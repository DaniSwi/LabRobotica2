# lab 2 robotica - nav reactiva con kalman y media movil
# adaptado al comportamiento real de los sensores del epuck

from controller import Robot, DistanceSensor
import math
import csv

TIME_STEP    = 64          # ms base de webots
MAX_SPEED    = 6.28        # tope del motor en rad/s
WHEEL_RADIUS = 0.0205
AXLE_LENGTH  = 0.052

# los sensores infrarrojos se ciegan mas alla de los 5cm en el simulador
# asi que los umbrales de reaccion tienen que ser cortitos
SAFETY_DISTANCE_CM     = 3.5  # a esta distancia entra en panico y frena
HYSTERESIS_DISTANCE_CM = 4.5  # le pedimos un poco mas de margen para salir del giro y no quedarse tiritando

MA_WINDOW = 5

# el q y r de kalman. si el filtro responde muy lento o vibra mucho, es aca
Q_PROCESS  = 1e-3    # confio harto en la odometria cuando va en linea recta
R_MEASURE  = 0.5     # ruido estimado del infrarrojo


def raw_to_cm(raw: float) -> float:
    # la curva de este sensor no es lineal. si da 0 es pq no hay nada a la vista
    if raw <= 0:
        return 5.0
    
    # constante sacada al ojo ajustando la funcion inversa
    k = 900.0
    d = k / raw
    # capeamos por arriba y por abajo para no mandar basura al filtro
    return max(0.1, min(d, 5.0))


class MovingAverage:
    # clase basica para limpiar la señal, la usamos mas que nada para comparar dps
    def __init__(self, window: int = MA_WINDOW):
        self.window = window
        self.buffer: list = []

    def update(self, value: float) -> float:
        self.buffer.append(value)
        if len(self.buffer) > self.window:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)


class KalmanFilter1D:
    # aca estimamos la distancia frontal aislando un poco el ruido del entorno
    def __init__(self, initial_dist: float = 5.0,
                 q: float = Q_PROCESS, r: float = R_MEASURE):
        self.d_hat = initial_dist
        self.P     = 1.0 # incertidumbre inicial, se acomoda sola rapido
        self.Q     = q
        self.R     = r

    def predict(self, delta_d: float) -> float:
        # si las ruedas me dicen que avance, la distancia a la pared deberia bajar
        self.d_hat = self.d_hat - delta_d 
        self.P = self.P + self.Q
        return self.d_hat

    def correct(self, measurement: float) -> float:
        # ponderamos segun a quien le creemos mas (modelo vs sensor)
        K = self.P / (self.P + self.R)
        self.d_hat = self.d_hat + K * (measurement - self.d_hat)
        self.P = (1 - K) * self.P
        return self.d_hat

    @property
    def kalman_gain(self) -> float:
        return self.P / (self.P + self.R)


class SignalLogger:
    # bota todo a un csv para graficar despues en el informe
    def __init__(self, filename: str = "sensor_log.csv"):
        self.filename = filename
        self.records: list = []
        self.headers = [
            "time_s", "ps0_raw", "ps7_raw", "ps1_raw", "ps6_raw",
            "enc_left_rad", "enc_right_rad", "delta_d_cm", "delta_theta_rad",
            "front_raw_cm", "front_filtered_cm", "front_kalman_cm",
            "kalman_gain", "action"
        ]

    def log(self, **kwargs):
        self.records.append(kwargs)

    def save(self):
        if not self.records:
            return
        with open(self.filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.headers, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.records)


class EPuckController:
    def __init__(self):
        self.robot = Robot()
        self.ts    = TIME_STEP

        self.left_motor  = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # metemos los 8 sensores en una lista para iterarlos facil
        self.ps: list[DistanceSensor] = []
        for i in range(8):
            sensor = self.robot.getDevice(f"ps{i}")
            sensor.enable(self.ts)
            self.ps.append(sensor)

        self.enc_left  = self.robot.getDevice("left wheel sensor")
        self.enc_right = self.robot.getDevice("right wheel sensor")
        self.enc_left.enable(self.ts)
        self.enc_right.enable(self.ts)

        self.enc_left_prev  = 0.0
        self.enc_right_prev = 0.0
        self.encoders_initialized = False

        self.ma_filter = MovingAverage(window=MA_WINDOW)
        self.kf        = KalmanFilter1D(initial_dist=5.0)

        self.estado   = "AVANZANDO"
        self.dir_giro = ""

        self.logger     = SignalLogger("sensor_log.csv")
        self.step_count = 0
        self.time_s     = 0.0

    def read_sensors(self):
        return [s.getValue() for s in self.ps]

    def estimate_kinematics(self) -> tuple[float, float]:
        # sacamos el delta lineal y angular desde los encoders
        el = self.enc_left.getValue()
        er = self.enc_right.getValue()

        if not self.encoders_initialized:
            self.enc_left_prev  = el
            self.enc_right_prev = er
            self.encoders_initialized = True
            return 0.0, 0.0

        delta_l = el - self.enc_left_prev
        delta_r = er - self.enc_right_prev
        self.enc_left_prev  = el
        self.enc_right_prev = er

        # formula de traccion diferencial estandar
        delta_d_m   = WHEEL_RADIUS * (delta_l + delta_r) / 2.0
        delta_theta = WHEEL_RADIUS * (delta_r - delta_l) / AXLE_LENGTH
        
        return delta_d_m * 100.0, delta_theta

    def frontal_raw_cm(self, ps_values: list) -> float:
        # ps0 y ps7 son los ojitos delanteros, nos quedamos con el peor caso
        raw_front = max(ps_values[0], ps_values[7])
        return raw_to_cm(raw_front)

    def decide_action(self, front_cm: float, ps_values: list) -> tuple[float, float, str]:
        # sumamos los laterales a ver que lado esta mas comprometido
        left_side  = ps_values[5] + ps_values[6]
        right_side = ps_values[1] + ps_values[2]

        if self.estado == "AVANZANDO":
            if front_cm < SAFETY_DISTANCE_CM:
                self.estado = "GIRANDO"
                # miramos una sola vez pa donde escapar y lo guardamos
                self.dir_giro = "izq" if right_side > left_side else "der"
                print(f"\n [t={self.time_s:.1f}s] OBSTÁCULO DETECTADO a {front_cm:.1f} cm")
                print(f"   -> Cambiando estado a: GIRANDO hacia la {self.dir_giro.upper()}")

        elif self.estado == "GIRANDO":
            # no soltamos el giro apenas pasemos el umbral de choque
            # le damos un pelin mas de holgura por la histeresis
            if front_cm > HYSTERESIS_DISTANCE_CM:
                self.estado = "AVANZANDO"
                self.dir_giro = ""
                print(f"\n[t={self.time_s:.1f}s] CAMINO DESPEJADO (Distancia libre: {front_cm:.1f} cm)")
                print(f"   -> Cambiando estado a: AVANZANDO")

        if self.estado == "AVANZANDO":
            # ir al maximo lo hace inestable en los giros cerrados, mejor un 70%
            vl = MAX_SPEED * 0.7
            vr = MAX_SPEED * 0.7
            action = "avanzar"
        else:
            if self.dir_giro == "izq":
                vl = -MAX_SPEED * 0.4
                vr =  MAX_SPEED * 0.4
                action = "girar_izq"
            else:
                vl =  MAX_SPEED * 0.4
                vr = -MAX_SPEED * 0.4
                action = "girar_der"

        return vl, vr, action

    def step(self) -> bool:
        if self.robot.step(self.ts) == -1:
            return False

        self.time_s = self.step_count * (self.ts / 1000.0)
        self.step_count += 1

        ps_vals = self.read_sensors()
        delta_d_cm, delta_theta = self.estimate_kinematics()
        front_raw = self.frontal_raw_cm(ps_vals)

        # la media movil la dejamos corriendo sola para poder plotearla despues
        front_filtered = self.ma_filter.update(front_raw)

        # truco feo pero util: si el robot esta rotando, el modelo 1d falla pesimo
        # le subimos artificialmente el ruido de proceso pa que no le crea a la cinematica
        if abs(delta_theta) > 0.05:
            self.kf.Q = Q_PROCESS * 50
        else:
            self.kf.Q = Q_PROCESS

        # le pasamos la señal sucia directo al kalman como dice la teoria
        self.kf.predict(delta_d_cm)
        front_kalman = self.kf.correct(front_raw)
        gain = self.kf.kalman_gain

        vl, vr, action = self.decide_action(front_kalman, ps_vals)

        self.left_motor.setVelocity(vl)
        self.right_motor.setVelocity(vr)

        self.logger.log(
            time_s            = round(self.time_s, 4),
            ps0_raw           = round(ps_vals[0], 2),
            ps7_raw           = round(ps_vals[7], 2),
            ps1_raw           = round(ps_vals[1], 2),
            ps6_raw           = round(ps_vals[6], 2),
            enc_left_rad      = round(self.enc_left.getValue(), 4),
            enc_right_rad     = round(self.enc_right.getValue(), 4),
            delta_d_cm        = round(delta_d_cm, 4),
            delta_theta_rad   = round(delta_theta, 4),
            front_raw_cm      = round(front_raw, 3),
            front_filtered_cm = round(front_filtered, 3),
            front_kalman_cm   = round(front_kalman, 3),
            kalman_gain       = round(gain, 5),
            action            = action
        )

        return True

    def run(self):
        try:
            while self.step():
                pass
        except KeyboardInterrupt:
            # matar el script limpio si hacemos ctrl+c
            pass
        finally:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            self.logger.save()

if __name__ == "__main__":
    controller = EPuckController()
    controller.run()
