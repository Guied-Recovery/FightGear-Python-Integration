import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import socket
from flightgear_python.fg_if import FDMConnection
import time

# --- Funções auxiliares ---
def deg2rad(d):
    return math.pi * d / 180.0

def rad2deg(r):
    return 180.0 * r / math.pi

def compute_bearing(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    lat1 = deg2rad(lat1_deg)
    lon1 = deg2rad(lon1_deg)
    lat2 = deg2rad(lat2_deg)
    lon2 = deg2rad(lon2_deg)
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    bearing = math.atan2(y, x)
    return (rad2deg(bearing) + 360) % 360

def haversine(lat1, lon1, lat2, lon2):
    # distância em metros
    R = 6371000
    phi1 = deg2rad(lat1)
    phi2 = deg2rad(lat2)
    dphi = deg2rad(lat2-lat1)
    dlambda = deg2rad(lon2-lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R*c

# --- Controle via Telnet ---
class FGController:
    def __init__(self, host="localhost", port=5401):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

    def set_property(self, prop_path, value):
        cmd_str = f"set {prop_path} {value}\r\n"
        self.sock.sendall(cmd_str.encode('ascii'))

    def set_aileron(self, value):
        value = max(-1.0, min(1.0, value))
        self.set_property("/controls/flight/aileron", value)

    def set_elevator(self, value):
        value = max(-1.0, min(1.0, value))
        self.set_property("/controls/flight/elevator", value)

    def set_chute_cmd_norm(self, deployed=True):
        val = 1 if deployed else 0
        self.set_property("/fdm/jsbsim/systems/chute/chute-cmd-norm", val)

    def close(self):
        self.sock.close()

# --- Ambiente Gym para RL ---
class ParachuteEnv(gym.Env):
    def __init__(self, target_lat, target_lon):
        super().__init__()
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.fg = FGController()
        self.fdm_conn = FDMConnection(fdm_version=24)
        self.fdm_conn.connect_rx('localhost', 5501, self._fdm_callback)

        # Estado: [lat, lon, alt, heading, vel_x, vel_y, vel_z]
        high = np.array([180, 180, 10000, 360, 100, 100, 100], dtype=np.float32)
        low  = np.array([-180, -180, 0, 0, -100, -100, -100], dtype=np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        # Ações contínuas: aileron e elevator
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)

        self.state = None
        self.done = False
        self.parachute_opened = False

    def _fdm_callback(self, fdm_data, event_pipe):
        lat = fdm_data.lat_rad * 180/math.pi
        lon = fdm_data.lon_rad * 180/math.pi
        alt = fdm_data.alt_m
        heading = (fdm_data.psi_rad * 180/math.pi) % 360
        vel_x = fdm_data.Va * math.cos(fdm_data.psi_rad)
        vel_y = fdm_data.Va * math.sin(fdm_data.psi_rad)
        vel_z = -fdm_data.Vz if hasattr(fdm_data, "Vz") else 0.0
        self.state = np.array([lat, lon, alt, heading, vel_x, vel_y, vel_z], dtype=np.float32)
        return fdm_data

    def reset(self):
        # reinicia variáveis
        self.done = False
        self.parachute_opened = False
        # abre chute se quiser iniciar já aberto
        self.fg.set_chute_cmd_norm(False)
        self.state = None
        # espera receber primeiros dados FDM
        while self.state is None:
            time.sleep(0.05)
        return self.state

    def step(self, action):
        aileron, elevator = action
        self.fg.set_aileron(aileron)
        self.fg.set_elevator(elevator)

        # abre paraquedas se altura < 4000
        if self.state[2] < 4000 and not self.parachute_opened:
            self.fg.set_chute_cmd_norm(True)
            self.parachute_opened = True

        # espera próximo frame
        time.sleep(0.05)

        # cálculo da recompensa
        dist = haversine(self.state[0], self.state[1], self.target_lat, self.target_lon)
        reward = -dist/1000.0  # penaliza distância (quanto menor, melhor)

        # termina episódio se altitude < 0
        self.done = self.state[2] <= 0

        return self.state, reward, self.done, {}

    def close(self):
        self.fg.close()

# --- Exemplo de uso ---
if __name__ == "__main__":
    import stable_baselines3 as sb3

    TARGET_LAT = -79.616
    TARGET_LON = 43.668

    env = ParachuteEnv(TARGET_LAT, TARGET_LON)

    model = sb3.PPO("MlpPolicy", env, verbose=1)
    # Treinamento (pode demorar muito dependendo da simulação)
    # model.learn(total_timesteps=10000)

    obs = env.reset()
    print("Estado inicial:", obs)
    for _ in range(10):
        action = env.action_space.sample()  # ação aleatória
        obs, reward, done, info = env.step(action)
        print(obs, reward, done)
    done = False
    while not done:
        action = env.action_space.sample()  # teste com ação aleatória
        obs, reward, done, info = env.step(action)
        print(f"Alt={obs[2]:.1f} m, Dist ao alvo={haversine(obs[0], obs[1], TARGET_LAT, TARGET_LON):.1f} m, Reward={reward:.2f}")
