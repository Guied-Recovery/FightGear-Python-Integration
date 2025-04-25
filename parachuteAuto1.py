import time
import math
import socket
from flightgear_python.fg_if import FDMConnection

# --- Funções de conversão e bearing ---
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
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2)*math.cos(dlon)
    bearing = math.atan2(y, x)
    return (rad2deg(bearing) + 360) % 360

# --- Classe para enviar comandos de flight controls via socket (Telnet) ---
class FGController:
    def __init__(self, host="localhost", port=5401):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

    def set_property(self, prop_path, value):
        cmd_str = f"set {prop_path} {value}\r\n"
        self.sock.sendall(cmd_str.encode('ascii'))
        # Se quiser ler resposta:
        # data = self.sock.recv(1024)

    def set_aileron(self, value):
        # Clampa o valor entre -1 e 1
        value = max(-1.0, min(1.0, value))
        self.set_property("/controls/flight/aileron", value)

    def set_elevator(self, value):
        # Clampa o valor entre -1 e 1
        value = max(-1.0, min(1.0, value))
        self.set_property("/controls/flight/elevator", value)

    def set_chute_cmd_norm(self, deployed=True):
        val = 1 if deployed else 0
        self.set_property("/fdm/jsbsim/systems/chute/chute-cmd-norm", val)

    def close(self):
        self.sock.close()

# --- Classe principal que recebe dados FDM e faz controle ---
class AutonomousParachute:
    def __init__(self, target_lat, target_lon, telnet_host="localhost", telnet_port=5401):
        # Conexão para enviar comandos
        self.fg = FGController(telnet_host, telnet_port)
        # Conexão para receber dados de voo
        self.fdm_conn = FDMConnection(fdm_version=24)
        self.fdm_conn.connect_rx('localhost', 5501, self.fdm_callback)

        # Ponto de pouso alvo
        self.target_lat = target_lat
        self.target_lon = target_lon

        # Flag para não abrir o paraquedas duas vezes
        self.parachute_opened = False
        # Exemplo de altitude para abrir: 4000m
        self.deploy_alt = 4000.0
        # Exemplo de altitude para flare final: 50m
        self.flare_alt = 50.0

    def fdm_callback(self, fdm_data, event_pipe):
        """
        Recebemos:
          - fdm_data.lat_rad (radianos)
          - fdm_data.lon_rad (radianos)
          - fdm_data.alt_m (metros)
          - fdm_data.psi_rad (yaw, heading, em radianos) -> se for fornecido
        """
        lat_deg = fdm_data.lat_rad * (180.0 / math.pi)
        lon_deg = fdm_data.lon_rad * (180.0 / math.pi)
        altitude = fdm_data.alt_m

        # Exemplo de debug:
        print(f"Pos atual: lat={lat_deg:.5f}, lon={lon_deg:.5f}, alt={altitude:.1f}m")

        # 1) Abrir paraquedas em altitude > 0 e < 4000m
        if altitude < self.deploy_alt and not self.parachute_opened:
            print("Abrindo paraquedas!")
            self.fg.set_chute_cmd_norm(True)
            self.parachute_opened = True

        # 2) Calcular rota se paraquedas estiver aberto
        if self.parachute_opened:
            # a) Calcular bearing para o alvo
            bearing_to_target = compute_bearing(lat_deg, lon_deg,
                                                self.target_lat, self.target_lon)

            # b) Heading atual: se "psi_rad" for confiável, use-o; senão, crie heurística
            heading_deg = (fdm_data.psi_rad * 180.0 / math.pi) % 360

            # c) Erro de rumo
            heading_error = bearing_to_target - heading_deg

            # Normalizar para intervalo [-180, 180]
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360

            # d) Controlar aileron com base no erro (controlador proporcional simples)
            Kp_ail = 0.02  # ajuste conforme necessidade
            aileron_cmd = Kp_ail * heading_error
            # Clampa em [-1, 1] dentro do set_aileron
            self.fg.set_aileron(aileron_cmd)

            # e) Definir elevator conforme estratégia de descida
            #    Ex: acima de 50m, mantenha elevator moderado (-0.3) para planeio
            #        abaixo de 50m, "flare" (1.0) para pouso suave
            if altitude > self.flare_alt:
                self.fg.set_elevator(-0.3)  # descer de forma moderada
            else:
                self.fg.set_elevator(1.0)   # flare final

        return fdm_data

    def start(self):
        self.fdm_conn.start()
        try:
            while True:
                time.sleep(0.05)  # evita loop infinito agressivo
        except KeyboardInterrupt:
            pass
        finally:
            self.fg.close()

# --- Script principal ---
if __name__ == "__main__":
    # Defina o local de pouso desejado (lat/lon em graus)
    TARGET_LAT = -79.616  # Exemplo: próximo ao KSFO
    TARGET_LON = 43.668

    autoparachute = AutonomousParachute(
        target_lat=TARGET_LAT,
        target_lon=TARGET_LON,
        telnet_host="localhost",
        telnet_port=5401
    )
    autoparachute.start()
