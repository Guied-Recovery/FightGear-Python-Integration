import time
import math
import socket
import csv
from flightgear_python.fg_if import FDMConnection

CSV_FILE = open("flight_data_controled.csv", "w", newline="")
CSV_WRITER = csv.writer(CSV_FILE)
CSV_WRITER.writerow([
    "system_time_s",      # tempo de execução do script (real)
    "sim_time_s",         # tempo interno da simulação do FG
    "latitude_deg",
    "longitude_deg",
    "altitude_m"
])
CSV_FILE.flush()
START_TIME = time.time()

PARACHUTE_DEPLOYED = False
PARACHUTE_DEPLOY_ALT = 4000.0

# --- Funções auxiliares ---
def is_valid_position(lat, lon, alt):
    """Verifica se os dados FDM são fisicamente plausíveis."""
    return (-90 <= lat <= 90) and (-180 <= lon <= 180) and (0 < alt < 20000)


# --- Controlador Telnet ---
class FGController:
    def __init__(self, host="localhost", port=5401):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        print("[INFO] Conectado ao FlightGear Telnet.")

    def set_property(self, path, value):
        cmd = f"set {path} {value}\r\n"
        self.sock.sendall(cmd.encode("ascii"))

    def set_aileron(self, value):
        self.set_property("/controls/flight/aileron", max(-1, min(1, value)))

    def set_elevator(self, value):
        self.set_property("/controls/flight/elevator", max(-1, min(1, value)))

    def set_chute_cmd_norm(self, deployed=True):
        self.set_property("/fdm/jsbsim/systems/chute/chute-cmd-norm", 1 if deployed else 0)

    def close(self):
        self.sock.close()
        print("[INFO] Conexão Telnet encerrada.")


FG = FGController()

# --- Callback de dados FDM ---
def fdm_callback(fdm_data, event_pipe):
    global PARACHUTE_DEPLOYED

    lat = math.degrees(fdm_data.lat_rad)
    lon = math.degrees(fdm_data.lon_rad)
    alt = fdm_data.alt_m
    t_system = time.time() - START_TIME

    # Obtém tempo de simulação (segundos desde o início do FG)
    try:
        t_sim = fdm_data.sim_time_sec
    except AttributeError:
        # Alguns builds antigos do FDM não possuem esse campo
        t_sim = None

    # Verifica se dados são válidos antes de salvar e controlar
    if not is_valid_position(lat, lon, alt):
        print(f"[WARN] Dados inválidos: lat={lat:.4f}, lon={lon:.4f}, alt={alt:.1f}")
        return fdm_data

    CSV_WRITER.writerow([t_system, t_sim, lat, lon, alt])
    CSV_FILE.flush()

    if not PARACHUTE_DEPLOYED and alt < PARACHUTE_DEPLOY_ALT:
        print("[INFO] Abrindo paraquedas.")
        FG.set_chute_cmd_norm(True)
        PARACHUTE_DEPLOYED = True
        time.sleep(1.5)  # dá tempo do FDM estabilizar

    if PARACHUTE_DEPLOYED:
        # Gira em círculos suaves
        aileron = 0.3 * math.sin(t_system * 0.2)
        elevator = -0.25
        FG.set_aileron(aileron)
        FG.set_elevator(elevator)
        print(f"[CTRL] aileron={aileron:.2f}, elevator={elevator:.2f}, alt={alt:.1f}m")

    return fdm_data


class CircularParaglider:
    def __init__(self):
        self.fdm_conn = FDMConnection(fdm_version=24)
        self.fdm_conn.connect_rx("localhost", 5501, fdm_callback)

    def start(self):
        print("[INFO] Aguardando inicialização do FDM...")
        time.sleep(5)
        print("[INFO] Iniciando controle circular do paraglider...")
        self.fdm_conn.start()
        try:
            while True:
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n[INFO] Encerrando simulação...")
        finally:
            CSV_FILE.close()
            FG.close()
            print("[INFO] Arquivo CSV salvo com sucesso.")


if __name__ == "__main__":
    controller = CircularParaglider()
    controller.start()
