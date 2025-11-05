import time
import math
import socket
import csv
from flightgear_python.fg_if import FDMConnection

CSV_FILE = open("flight_data_no_control.csv", "w", newline="")
CSV_WRITER = csv.writer(CSV_FILE)
CSV_WRITER.writerow([
    "system_time_s",      # tempo real do sistema
    "sim_time_s",         # tempo da simulação (FG)
    "latitude_deg",
    "longitude_deg",
    "altitude_m"
])
CSV_FILE.flush()
START_TIME = time.time()

PARACHUTE_DEPLOYED = False
PARACHUTE_DEPLOY_ALT = 4000.0


# --- Função auxiliar ---
def is_valid_position(lat, lon, alt):
    """Verifica se os dados FDM são fisicamente plausíveis."""
    return (-90 <= lat <= 90) and (-180 <= lon <= 180) and (0 < alt < 20000)


# --- Controlador Telnet (somente para abrir paraquedas) ---
class FGController:
    def __init__(self, host="localhost", port=5401):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        print("[INFO] Conectado ao FlightGear Telnet.")

    def set_property(self, path, value):
        cmd = f"set {path} {value}\r\n"
        self.sock.sendall(cmd.encode("ascii"))

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

    try:
        t_sim = fdm_data.sim_time_sec
    except AttributeError:
        t_sim = None

    if not is_valid_position(lat, lon, alt):
        print(f"[WARN] Dados inválidos: lat={lat:.4f}, lon={lon:.4f}, alt={alt:.1f}")
        return fdm_data

    CSV_WRITER.writerow([t_system, t_sim, lat, lon, alt])
    CSV_FILE.flush()

    # Abre paraquedas automaticamente ao atingir altitude limite
    if not PARACHUTE_DEPLOYED and alt < PARACHUTE_DEPLOY_ALT:
        print("[INFO] Abrindo paraquedas.")
        FG.set_chute_cmd_norm(True)
        PARACHUTE_DEPLOYED = True

    # Sem controle: não envia comandos de aileron ou elevator
    return fdm_data


# --- Classe principal da simulação ---
class FreeFallParaglider:
    def __init__(self):
        self.fdm_conn = FDMConnection(fdm_version=24)
        self.fdm_conn.connect_rx("localhost", 5501, fdm_callback)

    def start(self):
        print("[INFO] Aguardando inicialização do FDM...")
        time.sleep(5)
        print("[INFO] Iniciando voo sem controle (queda livre + paraquedas)...")
        self.fdm_conn.start()
        try:
            while True:
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n[INFO] Encerrando simulação...")
        finally:
            CSV_FILE.close()
            FG.close()
            print("[INFO] Arquivo CSV salvo com sucesso (flight_data_no_control.csv).")


if __name__ == "__main__":
    controller = FreeFallParaglider()
    controller.start()
