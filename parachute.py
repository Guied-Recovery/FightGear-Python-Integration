import time
import socket
from flightgear_python.fg_if import FDMConnection


def set_parachute_main_deployed(host="localhost", port=5401):
    """
    Define se o paraquedas principal está aberto (True) ou fechado (False)
    diretamente via socket TCP.
    """
    command_str = "set /fdm/jsbsim/systems/chute/chute-cmd-norm 1\r\n"

    # Conecta no socket TCP na porta do servidor "telnet" do FlightGear
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(command_str.encode('ascii'))
        # Se quiser ler alguma resposta do FG, descomente:
        # data = s.recv(1024)
        # print("Resposta FG:", data.decode('ascii'))


class FlightGearConnector:
    def __init__(self):
        # Conexão com FlightGear via flightgear_python
        self.fdm_conn = FDMConnection(fdm_version=24)

        # Recebendo dados (FDM) de altitude, velocidade, etc.
        # "connect_rx" => para receber do FG, usando socket (out, 30, localhost, 5501, udp)
        self.fdm_conn.connect_rx('localhost', 5501, self.fdm_callback)

        # Flag para garantir que abriremos o paraquedas só uma vez
        self.parachute_opened = False

        # Defina a altitude limite (em metros) abaixo da qual o paraquedas será aberto
        self.altitude_trigger = 3000.0

    def fdm_callback(self, fdm_data, event_pipe):
        """
        Este callback é chamado toda vez que chegam dados de voo (FDM).
        fdm_data.alt_m = altitude em metros.
        """
        altitude = fdm_data.alt_m
        lat_rad = fdm_data.lat_rad
        lon_rad = fdm_data.lon_rad
        # Exemplo de debug
        print(f"Altitude atual: {altitude:.2f} m")
        print(f"longitude: {lat_rad} m")
        print(f"latitude: {lon_rad} m")

        # Se a altitude for < 3000 m e ainda não abrimos o paraquedas, abra
        if altitude < self.altitude_trigger and not self.parachute_opened:
            print("Abrindo paraquedas!")
            set_parachute_main_deployed()  # envia comando para abrir
            self.parachute_opened = True

        return fdm_data  # Retorna sem alterações

    def start(self):
        # Inicia a thread que recebe FDM
        self.fdm_conn.start()
        # Loop principal do script
        while True:
            # Aguarda um pouquinho para não sobrecarregar CPU
            time.sleep(0.05)

if __name__ == '__main__':
    fg_connector = FlightGearConnector()
    fg_connector.start()
