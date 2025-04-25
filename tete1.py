import time
import math
from flightgear_python.fg_if import FDMConnection


class FlightGearControl:
    def __init__(self):
        self.start_time = time.time()

        # Cria conexão
        self.fdm_conn = FDMConnection(fdm_version=24)

        # RX (recebe estrutura de dados e atualiza com callback)
        self.fdm_conn.connect_rx('localhost', 5501, self.fdm_callback)

        # TX (envia para o FlightGear)
        self.fdm_conn.connect_tx('localhost', 5502)

    def fdm_callback(self, fdm_data, event_pipe):
        t = time.time() - self.start_time

        # Comandos de controle
        fdm_data.elevator = 0.1 * math.sin(t)
        fdm_data.left_aileron = 0.1 * math.sin(t / 2)
        fdm_data.right_aileron = 0.1 * math.sin(t / 2)
        fdm_data.rudder = 0.05 * math.sin(t / 3)
        fdm_data.throttle = 0.8

        return fdm_data

    def run(self):
        self.fdm_conn.start()  # Inicia o loop RX/TX


if __name__ == '__main__':
    controller = FlightGearControl()
    controller.run()
