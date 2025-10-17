import socket
import time

class FGEnvironmentReader:
    def __init__(self, host="localhost", port=5401):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.sock.settimeout(2)
        # Ir para diretório /environment
        self.sock.sendall(b"cd /environment\r\n")
        time.sleep(0.1)
        self.sock.recv(1024)  # limpar buffer inicial

    def get_property(self, prop):
        self.sock.sendall(f"get {prop}\r\n".encode("ascii"))
        data = self.sock.recv(1024).decode("ascii").strip()
        if "=" in data:
            try:
                return float(data.split("=")[1].strip().strip("'"))
            except:
                return None
        return None

    def read_environment(self):
        props = [
            "wind-speed-kt",
            "wind-from-heading-deg",
            "wind-from-north-fps",
            "wind-from-east-fps",
            "wind-from-down-fps",
            "temperature-degc",
            "pressure-inhg",
            "relative-humidity",
            "visibility-m",
            "ground-elevation-m",
            "gravitational-acceleration-mps2"
        ]
        result = {}
        for p in props:
            result[p] = self.get_property(p)
        return result

    def close(self):
        self.sock.close()


if __name__ == "__main__":
    fg = FGEnvironmentReader()
    try:
        while True:
            env = fg.read_environment()
            print("------------------------------------------------------------")
            print("🌤️ Ambiente FlightGear (tempo real):")
            print(f"💨 Vento: {env['wind-speed-kt']} kt a {env['wind-from-heading-deg']}°")
            print(f"🧭 Componentes vento: N={env['wind-from-north-fps']} fps, E={env['wind-from-east-fps']} fps, D={env['wind-from-down-fps']} fps")
            print(f"🌡️ Temperatura: {env['temperature-degc']} °C")
            print(f"⏱️ Pressão: {env['pressure-inhg']} inHg")
            print(f"💧 Umidade: {env['relative-humidity']} %")
            print(f"👁️ Visibilidade: {env['visibility-m']} m")
            print(f"🌍 Elevação do solo: {env['ground-elevation-m']} m")
            print(f"⚡ Gravidade: {env['gravitational-acceleration-mps2']} m/s²")
            time.sleep(1)
    except KeyboardInterrupt:
        fg.close()
        print("Encerrado.")
