from pymodbus.client import ModbusTcpClient
from dataclasses import dataclass, field
import ipaddress

@dataclass
class InxpectNetwork:
    ip: str
    client: ModbusTcpClient = field(default=None, init=False)
    radars: dict = field(default=None, init=False)
    n_radars: int = field(default=0, init=False)
    state: dict = field(default=None, init=False)

    def __post_init__(self):
        try:
            ipaddress.ip_address(self.ip)
        except ValueError("Invalid IP Address"):
            raise ValueError("Invalid IP Address")
        
        self.client = ModbusTcpClient(self.ip, 502)
        try:
            self.client.connect()
        except Exception:
            raise Exception(f"Unable to connect to: {self.ip}")
        
        self.radars = {}
        self.state = {}
        

    def add_radar(self, new_radar):
        self.radars[new_radar.name] = new_radar
        self.n_radars += 1


    def update_state(self):
        for i in range(self.n_radars):
            distance, angle, \
            distance_preproc, angle_preproc, \
            distance_filtered, angle_filtered = self.radars["Radar" + str(i+1)].read_windowed()

            self.state["Radar" + str(i+1)] = {"distance" : distance, "angle": angle, \
                                            "distance_preproc" : distance_preproc, "angle_preproc": angle_preproc, \
                                            "distance_filtered" : distance_filtered, "angle_filtered": angle_filtered}




