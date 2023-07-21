import pymodbus.exceptions
from pymodbus.client import ModbusTcpClient
from dataclasses import dataclass, field
import ipaddress

REGISTER_ADDRESS = {"distance": 41014, "angle": 41015}
MM_TO_M = 1e-3
DEG_TO_RAD = 0.008726646259971648

@dataclass
class Inxpect:
    ip: str
    client: ModbusTcpClient = field(default=None, init=False)

    def __post_init__(self):
        try:
            ipaddress.ip_address(self.ip)
        except ValueError("Invalid IP Address"):
            raise ValueError("Invalid IP Address")
        self.client = ModbusTcpClient(self.ip, 502)

    def create_connection(self):
        try:
            self.client.connect()
        except Exception:
            raise Exception(f"Unable to connect to: {self.ip}")

    def read(self):
        self.create_connection()
        try:
            data_raw = self.client.read_holding_registers(REGISTER_ADDRESS["distance"], 1).registers
        except pymodbus.exceptions.ModbusIOException as ex_msg:
            raise Exception(ex_msg)
        if len(data_raw) == 1:
            distance = data_raw[0] * MM_TO_M
        else:
            raise ValueError("Invalid reading")

        self.create_connection()
        try:
            data_raw = self.client.read_holding_registers(REGISTER_ADDRESS["angle"], 1).registers
        except pymodbus.exceptions.ModbusIOException as ex_msg:
            raise Exception(ex_msg)
        if len(data_raw) == 1:
            angle = data_raw[0] * DEG_TO_RAD
        else:
            raise ValueError("Invalid reading")

        return distance, angle
