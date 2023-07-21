import pymodbus.exceptions
from pymodbus.client import ModbusTcpClient
from dataclasses import dataclass, field
import ipaddress

REGISTER_ADDRESS = {"distance": 41014, "angle": 41015}


@dataclass
class Inxpect:
    ip: str
    client: ModbusTcpClient = field(default=None, init=False)

    def __pos__(self):
        try:
            ipaddress.ip_address(self.ip)
        except ValueError("Invalid IP Address"):
            raise ValueError("Invalid IP Address")
        self.client = ModbusTcpClient(self.ip, 502)

    def create_connection(self):
        self.client.connect()

    def read(self):
        self.create_connection()
        try:
            data_raw = self.client.read_holding_registers(REGISTER_ADDRESS["distance"], 1)
        except pymodbus.exceptions.ModbusIOException as ex_msg:
            raise Exception(ex_msg)
        if len(data_raw) == 1:
            distance = data_raw[0]
        else:
            raise ValueError("Invalid reading")

        self.create_connection()
        try:
            data_raw = self.client.read_holding_registers(REGISTER_ADDRESS["angle"], 1)
        except pymodbus.exceptions.ModbusIOException as ex_msg:
            raise Exception(ex_msg)
        if len(data_raw) == 1:
            angle = data_raw[0]
        else:
            raise ValueError("Invalid reading")

        return distance, angle
