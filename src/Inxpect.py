import pymodbus.exceptions
from pymodbus.client import ModbusTcpClient
from dataclasses import dataclass, field
from typing import Optional
import numpy as np
import ipaddress
from scipy import signal

REGISTER_ADDRESS = {"distance": 41014, "angle": 41015}
N_SENSOR_DATA = 2
MM_TO_M = 1e-3
DEG_TO_RAD = 0.008726646259971648


@dataclass
class RadarData:
    distance: float
    angle: float

    def get_data(self):
        return self.distance, self.angle

    def get_distance(self):
        return self.distance

    def get_angle(self):
        return self.angle


@dataclass
class Inxpect:
    ip: str

    lambda_: Optional[float] = field(default=1, init=True)

    client: ModbusTcpClient = field(default=None, init=False)

    state_obs_initialized: bool = field(default=False, init=False)
    sensor_data: np.array = field(default=None, init=False)
    sensor_data_filtered: np.array = field(default=None, init=False)
    no_signal_samples: int = field(default=0, init=False)
    filter_win_length: int = field(default=1000, init=True)
    n_cycles: int = field(default=0, init=False)

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

        sensor_data_new = np.array([distance, angle])

        self.n_cycles += 1
        if self.n_cycles == 1:
            self.sensor_data = np.reshape(sensor_data_new,(N_SENSOR_DATA,1))
            self.sensor_data_filtered = self.sensor_data
        else:          
            if self.sensor_data.shape[1] < self.filter_win_length:
                self.sensor_data = np.append(self.sensor_data,np.reshape(sensor_data_new,(N_SENSOR_DATA,1)), axis=1)
                self.sensor_data_filtered = self.sensor_data
            
            else:
                self.sensor_data[:,:-1] = self.sensor_data[:,1:]
                self.sensor_data[:,-1] = sensor_data_new
                self.moving_average_filter()

        # if not self.state_obs_initialized:
        #     self.initialize_state_observer(sensor_data_new)
        # else:
        #     # if distance < 0.1 and self.no_signal_samples < 300:
        #     #     sensor_data_new = self.sensor_data
        #     #     self.no_signal_samples += 1
        #     # else:
        #     #     self.no_signal_samples = 0
        #     self.update_state_observer(sensor_data_new)
        #     # self.first_order_filter(sensor_data_new)

        return self.sensor_data_filtered[0,-1], self.sensor_data_filtered[1,-1]
        

    def initialize_state_observer(self, sensor_data_new: np.array) -> None:
        self.sensor_data = sensor_data_new
        self.state_obs_initialized = True

    def update_state_observer(self, sensor_data_new: np.array):
        self.sensor_data = self.sensor_data + self.lambda_ * (sensor_data_new - self.sensor_data)

    def first_order_filter(self, sensor_data_new: np.array):
        self.sensor_data = self.lambda_ * self.sensor_data + (1 - self.lambda_) * sensor_data_new

    def moving_average_filter(self):
        b = (np.ones(self.filter_win_length))/self.filter_win_length #numerator co-effs of filter transfer function
        a = np.ones(1)  #denominator co-effs of filter transfer function
        
        for i in range(0,N_SENSOR_DATA):
            # self.sensor_data_filtered[i] = signal.convolve(self.sensor_data[i,:],b) #filter output using convolution
            self.sensor_data_filtered[i] = signal.lfilter(b,a,self.sensor_data[i,:]) #filter output using lfilter function