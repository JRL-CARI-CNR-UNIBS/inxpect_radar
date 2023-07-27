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
DEG_TO_RAD = 1 # 0.008726646259971648


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
    sensor_data_preproc: np.array = field(default=None, init=False)
    sensor_data_filtered: np.array = field(default=None, init=False)
    no_signal_samples: int = field(default=0, init=False)
    filter_win_length: int = field(default=100, init=True)
    n_cycles: int = field(default=0, init=False)
    ignore_win_length: int = field(default=10, init=True)


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
        # Read raw data from MODBUS registers
        distance = self.read_raw_data("distance", "distance")
        angle = self.read_raw_data("angle", "angle")
        sensor_data_new = np.array([distance, angle])

        # Save a window of samples, preprocess, and filter them
        self.n_cycles += 1
        if self.n_cycles == 1:
            self.sensor_data = np.reshape(sensor_data_new,(N_SENSOR_DATA,1))
            self.sensor_data_filtered = np.copy(self.sensor_data)
            self.sensor_data_preproc = np.copy(self.sensor_data)

        else:         
            if self.sensor_data.shape[1] < self.filter_win_length:
                self.sensor_data = np.append(self.sensor_data,np.reshape(sensor_data_new,(N_SENSOR_DATA,1)), axis=1)
                self.sensor_data_filtered = np.copy(self.sensor_data)
                self.sensor_data_preproc = np.copy(self.sensor_data)
            
            else:
                self.sensor_data[:,:-1] = self.sensor_data[:,1:]
                self.sensor_data[:,-1] = sensor_data_new

                latest_data = self.sensor_data[:,-self.ignore_win_length:]

                print("Latest data in window: ")
                print(latest_data)

                if self.human_detected(latest_data):
                    print("Person detected", end="\n")
                    
                    # for i in range(0,N_SENSOR_DATA): # remove unexpected zeros
                    #     null_condition = (latest_data[i] < 1e-3)
                    #     idx_zero = np.where(null_condition)[0]
                    #     idx_not_zero = np.where(np.invert(null_condition))[0]

                    #     print("Measurement #" + str(i) + " before processing: " + str(latest_data[i]), end="\n")
                    #     if len(idx_not_zero.tolist()) != 0 and len(idx_zero.tolist()) != 0:
                    #         latest_data[i][idx_zero] = np.mean(latest_data[i][idx_not_zero])
                    #     print("Measurement #" + str(i) + " after processing: " + str(latest_data[i]), end="\n\n")

                    # self.sensor_data_preproc[:,-self.ignore_win_length:] = latest_data

                    # self.moving_average_filter() # filter data

                else:
                    print("Person NOT detected", end="\n\n")
                    self.sensor_data_preproc[:,-self.ignore_win_length:] = np.zeros((N_SENSOR_DATA,self.ignore_win_length),dtype=float)

        # if not self.state_obs_initialized:
        #     self.initialize_state_observer(sensocr_data_new)
        # else:
        #     # if distance < 0.1 and self.no_signal_samples < 300:
        #     #     sensor_data_new = self.sensor_data
        #     #     self.no_signal_samples += 1
        #     # else:
        #     #     self.no_signal_samples = 0
        #     self.update_state_observer(sensor_data_new)
        #     # self.first_order_filter(sensor_data_new)

        return self.sensor_data[0,-1], self.sensor_data[1,-1], \
               self.sensor_data_preproc[0,-1], self.sensor_data_preproc[1,-1], \
               self.sensor_data_filtered[0,-1], self.sensor_data_filtered[1,-1]
        

    def read_raw_data(self, register_name, meas_type):
        self.create_connection()
        try:
            data_raw = self.client.read_holding_registers(REGISTER_ADDRESS[register_name], 1).registers
        except pymodbus.exceptions.ModbusIOException as ex_msg:
            raise Exception(ex_msg)
        if len(data_raw) == 1:
            if meas_type == "distance":
                return data_raw[0] * MM_TO_M
            elif meas_type == "angle":
                return data_raw[0] * DEG_TO_RAD
        else:
            raise ValueError("Invalid reading")


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
            self.sensor_data_filtered[i] = signal.lfilter(b,a,self.sensor_data_preproc[i,:]) #filter output using lfilter function


    def human_detected(self, data):
        n_dect = 0
        for i in range(0,data.shape[1]):
            if np.any(np.abs(data[:,i]) > 1e-3):
                n_dect +=1
        print("Number of detection in window: " + str(n_dect))

        return n_dect > 0.5*self.ignore_win_length