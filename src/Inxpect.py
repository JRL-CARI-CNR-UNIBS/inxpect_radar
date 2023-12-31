import pymodbus.exceptions
from pymodbus.client import ModbusTcpClient
from dataclasses import dataclass, field
import numpy as np
from scipy import signal
import time

N_SENSOR_DATA = 2
MM_TO_M = 1e-3
DEG_TO_RAD = 1 # 0.008726646259971648 # the angle is still expressed in deg


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
    name: str
    client: ModbusTcpClient
    modbus_fields: dict

    lambda_: float = field(default=1, init=True)
    state_obs_initialized: bool = field(default=False, init=False)
    sensor_data: np.array = field(default=None, init=False)
    sensor_data_preproc: np.array = field(default=None, init=False)
    sensor_data_filtered: np.array = field(default=None, init=False)
    no_signal_samples: int = field(default=0, init=False)
    filter_win_length: int = field(default=20, init=True) # window for moving-average filtering
    n_cycles: int = field(default=0, init=False)
    ignore_win_length: int = field(default=5, init=True) # window for null outlier rejection
    is_detecting: bool = field(default=False, init=True) # is the sensor detecting the human along the current window?
    time_old: float = field(default=0, init=False)
    rejection_time: float = field(default=0, init=False)
    rejection_time_thresh: float = field(default=2.0, init=True) # threshold in seconds to ignore null samples


    def __post_init__(self):
        self.sensor_data = np.zeros((N_SENSOR_DATA,1), dtype=float)
        self.sensor_data_preproc = np.zeros((N_SENSOR_DATA,1), dtype=float)
        self.sensor_data_filtered = np.zeros((N_SENSOR_DATA,1), dtype=float)
        self.time_old = time.time()

    #TODO to be removed
    def read_windowed(self):
        print(self.name + " reading...")
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
                # Slide window
                self.sensor_data[:,:-1] = self.sensor_data[:,1:]
                self.sensor_data_preproc[:,:-1] = self.sensor_data_preproc[:,1:]

                # Add new raw sample
                self.sensor_data[:,-1] = sensor_data_new

                # Get self.ignore_win_length latest raw samples
                latest_data = self.sensor_data[:,-self.ignore_win_length:]

                # print("Latest data in window: ")
                # print(latest_data)

                if self.human_detected(latest_data):
                    print("Person detected", end="\n")

                    # Add new preprocessed sample
                    self.sensor_data_preproc[:,-1] = sensor_data_new

                    # Activate filtering after a given number of cycles (to fill the whole filtering window)
                    if self.n_cycles > self.filter_win_length:
                        self.remove_zeros(latest_data)

                    # Filter data using moving-average filter                   
                    self.moving_average_filter()

                    # Filter data using either state-observer or first-order LPF
                    # if not self.state_obs_initialized:
                    #     self.initialize_state_observer(sensor_data_new)
                    # else:
                    #     self.update_state_observer(sensor_data_new)
                    #     # self.first_order_filter(sensor_data_new)

                    self.is_detecting = True

                else:
                    print("Person NOT detected", end="\n\n")

                    # Ignore new sample from sensor data
                    if self.is_detecting: # if the sensor was detecting, keep the latest measured value
                        self.sensor_data_preproc[:,-1] = self.sensor_data_preproc[:,-2]
                        self.sensor_data_filtered[:,-1] = self.sensor_data_filtered[:,-2]
                    else:   # if the sensor was not detecting, add a zero measurement        
                        self.sensor_data_preproc[:,-1] = np.zeros((N_SENSOR_DATA,),dtype=float)
                        self.sensor_data_filtered[:,-1] = np.zeros((N_SENSOR_DATA,),dtype=float)
                    
                    if self.count_detections(latest_data) == 0:
                        self.is_detecting = False

        return self.sensor_data[0,-1], self.sensor_data[1,-1], \
               self.sensor_data_preproc[0,-1], self.sensor_data_preproc[1,-1], \
               self.sensor_data_filtered[0,-1], self.sensor_data_filtered[1,-1]
        

    def read(self):
        # print(self.name + " reading...")
        # Read raw data from MODBUS registers
        distance = self.read_raw_data("distance", "distance")
        angle = self.read_raw_data("angle", "angle")
        sensor_data_new = np.reshape(np.array([distance, angle]),(N_SENSOR_DATA,1))

        # Elapsed time from previous reading
        dt = time.time() - self.time_old
        self.time_old = time.time()

        print(self.name + ": " + str(self.rejection_time))
        if np.linalg.norm(sensor_data_new) < 1e-3:
            if self.rejection_time < self.rejection_time_thresh:
                self.rejection_time += dt
                self.sensor_data_preproc = np.append(self.sensor_data_preproc, \
                                                        np.reshape(self.sensor_data_preproc[:,-1],(N_SENSOR_DATA,1)), axis=1)
            
            else:
                self.sensor_data_preproc = np.append(self.sensor_data_preproc, \
                                                     np.zeros((N_SENSOR_DATA,1),dtype=float), axis=1)
        else:
            self.sensor_data_preproc = np.append(self.sensor_data_preproc,sensor_data_new, axis=1)
            self.rejection_time = 0.0

        # Slide window
        if self.sensor_data_preproc.shape[1] >= self.filter_win_length:
            # Filter data using moving-average filter                   
            filtered_signal = self.moving_average_filter()
            self.sensor_data_filtered = np.append(self.sensor_data_filtered,filtered_signal, axis=1)
            
            # Remove oldest value in sliding window
            self.sensor_data_preproc = np.delete(self.sensor_data_preproc, 0, axis=1)   
            self.sensor_data_filtered = np.delete(self.sensor_data_filtered, 0, axis=1)   

        else:
            self.sensor_data_filtered = np.copy(self.sensor_data_preproc)
            
        return sensor_data_new[0][0], sensor_data_new[1][0], \
               self.sensor_data_preproc[0,-1], self.sensor_data_preproc[1,-1], \
               self.sensor_data_filtered[0,-1], self.sensor_data_filtered[1,-1]


    def read_raw_data(self, register_name, meas_type):
        try:
            data_raw = self.client.read_holding_registers(self.modbus_fields[register_name], 1).registers
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
        
        filtered_signal = np.zeros((N_SENSOR_DATA,self.filter_win_length),dtype=float)
        for i in range(0,N_SENSOR_DATA):
            filtered_signal[i] = signal.lfilter(b,a,self.sensor_data_preproc[i,-self.filter_win_length:]) # filter output using lfilter function

        return np.reshape(filtered_signal[:,-1],((N_SENSOR_DATA,1)))


    def human_detected(self, data):
        n_dect = self.count_detections(data)
        # print("Number of detections in window: " + str(n_dect))
        return n_dect == self.ignore_win_length
    

    def count_detections(self, data):
        n_dect = 0
        for i in range(0,data.shape[1]):
            if np.any(np.abs(data[:,i]) > 1e-3):
                n_dect +=1
        return n_dect
    

    def remove_zeros(self, data):
        for i in range(0,N_SENSOR_DATA): # remove unexpected zeros
            null_condition = (data[i] < 1e-3)
            idx_zero = np.where(null_condition)[0]
            idx_not_zero = np.where(np.invert(null_condition))[0]

            # print("Measurement #" + str(i) + " before processing: " + str(data[i]), end="\n")
            if len(idx_not_zero.tolist()) != 0 and len(idx_zero.tolist()) != 0:
                data[i][idx_zero] = np.mean(data[i][idx_not_zero])
            # print("Measurement #" + str(i) + " after processing: " + str(data[i]), end="\n\n")

        self.sensor_data_preproc[:,-self.filter_win_length:] = data