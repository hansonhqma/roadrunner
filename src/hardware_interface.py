import serial
import time
import math
import struct
from kinematics import robot_model
import numpy as np
from multiprocessing import Process, Value, Array

class hardware_interface:

    def __init__(self, device_name='/dev/myserial'):
        self.__device_name = device_name
        self.__com = serial.Serial(self.__device_name, 115200)

        self.__packet_head = 0xff
        self.__id = 0xfc
        self.__complement = 257 - self.__id
        self.__motor_control_msg_type = 0x10
        
        self.report_time = 0
        self.packet = None

        self.__vel_data_type = 0x0a
        self.__mpu_data_type = 0x0b
        self.__encoder_data_type = 0x0d
        self.__valid_msg_types = (self.__vel_data_type, self.__mpu_data_type, self.__encoder_data_type)

        self.__vel_gain = 1/1000.0
        self.__mpu_gain = 1/3754.9
        self.__ticks_per_rev = 1320.0
        self.__mpu_correction = 0

        self.__model = robot_model(0.03, 0.18, 0.2)
        self.__FK_matrix = self.__model.forward_matrix

        self.t_vel = Array('d', 3) # translational velocities x y z
        self.r_vel = Value('d')    # rotational velocity z
        self.rot = Value('d')      # rotational position z
        self.batt = Value('d')     # battery voltage

        # integrator
        self.__INTEGRATION_TIME = 0
        self.__previous_r_vel = 0
        self.__delta_r_IMU = 0

        self.__listener_process = None

        # internal encoder information
        self.__wheel_values = np.array([0, 0, 0, 0])
        self.__previous_wheel_values = None

        # sensor fusion values
        self.__imu_fusion_gain = 0.9
        self.__encoder_fusion_gain = 0.1

    def __read_data(self, t_vel, r_vel, rot, batt):
        while True:
            if not self.__com.read()[0] == self.__packet_head:
                continue
            if not self.__com.read()[0] == self.__id - 1:
                continue

            # parse data
            CHECK_SUM = 0
            CHECK_VAL = 0
            packet_len = self.__com.read()[0]
            packet_type = self.__com.read()[0]
            data_length = packet_len - 2 # already read two bytes of the packet, rest is data

            if packet_type not in self.__valid_msg_types:
                continue


            if self.report_time != 0:
                dt = time.time() - self.report_time
            self.report_time = time.time()

            packet_data = self.__com.read(data_length)
            self.packet = packet_data
            checksum = packet_len + packet_type + sum(packet_data[:packet_len-3])
            if not checksum & 0xff == packet_data[-1]:
                #invalid checksum, dump message
                continue

            # valid checksum, parse message

            
            if packet_type == self.__vel_data_type:
                t_vel[0] = struct.unpack('h', packet_data[0:2])[0] * self.__vel_gain
                t_vel[1] = struct.unpack('h', packet_data[2:4])[0] * self.__vel_gain
                t_vel[2] = struct.unpack('h', packet_data[4:6])[0] * self.__vel_gain
                batt.value = struct.unpack('B', packet_data[6:7])[0]

            elif packet_type == self.__mpu_data_type: #packet type is 0x0b
                raw_value = struct.unpack('h', packet_data[4:6])[0] * self.__mpu_gain - self.__mpu_correction
                r_vel.value = raw_value
                # integrate with new rotational velocity value

                if self.__INTEGRATION_TIME == 0:
                    self.__INTEGRATION_TIME = time.time()
                    self.__previous_r_vel = raw_value

                    # assumes robot is initially stationary
                    self.__mpu_correction = raw_value
                    continue

                dt = time.time() - self.__INTEGRATION_TIME
                self.__INTEGRATION_TIME = time.time()

                # rot.value as of line 89 is purely estimate based on imu
                self.__delta_r_IMU = dt*(self.__previous_r_vel + raw_value)/2
                #old way of doing it
                #rot.value += dt*(self.__previous_r_vel + raw_value)/2
                #print(rot.value, raw_value)
                self.__previous_r_vel = raw_value

            elif packet_type == self.__encoder_data_type:
                self.__wheel_values[0] = struct.unpack('i', packet_data[0:4])[0]
                self.__wheel_values[2] = struct.unpack('i', packet_data[4:8])[0]
                self.__wheel_values[1] = struct.unpack('i', packet_data[8:12])[0]
                self.__wheel_values[3] = struct.unpack('i', packet_data[12:16])[0]
                #self.__wheel_values[0] = self.__wheel_values[0]/self.__ticks_per_rev*2*self.__pi
                #self.__wheel_values[2] = self.__wheel_values[2]/self.__ticks_per_rev*2*self.__pi
                #self.__wheel_values[1] = self.__wheel_values[1]/self.__ticks_per_rev*2*self.__pi
                #self.__wheel_values[3] = self.__wheel_values[3]/self.__ticks_per_rev*2*self.__pi
                #convert wheel ticks to radians

                self.__wheel_values = self.__wheel_values/self.__ticks_per_rev*2*math.pi
                #print values to console

                # first tick
                if type(self.__previous_wheel_values) == type(None):
                    self.__previous_wheel_values = np.array([x for x in self.__wheel_values])
                    continue

                #calculate change in wheel radians from last time step
                delta_theta = self.__wheel_values - self.__previous_wheel_values

                #calculate change kinematic vector from last time step
                delta_kin_vector = self.__FK_matrix @ delta_theta
                #pull off the rotation value from the kinematics
                delta_r_enc = delta_kin_vector[2]
                #sensor fusion to update rotation value
                rot.value += self.__imu_fusion_gain*(self.__delta_r_IMU) + self.__encoder_fusion_gain*(delta_r_enc)
                print("IMU estimate: {:.3f}\tEncoder estimate: {:.3f}\t Fused estimate: {:.3f}".format(self.__delta_r_IMU, delta_r_enc, rot.value))
                #store current value to old value
                self.__previous_wheel_values = np.array([x for x in self.__wheel_values])

            else:
                #placeholder
                continue

    def drive(self, w1, w2, w3, w4):
        # pwm duty cycles between -100 and 100
        w1_byte = struct.pack('b', w1)[0]
        w2_byte = struct.pack('b', w2)[0]
        w3_byte = struct.pack('b', w3)[0]
        w4_byte = struct.pack('b', w4)[0]

        # wheels 2 and 3 are switched around due to descrepancies between our kinematics model and the robot hardware
        packet = [self.__packet_head, self.__id, 0, self.__motor_control_msg_type,
                    w1_byte, w3_byte, w2_byte, w4_byte]
        packet[2] = len(packet) - 1
        checksum = sum(packet, self.__complement) & 0xff
        packet.append(checksum)

        self.__com.write(packet)
        

    def start_ipc(self):
        self.__listener_process = Process(target=self.__read_data, args=(self.t_vel, self.r_vel, self.rot, self.batt))
        self.__listener_process.start()

        print("STARTED HARDWARE LISTENER PROCESS AT", self.__listener_process.pid)




















