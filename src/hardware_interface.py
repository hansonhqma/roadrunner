import serial
import time
import struct

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

        self.__vel_gain = 1/1000.0
        self.__mpu_gain = 1/3754.9

        self.t_vel = Array('d', 3) # translational velocities x y z
        self.r_vel = Value('d')    # rotational velocity z
        self.rot = Value('d')      # rotational position z
        self.batt = Value('d')     # battery voltage

        # integrator
        self.__INTEGRATION_TIME = 0
        self.__previous_r_vel = 0

        self.__listener_process = None

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

            if packet_type not in (0x0a, 0x0b):
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
            else: #packet type is 0x0b
                raw_value = struct.unpack('h', packet_data[4:6])[0] * self.__mpu_gain
                r_vel.value = raw_value
                # integrate with new rotational velocity value
                if self.__INTEGRATION_TIME == 0:
                    self.__INTEGRATION_TIME = time.time()
                    self.__previous_r_vel = raw_value
                    continue
                dt = time.time() - self.__INTEGRATION_TIME
                self.__INTEGRATION_TIME = time.time()

                rot.value += dt*(self.__previous_r_vel + raw_value)/2
                #print(rot.value, raw_value)
                self.__previous_r_vel = raw_value

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




















