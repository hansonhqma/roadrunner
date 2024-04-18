import evdev
from multiprocessing import Process, Value, Array
import time

class hid:
    
    def __init__(self):
        self.__controller = evdev.InputDevice('/dev/input/event2')

        self.__axis_codes = (0, 1, 3)
        self.__AXIS_MAX = 1<<15
        self.__deadzone = 0.2

        self.vtx = Value('d')
        self.vty = Value('d')
        self.vrz = Value('d')

    def __event_loop(self, vtx, vty, vrz):
        for event in self.__controller.read_loop():

        
            if event.type is not evdev.ecodes.EV_ABS:
                continue

            if event.code not in self.__axis_codes:
                continue
            
            # only desired axis codes persist
            
            if event.code == 0:
                throttle = event.value/self.__AXIS_MAX
                if abs(throttle) > self.__deadzone:
                    vtx.value = throttle
                else:
                    vtx.value = 0

            elif event.code == 1:
                throttle = -event.value/self.__AXIS_MAX
                if abs(throttle) > self.__deadzone:
                    vty.value = throttle
                else:
                    vty.value = 0

            elif event.code == 3:
                throttle = -event.value/self.__AXIS_MAX
                if abs(throttle) > self.__deadzone:
                    vrz.value = throttle
                else:
                    vrz.value = 0

    def start_ipc(self):
        self.__hid_process = Process(target=self.__event_loop, args=(self.vtx, self.vty, self.vrz))
        self.__hid_process.start()

        print("STARTED HID LISTENER PROCESS AT", self.__hid_process.pid)
        while True:
            time.sleep(0.02)
            print(self.vtx.value, self.vty.value, self.vrz.value)
