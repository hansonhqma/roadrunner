import kinematics
import numpy as np
import evdev
from Rosmaster_Lib import Rosmaster

controller = evdev.InputDevice('/dev/input/event5')

robot = Rosmaster()
robot.create_receive_threading()

kinematics_model = kinematics.robot_model(0.03, 0.18, 0.2)

axis_codes = (0, 1, 3)

AXIS_MAX = 1<<15

deadzone = 0.2
drive_gain = 2
rotate_gain = 5

# vx, vy, vrz
steer_desired = np.array([0, 0, 0], dtype=np.float64)

for event in controller.read_loop():

    if event.type is not evdev.ecodes.EV_ABS:
        continue

    if event.code not in axis_codes:
        continue

    if event.code == 0:
        throttle = event.value/AXIS_MAX
        if abs(throttle) > deadzone:
            steer_desired[0] = throttle
        else:
            steer_desired[0] = 0


    elif event.code == 1:
        throttle = -event.value/AXIS_MAX
        if abs(throttle) > deadzone:
            steer_desired[1] = throttle
        else:
            steer_desired[1] = 0

    elif event.code == 3:
        throttle = -event.value/AXIS_MAX
        if abs(throttle) > deadzone:
            steer_desired[2] = throttle*rotate_gain
        else:
            steer_desired[2] = 0

    else:
        continue


    print("Vx: {:.5f}, Vy: {:.5f}, Vrz: {:.5f}".format(steer_desired[0], steer_desired[1], steer_desired[2]))

    drive_input = drive_gain * (kinematics_model.inverse_matrix @ steer_desired)

    robot.set_motor(drive_input[0], drive_input[2], drive_input[1], drive_input[3])

