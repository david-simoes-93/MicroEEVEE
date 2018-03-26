from CiberEEVEE.croblink3 import CRobLinkAngs
from CiberEEVEE.mapgrid import Maze
from CiberEEVEE.Utils import updateRobotPos
import sys


def filter_buffer(buffer, val):
    if val < 0.4:
        val = 0.4
    val = 1 / val

    buffer.pop(0)
    buffer.append(val)
    return sorted(buffer)[int(len(buffer) / 2)]  # median
    # return np.mean(buffer)                  # average


def stop():
    cif.driveMotors(0.0, 0.0)
    # updateRobotPos(0.0, 0.0)


# Connect to server
if len(sys.argv) < 2:
    sys.argv.append("localhost")
cif = CRobLinkAngs("EEVEE", 0, [0.0, 90.0, -90.0, 180], sys.argv[1])
if cif.status != 0:
    print("Connection refused or error")
    quit()

pos_cheese = None
my_x, my_y, prev_left, pref_right = 0, 0, 0, 0
pos_start = [my_x, my_y]
my_map = Maze()

front_buffer = [0, 0, 0]
left_buffer = [0, 0, 0]
right_buffer = [0, 0, 0]
back_buffer = [0, 0, 0]

motor_command_left, motor_command_right = 0, 0

while True:
    # Read sensors
    cif.readSensors()
    front_sensor = filter_buffer(front_buffer, cif.measures.irSensor[0])
    left_sensor = filter_buffer(left_buffer, cif.measures.irSensor[1])
    right_sensor = filter_buffer(right_buffer, cif.measures.irSensor[2])
    back_sensor = filter_buffer(back_buffer, cif.measures.irSensor[3])
    compass = -cif.measures.compass

    # Read cheese ground
    if pos_cheese is not None:
        if cif.measures.ground == 0:
            pos_cheese = (my_x, my_y)
            cif.setVisitingLed(1)
        else:
            cif.setVisitingLed(0)

    if not cif.measures.collision:
        my_x, my_y, my_dir, prev_left, pref_right = \
            updateRobotPos(prev_left, pref_right, motor_command_left, motor_command_right,
                           my_x, my_y, compass)
    my_map.update(my_x, my_y, left_sensor, front_sensor, right_sensor, back_sensor,
                  compass, cif.measures.ground)
    my_map.render()

    print("%4.2f %4.2f %4.2f %4.2fº %d" % (
          left_sensor, front_sensor, right_sensor, compass, cif.measures.ground))
    print()

    #input("rdy?")

    if cif.measures.time == 0:
        continue

    motor_command_left, motor_command_right = 0.1, 0.1
    cif.driveMotors(motor_command_left, motor_command_right)


    # if sim_state == STOP:
    #    stop()
    #    print("End.")
    #    exit()

my_map.render(close=True)
