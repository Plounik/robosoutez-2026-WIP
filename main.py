from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Axis, Button, Color, Direction, Icon, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task

hub = PrimeHub()



left = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right = Motor(Port.F)
sorter = Motor(Port.B)
fixer = Motor(Port.E)
line_sensor = ColorSensor(Port.A)
brick_sensor = ColorSensor(Port.C)


print(f"The battery voltage is {hub.battery.voltage()/1000}V.")



sorter.control.limits(2000, 20000, 600)
fixer.control.limits(2000, 20000, 600)

base = DriveBase(left, right, 88, 290)
timer = StopWatch()

release_timer = StopWatch()
scan_iter = 0
push_stamps = [100000, 100000, 100000, 100000, 100000]
push_iter = 0

release_stamps = [100000, 100000, 100000, 100000, 100000]
release_iter = 0

fix_stamps = [100000, 100000, 100000, 100000, 100000]
fix_iter = 0

last_error = 0
def PD(kp, kd):
    global last_error
    error = line_sensor.reflection() - 50
    derivative = error - last_error
    power = error * kp + derivative * kd
    base.drive(200, power)    
    last_error = error

def unload():
    target_angle = 220
    base_run_angle(-180, 200)
    sorter.run_target(1500, 110, Stop.HOLD)


def base_run_angle(angle, speed = 500):
    right.run_angle(speed, angle, wait=False)
    left.run_angle(speed, angle)
def store_short():
    global release_timer
    global scan_iter
    global push_stamps
    global push_iter

    global release_stamps
    global release_iter

    global fix_stamps
    global fix_iter


    s = brick_sensor.reflection()
    if s < 15 and release_timer.time() > 250:
        release_timer.reset()
        push_stamps[scan_iter] = timer.time() + 1330
        fix_stamps[fix_iter] = timer.time() + 250
        scan_iter += 1 if scan_iter < 4 else 0
        hub.speaker.beep(500, 20)

    if timer.time() > fix_stamps[fix_iter]:
        fix_iter += 1 if fix_iter < 4 else 0
        fixer.run_angle(1500, 360, Stop.HOLD, wait=False)

    if timer.time() > push_stamps[push_iter]:
        release_stamps[push_iter] = timer.time() + 150
        push_iter += 1 if push_iter < 4 else 0
        sorter.run_target(1500, 110, Stop.HOLD, wait=False)

    if  timer.time() > release_stamps[release_iter]:
        if release_iter != 3: sorter.run_target(1500, -150, Stop.HOLD, wait=False)
        release_iter += 1


sorter.run_target(2000, -150, Stop.HOLD, wait=False)
fixer.run_target(2000, 0, Stop.HOLD, wait=False)

base.drive(200, 0)
wait(10)
while release_iter < 4:
    if scan_iter != 4:
        PD(0.5, 0)
    elif push_iter != 4:
        base.drive(200, 0)
        print(1)
    else:
        base.stop()
        print(2)
    store_short()
base.stop()
right.run_angle(500, -270)
base_run_angle(-450)
right.run_angle(500, 270)





wait(3000)