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

reset_gyro = hub.imu.rotation(Axis.Z, False)



print(f"The battery voltage is {hub.battery.voltage()/1000}V.")



sorter.control.limits(2000, 20000, 600)
fixer.control.limits(2000, 20000, 600)

base = DriveBase(left, right, 88, 290)
timer = StopWatch()
run_timer = StopWatch()

release_timer = StopWatch()

last_error = 0
def PD(kp, kd, speed = 200):
    global last_error
    error = line_sensor.reflection() - 16
    derivative = error - last_error
    power = error * kp + derivative * kd
    base.drive(speed, power)    
    last_error = error



def base_run_angle(angle, speed = 700):
    base.straight(angle*0.7675)

def base_turn_angle(angle, speed = 700):
    right.run_angle(speed, -angle, wait=False)
    left.run_angle(speed, angle)   


def store_reset():
    global release_timer
    global scan_iter
    global push_stamps
    global push_iter
    global release_stamps
    global release_iter
    global fix_stamps
    global fix_iter
    release_timer.reset()
    scan_iter = 0
    push_stamps = [100000, 100000, 100000, 100000, 100000]
    push_iter = 0
    release_stamps = [100000, 100000, 100000, 100000, 100000]
    release_iter = 0
    fix_stamps = [100000, 100000, 100000, 100000, 100000]
    fix_iter = 0


def store(bricks = 4):
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
        push_stamps[scan_iter] = timer.time() + 1230
        fix_stamps[fix_iter] = timer.time() + 250
        scan_iter += 1 if scan_iter < bricks else 0
        hub.speaker.beep(500, 20)

    if timer.time() > fix_stamps[fix_iter]:
        fix_iter += 1 if fix_iter < bricks else 0
        fixer.run_angle(1500, 360, Stop.HOLD, wait=False)
        print(0)

    if timer.time() > push_stamps[push_iter]:
        release_stamps[push_iter] = timer.time() + 150
        push_iter += 1 if push_iter < bricks else 0
        sorter.run_target(1500, 70, Stop.HOLD, wait=False)

    if  timer.time() > release_stamps[release_iter]:
        if release_iter != bricks - 1: sorter.run_target(1500, -150, Stop.HOLD, wait=False)
        release_iter += 1

def print_angle(): 
    print(hub.imu.rotation(Axis.Z, False) - reset_gyro)

def turn(target):
    angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
    difference = angle + target
    speed = 500 * (difference / abs(difference))
    left.run(speed), right.run(-speed)
    while 20 < abs(difference):
        angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
        difference = angle + target
    speed = 400 * (difference / abs(difference))
    left.run(speed/3), right.run(-speed/3)
    while 5 < abs(difference):
        angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
        difference = angle + target
    left.stop(), right.stop()

def turn_by(target):
    angle = hub.imu.rotation(Axis.Z, False)
    target = angle - target
    difference = angle + target
    speed = 500 * (difference / abs(difference))
    left.run(speed), right.run(-speed)
    while 20 < abs(difference):
        angle = hub.imu.rotation(Axis.Z, False)
        difference = angle + target
    speed = 400 * (difference / abs(difference))
    left.run(speed/2), right.run(-speed/2)
    while 1 < abs(difference):
        angle = hub.imu.rotation(Axis.Z, False)
        difference = angle + target
    left.stop(), right.stop()


def fix_angle(target):
    angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
    difference = angle + target

    if difference != 0:
        speed = 100 * (difference / abs(difference))
        left.run(speed), right.run(-speed)
        while 0.6 < abs(difference):
            angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
            difference = angle + target
        left.stop(), right.stop()

def fix_angle_drive(target):
    angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
    difference = angle + target
    if difference != 0:
        print(1)
        base.drive(200, difference * 10)
        while 1 < abs(difference):
            angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
            difference = angle + target
            base.drive(200, difference * 5)
        base.stop()


def unload():
    print_angle()
    sorter.run_angle(1500, -40)
    wait(100)
    for i in range(4):
        base_run_angle(-145, 1000)
        if i != 3: sorter.run_angle(1500, 150, Stop.HOLD)
    sorter.run_angle(800, -360, wait=False)


def PD_till_target_black(target):
    saw_black = 0
    saw_black_timer = StopWatch()
    while saw_black != target:
        PD(2, 0, 300)
        s = brick_sensor.reflection()
        if s < 15 and saw_black_timer.time() > 250:
            saw_black_timer.reset()
            saw_black += 1 if saw_black < target else 0
            hub.speaker.beep(500, 20)
    print_angle()
    turn(0)
    base_run_angle(405)


def line(bricks = 4, once = True):
    store_reset()
    while release_iter < bricks:
        if scan_iter < bricks:
            PD(2, 0)
        elif push_iter < bricks:
            if once: fix_angle_drive(0)
            base.drive(200, 0)
            once = False
        else:
            base.stop()
            
        store()

fixer.run_target(4000, 0, Stop.HOLD, wait=False)
sorter.run_until_stalled(-2000)

base.drive(450, 0)


line()

base_run_angle(90)
turn(-90)
right.run_angle(1000, 250)
base_run_angle(-200)
right.run_angle(1000, -200)
fix_angle(-90)

sorter.run_target(1500, -150, Stop.HOLD, wait=False)
line(1, False)
fixer.run_angle(1500, 360, Stop.HOLD, wait=False)
sorter.run_target(1500, -80, Stop.HOLD, wait=False)


print(f"Run finished in {run_timer.time()/1000}s")
wait(3000)