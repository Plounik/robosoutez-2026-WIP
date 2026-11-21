from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Axis, Button, Color, Direction, Icon, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task

hub = PrimeHub()
left = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right = Motor(Port.D)
sorter = Motor(Port.B)
fixer = Motor(Port.E)
line_sensor = ColorSensor(Port.A)
brick_sensor = ColorSensor(Port.C)
base = DriveBase(left, right, 88, 290)
base.settings(1000, 5000)



print(f"The battery voltage is {hub.battery.voltage()/1000}V.")
if hub.charger.connected(): print(f"The charging current is {hub.charger.current()/1000}A")


def initialize():
    global reset_gyro, last_error, timer, release_timer, run_timer, sorter_start
    reset_gyro = hub.imu.rotation(Axis.Z, False)
    last_error = 0
    fixer.run_target(400, 0, Stop.HOLD, wait=False)
    sorter.run_until_stalled(-2000)
    timer = StopWatch()
    release_timer = StopWatch() 
    run_timer = StopWatch()
    sorter.control.limits(2000, 20000, 600)
    fixer.control.limits(2000, 20000, 600)
    wait(100)
    sorter.reset_angle(-185)
    sorter_start = -185


def beep():
    hub.speaker.beep() 


def base_run_angle(angle, speed = 700):
    base.straight(angle*0.7675)


def base_turn_angle(angle, speed = 700):
    right.run_angle(speed, -angle, wait=False)
    left.run_angle(speed, angle)   


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
    wait(50)


def turn_by(target):
    angle = hub.imu.rotation(Axis.Z, False)
    target = angle - target
    difference = angle - target
    speed = 500 * (difference / abs(difference))
    left.run(speed), right.run(-speed)
    while 20 < abs(difference):
        angle = hub.imu.rotation(Axis.Z, False)
        difference = angle - target
    speed = 400 * (difference / abs(difference))
    left.run(speed/2), right.run(-speed/2)
    while 3 < abs(difference):
        angle = hub.imu.rotation(Axis.Z, False)
        difference = angle - target
    left.stop(), right.stop()


def fix_angle(target):
    angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
    difference = angle + target

    if difference != 0:
        speed = 100 * (difference / abs(difference))
        left.run(speed), right.run(-speed)
        while 1.5 < abs(difference):
            angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
            difference = angle + target
        left.stop(), right.stop()


def fix_angle_drive(target):
    angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
    difference = angle + target
    if difference != 0:
        base.drive(200, difference * 10)
        while 1 < abs(difference):
            angle = hub.imu.rotation(Axis.Z, False) - reset_gyro
            difference = angle + target
            base.drive(200, difference * 5)
        base.stop()


def PD(kp, kd, speed = 200):
    global last_error
    error = line_sensor.reflection() - 16
    derivative = error - last_error
    power = error * kp + derivative * kd
    base.drive(speed, power)    
    last_error = error
    return error


def print_angle(): 
    print(f"The gyro angle is: {hub.imu.rotation(Axis.Z, False) - reset_gyro}")


def get_angle(): 
    return (hub.imu.rotation(Axis.Z, False) - reset_gyro)


def store_reset():
    global release_timer, scan_iter, push_stamps, push_iter, release_stamps, release_iter, fix_stamps, fix_iter

    release_timer.reset()
    scan_iter = 0
    push_stamps = [100000, 100000, 100000, 100000, 100000]
    push_iter = 0
    release_stamps = [100000, 100000, 100000, 100000, 100000]
    release_iter = 0
    fix_stamps = [100000, 100000, 100000, 100000, 100000]
    fix_iter = 0


def store(bricks = 4, delay = 0):
    global release_timer, scan_iter, push_stamps, push_iter, release_stamps, release_iter, fix_stamps, fix_iter

    s = brick_sensor.reflection()
    if s < 15 and release_timer.time() > 250:
        release_timer.reset()
        push_stamps[scan_iter] = timer.time() + 1430
        fix_stamps[fix_iter] = timer.time() + 300 + delay
        scan_iter += 1 if scan_iter < bricks else 0
        hub.speaker.beep(500, 20)

    if timer.time() > fix_stamps[fix_iter]:
        fix_iter += 1 if fix_iter < bricks else 0
        fixer.run_angle(1500, 360, Stop.HOLD, wait=False)

    if timer.time() > push_stamps[push_iter]:
        release_stamps[push_iter] = timer.time() + 150
        push_iter += 1 if push_iter < bricks else 0
        sorter.run_target(1500, 70, Stop.HOLD, wait=False)

    if  timer.time() > release_stamps[release_iter]:
        if release_iter != bricks - 1: sorter.run_target(1500, -185, Stop.HOLD, wait=False)
        release_iter += 1


def line(bricks = 4, once = True):
    store_reset()
    angle = round(get_angle() /90, 0) * -90
    print(angle)
    while release_iter < bricks:
        if scan_iter < bricks:
            PD(2, 0, 180)
        elif push_iter < bricks:
            if once: fix_angle_drive(angle)
            base.drive(200, 0)
            once = False
        else:
            base.stop()
            
        store()


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


def unload(count):
    print_angle()
    move_by = 145
    drive_by = -110
    end = 410
    sorter.run_target(1500, end - 5 - (move_by * (count - 1)))
    base.reset(0, 0)
    wait(50)
    base.straight(drive_by - 10)

    for i in range(count-1):
        if i < count - 2:
            sorter.run_angle(1500, move_by + 20)
            sorter.run_angle(1500, -20, wait=False)
            wait(10)
        elif i == count - 2:
            sorter.run_target(1500, end)
        base.straight(drive_by)
        print(i)
    wait(10)
    base.straight(-100)
    sorter.run_target(500, sorter_start, wait=False)
    
    fixer.run_angle(1000, 270)
    goal = -count * 115
    distance = goal - base.distance()
    base.settings(150)
    base.straight(distance)
    base.settings(800)
    base.straight(-150, wait=True)

    fixer.run_angle(1000, -270, wait=False)


def unload_fast(count):
    print_angle()
    move_by = 145
    end = 410
    sorter.run_target(1500, end - (move_by * (count-1)))
    
    base.reset(0, 0)

    base.drive(-350, 0)

    for i in range(count):
        if i == 0 :wait(440)
        else:wait(40)
        if i < count - 2: sorter.run_angle(1500, move_by, Stop.HOLD)
        elif i == count - 2: sorter.run_target(1500, end)
    base.straight(-100, wait=False)
    sorter.run_until_stalled(-2000)
    fixer.run_angle(1000, 270)

    goal = -count * 115
    distance = goal - base.distance()
    base.settings(150)
    base.straight(distance)
    base.settings(800)

    base.straight(-150, wait=True)

    fixer.run_angle(1000, -270, wait=False)



def drive_straight_pd(speed=200, kp=4, kd=0.2, target_angle=0):
    """Update steering toward an angle relative to reset_gyro."""
    angle = get_angle()
    yaw_rate = hub.imu.angular_velocity(Axis.Z, False)
    # Positive drive-base steering decreases the raw Z angle.
    turn_rate = kp * (angle - target_angle) + kd * yaw_rate
    base.drive(speed, max(-90, min(90, turn_rate)))


def PD_drive_distance(distance, speed=200, kp=4, kd=0.2, target_angle=0):
    """Drive a signed distance in mm toward a relative gyro angle."""
    if speed <= 0:
        raise ValueError("speed must be positive")
    direction = 1 if distance >= 0 else -1
    start = base.distance()
    try:
        while direction * (base.distance() - start) < abs(distance):
            drive_straight_pd(direction * speed, kp, kd, target_angle)
            wait(10)
    finally:
        base.stop()


def main():
    global reset_gyro
    base.drive(450, 0)

    line()

    turn(-80)
    fix_angle(-80)
    #               wait(50)
    #               base.straight(-50)
    #               base.drive(500, 30)
    #                wait(200)
    #               base.stop()
    sorter.run_until_stalled(-2000)
    wait(50)

    store_reset()
    while release_iter < 1:
        if push_iter < 1:
            PD(3, 0, 180)
        else:
            wait(150)
            base.stop()
        store(delay= 5000)


    base_run_angle(-180, 1500)
    turn(-180)
    base_run_angle(650, 1500)
    turn(-90)
    wait(50)
    fix_angle(-90)
    wait(50)
    

    fixer.run_angle(1500, 720, Stop.HOLD, wait=False)
    sorter.run_target(1500, -80, Stop.HOLD, wait=False)
    PD_drive_distance(550, 400, target_angle= 90)
    fix_angle(-90)

    unload(5)

    turn(-180)
    base_run_angle(-600, 1500)
    turn(-90)
    fix_angle(-90)
    base.reset(0, 0)
    error = PD(2, 0, 500)
    while abs(error) > 10:
        error = PD(2, 0, 500)
    line(3)
    sorter.run_target(1500, 155)
    fixer.run_angle(1500, 360, Stop.HOLD, wait=False)

    PD_drive_distance(-550, 800, target_angle=90)
    turn(-180)
    fixer.run_angle(1500, -360, Stop.HOLD, wait=False)

    PD_drive_distance(200, 400, target_angle=180)
    unload_fast(3)






initialize()


# main()

def get_frequency(speed):
    base.drive(speed, 0)
    b = 0
    a = 0
    c = []
    while a < 2:
        s = brick_sensor.reflection()
        if s < 15 and release_timer.time() > 150:
            if b == 0:
                b = timer.time()
            else:
                c.append(timer.time() - b)
                b = 0
                a += 1
            release_timer.reset()
    base.stop()
    return c
    
def meassure_min_push_time(time, min_angle_should_be=-10):
    run_timer.reset()
    sorter.track_target(70)
    wait(time)
    a = sorter.angle()
    sorter.run_target(1500, -185)
    return a



# print(get_frequency(280))
print(get_frequency(420))
print(f"Run finished in {run_timer.time()/1000}s")