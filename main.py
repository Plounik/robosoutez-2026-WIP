from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Axis, Button, Color, Direction, Icon, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task

from turning import turn_to

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
    right.run_angle(speed, angle, wait=False)
    left.run_angle(speed, -angle)   


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
    print(c)

def turn(target):
    """Turn to an absolute CCW-positive heading relative to reset_gyro."""
    if not turn_to(hub, left, right, reset_gyro, target, base=base):
        raise RuntimeError("Turn did not settle")


def turn_by(angle):
    """Turn counterclockwise by angle degrees (negative turns clockwise)."""
    turn(get_angle() + angle)


def fix_angle(target):
    turn(target)


def fix_angle_drive(target):
    difference = get_angle() - target
    if difference != 0:
        base.drive(200, difference * 10)
        while 1 < abs(difference):
            difference = get_angle() - target
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
    print(f"The gyro angle is: {get_angle()}")


def get_angle(): 
    return (hub.imu.rotation(Axis.Z, False) - reset_gyro)


# Speeds are mm/s. Delays are ms from brick detection.
short = {
    "last_brick_extra_mm": 10,
    "bricks": 4, "speed": 250, "kp": 2, "kd": 0,
    "brick_threshold": 15, "detection_gap_ms": 250,
    "push_delay_ms": 950, "fix_delay_ms": 270,
    "fix_delay_step_ms": 5,
    "heading_gain": 5, "heading_tolerance": 1,
}

long = {
    "last_brick_extra_mm": 5,
    "bricks": 3, "speed": 350, "kp": 2, "kd": 0,
    "brick_threshold": 15, "detection_gap_ms": 178,
    "push_delay_ms": 679, "fix_delay_ms": 193,
    "fix_delay_step_ms": 5,
    "heading_gain": 5, "heading_tolerance": 1,
}


def store_reset(bricks=4):
    global final_push_distance
    final_push_distance = None
    global release_timer, scan_iter, push_stamps, push_iter, release_iter, fix_stamps, fix_iter

    release_timer.reset()
    scan_iter = 0
    push_stamps = [0] * bricks
    push_iter = 0
    release_iter = 0
    fix_stamps = [0] * bricks
    fix_iter = 0


def store(bricks=4, delay=0, profile=short):
    global final_push_distance
    global release_timer, scan_iter, push_stamps, push_iter, release_iter, fix_stamps, fix_iter

    s = brick_sensor.reflection()
    if scan_iter < bricks and s < profile["brick_threshold"] and release_timer.time() > profile["detection_gap_ms"]:
        release_timer.reset()
        push_stamps[scan_iter] = timer.time() + profile["push_delay_ms"]
        fix_delay = profile["fix_delay_ms"] + scan_iter * profile["fix_delay_step_ms"]
        fix_stamps[scan_iter] = timer.time() + fix_delay + delay
        scan_iter += 1
        hub.speaker.beep(500, 20)

    if fix_iter < scan_iter and timer.time() > fix_stamps[fix_iter]:
        fix_iter += 1
        fixer.run_angle(1500, 360, Stop.HOLD, wait=False)

    if release_iter < push_iter and sorter.angle() >= -40:
        if release_iter != bricks - 1:
            sorter.run_target(1500, sorter_start, Stop.HOLD, wait=False)
        release_iter += 1

    if push_iter == release_iter and push_iter < scan_iter and timer.time() > push_stamps[push_iter]:
        if push_iter == 0 or sorter.done():
            if push_iter == bricks - 1 and profile["last_brick_extra_mm"] > 0:
                distance = base.distance()
                if final_push_distance is None:
                    final_push_distance = distance + profile["last_brick_extra_mm"]
                if distance < final_push_distance:
                    return
            sorter.track_target(70)
            push_iter += 1



def single_store_reset():
    global release_timer, scan_iter, push_stamps, push_iter, release_stamps, release_iter, fix_stamps, fix_iter

    release_timer.reset()
    scan_iter = 0
    push_stamps = [100000, 100000, 100000, 100000, 100000]
    push_iter = 0
    release_stamps = [100000, 100000, 100000, 100000, 100000]
    release_iter = 0
    fix_stamps = [100000, 100000, 100000, 100000, 100000]
    fix_iter = 0


def single_store(bricks = 4, delay = 0):
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



def line(profile=short, once=True):
    bricks = profile["bricks"]
    speed = profile["speed"]
    store_reset(bricks)
    angle = round(get_angle() /90, 0) * 90
    print(angle)
    while release_iter < bricks:
        if scan_iter < bricks:
            PD(profile["kp"], profile["kd"], speed)
        elif push_iter < bricks:
            difference = get_angle() - angle
            once = once and abs(difference) > profile["heading_tolerance"]
            base.drive(speed, difference * profile["heading_gain"] if once else 0)
        else:
            base.stop()
            
        store(bricks, profile=profile)


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
    print("this")
    print_angle()
    move_by = 145
    drive_by = -120
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

    line(short)

    turn(80)
    #               wait(50)
    #               base.straight(-50)
    #               base.drive(500, 30)
    #                wait(200)
    #               base.stop()
    sorter.run_until_stalled(-2000)
    wait(50)

    single_store_reset()
    while release_iter < 1:
        if push_iter < 1:
            PD(3, 0, 180)
        else:
            wait(150)
            base.stop()
        single_store(delay= 5000)


    base_run_angle(-180, 1500)
    turn(180)
    base_run_angle(650, 1500)
    turn(90)
    

    fixer.run_angle(1500, 720, Stop.HOLD, wait=False)
    sorter.run_target(1500, -80, Stop.HOLD, wait=False)
    PD_drive_distance(550, 400, target_angle= 90)
    fix_angle(90)
    wait(50)
    unload(5)

    turn(180)
    base_run_angle(-600, 1500)
    turn(90)
    base.reset(0, 0)
    error = PD(2, 0, 500)
    while abs(error) > 10:
        error = PD(2, 0, 500)
    line(long)
    sorter.run_target(1500, 155)
    fixer.run_angle(1500, 360, Stop.HOLD, wait=False)

    PD_drive_distance(-650, 800, target_angle=90)
    turn(180)
    fixer.run_angle(1500, -360, Stop.HOLD, wait=False)

    PD_drive_distance(250, 400, target_angle=180)
    unload(3)


initialize()

main()


print(f"Run finished in {run_timer.time()/1000}s")
