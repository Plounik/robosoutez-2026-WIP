from math import sqrt
from pybricks.parameters import Axis
from pybricks.tools import StopWatch, wait


def turn_to(hub, left, right, zero, target, method="auto", base=None):
    """Turn to an unwrapped CCW-positive heading from the starting zero."""
    clock = StopWatch()
    settled_since = None
    initial = hub.imu.rotation(Axis.Z, False) - zero
    distance = abs(target - initial)
    if method == "auto":
        if distance <= 30:
            method = "sprint"
        elif distance < 135:
            method = "gyro_turn"
        else:
            method = "gyro_fast"
    limit = 1800 + 25 * distance
    native = method in ("gyro_turn", "gyro_fast")
    try:
        if native:
            settings = base.settings()
            base.stop()
            base.use_gyro(True)
            rate, acceleration = (220, 700) if method == "gyro_turn" else (310, 1500)
            base.settings(turn_rate=rate, turn_acceleration=acceleration)
            base.turn(initial - target, wait=False)
            while not base.done():
                if clock.time() >= limit:
                    return False
                wait(5)
            base.stop()
        elif method == "encoder_trim":
            # Convert chassis degrees into wheel degrees for the 290 / 88 mm base.
            travel = (target - initial) * 290 / 88
            left.run_angle(850, -travel, wait=False)
            right.run_angle(850, travel, wait=False)
            while not (left.done() and right.done()):
                if clock.time() >= limit:
                    return False
                wait(5)
        while clock.time() < limit:
            error = target - (hub.imu.rotation(Axis.Z, False) - zero)
            rate = hub.imu.angular_velocity(Axis.Z, False)
            now = clock.time()
            if abs(error) <= 0.8 and abs(rate) <= 5:
                left.brake()
                right.brake()
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= 20:
                    return True
            else:
                settled_since = None
                if method in ("duty", "duty_fast"):
                    # Direct duty control: gyro rate damps the turn before reversal.
                    if method == "duty":
                        duty = 3.4 * error - 0.24 * rate
                    else:
                        duty = 5.0 * error - 0.30 * rate
                    if abs(error) > 0.8 and abs(rate) < 12:
                        duty += 9 if error > 0 else -9
                    duty = max(-85, min(85, duty))
                    left.dc(-duty)
                    right.dc(duty)
                    wait(5)
                    continue
                if native or method in ("staged", "encoder_trim"):
                    magnitude = 500 if abs(error) > 20 else min(135, 22 * abs(error))
                    speed = magnitude if error >= 0 else -magnitude
                elif method == "sprint":
                    if abs(error) > 34:
                        magnitude = 850
                    elif abs(error) > 8:
                        magnitude = 270
                    else:
                        magnitude = min(145, 37 * abs(error))
                    speed = magnitude if error >= 0 else -magnitude
                elif method == "braking":
                    desired_rate = min(240, sqrt(2 * 410 * abs(error)), 8 * abs(error))
                    if error < 0:
                        desired_rate = -desired_rate
                    speed = 3.3 * desired_rate + 1.4 * (desired_rate - rate)
                elif method == "pd_fast":
                    speed = 36 * error - 2.4 * rate
                elif method == "pd":
                    speed = 23 * error - 1.6 * rate
                else:
                    raise ValueError("unknown turn method")
                speed = max(-900, min(900, speed))
                left.run(-speed)
                right.run(speed)
            wait(5)
        return False
    finally:
        if native:
            base.stop()
            base.use_gyro(False)
            base.settings(turn_rate=settings[2], turn_acceleration=settings[3])
        left.brake()
        right.brake()
