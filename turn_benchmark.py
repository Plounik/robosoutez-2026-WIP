from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from turning import turn_to


hub = PrimeHub()
left = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right = Motor(Port.D)
base = DriveBase(left, right, 88, 290)
clock = StopWatch()
methods = ("staged", "pd", "pd_fast", "braking", "encoder_trim", "sprint", "duty", "duty_fast", "gyro_turn", "gyro_fast")
results = {name: [] for name in methods}
while not hub.imu.ready():
    if clock.time() > 15000:
        raise RuntimeError("IMU could not calibrate")
    wait(20)
zero = hub.imu.rotation(Axis.Z, False)
print("trial method delta settle_ms final_error_deg qualified")
for trial in range(3):
    for magnitude in (15, 90, 180):
        for slot in range(len(methods)):
            method = methods[(trial + slot) % len(methods)]
            first = -magnitude if (trial + slot) % 2 else magnitude
            for delta in (first, -first):
                start_angle = hub.imu.rotation(Axis.Z, False) - zero
                target = start_angle + delta
                clock.reset()
                settled = turn_to(hub, left, right, zero, target, method, base)
                elapsed = clock.time()
                wait(150)
                final_angle = hub.imu.rotation(Axis.Z, False) - zero
                error = abs(target - final_angle)
                ok = settled and error <= 1
                print(trial + 1, method, delta, elapsed, error, ok)
                results[method].append((elapsed, error, ok))
                if not settled and abs(final_angle - start_angle) < 2:
                    raise RuntimeError("Chassis did not rotate")
print("SUMMARY method passed trials median_ms worst_error_deg")
for method in methods:
    rows = results[method]
    times = sorted(row[0] for row in rows)
    n = len(times)
    print(method, sum(int(row[2]) for row in rows), n,
          (times[n // 2] + times[(n - 1) // 2]) / 2, max(row[1] for row in rows))
