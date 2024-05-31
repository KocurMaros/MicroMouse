import math

PI = math.pi
TAU = 2*PI
def smallestSignedAngleBetween(x, y):
    a = (x - y) % TAU
    b = (y - x) % TAU
    return -a if a < b else b

desired_angle = 0
current_angle = 300
prev_angle = current_angle
prev_error = 0

for i in range(0, 360):
    current_angle += 1
    if current_angle >= 360:
        current_angle = 0
    error = abs(math.atan2(math.sin(math.radians(desired_angle) - math.radians(current_angle)), math.cos(math.radians(desired_angle) - math.radians(current_angle))))
    print(f"Current angle: {current_angle}, Error: {int(error*180/PI)}")