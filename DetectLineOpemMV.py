import sensor, image, time
from pyb import UART
import math

# ---------- Camera setup ----------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

sensor.skip_frames(time=2000)

sensor.set_brightness(1)
sensor.set_contrast(2)

# IMPORTANT: lock exposure (prevents green drift issues)
sensor.set_auto_exposure(False, exposure_us=sensor.get_exposure_us())

clock = time.clock()
uart = UART(3, 115200)

# ---------- Thresholds ----------
BLACK_THRESHOLD = (0, 10, -128, 127, -128, 127)
GREEN_THRESHOLD = (30, 80, -70, -15, -20, 40)

# ---------- Config ----------
IMG_W = sensor.width()
IMG_H = sensor.height()
CENTER_X = IMG_W // 2

NUM_SLICES = 8
SLICE_HEIGHT = 13
BOTTOM_OFFSET = 20

X_TOLERANCE = 100

DIS_THREASHOLD = 100
DIS_HELP_NUM = 1.5

points = []

# ---------- MAIN LOOP ----------
while True:
    clock.tick()
    img = sensor.snapshot()
    points.clear()

    # ---------- LINE DETECTION ----------
    for i in range(NUM_SLICES):
        y = IMG_H - BOTTOM_OFFSET - (i + 1) * SLICE_HEIGHT
        roi = (0, y, IMG_W, SLICE_HEIGHT)

        blobs = img.find_blobs(
            [BLACK_THRESHOLD],
            roi=roi,
            pixels_threshold=300,
            area_threshold=300,
            merge=True
        )

        best = None
        best_dist = 9999

        for b in blobs:
            cx = b.cx()
            dist = abs(cx - CENTER_X)

            if dist < best_dist and dist <= X_TOLERANCE:
                best = b
                best_dist = dist

        if best:
            cx = best.cx()
            cy = best.cy()
            points.append((cx, cy))
            img.draw_cross(cx, cy)

    # ---------- DRAW LINE ----------
    for i in range(len(points) - 1):
        img.draw_line(
            points[i][0], points[i][1],
            points[i+1][0], points[i+1][1]
        )

    # ---------- GREEN DETECTION ----------
    green_blobs = img.find_blobs(
        [GREEN_THRESHOLD],
        pixels_threshold=100,
        area_threshold=100
    )

    left_detected = False
    right_detected = False

    if points:
        line_x = points[0][0]
    else:
        line_x = CENTER_X

    for b in green_blobs:
        ratio = b.w() / b.h()

        # debug (optional)
        print("green blobs:", len(green_blobs))

        # relaxed + safer filter
        if 0.7 < ratio < 1.4 and b.area() > 200:
            cx = b.cx()

            img.draw_rectangle(b.rect())
            img.draw_cross(cx, b.cy())

            if cx < line_x:
                left_detected = True
            else:
                right_detected = True

    # ---------- DECISION ----------
    if left_detected and right_detected:
        uart.write("1003\n")

    elif right_detected:
        uart.write("1001\n")

    elif left_detected:
        uart.write("1002\n")

    else:
        # ---------- NORMAL LINE FOLLOW ----------
        if points:
            error = points[0][0] - CENTER_X

            first_point = points[0]
            last_point = points[-1]

            dis = int(math.sqrt(
                (first_point[0] - last_point[0])**2 +
                (first_point[1] - last_point[1])**2
            ))

            if dis > DIS_THREASHOLD:
                error *= DIS_HELP_NUM

            uart.write("%d\n" % error)

        else:
            uart.write("999\n")
