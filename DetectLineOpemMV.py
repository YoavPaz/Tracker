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

# lock exposure
sensor.set_auto_exposure(False, exposure_us=35000)

clock = time.clock()
uart = UART(3, 115200)

# ---------- Thresholds ----------
BLACK_THRESHOLD = (0, 10, -128, 127, -128, 127)
#GREEN_THRESHOLD = (10, 100, -20, 20, -30, 40)
GREEN_THRESHOLD = (15, 100, -80, -10, -40, 35)
# ---------- Config ----------
IMG_W = sensor.width()
IMG_H = sensor.height()
CENTER_X = IMG_W // 2

NUM_SLICES = 10
SLICE_HEIGHT = 13
BOTTOM_OFFSET = 20

X_TOLERANCE = 100

DIS_THREASHOLD = 120
DIS_HELP_NUM = 1.5

# ---------- State ----------
use_green_override = False
override_points = []
points = []

# ---------- MAIN LOOP ----------
while True:
    clock.tick()
    img = sensor.snapshot()

    points.clear()

    # =========================================================
    # 1. NORMAL BLACK LINE DETECTION (BOTTOM LINE)
    # =========================================================
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
            points.append((best.cx(), best.cy()))
            img.draw_cross(best.cx(), best.cy())

    # draw line
    for i in range(len(points) - 1):
        img.draw_line(points[i][0], points[i][1],
                      points[i+1][0], points[i+1][1])

    # =========================================================
    # 2. GREEN DETECTION (TRIGGER OVERRIDE MODE)
    # =========================================================
    green_roi = (0, IMG_H - 100, IMG_W, 100)

    green_blobs = img.find_blobs(
        [GREEN_THRESHOLD],
        roi=green_roi,
        pixels_threshold=50,
        area_threshold=50,
        merge=False
    )

    if points:
        line_x = points[0][0]
    else:
        line_x = CENTER_X

    found_green = False

    for b in green_blobs:
        ratio = b.w() / b.h()
        #print("Green")

        if 0.7 < ratio < 1.4 and b.area() > 200:
            found_green = True
            cx = b.cx()
            cy = b.cy()

            img.draw_rectangle(b.rect())
            img.draw_cross(cx, cy)

            # decide direction
            direction = -1 if cx < line_x else 1

            # ROI ABOVE GREEN
            top = max(0, cy - 80)
            bottom = cy - 10

            if direction == -1:
                roi = (0, top, IMG_W // 2, bottom - top)
            else:
                roi = (IMG_W // 2, top, IMG_W // 2, bottom - top)

            # find BLACK line ABOVE green
            blobs = img.find_blobs(
                [BLACK_THRESHOLD],
                roi=roi,
                pixels_threshold=100,
                area_threshold=100,
                merge=True
            )

            override_points.clear()

            best = None
            best_dist = 9999

            for bl in blobs:
                dist = abs(bl.cx() - CENTER_X)

                if dist < best_dist:
                    best = bl
                    best_dist = dist

            if best:
                override_points.append((best.cx(), best.cy()))
                img.draw_cross(best.cx(), best.cy(), color=(255, 0, 0))
                img.draw_rectangle(best.rect(), color=(255, 0, 0))

                if not use_green_override:
                    uart.write("1100\n")
                    print("FORWARD")

                use_green_override = True

    # =========================================================
    # 3. RESET OVERRIDE IF LOST
    # =========================================================
    if use_green_override and not found_green:
        use_green_override = False

    # =========================================================
    # 4. STEERING DECISION
    # =========================================================
    if use_green_override and override_points:

        # FOLLOW GREEN-DIRECTED LINE
        error = override_points[0][0] - CENTER_X
        uart.write("%d\n" % error)

    else:

        # NORMAL LINE FOLLOW
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
            #print(f"Sending error: {error}, dis: {dis}, passed dis: {dis > DIS_THREASHOLD}")

        else:
            uart.write("999\n")
