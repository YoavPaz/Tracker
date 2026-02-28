import sensor, image, time
from pyb import UART

# ---------- Camera setup ----------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240 
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

sensor.set_brightness(1)
sensor.set_contrast(2)
clock = time.clock()

uart = UART(3, 115200) # defualt pins: TX=Pin P4, RX=Pin P5

BLACK_THRESHOLD = (0, 45)

IMG_W = sensor.width()
IMG_H = sensor.height()
CENTER_X = IMG_W // 2

NUM_SLICES = 6
SLICE_HEIGHT = 10
BOTTOM_OFFSET = 5

X_TOLERANCE = 50

points = []

while True:
    clock.tick()
    img = sensor.snapshot()
    points.clear()

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

            img.draw_cross(cx, cy, color=(0, 255, 255))  # cyan cross

    for i in range(len(points) - 1):
        img.draw_line(
            points[i][0], points[i][1],
            points[i+1][0], points[i+1][1],
            color=(0, 255, 255)
        )

    if points:
        error = points[0][0] - CENTER_X
        uart.write("%d\n" % error)
        print(f"Results: error: {error}, fps: {clock.fps()}")
    else:
        uart.write("999\n")
