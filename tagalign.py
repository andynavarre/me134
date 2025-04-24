from XRPLib.defaults import *
from Husky.huskylensPythonLibrary import HuskyLensLibrary
import time

# Initialize HuskyLens and drivetrain
husky = HuskyLensLibrary("I2C")
drivetrain = DifferentialDrive.get_default_differential_drive()

# Constants
TAG_CENTER_TOLERANCE = 2         # +/- pixels from center (x=160)
DESIRED_TAG_WIDTH = 55            # Width where tag is "close enough"
TOO_CLOSE_TAG_WIDTH = 60          # Width threshold where robot is too close
AVERAGE_WINDOW_SIZE = 5           # For median filtering

# Alignment states
ALIGN_YAW = "YAW_ALIGNMENT"
ALIGN_APPROACH = "APPROACH"
ALIGN_FINE = "FINAL_CORRECTION"
ALIGN_DONE = "ALIGNED"

alignment_state = ALIGN_YAW
x_buffer = []
width_buffer = []

# Ensure tag recognition mode is active
while not husky.tag_recognition_mode():
    husky.tag_recognition_mode()

# Median filter logic
def update_filtered_measurements(tag):
    global x_buffer, width_buffer

    x_center = tag[0]
    width = tag[2]

    x_buffer.append(x_center)
    width_buffer.append(width)

    if len(x_buffer) > AVERAGE_WINDOW_SIZE:
        x_buffer.pop(0)
        width_buffer.pop(0)

    x_median = sorted(x_buffer)[len(x_buffer) // 2]
    width_median = sorted(width_buffer)[len(width_buffer) // 2]
    return x_median, width_median

# Main alignment loop
while True:
    tag_data = husky.command_request_blocks()

    if len(tag_data) > 0:
        tag = tag_data[0]  # Use the first tag detected
        print("[DEBUG] Raw tag data:", tag)
        x, width = update_filtered_measurements(tag)

        print(f"[ALIGN] State: {alignment_state} | X: {x}, Width: {width}")

        if alignment_state == ALIGN_YAW:
            error = x - 160
            if abs(error) <= TAG_CENTER_TOLERANCE:
                drivetrain.set_speed(0, 0)
                alignment_state = ALIGN_APPROACH
            else:
                turn_speed = 10 if error > 0 else -10
                drivetrain.set_speed(turn_speed, -turn_speed)

        elif alignment_state == ALIGN_APPROACH:
            if width < DESIRED_TAG_WIDTH:
                drivetrain.set_speed(15, 15)  # Move forward

            elif width > TOO_CLOSE_TAG_WIDTH:
                print("[ALIGN] Too close to tag. Backing up...")
                drivetrain.set_speed(-15, -15)
            else:
                drivetrain.set_speed(0, 0)
                alignment_state = ALIGN_FINE

        elif alignment_state == ALIGN_FINE:
            error = x - 160
            if abs(error) <= TAG_CENTER_TOLERANCE:
                drivetrain.set_speed(0, 0)
                alignment_state = ALIGN_DONE
            else:
                turn_speed = 5 if error > 0 else -5
                drivetrain.set_speed(turn_speed, -turn_speed)

        elif alignment_state == ALIGN_DONE:
            print("[ALIGN] Alignment complete.")
            drivetrain.set_speed(0, 0)
            break

    else:
        print("[ALIGN] No tag detected.")
        x_buffer.clear()
        width_buffer.clear()
        drivetrain.set_speed(0, 0)
        # Remain in current state and reattempt when tag reappears
