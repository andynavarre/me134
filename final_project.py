import time
import random
from MQTT.mqttconnect import connect_mqtt
from XRPLib.differential_drive import DifferentialDrive
from XRPLib.board import Board
from XRPLib.rangefinder import Rangefinder
from Husky.huskylensPythonLibrary import HuskyLensLibrary
from XRPLib.reflectance import Reflectance
from XRPLib.servo import Servo

#Initialize sensors and drivetrain
drivetrain = DifferentialDrive.get_default_differential_drive()
board = Board.get_default_board()
rangefinder = Rangefinder.get_default_rangefinder()
servo = Servo.get_default_servo(index=1)
reflect = Reflectance.get_default_reflectance()

# PID controller class
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_sum = 0
        self.error_delta = 0
        self.prev_error = 0
        
    def update(self, error):
        #### TODO: implement the PID controller here ####
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.error_sum += error #* dt
        I = self.Ki * self.error_sum
        
        # Derivative term
        error_delta = (error - self.prev_error) #/ dt #if dt > 0 else 0
        D = self.Kd * error_delta
        
        # Save the error for the next loop
        self.prev_error = error
        
        # Total output effort
        effort = P + I + D
        return effort

# Line-following setup
line_pid = PIDController(Kp=30, Ki=1, Kd=0) # 40, 1, 1 worksish

# Camera Setup
husky = HuskyLensLibrary("I2C")
while not husky.tag_recognition_mode():
    husky.tag_recognition_mode()

# Constants and State
ROLES = ["BASE", "MIDDLE", "TOP"]
STATE_INIT = "INIT"
STATE_WAIT_ROLE = "WAIT_ROLE"
STATE_ROLE_BEHAVIOR = "STATE_ROLE_BEHAVIOR"
STATE_STOPPED = "STATE_STOPPED"

MQTT_ROLE_TOPIC = "swarm/role_assignment"
MQTT_COMMAND_TOPIC = "swarm/commands"
MQTT_STATUS_TOPIC = "swarm/status"
MQTT_BROADCAST_TOPIC = "swarm/broadcast"

# Constant and setup for role communication
robot_id = str(1) # str(time.ticks_ms() % 10000)  Unique per boot
known_ids = set()
assigned_ids = {}
state = STATE_INIT
role = None

start_time = time.time()
role_assignment_delay = 3.0  # seconds

# Constant for checks between robots
top_ready = False
top_button_pressed = False
top_starting = False
base_ready = False

# Sensor Readings
distance_buffer = []

# Servo Stuff
DOWN = 80
UP = 0
servo.set_angle(DOWN)

# Base robot behavior states
BASE_STATE_WAIT_FOR_BUTTON = "BASE_WAIT_FOR_BUTTON"
BASE_STATE_RANDOM_WALK = "BASE_RANDOM_WALK"
BASE_STATE_ALIGN_APRILTAG = "BASE_ALIGN_APRILTAG"
BASE_STATE_READY_FOR_UNSTACK = "BASE_READY_FOR_UNSTACK"
BASE_STATE_WALL_FOLLOWING = "BASE_WALL_FOLLOWING"
BASE_STATE_ALIGN_APRILTAG2 = "BASE_ALIGN_APRILTAG2"
BASE_STATE_WAIT_FOR_PICKUP = "BASE_WAIT_FOR_PICKUP"
BASE_STATE_REJOIN_STACK = "BASE_REJOIN_STACK"
BASE_STATE_FINAL_MOVES = "BASE_FINAL_MOVES"
BASE_STATE_DONE = "BASE_DONE"

base_state = BASE_STATE_WAIT_FOR_BUTTON

# Middle robot behavior states
MIDDLE_STATE_IDLE = "MIDDLE_IDLE"
MIDDLE_STATE_PREPARE_FOR_RESTACK = "MIDDLE_PREPARE_FOR_RESTACK"

middle_state = MIDDLE_STATE_IDLE

# Top-specific flags and setup
# Top robot behavior states
TOP_STATE_WAIT_FOR_START = "TOP_STATE_WAIT_FOR_START"
TOP_STATE_STACKING = "TOP_STATE_STACKING"
TOP_STATE_EXIT = "TOP_STATE_EXIT"
TOP_STATE_LINE_FOLLOWING = "TOP_STATE_LINE_FOLLOWING"
TOP_STATE_ALIGN = "TOP_STATE_ALIGN"
TOP_STATE_REJOIN = "TOP_STATE_REJOIN"
TOP_STATE_UNSTACKING = "TOP_STATE_UNSTACKING"
TOP_STATE_DONE = "TOP_STATE_DONE"

top_state = TOP_STATE_WAIT_FOR_START

# Check if this robot is the assigner
def is_role_assigner():
    all_ids = list(known_ids.union({robot_id}))
    if robot_id == min(all_ids):
        print("Role Assigner")
        return True
    else:
        print("Not role assigner")
        return False

# MQTT Callback
def handle_mqtt_message(topic, msg):
    global role, top_ready, top_button_pressed

    # Decoding the messages and topics Ex: b\topic
    topic = topic.decode("utf-8") if isinstance(topic, bytes) else topic
    msg = msg.decode("utf-8") if isinstance(msg, bytes) else msg
    print("MQTT received | Topic:", topic, "Msg:", msg)

    #New Robot detected: add to personal list
    if topic == MQTT_BROADCAST_TOPIC and msg.startswith("NEW_ROBOT:"):
        incoming_id = msg.split(":")[1]
        known_ids.add(incoming_id)

    #If another robot is assigning roles, read the assignment
    elif topic == MQTT_ROLE_TOPIC:
        parts = msg.split(":")
        if len(parts) == 2:
            target_id, assigned_role = parts
            if target_id == robot_id:
                if role is None:
                    role = assigned_role
                    print("This robot", robot_id, "assigned:", role)
                    assigned_ids = {1, 2, 3}
                    client.publish(MQTT_STATUS_TOPIC, f"{role} READY")
                elif role != assigned_role:
                    print("Role conflict: already", role, "ignoring", assigned_role)

    # Check status of other robots while doing the circuit
    elif topic == MQTT_COMMAND_TOPIC:
        if msg == "TOP_BUTTON_PRESSED":
            top_button_pressed = True
        elif msg == "TOP_READY" or msg == "TOP_WAITING_AT_TAG":
            top_ready = True
        elif msg == "BASE_READY" or msg == "BASE_WAITING_AT_TAG":
            base_ready = True

# Just check if the robot has detected an April tag
def detect_april_tag():
    tag_data = husky.command_request_blocks()
    return len(tag_data) > 0

# AprilTag Alignment Behavior for an April Tag on the wall
def align_to_wall_apriltag():
    global x_buffer, width_buffer
    
    TAG_CENTER_TOLERANCE = 20
    DESIRED_TAG_WIDTH = 118
    TOO_CLOSE_TAG_WIDTH = 125
    AVERAGE_WINDOW_SIZE = 5
    TILT_RATIO_THRESHOLD = 1.1
    MAX_RATIO_HISTORY = 3

    STALL_DETECTION_WINDOW = 0.5
    STALL_X_THRESHOLD = 5

    ALIGN_YAW = "YAW_ALIGNMENT"
    ALIGN_APPROACH = "APPROACH"
    ALIGN_FINE = "FINAL_CORRECTION"
    ALIGN_DONE = "ALIGNED"

    alignment_state = ALIGN_YAW
    x_buffer = []
    width_buffer = []
    ratio_history = []
    turn_direction = 1

    last_seen_time = time.time()
    x = 160
    last_x_position = 160
    stall_start_time = None

    print("[ALIGN] Starting AprilTag alignment...")

    while True:
        tag_data = husky.command_request_blocks()

        tag = None
        if len(tag_data) > 0:
            tag = tag_data[0]

        if tag is not None:
            last_seen_time = time.time()
            x_center = tag[0]
            width = tag[2]

            x_buffer.append(x_center)
            width_buffer.append(width)

            if len(x_buffer) > AVERAGE_WINDOW_SIZE:
                x_buffer.pop(0)
                width_buffer.pop(0)

            x_median = sorted(x_buffer)[len(x_buffer) // 2]
            width_median = sorted(width_buffer)[len(width_buffer) // 2]

            x = x_median

        # --------- STATE MACHINE -----------

        if alignment_state == ALIGN_YAW:
            if tag:
                error = x - 160
                if abs(error) <= TAG_CENTER_TOLERANCE:
                    drivetrain.set_speed(0, 0)
                    alignment_state = ALIGN_APPROACH
                else:
                    turn_speed = 30 if error > 0 else -30
                    drivetrain.set_speed(turn_speed, -turn_speed)
                    time.sleep(0.1)
            else:
                drivetrain.set_speed(60, -60)
                time.sleep(0.1)
                drivetrain.set_speed(0, 0)

        elif alignment_state == ALIGN_APPROACH:
            if tag:
                if width_median < DESIRED_TAG_WIDTH:
                    drift_error = x - 160

                    if abs(drift_error) > TAG_CENTER_TOLERANCE:
                        if drift_error > 0:
                            print("[ALIGN] Correcting drift: steering left.")
                            drivetrain.set_speed(50, 30)
                        else:
                            print("[ALIGN] Correcting drift: steering right.")
                            drivetrain.set_speed(30, 50)

                        if stall_start_time is None:
                            stall_start_time = time.time()
                            last_x_position = x
                        elif time.time() - stall_start_time > STALL_DETECTION_WINDOW:
                            if abs(x - last_x_position) < STALL_X_THRESHOLD:
                                print("[STALL] No x movement while steering. Boosting forward!")
                                drivetrain.set_speed(40, 40)
                                time.sleep(0.5)
                                drivetrain.set_speed(0, 0)
                                stall_start_time = None
                            else:
                                stall_start_time = time.time()
                                last_x_position = x

                    else:
                        # Centered, check tilt
                        width_now = tag[2]
                        height_now = tag[3]
                        if height_now == 0:
                            ratio_now = 1.0
                        else:
                            ratio_now = width_now / height_now
                        print(f"[DEBUG] Tilt ratio = {ratio_now:.2f}")

                        if ratio_now < TILT_RATIO_THRESHOLD:
                            print("[ALIGN] Tag is tilted — rotating to correct...")
                            drivetrain.set_speed(30 * turn_direction, -30 * turn_direction)
                            time.sleep(0.2)
                            drivetrain.set_speed(0, 0)
                        else:
                            # Good, centered and not tilted
                            print("[ALIGN] Centered and straight. Moving forward.")
                            drivetrain.set_speed(50, 50)
                            stall_start_time = None

                elif width_median > TOO_CLOSE_TAG_WIDTH:
                    print("[ALIGN] Too close to tag. Backing up...")
                    drift_error = x - 160
                    if abs(drift_error) <= TAG_CENTER_TOLERANCE:
                        drivetrain.set_speed(-50, -50)
                    elif drift_error > 0:
                        drivetrain.set_speed(-65, -30)
                    else:
                        drivetrain.set_speed(-30, -65)

                    drivetrain.set_speed(0, 0)

                else:
                    drivetrain.set_speed(0, 0)
                    alignment_state = ALIGN_FINE

            else:
                if x < 160:
                    drivetrain.set_speed(30, -30)
                else:
                    drivetrain.set_speed(-30, 30)

                if time.time() - last_seen_time > 2:
                    print("[ALIGN] Tag lost too long. Cancelling alignment.")
                    drivetrain.set_speed(0, 0)
                    break

        elif alignment_state == ALIGN_FINE:
            if tag:
                error = x - 160
                if abs(error) <= TAG_CENTER_TOLERANCE:
                    drivetrain.set_speed(0, 0)
                    alignment_state = ALIGN_DONE
                else:
                    turn_speed = 60 if error > 0 else -60
                    drivetrain.set_speed(turn_speed, -turn_speed)
            else:
                if x < 160:
                    drivetrain.set_speed(50, -50)
                else:
                    drivetrain.set_speed(-50, 50)

                if time.time() - last_seen_time > 3:
                    print("[ALIGN] Tag lost too long. Cancelling alignment.")
                    drivetrain.set_speed(0, 0)
                    break

        elif alignment_state == ALIGN_DONE:
            print("[ALIGN] Alignment complete!")
            drivetrain.set_speed(0, 0)
            servo.set_angle(DOWN)
            break

        time.sleep(0.1)

# April Tag Alignment for an April Tag located on the floor
def align_to_floor_apriltag():

    # === AprilTag Alignment Constants ===
    CAMERA_CENTER_X = 160
    CAMERA_CENTER_Y = 120

    X_TOLERANCE = 30
    Y_TOLERANCE = 30

    ALIGN_TRANSLATE = "TRANSLATE"
    ALIGN_FINE = "FINE"
    ALIGN_DONE = "DONE"

    alignment_state = ALIGN_TRANSLATE

    print("[ALIGN] Starting alignment to floor AprilTag...")

    while True:
        tag_data = husky.command_request_blocks()

        if len(tag_data) > 0:
            tag = tag_data[0]
            x, y = tag[0], tag[1]
            width = tag[2]

            print(f"[ALIGN] State: {alignment_state} | X: {x}, Y: {y}, Width: {width}")

            dx = x - CAMERA_CENTER_X
            dy = y - CAMERA_CENTER_Y

            if alignment_state == ALIGN_TRANSLATE:
                # Translate left/right
                if abs(dx) > X_TOLERANCE:
                    if dx > 0:
                        drivetrain.set_speed(30, -30)  # Turn right
                    else:
                        drivetrain.set_speed(-30, 30)  # Turn left
                    continue

                # Translate forward/backward
                if abs(dy) > Y_TOLERANCE:
                    if dy > 0:
                        drivetrain.set_speed(-30, -30)  # Move backward
                    else:
                        drivetrain.set_speed(30, 30)    # Move forward
                    continue

                # If within tolerance
                drivetrain.set_speed(0, 0)
                alignment_state = ALIGN_FINE

            elif alignment_state == ALIGN_FINE:
                if abs(dx) <= 15 and abs(dy) <= 15:
                    drivetrain.set_speed(0, 0)
                    alignment_state = ALIGN_DONE
                else:
                    x_correction = 20 if dx > 0 else -20
                    y_correction = 20 if dy > 0 else -20
                    drivetrain.set_speed(y_correction + x_correction,
                                        y_correction - x_correction)

            elif alignment_state == ALIGN_DONE:
                print("[ALIGN] Alignment complete.")
                drivetrain.set_speed(0, 0)
                break
        else:
            print("[ALIGN] No AprilTag detected.")
            drivetrain.set_speed(0, 0)
    
        time.sleep(0.001)

def get_filtered_distance():
    new_distance = Rangefinder.distance()
    distance_buffer.append(new_distance)
    if len(distance_buffer) > 5:
        distance_buffer.pop(0)
    return sorted(distance_buffer)[len(distance_buffer) // 2]

# Behavior for BASE role
def handle_base_behavior():
    global base_state, base_ready, top_ready, top_button_pressed

    if base_state == BASE_STATE_WAIT_FOR_BUTTON:
        print("[BASE] Waiting for MQTT signal that top robot's button was pressed...")
        if top_button_pressed:
            print("[BASE] Received signal – beginning random walk.")
            time.sleep(1)
            base_state = BASE_STATE_RANDOM_WALK

    elif base_state == BASE_STATE_RANDOM_WALK:
        servo.set_angle(UP)
        top_button_pressed = False
        print("[BASE] Random walking toward table...")

        base_speed = random.randint(40, 80)
        turn_bias = random.randint(-20, 20)

        left_speed = max(min(base_speed + turn_bias, 60), 40)
        right_speed = max(min(base_speed - turn_bias, 60), 40)

        drivetrain.set_speed(left_speed, right_speed)
        time.sleep(2)
        drivetrain.set_speed(0, 0)

        if detect_april_tag():
            print("[BASE] AprilTag detected! Transitioning to alignment.")
            base_state = BASE_STATE_ALIGN_APRILTAG

    elif base_state == BASE_STATE_ALIGN_APRILTAG:
        print("[BASE] Aligning with AprilTag...")
        align_to_wall_apriltag()
        servo.set_angle(DOWN)
        base_state = BASE_STATE_READY_FOR_UNSTACK
        client.publish(MQTT_COMMAND_TOPIC, "BASE_READY")

    elif base_state == BASE_STATE_READY_FOR_UNSTACK:
        print("[BASE] Ready for top to unstack.")
        if top_ready:
            base_state = BASE_STATE_WALL_FOLLOWING

    elif base_state == BASE_STATE_WALL_FOLLOWING:
        top_ready = False
        print("[BASE] Starting wall-following behavior...")

        # Initial back-up and turn
        time.sleep(2)
        drivetrain.set_speed(-30, -30)
        time.sleep(0.5)
        drivetrain.set_speed(0, 0)

        drivetrain.turn(-90, 1)

        # Constants for wall-following
        TARGET_DISTANCE = 10       # cm
        DEADBAND = 1.0             # cm range for no correction
        KP = 1                   # proportional gain
        MIN_SPEED = 30
        MAX_SPEED = 50
        BASE_SPEED = 35
        WALL_LOST_THRESHOLD = 50

        has_turned_corner = False

        while True:
            distance = get_filtered_distance()
            print(f"[WALL FOLLOW] Distance to wall: {distance:.2f} cm")

            # Check if an AprilTag is detected
            if detect_april_tag():
                print("[BASE] AprilTag detected – transitioning to ALIGN_APRILTAG2.")
                drivetrain.set_speed(0, 0)
                base_state = BASE_STATE_ALIGN_APRILTAG2
                break

            # Check for corner (wall lost)
            if not has_turned_corner and distance > WALL_LOST_THRESHOLD:
                print("[WALL FOLLOW] Wall lost. Turning left at corner...")
                drivetrain.set_speed(BASE_SPEED, BASE_SPEED)
                sleep(1.2)
                drivetrain.turn(90, 1)
                drivetrain.set_speed(BASE_SPEED, BASE_SPEED)
                sleep(1)
                drivetrain.set_speed(0,0)
                has_turned_corner = True
                continue

            # Wall-following using proportional control
            error = TARGET_DISTANCE - distance
            if abs(error) < DEADBAND:
                correction = 0
            else:
                correction = -KP * error

            left_speed = BASE_SPEED - correction
            right_speed = BASE_SPEED + correction

            left_speed = max(min(left_speed, MAX_SPEED), MIN_SPEED)
            right_speed = max(min(right_speed, MAX_SPEED), MIN_SPEED)

            drivetrain.set_speed(left_speed, right_speed)
            sleep(0.1)

    elif base_state == BASE_STATE_ALIGN_APRILTAG2:
        print("[BASE] Aligning with AprilTag...")
        drivetrain.turn(-90, 1)
        drivetrain.set_speed(-60, -60)
        sleep(1.5)
        drivetrain.set_speed(0, 0)
        base_state = BASE_STATE_WAIT_FOR_PICKUP
        client.publish(MQTT_COMMAND_TOPIC, "BASE_WAITING_AT_TAG")
        
    elif base_state == BASE_STATE_WAIT_FOR_PICKUP:
        print("[BASE] Waiting for top to be ready for pickup...")
        if top_ready:
            base_state = BASE_STATE_REJOIN_STACK

    elif base_state == BASE_STATE_REJOIN_STACK:
        print("[BASE] Rejoining stack...")
        # TODO: Implement check to know if it's actually stacked
        if top_ready:
            base_state = BASE_STATE_FINAL_MOVES

    elif base_state == BASE_STATE_FINAL_MOVES:
        print("[BASE] Final random moves...")
        base_speed = random.randint(40, 80)
        turn_bias = random.randint(-20, 20)

        left_speed = max(min(base_speed + turn_bias, 100), 20)
        right_speed = max(min(base_speed - turn_bias, 100), 20)

        drivetrain.set_speed(left_speed, right_speed)
        time.sleep(2)
        drivetrain.set_speed(0, 0)
        base_state = BASE_STATE_DONE

    elif base_state == BASE_STATE_DONE:
        print("[BASE] Done. Stopping motors.")
        stop_all_motors()

# Behavior Placeholders for other roles
def handle_middle_behavior():
    global middle_state

    if middle_state == MIDDLE_STATE_IDLE:
        print("[MIDDLE] Waiting... (idle)")
        if base_ready and top_ready:
            middle_state = MIDDLE_STATE_PREPARE_FOR_RESTACK

    elif middle_state == MIDDLE_STATE_PREPARE_FOR_RESTACK:
        print("[MIDDLE] Stack ready. Preparing for re-stack or shutdown.")
        # TODO: Figure out if the middle robot would do anything
        time.sleep(2)
        middle_state = MIDDLE_STATE_IDLE


def handle_top_behavior():
    print("TOP behavior running...")
    global top_state, base_ready, top_ready, top_button_pressed, top_starting
    if top_state == TOP_STATE_WAIT_FOR_START:
        if not top_starting:
            while not board.is_button_pressed():
                time.sleep(0) # unsure how to just make this run until button is pressed, while also still going through main loop
                
            top_starting = True
            #client.publish(MQTT_COMMAND_TOPIC, "TOP_BUTTON_PRESSED") GROUND BOT DOESNT CARE YET
        elif top_starting:
            # maybe move back and forth briefly? indicates it's ready to be stacked
            top_state = TOP_STATE_STACKING
            top_starting = False

    elif top_state == TOP_STATE_STACKING: 
        """Process is manual for now, just need to press button when done stacking."""
        if not top_ready:
            while not board.is_button_pressed():
                time.sleep(0) # unsure how to just make this run until button is pressed, while also still going through main loop
                
            top_ready = True
            client.publish(MQTT_COMMAND_TOPIC, "TOP_BUTTON_PRESSED") # Tell base bot that stacking is complete
        elif top_ready:
            if base_ready: 
                top_state = TOP_STATE_EXIT
                top_ready = False
                base_ready = False
    
    elif top_state == TOP_STATE_EXIT:
        drivetrain.set_speed(30,30)
        time.sleep(0.5)
        client.publish(MQTT_COMMAND_TOPIC, "TOP_READY") # Tell base bot that it can start wall following
        top_state = TOP_STATE_LINE_FOLLOWING

    elif top_state == TOP_STATE_LINE_FOLLOWING:
        error = reflect.get_left() - reflect.get_right() # initiate reflectance above
        correction = line_pid.update(error) # add PID class and initiate above
        speed_left = 20 - correction
        speed_right = 20 + correction
        drivetrain.set_speed(speed_left,speed_right)
        if detect_april_tag():
            drivetrain.set_speed(0,0)
            top_state = TOP_STATE_ALIGN
        #time.sleep(0.05) Already a 0.1 delay in main loop
    
    elif top_state == TOP_STATE_ALIGN:
        print("[TOP] Aligning with AprilTag...")
        drivetrain.set_speed(0,-30)
        time.sleep(0.6)
        drivetrain.set_speed(0,0)
        top_state = TOP_STATE_REJOIN
        client.publish(MQTT_COMMAND_TOPIC, "TOP_WAITING_AT_TAG") # Tell base bot that it can start wall following

    elif top_state == TOP_STATE_REJOIN:
        if base_ready:
            drivetrain.set_speed(30,30)
            time.sleep(1.3)
            drivetrain.set_speed(0,0)
            base_ready = False
            client.publish(MQTT_COMMAND_TOPIC, "TOP_READY") # Tell base bot that it restacked
            top_state = TOP_STATE_UNSTACKING

    elif top_state == TOP_STATE_UNSTACKING:
        """Process is manual for now, just need to press button when done unstacking."""
        if not top_ready:
            while not board.is_button_pressed():
                time.sleep(0) # unsure how to just make this run until button is pressed, while also still going through main loop
                
            top_ready = True
            client.publish(MQTT_COMMAND_TOPIC, "TOP_BUTTON_PRESSED") # Tell base bot that unstacking is complete
            top_state = TOP_STATE_DONE

    elif top_state == TOP_STATE_DONE:
        print("[TOP] Done. Stopping motors.")
        stop_all_motors()

def stop_all_motors():
    drivetrain.set_speed(0, 0)
    print("Motors stopped.")

# MQTT Setup
client = connect_mqtt()
client.set_callback(handle_mqtt_message)
client.subscribe(MQTT_ROLE_TOPIC)
client.subscribe(MQTT_COMMAND_TOPIC)
client.subscribe(MQTT_BROADCAST_TOPIC)

print("Robot ID:", robot_id)

# Main Loop
while True:
    try:
        client.check_msg()
    except Exception as e:
        print("MQTT check failed:", e)
    # If robot has just been initialized, communicate "NEW ROBOT" message
    if state == STATE_INIT:
        print("Broadcasting NEW_ROBOT message...")
        for _ in range(3):
            client.publish(MQTT_BROADCAST_TOPIC, f"NEW_ROBOT:{robot_id}")
            time.sleep(0.2)
        known_ids.add(robot_id)
        start_time = time.time()
        state = STATE_WAIT_ROLE

    # If robot is assigning roles, wait for three seconds, and proceed to assign roles
    elif state == STATE_WAIT_ROLE:

        if len(known_ids.union({robot_id})) < len(ROLES):
            print("Waiting for all 3 robots to join...", len(known_ids.union({robot_id})), "detected")
        elif time.time() - start_time > role_assignment_delay and is_role_assigner():
            print("Acting as assigner")
            all_detected = known_ids.union({robot_id})
            if len(all_detected) < len(ROLES):
                print("Not enough robots yet, waiting...")
            else:
                for i, rid in enumerate(sorted(all_detected)):
                    if i < len(ROLES):
                        role_to_assign = ROLES[i]
                        assigned_ids[rid] = role_to_assign
                        print("Assigning role", role_to_assign, "to", rid)
                        client.publish(MQTT_ROLE_TOPIC, f"{rid}:{role_to_assign}")
    
        if role is not None:
            print("All roles assigned and my role is", role, "- proceeding.")
            state = STATE_ROLE_BEHAVIOR
        else:
            # This robot hasn't received its role yet, wait a bit longer
            print("Waiting for my role assignment...")
        else:
            print(f"Waiting for role assignment... ({len(assigned_ids)}/{len(ROLES)} assigned)")



    elif state == STATE_ROLE_BEHAVIOR:
        if role == "BASE":
            handle_base_behavior()
        elif role == "MIDDLE":
            handle_middle_behavior()
        elif role == "TOP":
            handle_top_behavior()

    elif state == STATE_STOPPED:
        stop_all_motors()

    time.sleep(0.05)
