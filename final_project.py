import time
import random
from MQTT.mqttconnect import connect_mqtt
from XRPLib.differential_drive import DifferentialDrive
from XRPLib.board import Board
from .rangefinder import Rangefinder
from Husky.huskylensPythonLibrary import HuskyLensLibrary

#Initialize sensors and drivetrain
drivetrain = DifferentialDrive.get_default_differential_drive()
board = Board.get_default_board()
rangefinder = Rangefinder.get_default_rangefinder()

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
robot_id = str(time.ticks_ms() % 10000)  # Unique per boot
known_ids = set()
assigned_ids = {}
state = STATE_INIT
role = None

start_time = time.time()
role_assignment_delay = 3.0  # seconds

# Constant for checks between robots
top_ready = False
top_button_pressed = False
base_ready = False

# Sensor Readings
distance_buffer = []

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


# Check if this robot is the assigner
def is_role_assigner():
    all_ids = list(known_ids.union({robot_id}))
    return robot_id == min(all_ids)

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
                    client.publish(MQTT_STATUS_TOPIC, f"{role} READY")
                elif role != assigned_role:
                    print("Role conflict: already", role, "ignoring", assigned_role)

    # Check status of other robots while doing the circuit
    elif topic == MQTT_COMMAND_TOPIC:
        if msg == "TOP_BUTTON_PRESSED":
            top_button_pressed = True
        elif msg == "TOP_READY" or msg == "TOP_WAITING_AT_TAG":
            top_ready = True

# Just check if the robot has detected an April tag
def detect_april_tag():
    tag_data = husky.command_request_blocks()
    return len(tag_data) > 0

def get_filtered_distance():
    new_distance = Rangefinder.distance()
    distance_buffer.append(new_value)
    if len(distance_buffer) > 5:
        distance_buffer.pop(0)
    return sorted(distance_buffer[len(distance_buffer)//2]

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
        top_button_pressed = False
        print("[BASE] Random walking toward table...")

        base_speed = random.randint(40, 80)
        turn_bias = random.randint(-20, 20)

        left_speed = max(min(base_speed + turn_bias, 100), 20)
        right_speed = max(min(base_speed - turn_bias, 100), 20)

        drivetrain.set_speed(left_speed, right_speed)
        time.sleep(2)
        drivetrain.set_speed(0, 0)

        if detect_april_tag():
            print("[BASE] AprilTag detected! Transitioning to alignment.")
            base_state = BASE_STATE_ALIGN_APRILTAG

    elif base_state == BASE_STATE_ALIGN_APRILTAG:
        print("[BASE] Aligning with AprilTag...")
        # TODO: Implement AprilTag centering logic
        time.sleep(2)
        base_state = BASE_STATE_READY_FOR_UNSTACK
        client.publish(MQTT_COMMAND_TOPIC, "BASE_READY")

    elif base_state == BASE_STATE_READY_FOR_UNSTACK:
        print("[BASE] Ready for top to unstack.")
        if top_ready:
            base_state = BASE_STATE_WALL_FOLLOWING

    elif base_state == BASE_STATE_WALL_FOLLOWING:
        print("[BASE] Backing up from the wall...")
        drivetrain.set_speed(-20, -20)
        time.sleep(1.0)
        drivetrain.set_speed(0, 0)

        print("[BASE] Turning 90 degrees to the right...")
        drivetrain.turn(90)
        time.sleep(1)

        print("[BASE] Starting wall follow...")

        has_turned_corner = False
        Kp = 1.2
        target_distance = 10  # cm

        while True:
            distance = get_filtered_distance()
            print(f"[BASE] Left distance: {distance} cm")

            # AprilTag check (stop condition)
            if detect_april_tag():
                print("[BASE] AprilTag detected – stopping.")
                drivetrain.set_speed(0, 0)
                base_state = BASE_STATE_APRILTAG2
                break

            # Detect left wall loss to find corner
            if not has_turned_corner and distance > 50:
                print("[BASE] Left wall lost – 90° corner detected.")
                drivetrain.set_speed(0, 0)
                time.sleep(0.5)
                drivetrain.turn(-90)
                time.sleep(1)
                has_turned_corner = True
                continue

            # Wall following logic (P-control)
            error = target_distance - distance
            correction = Kp * error

            base_speed = 60
            left_speed = max(min(base_speed - correction, 100), 20)
            right_speed = max(min(base_speed + correction, 100), 20)

            drivetrain.set_speed(left_speed, right_speed)
            time.sleep(0.1)

    elif base_state == BASE_STATE_ALIGN_APRILTAG2:
        print("[BASE] Aligning with AprilTag...")
        # TODO: Implement AprilTag centering logic
        time.sleep(2)
        base_state = BASE_STATE_WAIT_FOR_PICKUP:
        client.publish(MQTT_COMMAND_TOPIC, "BASE_WAITING_AT_TAG")
        
    elif base_state == BASE_STATE_WAIT_FOR_PICKUP:
        print("[BASE] Waiting for top to be ready for pickup...")
        if top_ready:
            base_state = BASE_STATE_REJOIN_STACK

    elif base_state == BASE_STATE_REJOIN_STACK:
        print("[BASE] Rejoining stack...")
        # TODO: Implement check to know if it's actually stacked
        time.sleep(2)
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
        if time.time() - start_time > role_assignment_delay and is_role_assigner():
            print("Acting as assigner")
            unassigned = [rid for rid in known_ids if rid not in assigned_ids]
            for i, rid in enumerate(sorted(unassigned)):
                if i < len(ROLES):
                    role_to_assign = ROLES[i]
                    assigned_ids[rid] = role_to_assign
                    print("Assigning role", role_to_assign, "to", rid)
                    client.publish(MQTT_ROLE_TOPIC, f"{rid}:{role_to_assign}")
                    
        # If robot is not assigner, and has received a role, follow that behavior
        if role is not None:
            print("Role assigned:", role)
            state = STATE_ROLE_BEHAVIOR
        else:
            print("Waiting for role assignment...")

    elif state == STATE_ROLE_BEHAVIOR:
        if role == "BASE":
            handle_base_behavior()
        elif role == "MIDDLE":
            handle_middle_behavior()
        elif role == "TOP":
            handle_top_behavior()

    elif state == STATE_STOPPED:
        stop_all_motors()

    time.sleep(0.1)
