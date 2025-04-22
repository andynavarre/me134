import time
import random
from MQTT.mqttconnect import connect_mqtt
from XRPLib.differential_drive import DifferentialDrive
from XRPLib.board import Board

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

robot_id = str(time.ticks_ms() % 10000)  # Unique per boot
known_ids = set()
assigned_ids = {}
state = STATE_INIT
role = None

start_time = time.time()
role_assignment_delay = 3.0  # seconds

# Check if this robot is the assigner
def is_role_assigner():
    all_ids = list(known_ids.union({robot_id}))
    return robot_id == min(all_ids)

# MQTT Callback
def handle_mqtt_message(topic, msg):
    global role

    topic = topic.decode("utf-8") if isinstance(topic, bytes) else topic
    msg = msg.decode("utf-8") if isinstance(msg, bytes) else msg
    print("MQTT received | Topic:", topic, "Msg:", msg)

    if topic == MQTT_BROADCAST_TOPIC and msg.startswith("NEW_ROBOT:"):
        incoming_id = msg.split(":")[1]
        known_ids.add(incoming_id)

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

# Behavior Placeholders
def handle_base_behavior():
    print("BASE behavior running...")

def handle_middle_behavior():
    print("MIDDLE behavior running...")

def handle_top_behavior():
    print("TOP behavior running...")

def stop_all_motors():
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

    if state == STATE_INIT:
        print("Broadcasting NEW_ROBOT message...")
        for _ in range(3):
            client.publish(MQTT_BROADCAST_TOPIC, f"NEW_ROBOT:{robot_id}")
            time.sleep(0.2)
        known_ids.add(robot_id)
        start_time = time.time()
        state = STATE_WAIT_ROLE

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
