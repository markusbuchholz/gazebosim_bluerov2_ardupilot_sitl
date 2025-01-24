from pymavlink import mavutil
import time

# Connect to the vehicle
conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the heartbeat message to find the system ID
conn.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (conn.target_system, conn.target_component))

def arm_vehicle(conn):
    """ Arms the vehicle """
    conn.mav.command_long_send(
        conn.target_system, 
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        1,  # Confirmation
        1,  # Param 1: 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0)
    conn.motors_armed_wait()
    print("Vehicle armed")

def disarm_vehicle(conn):
    """ Disarms the vehicle """
    conn.mav.command_long_send(
        conn.target_system, 
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        1,  # Confirmation
        0,  # Param 1: 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0)
    conn.motors_disarmed_wait()
    print("--- Vehicle disarmed ---")

def set_manual_mode(conn):
    conn.set_mode_apm("MANUAL")
    print("--- Vehicle set to manual mode ---")

def send_motor_test(conn, motor_number, thrust_type, thrust_value, duration):
    """
    https://mavlink.io/en/messages/common.html#MOTOR_TEST_THROTTLE_TYPE
    
    Sends a MAV_CMD_DO_MOTOR_TEST command to the vehicle.
    
    :param motor_number: Motor number (1-8 for an 8-thruster vehicle)
    :param thrust_type: Thrust type (0 = Percent, 1 = PWM, 2 = Pilot, 3 = Control)
    :param thrust_value: Thrust value (0.0 - 100.0 for percent)
    :param duration: Duration of the motor test in seconds
    """
    conn.mav.command_long_send(
        conn.target_system,  # target_system
        conn.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,  # command
        0,  # confirmation
        motor_number,  # param1 (motor number)
        thrust_type,  # param2 (thrust type)
        thrust_value,  # param3 (thrust value)
        duration,  # param4 (duration)
        0, 0, 0)  # param5, param6, param7 (unused)

# Arm the vehicle
arm_vehicle(conn)

# Set to manual mode
set_manual_mode(conn)

# Test each thruster individually
num_thrusters = 8
thrust_type = 0  # 0 = Percent
thrust_value = 50.0  # 50% thrust
duration = 5  # 5 seconds for each test

for i in range(1, num_thrusters + 1):
    print(f"Testing motor {i}")
    send_motor_test(conn, i, thrust_type, thrust_value, duration)
    time.sleep(duration + 1)  # Wait for the test to complete plus a buffer

# Disarm the vehicle
disarm_vehicle(conn)

print("Motor tests completed !!")
