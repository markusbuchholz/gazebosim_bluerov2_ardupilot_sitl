#Markus Buchholz
#!/usr/bin/env python
from pymavlink import mavutil
from time import sleep

# Function to create a connection to the autopilot
def create_connection():
    #return mavutil.mavlink_connection('udpin:192.168.2.1:14550')
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')

def set_servo_function(master, servo_num, function):
    """ Set the servo function for the given servo number. """
    param_name = f'SERVO{servo_num}_FUNCTION'
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        function,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )

def enable_passthrough_mode(master, num_thrusters):
    """ Enable RC Passthrough mode by setting SERVOX_FUNCTION parameters to 1. """
    for i in range(1, num_thrusters + 1):
        set_servo_function(master, i, 1)
        sleep(0.1)  # Give some time for each parameter to be set

def disable_passthrough_mode(master, num_thrusters, backup_params):
    """ Disable RC Passthrough mode by restoring the backup SERVOX_FUNCTION parameters. """
    for i in range(1, num_thrusters + 1):
        set_servo_function(master, i, backup_params[i-1])
        sleep(0.1)  # Give some time for each parameter to be set

def backup_thruster_params(master, num_thrusters):
    """ Backup the current SERVOX_FUNCTION parameters. """
    backup_params = []
    for i in range(1, num_thrusters + 1):
        param_name = f'SERVO{i}_FUNCTION'
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            param_name.encode('utf-8'),
            -1
        )
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        backup_params.append(int(message.param_value))
    return backup_params

def send_rc_override(master, channels):
    """ Send RC override command with the specified PWM values for each channel. """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7]
    )

def control_motor(master, motor_number, pwm_value=1600, duration=3.0):
    """ Control a specific motor by sending an RC override command. """
    channels = [1500] * 8  # Initialize all channels to neutral (1500)
    channels[motor_number - 1] = pwm_value  # Set the specified motor to the test PWM value
    
    send_rc_override(master, channels)
    sleep(duration)
    
    channels[motor_number - 1] = 1500  # Reset the motor to neutral after the test
    send_rc_override(master, channels)

def arm_vehicle(master):
    """ Arm the vehicle and wait for confirmation. """
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed")

def disarm_vehicle(master):
    """ Disarm the vehicle and wait for confirmation. """
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Vehicle disarmed")

if __name__ == '__main__':
    master = create_connection()
    master.wait_heartbeat()
    print("Heartbeat received")

    num_thrusters = 8  # Number of thrusters to control

    try:
        backup_params = backup_thruster_params(master, num_thrusters)
        print("Thruster parameters backed up")

        enable_passthrough_mode(master, num_thrusters)
        print("RC Passthrough mode enabled")

        arm_vehicle(master)
        print('Armed!')

        while True:
            cmd = input("Enter motor number (1-8 to activate, 0 to stop all, q to quit): ")
            if cmd == 'q':
                print("Exiting...")
                break
            elif cmd == '0':
                send_rc_override(master, [1500] * 8)  # Set all motors to neutral
                print("All motors stopped")
            elif cmd in [str(i) for i in range(1, 9)]:
                motor_number = int(cmd)
                control_motor(master, motor_number)
            else:
                print("Invalid command:", cmd)
        
        disarm_vehicle(master)
        disable_passthrough_mode(master, num_thrusters, backup_params)
        print("RC Passthrough mode disabled")
    except Exception as e:
        print("Error:", e)
        disarm_vehicle(master)
        disable_passthrough_mode(master, num_thrusters, backup_params)
        print("RC Passthrough mode disabled due to error")

