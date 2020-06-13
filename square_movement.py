from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

# Connect to the Vehicle.
print("Connecting to vehicle on: serial0")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    while not vehicle.mode.name == 'GUIDED':  # Wait until mode has changed
        print(" Waiting for mode change ...")
        time.sleep(1)
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


# Arm and take of to altitude of 6 meters
arm_and_takeoff(6)

"""
Convenience functions for sending immediate/guided mode commands to control the Copter.
The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
* MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed (latitude, longitude, altitude)
* MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.
The full set of available commands are listed here:
http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
"""


def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see:
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

"""
Functions that move the vehicle by specifying the velocity components in each direction.
The two functions use different MAVLink commands. The main difference is
that depending on the frame used, the NED velocity can be relative to the vehicle
orientation.
The methods include:
* send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
* send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
"""


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

DURATION = 5

# Set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
NORTH = 1
SOUTH = -1

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
EAST = 1
WEST = -1

# Note for vz:
# vz < 0 => ascend
# vz > 0 => descend
UP = -0.5
DOWN = 0.5

print("SQAURE path using SET_POSITION_TARGET_GLOBAL_INT and velocity parameters")
# vx, vy are parallel to North and East (independent of the vehicle orientation)

print("Yaw 225 absolute")
#condition_yaw(0)

print("Velocity South, West and Up")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

print("Velocity North, West and Down")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

print("Velocity North and East")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

print("Velocity South and East")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

"""
The example is completing. RETURN_TO_LAUNCH.
"""

print("Setting RETURN_TO_LAUNCH mode...")
vehicle.mode = VehicleMode("RTL")
while not vehicle.mode.name == "RTL":  # Wait until mode has changed
    print(" Waiting for mode change ...")
    time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

print("Completed")
