# USAGE
# python square_movement_test.py

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

# connect to the Vehicle.
print("Connecting to vehicle on: serial0")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)

"""
The following functions are used to control the movement of the drone
"""

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # copter should arm in GUIDED mode
    while not vehicle.mode.name == 'GUIDED':  # wait until mode has changed
        print(" Waiting for mode change ...")
        time.sleep(1)

    # arm the copter
    print("Arming motors")
    vehicle.armed = True
    while not vehicle.armed:  # wait until copter is armed
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # take off to target altitude

    # wait until the vehicle reaches a safe height before processing the goto (otherwise the command after
    # Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

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
        
# take off to 10m
arm_and_takeoff(10)

# set duration time to 10 seconds
DURATION = 10

# set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
NORTH = 1  # fly North at 1 km/h
SOUTH = -1  # fly South at 1 km/h

# vy > 0 => fly East
# vy < 0 => fly West
EAST = 1  # fly East at 1 km/h
WEST = -1  # fly West at 1 km/h

# Note for vz:
# vz < 0 => ascend
# vz > 0 => descend
UP = -0.5  # fly Up at 0.5 km/h
DOWN = 0.5  # fly Down at 0.5 km/h

print("SQAURE path using SET_POSITION_TARGET_GLOBAL_INT and velocity parameters")
# vx, vy are parallel to North and East (independent of the vehicle orientation)

# fly forward
print("Velocity forward")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

# rotate 90 degrees right
print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

# fly forward
print("Velocity forward")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

# rotate 90 degrees right
print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

# fly forward
print("Velocity forward")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

# rotate 90 degrees right
print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

# fly forward
print("Velocity forward")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

# rotate 90 degrees right
print("Yaw 90 relative (to previous yaw heading)")
condition_yaw(90, relative=True)

# wait 2 seconds
time.sleep(2)

"""
The example is completing. RETURN_TO_LAUNCH.
"""
# return back to launch location
print("Setting RETURN_TO_LAUNCH mode...")
vehicle.mode = VehicleMode("RTL")
while not vehicle.mode.name == "RTL":  # Wait until mode has changed
    print(" Waiting for mode change ...")
    time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

print("Completed")
