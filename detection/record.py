# USAGE
# python detect_video.py --model mobilenet_ssd_v2/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite --labels mobilenet_ssd_v2/coco_labels.txt

# import the necessary packages
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from edgetpu.detection.engine import DetectionEngine
from imutils.video import VideoStream
from pymavlink import mavutil
from imutils.video import FPS
from subprocess import call
from PIL import Image
import argparse
import time
import cv2
import math
import numpy as np

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # Copter should arm in GUIDED mode
    while not vehicle.mode.name == 'GUIDED':  # Wait until mode has changed
        print(" Waiting for mode change ...")
        time.sleep(1)

    print("Arming motors")
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

"""
Convenience functions for sending immediate/guided mode commands to control the Copter.
The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
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

flying = 0

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-m", "--model", required=True,
                help="path to TensorFlow Lite object detection model")
ap.add_argument("-l", "--labels", required=True,
                help="path to labels file")
ap.add_argument("-c", "--confidence", type=float, default=0.3,
                help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

# initialize the labels dictionary
print("[INFO] parsing class labels...")
labels = {}

# loop over the class labels file
for row in open(args["labels"]):
    # unpack the row and update the labels dictionary
    (classID, label) = row.strip().split(maxsplit=1)
    labels[int(classID)] = label.strip()

# load the Google Coral object detection model
print("[INFO] loading Coral model...")
model = DetectionEngine(args["model"])

# initialize the video stream and allow the camera sensor to warmup
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=False).start()
time.sleep(2.0)

# start video recording
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('/home/pi/SAR_Drone/detection/output.mp4', fourcc, 20.0, (640, 480))

fps = FPS().start()

clock_check = time.clock()

# loop over the frames from the video stream
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 500 pixels
    frame_height = 480
    frame_width = 360
    midframe_height = 180
    midframe_width = 240
    Xpov = midframe_width
    Ypov = frame_height - 125

    frame = vs.read()
    frame = cv2.resize(frame, (frame_height, frame_width))
    frame = cv2.flip(frame, 0)
    orig = frame.copy()

    # prepare the frame for object detection by converting (1) it
    # from BGR to RGB channel ordering and then (2) from a NumPy
    # array to PIL image format
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = Image.fromarray(frame)

    # make predictions on the input frame
    start = time.time()
    results = model.detect_with_image(frame, threshold=args["confidence"],
                                      keep_aspect_ratio=True, relative_coord=False)
    end = time.time()

    # loop over the results
    for r in results:
        # extract the bounding box and box and predicted class label
        box = r.bounding_box.flatten().astype("int")
        (startX, startY, endX, endY) = box
        label = labels[r.label_id]

        # draw the bounding box and label on the image
        cv2.rectangle(orig, (startX, startY), (endX, endY),
                      (0, 255, 0), 2)
        y = startY - 15 if startY - 15 > 15 else startY + 15
        text = "{}: {:.2f}%".format(label, r.score * 100)
        cv2.putText(orig, text, (startX, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if label == "person":
            box_centerX = ((endX - startX) // 2) + startX
            box_centerY = ((endY - startY) // 2) + startY
            cv2.circle(orig, (box_centerX, box_centerY), 5, (0, 0, 255), -1)

            # calculate angle of object to drone
            if box_centerX - Xpov != 0:
                angle = int(math.atan((box_centerY - Ypov) / (box_centerX - Xpov)) * 180 / math.pi)

            if angle < 0:
                angle += 90

            else:
                angle -= 90

            # create circle of where the drone is
            cv2.circle(orig, (Xpov, Ypov), 5, (0, 0, 255), -1)

            # create line connecting the drone and person location with the angle calculated too
            cv2.line(orig, (box_centerX, box_centerY), (Xpov, Ypov), (0, 0, 255), 1)
            cv2.putText(orig, str(angle), (Xpov, Ypov - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # create circle of center frame
            cv2.circle(orig, (midframe_width, midframe_height), 5, (0, 0, 255), -1)

            # calculate the distance from person to center of frame
            calcDistance = midframe_height - box_centerY
            cv2.putText(orig, str(calcDistance), (midframe_width, midframe_height + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 2)

    # show the output frame and wait for a key press
    orig = cv2.resize(orig, (640, 480))
    #cv2.imshow("Frame", orig)

    print(time.clock())

    if flying == 0 and time.clock() - clock_check >= 10:
        flying = 1

        # Connect to the Vehicle.
        print("Connecting to vehicle on: serial0")
        vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)

        # Arm and take of to altitude of 6 meters
        arm_and_takeoff(6)

        print("connected")

    if flying == 1 and vehicle.mode.name == "GUIDED":
        # update the FPS counter
        fps.update()

        out.write(orig)

    if flying == 1 and vehicle.mode.name != 'GUIDED':
        break

# stop the timer and display FPS information
fps.stop()
print("[INFO] elapse time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
out.release()
vs.stop()

print("Close vehicle object")
vehicle.close()

call("sudo shutdown -h now", shell=True)
