"""
balloon_video.py

This file includes functions to:
    Initialise the camera
    Initialise the output video

Image size is held in the balloon_finder.cnf
"""

import sys
from os.path import expanduser
import time
import math
from multiprocessing import Process, Pipe
import cv2
import balloon_config

# import pi camera if present
try:
    from picamera.array import PiRGBArray as PiRGBArray
    from picamera import PiCamera as PiCamera
except ImportError:
    print "picamera not installed - please install if running on RPI"

class BalloonVideo:

    def __init__(self):
        # camera type - 0=web cam, 1=RPI camera
        self.camera_type = balloon_config.config.get_integer('camera','type',1)

        # camera objects
        self.camera = None

        # get image resolution
        self.img_width = balloon_config.config.get_integer('camera','width',640)
        self.img_height = balloon_config.config.get_integer('camera','height',480)

        # get image center
        self.img_center_x = self.img_width / 2
        self.img_center_y = self.img_height / 2
        
        # define field of view
        self.cam_hfov = balloon_config.config.get_float('camera','horizontal-fov',70.42)
        self.cam_vfov = balloon_config.config.get_float('camera','vertical-fov',43.3)

        # define video output filename
        self.video_filename = balloon_config.config.get_string('camera','video_output_file','~/balloon-%Y-%m-%d-%H-%M.avi')
        self.video_filename = expanduser(self.video_filename)
        self.video_filename = time.strftime(self.video_filename)

        # background image processing variables
        self.proc = None            # background process object
        self.parent_conn = None     # parent end of communicatoin pipe
        self.img_counter = 0        # num images requested so far

    # __str__ - print position vector as string
    def __str__(self):
        return "BalloonVideo Object W:%d H:%d" % (self.img_width, self.img_height)

    # initialise camera
    def init_camera(self):
        # return immediately if already initialised
        if not self.camera is None:
            return
 
        # use webcam
        if self.camera_type == 0:
            self.camera = cv2.VideoCapture(0)
            #self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.img_width)
            #self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.img_height)

            # check we can connect to camera
            if not self.camera.isOpened():
                print "failed to open camera, exiting!"
                sys.exit(0)

        # use rpi camera
        if self.camera_type == 1:
            self.camera = PiCamera()
            self.camera.resolution = (self.img_width,self.img_height)
            # to-do: check we can connect to camera

    # close camera
    def close_camera(self):
        # return immediately if already initialised
        if not self.camera is None:
            return
        # use webcam
        if self.camera_type == 0:
            self.camera.release()
        # use rpi camera
        if self.camera_type == 1:
            self.camera.close()

    # capture image from camera
    def capture_image(self):
        # check camera is initialised
        self.init_camera()

        # use webcam
        if self.camera_type == 0:
            success_flag, image=self.camera.read()
            return image

        # use rpi camera
        if self.camera_type == 1:
            image_array = PiRGBArray(self.camera)
            self.camera.capture(image_array, format="bgr")
            image = image_array.array
            return image

    # open_video_writer - begin writing to video file
    def open_video_writer(self):
        # Define the codec and create VideoWriter object
        # Note: setting ex to -1 will display pop-up requesting user choose the encoder
        ex = cv2.VideoWriter_fourcc('M','J','P','G')
        self.video_writer = cv2.VideoWriter(self.video_filename, ex, 25, (self.img_width,self.img_height))
        if not self.video_writer is None:
            print "started recording video to %s" % self.video_filename
        else:
            print "failed to start recording video to %s" % self.video_filename
        return self.video_writer

    # pixels_to_angle_x - converts a number of pixels into an angle in radians 
    def pixels_to_angle_x(self, num_pixels):
        return num_pixels * math.radians(self.cam_hfov) / self.img_width
    
    # pixels_to_angle_y - converts a number of pixels into an angle in radians 
    def pixels_to_angle_y(self, num_pixels):
        return num_pixels * math.radians(self.cam_vfov) / self.img_height
    
    # angle_to_pixels_x - converts a horizontal angle (i.e. yaw) to a number of pixels
    #    angle : angle in radians
    def angle_to_pixels_x(self, angle):
        return int(angle * self.img_width / math.radians(self.cam_hfov))
    
    # angle_to_pixels_y - converts a vertical angle (i.e. pitch) to a number of pixels
    #    angle : angle in radians 
    def angle_to_pixels_y(self, angle):
        return int(angle * self.img_height / math.radians(self.cam_vfov))

    #
    # background image processing routines
    #

    # image_capture_background - captures all images from the camera in the background and returning the latest image via the pipe when the parent requests it
    def image_capture_background(self, imgcap_connection):
        # exit immediately if imgcap_connection is invalid
        if imgcap_connection is None:
            print "image_capture failed because pipe is uninitialised"
            return

        # open the camera
        self.init_camera()

        # clear latest image
        latest_image = None

        while True:
            # constantly get the image from the webcam
            image = self.capture_image()

            # if successful overwrite our latest image
            if not image is None:
                latest_image = image

            # check if the parent wants the image
            if imgcap_connection.poll():
                recv_obj = imgcap_connection.recv()
                # if -1 is received we exit
                if recv_obj == -1:
                    break

                # otherwise we return the latest image
                imgcap_connection.send(latest_image)

        # release camera when exiting
        self.close_camera()

    # start_background_capture - starts background image capture
    def start_background_capture(self):
        # create pipe
        self.parent_conn, imgcap_conn = Pipe()

        # create and start the sub process and pass it it's end of the pipe
        self.proc = Process(target=self.image_capture_background, args=(imgcap_conn,))
        self.proc.start()

    def stop_background_capture(self):
        # send exit command to image capture process
        self.parent_conn.send(-1)

        # join process
        self.proc.join()

    # get_image - returns latest image from the camera captured from the background process
    def get_image(self):
        # return immediately if pipe is not initialised
        if self.parent_conn == None:
            return None

        # send request to image capture for image
        self.parent_conn.send(self.img_counter)

        # increment counter for next interation
        self.img_counter = self.img_counter + 1

        # wait endlessly until image is returned
        recv_img = self.parent_conn.recv()

        # return image to caller
        return recv_img

    # main - tests BalloonVideo class
    def main(self):

        # start background process
        self.start_background_capture()

        while True:
            # send request to image capture for image
            img = self.get_image()
    
            # check image is valid
            if not img is None:
                # display image
                cv2.imshow ('image_display', img)
            else:
                print "no image"
    
            # check for ESC key being pressed
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
    
            # take a rest for a bit
            time.sleep(0.1)

        # send exit command to image capture process
        self.stop_background_capture()

        print "a2p 10 = %f" % self.angle_to_pixels_x(10)
        print "p2a 10 = %f" % self.pixels_to_angle_x(10)

# create a single global object
balloon_video = BalloonVideo()

if __name__ == "__main__":
    balloon_video.main()