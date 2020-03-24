# USAGE
# python detect_video.py --model mobilenet_ssd_v2/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite --labels mobilenet_ssd_v2/coco_labels.txt

# import the necessary packages
from edgetpu.detection.engine import DetectionEngine
from imutils.video import FPS
from imutils.video import VideoStream
from PIL import Image
import argparse
import time
import cv2
import math

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
#vs = VideoStream(usePiCamera=False).start()
time.sleep(2.0)

fps = FPS().start()

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
    frame = cv2.resize(frame, (frame_height,frame_width))
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
            angle = int(math.atan((box_centerY - Ypov) / (box_centerX - Xpov)) * 180 / math.pi)

            # create circle of where the arm is
            cv2.circle(orig, (Xpov, Ypov), 5, (0, 0, 255), -1)

            # create line connecting the arm and object location with the angle calculated too
            cv2.line(orig, (box_centerX, box_centerY), (Xpov, Ypov), (0, 0, 255), 1)
            cv2.putText(orig, str(angle), (Xpov, Ypov + 10, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # create circle of center frame
            cv2.circle(orig, (midframe_width, midframe_height), 5, (0, 0, 255), -1)

            # calculate the distance from person to center of frame
            calcDistance = midframe_width - box_centerY
            cv2.putText(orig, str(calcDistance), (midframewidth, midframe_height + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # show the output frame and wait for a key press
    #orig = cv2.resize(orig, (640, 480))
    cv2.imshow("Frame", orig)
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

    # update the FPS counter
    fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elapse time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
