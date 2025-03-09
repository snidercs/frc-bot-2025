# NOTE: This code runs in its own process, so we cannot access the robot here,
#       nor can we create/use/see wpilib objects
#
# To try this code out locally (if you have robotpy-cscore installed), you
# can execute `python3 -m cscore vision.py:main`
#

import cv2
import numpy as np

from cscore import CameraServer as CS

WIDTH = 1280
HEIGHT = 720
DEVICE = 0
STREAM_NAME = "Elevator Cam"

def main():
    CS.enableLogging()

    # Get the UsbCamera from CameraServer
    camera = CS.startAutomaticCapture (DEVICE)
    # Set the resolution
    camera.setResolution (WIDTH, HEIGHT)

    # Get a CvSink. This will capture images from the camera
    cvSink = CS.getVideo()
    # Setup a CvSource. This will send images back to the Dashboard
    outputStream = CS.putVideo ("Elevator Cam", WIDTH, HEIGHT)

    # Allocating new images is very expensive, always try to preallocate
    mat = np.zeros(shape=(HEIGHT, WIDTH, 3), dtype=np.uint8)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, mat = cvSink.grabFrame (mat)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        # Coordinates for the crosshair
        crosshair_x = 885 
        crosshair_y = 675

        # Define the length of the crosshair lines
        line_length = 50

        # Draw horizontal line
        cv2.line(mat, (crosshair_x - line_length, crosshair_y), (crosshair_x + line_length, crosshair_y), (255, 255, 255), 2)
        # Draw vertical line
        cv2.line(mat, (crosshair_x, crosshair_y - line_length), (crosshair_x, crosshair_y + line_length), (255, 255, 255), 2)

        # Give the output stream a new image to display
        outputStream.putFrame (mat)
