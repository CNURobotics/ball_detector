#!/usr/bin/env python

import sys
import numpy as np
import cv2


#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
#
class ImageProc:

    def __init__(self):
        pass

    def process(self, image, cv_show=False):
        """
        Basic image processing algorithms
        """
        try:
            img_blur = cv2.GaussianBlur(image, (5, 5), 0)

            # Segement in HSV color space HSV=hue-staturation-value (intensity)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            hue = hsv_image[:, :, 0]
            sat = hsv_image[:, :, 1]
            value = hsv_image[:, :, 2]
            sat_ret, sat_thrsh = cv2.threshold(sat, 64, 255, cv2.THRESH_BINARY)#+cv2.THRESH_OTSU)
            val_ret, val_thrsh = cv2.threshold(value, 48, 255, cv2.THRESH_BINARY)#+cv2.THRESH_OTSU)

            # Mostly pure color of reasonable intensity
            hsv_thrsh = cv2.bitwise_and(sat_thrsh, sat_thrsh, mask=val_thrsh)

            # -------------------------------------------------------------------
            # The work horse after basic thresholding
            #
            # To do, watershed or some segmentation to identify balls
            # https://docs.opencv.org/4.x/d3/db4/tutorial_py_watershed.html
            kernel = np.ones((3, 3), np.uint8)

            # Erode, then dilate to get rid of small speckles
            opening = cv2.morphologyEx(hsv_thrsh, cv2.MORPH_OPEN, kernel, iterations=2)

            # Make bigger to make sure we are not getting any background
            sure_not_bg = cv2.dilate(opening, kernel, iterations=3)

            dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
            divisor = np.max(np.max(dist_transform))
            if divisor > 0.0:
                dist_transform = dist_transform / divisor
            dist_transform = (255*dist_transform).astype(np.uint8)

            ret, sure_fg = cv2.threshold(dist_transform, 160, 255, cv2.THRESH_BINARY)
            sure_fg = sure_fg.astype(np.uint8)

            sure_bg = 255 - sure_not_bg
            unknown = cv2.subtract(sure_not_bg, sure_fg)

            # https://docs.opencv.org/4.x/d3/db4/tutorial_py_watershed.html
            # Marker labelling - label each region with unique integer
            ret, markers = cv2.connectedComponents(sure_fg)
            # Add one to all labels so that sure background is 1 not 0
            markers = markers+1
            # Now, mark the region of unknown with zero
            markers[unknown == 255] = 0

            watershed_image = img_blur.copy()  # Just for debug display

            # This is the magic
            markers = cv2.watershed(img_blur, markers)
            watershed_image[markers == -1] = [255, 255, 255]
            markers[markers == -1] = 0
            markers = markers*30   # Inflate values so they differences are visible in debug image

            if np.max(np.max(markers)) > 255:
                # Sanity check to make sure below conversion is valid
                print(f" Invalid max markers = {np.max(np.max(markers))}")
            markers = markers.astype(np.uint8)

            # Now, anything that is marked as not background or boundary is flagged in binary image
            ret, sure_marked = cv2.threshold(markers, 45, 255, cv2.THRESH_BINARY)
            sure_marked = sure_marked.astype(np.uint8)

            # Dilate, then erode to smooth gaps on edges
            closing = cv2.morphologyEx(sure_marked, cv2.MORPH_CLOSE, kernel, iterations=2)
            # Erode, then dilate to get rid of small speckles that are appendages to main blob
            ball_marked = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel, iterations=5)

            # Shrink just a bit
            labeled_balls = cv2.erode(ball_marked, kernel, iterations=2)
            ret, labeled_balls = cv2.connectedComponents(labeled_balls)

            # labeled_balls is now a uint32 type
            labeled_balls = labeled_balls.astype(np.uint8)  # Assuming < 255 labels

            return labeled_balls, hue, sat, value

        except  (RuntimeError, TypeError, NameError) as e:
            print("ImageProc.process error")
            print(e)
        except Exception as e:
            print("Unknown error in ImageProc.process")
            print(e)

        return None, None, None, None

def main():
    args = sys.argv
    print(args)

    image_proc = ImageProc()
    try:
        print(cv2.__version__)

        print(f"ImageProc demo read file<{args[1]}>")
        img = cv2.imread(args[1])

        cv2.imshow("ImageProc", img)

        print(f"Processing image of {img.shape}")
        image_proc.process(img, cv_show=True)
        print(30*"=")

        cv2.waitKey(0)

        print("Done!")

    except Exception as e:
        print("Exception - Shutting down")
        print(e)

if __name__ == '__main__':
    main()
