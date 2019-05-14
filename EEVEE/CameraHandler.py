import cv2
from picamera import PiCamera
# from picamera.array import PiRGBArray
import time


# Motor actuator
class CameraHandler:
    def __init__(self):
        self.camera = PiCamera()
        # maybe we need to invert resolution?
        # https://www.programcreek.com/python/example/106952/picamera.array.PiRGBArray
        # self.rawCapture = PiRGBArray(self.camera)

        self.template = cv2.imread('template2.jpg', cv2.IMREAD_COLOR)
        # print(self.template.shape)
        _, self.w, self.h = self.template.shape[::-1]

        # time.sleep(0.1)

        # All the 6 methods for comparison in a list
        self.method = cv2.TM_CCOEFF_NORMED
        """['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                        'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']"""

        # self.counter = 0

    def get(self):
        # grab an image from the camera
        # self.camera.capture(self.rawCapture, format="bgr")
        # img = self.rawCapture.array

        self.camera.capture('image.jpg')
        img = cv2.imread('image.jpg', cv2.IMREAD_COLOR)

        # Apply template Matching
        res = cv2.matchTemplate(img, self.template, self.method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        top_left = max_loc
        bottom_right = (top_left[0] + self.w, top_left[1] + self.h)

        cv2.rectangle(img, top_left, bottom_right, (255, 0, 0), 2)
        # cv2.imwrite('image' + str(self.counter) + '.jpg', img)
        pixel_avg = (top_left[0] + bottom_right[0]) / 2
        pixel_ratio = pixel_avg / 720
        theta = (pixel_ratio - 0.5) * 53.50
        if max_val > 0.8:
            # print("Confidence:",max_val, "Pixels:", pixel_ratio, "Angle:", theta)
            return theta
        # else:
        #    print("Unseen")
        return None

"""
ch = CameraHandler()
while True:
    print("Working...")
    ch.get()
    print("Done.")
    time.sleep(1)"""
