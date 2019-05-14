import cv2
from picamera import PiCamera
import time


# Motor actuator
class CameraHandler:
    def __init__(self):
        self.camera = PiCamera()
        self.template = cv2.imread('template2.jpg', cv2.IMREAD_COLOR)
        #print(self.template.shape)
        _, self.w, self.h = self.template.shape[::-1]

        # All the 6 methods for comparison in a list
        self.method = cv2.TM_CCOEFF_NORMED
        #self.methods =  ['cv2.TM_CCOEFF_NORMED']
        """['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                        'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']"""

        self.counter=0

    def get(self):

        self.camera.capture('image.jpg')
        img = cv2.imread('image.jpg', cv2.IMREAD_COLOR)
        #img2 = img.copy()

        #for meth in self.methods:
        #img = img2.copy()
        #method = eval(meth)

        # Apply template Matching
        res = cv2.matchTemplate(img, self.template, self.method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        #if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        #    top_left = min_loc
        #else:
        top_left = max_loc
        bottom_right = (top_left[0] + self.w, top_left[1] + self.h)

        cv2.rectangle(img, top_left, bottom_right, (255,0,0), 2)
        cv2.imwrite('image'+str(self.counter)+'.jpg', img)
        print("Pixels:", top_left, bottom_right)

        self.counter += 1

ch = CameraHandler()
while True:
    print("Working...")
    ch.get()
    print("Done.")
    time.sleep(1)
