import cv2
import math
import numpy as np

# Motor actuator
from Utils import normalize_radian_angle, intersects, seg_intersect, to_radian, dist_to_line_segment

class FakeCamera:
    def capture(self, file_name: str):
        pass


class CameraHandler:
    def __init__(self):
        try:
            from picamera import PiCamera
            self.camera = PiCamera()
        except Exception as e:
            print(e)
            self.camera = FakeCamera()
        
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

        self.triangulation_lines = [None, None, None]
        self.triangulation_index = 0
        self.beacon_estimation = None
        # self.counter = 0

    def get(self, my_x, my_y, my_theta, led0, led1):
        # if to close to previous point where a sighting was found, just skip
        for line in self.triangulation_lines:
            if line is None:
                continue
            if dist_to_line_segment([my_x, my_y], line[0], line[1]) < 20:
                # if dist([my_x, my_y], line[0]) < 40: #40cm
                return self.beacon_estimation

        # grab an image from the camera
        # self.camera.capture(self.rawCapture, format="bgr")
        # img = self.rawCapture.array
        led0.set(1)

        self.camera.capture('image.jpg')
        img = cv2.imread('image.jpg', cv2.IMREAD_COLOR)

        # Apply template Matching
        res = cv2.matchTemplate(img, self.template, self.method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        top_left = max_loc
        bottom_right = (top_left[0] + self.w, top_left[1] + self.h)

        cv2.rectangle(img, top_left, bottom_right, (255, 0, 0), 2)
        cv2.imwrite('image-CV.jpg', img)
        pixel_avg = (top_left[0] + bottom_right[0]) / 2
        pixel_ratio = pixel_avg / 720
        theta = (pixel_ratio - 0.5) * 53.50
        if max_val > 0.8:
            print("BEACON FOUND", theta)
            led1.set(1)
            self.add_new_sighting(to_radian(theta), my_x, my_y, my_theta)
            self.triangulate()
            # print("Confidence:",max_val, "Pixels:", pixel_ratio, "Angle:", theta)
            # return theta
        # else:
        #    print("Unseen")
        led0.set(0)

        return self.beacon_estimation

    def add_new_sighting(self, beacon_theta, my_x, my_y, my_theta):

        line_dir = normalize_radian_angle(beacon_theta + my_theta)

        # add a 3.6m line from our pos to beacon direction
        new_line = [np.array([my_x, my_y]), np.array(
            [my_x + math.cos(line_dir) * 360, my_y + math.sin(line_dir) * 360])]

        self.triangulation_lines[self.triangulation_index] = new_line
        self.triangulation_index = (self.triangulation_index + 1) % 3

    def triangulate(self):
        points = []
        if self.triangulation_lines[0] is not None and self.triangulation_lines[1] is not None:
            if intersects(self.triangulation_lines[0][0], self.triangulation_lines[0][1],
                          self.triangulation_lines[1][0], self.triangulation_lines[1][1]):
                points.append(seg_intersect(self.triangulation_lines[0][0], self.triangulation_lines[0][1],
                                            self.triangulation_lines[1][0], self.triangulation_lines[1][1]))
        if self.triangulation_lines[1] is not None and self.triangulation_lines[2] is not None:
            if intersects(self.triangulation_lines[2][0], self.triangulation_lines[2][1],
                          self.triangulation_lines[1][0], self.triangulation_lines[1][1]):
                points.append(seg_intersect(self.triangulation_lines[2][0], self.triangulation_lines[2][1],
                                            self.triangulation_lines[1][0], self.triangulation_lines[1][1]))
        if self.triangulation_lines[0] is not None and self.triangulation_lines[2] is not None:
            if intersects(self.triangulation_lines[2][0], self.triangulation_lines[2][1],
                          self.triangulation_lines[0][0], self.triangulation_lines[0][1]):
                points.append(seg_intersect(self.triangulation_lines[2][0], self.triangulation_lines[2][1],
                                            self.triangulation_lines[0][0], self.triangulation_lines[0][1]))

        self.beacon_estimation = np.mean(points, axis=0) if len(
            points) != 0 else self.triangulation_lines[0][1]


"""
ch = CameraHandler()
while True:
    print("Working...")
    ch.get()
    print("Done.")
    time.sleep(1)"""
