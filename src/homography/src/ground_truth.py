#!/usr/bin/env python
import roslib
roslib.load_manifest('homography')
import cv2
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class VideoAnnotator:
    def __init__(self):
        self.colors = None
        pass

    def init_colors(self):
        self.colors = [
            (0, 0, 255),
            (0, 255, 0),
            (255, 0, 0),
            (0, 255, 255)
            ]

    def get_color(self, index):
        if self.colors is None:
            self.init_colors()
        return self.colors[index%len(self.colors)]
        

    def frame_generator(self):
        # Define a generator that yields frames from the video.
        while(1):
            ret, frame = self.cap.read()
            if ret is not True:
                break
            yield frame
        self.cap.release()

    def annotate_position(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.trajectory.append((self.counter, x, y))
            print(self.trajectory)

    def run_on_video_file(self, video_file):
        self.counter = 0
        self.trajectory = []
        self.cap = cv2.VideoCapture(video_file)
        # Open windos and connect the callback function
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow('image', 1200, 600)
        cv2.setMouseCallback("image", self.annotate_position)

        # Look at each frame in the video one at a time.
        for frame in self.frame_generator():
            self.frame = frame
            self.counter += 1

            cv2.imshow("image", self.frame)
            k = cv2.waitKey(1000) & 0xff
            if k == 27:
                break

    def run_on_ros_stream(self):        
        print("Launching ground truth node")
        self.init_ros_stream()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    def init_ros_stream(self):
        # object
        self.counter = 0
        self.trajectory = []
        # opencv
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow('image', 2400, 1200)
        cv2.setMouseCallback("image", self.annotate_position)
        # ros
        rospy.init_node('ground_truth')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("ground_truth_input", Image, self.ros_callback)
        pass

    def ros_callback(self, data):
        self.counter += 1
        cv_image = self.to_opencv_image(data)
        for index, coord in enumerate(self.trajectory):
            cv_image = cv2.circle(cv_image, (coord[1], coord[2]), 30, self.get_color(index), 2)
        cv2.imshow("image", cv_image)
        k = cv2.waitKey(1) & 0xff
        pass

    def to_opencv_image(self, ros_img):
        try:
            return self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        except CvBridgeError as ex:
            print(ex)


if __name__ == "__main__":
    instance = VideoAnnotator()
    # instance.run_on_video_file(sys.argv[1])
    instance.run_on_ros_stream()