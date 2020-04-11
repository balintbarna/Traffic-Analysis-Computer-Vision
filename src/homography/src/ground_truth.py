#!/usr/bin/env python
import cv2
import sys

class VideoAnnotator:
    def __init__(self, video_file):
        self.cap = cv2.VideoCapture(video_file)
        self.counter = 0
        self.trajectory = []

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

    def main(self):
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



if __name__ == "__main__":
    instance = VideoAnnotator(sys.argv[1])
    instance.main()