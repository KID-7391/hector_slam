from gui import *
from GridMap import *
from SLAM import *
import time
import cv2
import rospy
from data_wrapper.msg import scan_odom
import sys

class TESTER:
    def __init__(self):
        self.slam = SLAM(GridMap(), LSLAMGUI())
        rospy.init_node('test_slam', anonymous=True)

    def test(self):
        rospy.Subscriber('scan_odom', scan_odom, self.Call)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.slam.gui.exit()
            sys.exit(0)

    def Call(self, msg):
        if self.slam.gui.state == -1:
            sys.exit(0)
        elif self.slam.gui.state == 0:
            self.slam.gui.start()
        t0 = time.time()
        self.slam.forward(msg)
        t1 = time.time()
        print (t1 - t0) * 1000


if __name__ == "__main__":
    tester = TESTER()
    tester.test()
    