from gui import *
from GridMap import *
from SLAM import *
from controller import Controller
import time
import cv2
import rospy
from data_wrapper.msg import scan_odom
import sys

# class TESTER:
#     def __init__(self):
#         self.slam = SLAM(GridMap(), LSLAMGUI())
#         rospy.init_node('test_slam', anonymous=True)

#     def test(self):
#         rospy.Subscriber('scan_odom', scan_odom, self.Call)
#         try:
#             rospy.spin()
#         except KeyboardInterrupt:
#             self.slam.gui.exit()
#             sys.exit(0)

#     def Call(self, msg):
#         if self.slam.gui.state == -1:
#             sys.exit(0)
#         elif self.slam.gui.state == 0:
#             self.slam.gui.start()
#         t0 = time.time()
#         self.slam.forward(msg)
#         t1 = time.time()
#         print (t1 - t0) * 1000

def test():
    ctrler = Controller()
    ctrler.main_control()

if __name__ == "__main__":
    # test()
    from math import cos, sin, pi
    p = np.array([259, 396, 0, 1])
    p[:2] = (p[:2] - np.array([500, 500])) / 40.
    # p = np.array([-1, 1, 0, 1])
    M = np.array([
        [cos(30*pi/180), -sin(30*pi/180), 0, 1],
        [sin(30*pi/180), cos(30*pi/180), 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    print np.matmul(p, np.linalg.inv(M).T)
    # tester = TESTER()
    # tester.test()
