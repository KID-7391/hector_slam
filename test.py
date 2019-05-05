from readbag import *
from gui import *
from GridMap import *
from SLAM import *
import time
import cv2

##########################
bagfile = 'h1.bag'
scan_topic = 'scan' 
odom_topic = 'odom'
start_time = 0
end_time = 800
##########################
image_file_name = 'map1.pgm'


class TESTER:
    def __init__(self):
        bagreader = BagReader(bagfile, scan_topic, odom_topic, start_time, end_time)
        self.slam = SLAM(bagreader.data, GridMap(), LSLAMGUI())

    def test(self):
        slam = self.slam
        slam.gui.start()
        while slam.idx < slam.T:
            t0 = time.time()
            # time.sleep(0.03)
            if slam.gui.state == 1:
                slam.forward()
                slam.idx += 1
            elif slam.gui.state == 2:
                slam.gui.state = 0
                slam.forward()
                slam.idx += 1
            elif slam.gui.state == 3:
                slam.gui.state = 0
                slam.forward()
                slam.idx -= 1
            t1 = time.time()
            print (t1 - t0) * 1000

        self.save()

    def save(self):
        prob_map = self.slam.gridmap.prob
        img = np.uint8(255 - prob_map*255)
        img = np.rot90(img)
        cv2.imwrite('map.pgm', img)

if __name__ == "__main__":
    tester = TESTER()
    tester.test()
    