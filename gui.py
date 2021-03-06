#!/usr/bin/python
# coding: UTF-8

from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import signal
import Queue
import sys
import time
import threading
import cv2
import pickle as pkl


class RobotItem(QtGui.QGraphicsItem):
    """a sample robot item"""
    def __init__(self, color):
        super(RobotItem, self).__init__()
        #self.setFlag(QtGui.QGraphicsItem.ItemIsMovable)
        self.setCacheMode(QtGui.QGraphicsItem.DeviceCoordinateCache)
        self.setZValue(1)
        self.color = color
        
    def boundingRect(self):
        adjust = 2.0
        return QtCore.QRectF(-10 - adjust, -10 - adjust, 20 + adjust,
                20 + adjust)

    def paint(self, painter, option, widget):
        #Draw a sample robot
        pen = QtGui.QPen()
        pen.setWidth(1)
        if self.color =='r':
            pen.setBrush(QtCore.Qt.red)
        elif self.color =='b':
            pen.setBrush(QtCore.Qt.blue)
        else:
            pen.setBrush(QtCore.Qt.green)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawEllipse(QtCore.QPointF(0.0, 0.0), 5, 5)
        painter.drawLine(0, 0, 5, 0)


class LSLAMGUI(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.q = Queue.Queue()
        self.state = 0

    def run(self):
        self.state = 1
        ## Always start by initializing Qt (only once per application)
        self.app = QtGui.QApplication([])

        ## Define a top-level widget to hold everything
        w = QtGui.QWidget()
        w.resize(800,500)
        w.setWindowTitle("LiDAR SLAM Viewer")

        ## Create some widgets to be placed inside
        #text = QtGui.QLineEdit('enter text')

        p2d = pg.GraphicsView()
        # p3d = gl.GLViewWidget()
        button_exit = QtGui.QPushButton('Exit')
        button_exit.setFixedWidth(110)
        button_exit.clicked.connect(self.exit)

        button_nav = QtGui.QPushButton('Navigate')
        button_nav.setFixedWidth(110)
        button_nav.clicked.connect(self.handleButton_nav)

        # button_back = QtGui.QPushButton('Back')
        # button_back.setFixedWidth(110)

        # button_back.clicked.connect(self.handleButton_back)
        # self.checkbox_gaussian = QtGui.QCheckBox("Use gaussian")
        # self.checkbox_gaussian.setChecked(False)
        #checkbox_gaussian.stateChanged.connect(self.handleCheckbox_gaussian)
        # p3d.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        #g = gl.GLGridItem()
        #p3d.addItem(g)

        ## Create a grid layout to manage the widgets size and position
        layout = QtGui.QGridLayout()
        w.setLayout(layout)

        ## Add widgets to the layout in their proper positions
        layout.addWidget(p2d, 0, 0, 1, 5)  
        # layout.addWidget(p3d, 1, 0, 1, 5) 
        #layout.addWidget(text, 2, 1) 
        layout.addWidget(button_exit, 2, 0)
        layout.addWidget(button_nav, 2, 1)
        # layout.addWidget(button_back, 2, 2)
        # layout.addWidget(self.checkbox_gaussian,2,3)
        #layout.addWidget(text, 2, 4)

        # Create a viewBox for 2D image
        vb = pg.ViewBox()
        vb.setAspectLocked()
        p2d.setCentralItem(vb)

        # self.prob_map = gl.GLSurfacePlotItem(z=np.zeros((80, 80)), shader='shaded', color=(0.5, 0.5, 1, 1))
        # self.prob_map.scale(0.5, 0.5, 1.0)
        # self.prob_map.translate(-20, -20, 0)
        # p3d.addItem(self.prob_map)


        #Create ImageItem for map
        self.img = pg.ImageItem(np.zeros((400,400)))
        vb.addItem(self.img)

        ## Display the widget as a new window
        w.show()

        ## Set image level
        self.img.setLevels([0, 1])

        #Create ScatterPlotItem for scan data 
        self.sct = pg.ScatterPlotItem(pen = pg.mkPen(None), 
                                      brush = pg.mkBrush("g"), 
                                      size =5, 
                                      antialias = False)
        self.sct.setParentItem(self.img)

        #Create RobotItem(custom) for showing robot pose 
        self.robot = RobotItem('b')
        self.robot.setParentItem(self.img)
        
        #Set timer
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(300)

        ## Start the Qt event loop
        self.app.exec_()

    def exit(self):
        # print 'exit'
        self.state = -1
        prob_map = self.prob
        res = {'map':self.prob,
                'init_pose':self.pose,
                'resolution':self.resolution,
                'original_point':self.ori_point
                }
        print 'start saving'
        with open('map.pkl', 'w') as f:
            pkl.dump(res, f)
        print 'done.'
        img = np.uint8(255 - prob_map*255)
        img = np.rot90(img)
        cv2.imwrite('map.pgm', img)

        try:
            self.app.exit(0)
            # sys.exit(0)
        except:
            print 'fail'

    def handleButton_nav(self):
        # try:
        #     self.app.exit(0)
        # except:
        #     pass
        self.state = 2

    def update(self):
        try:
            #Check is there any new data in queue
            prob, pose, newscan, resolution, ori_point = self.q.get(block=False)
            self.prob = prob
            self.pose = pose
            self.resolution = resolution
            self.ori_point = ori_point
            self.q.queue.clear()
            #remove previous laser scan data
            self.sct.clear()
            #update map
            # self.prob_map.setData(z=data)
            I = np.zeros(prob.shape)
            I.fill(1)
            self.img.setImage(I - prob)
            #update robot pose
            self.robot.setRotation(180.*pose[2]/np.pi)
            self.robot.setPos(pose[0],pose[1])
            #update laser scan
            #spots = [{'pos': pos} for pos in newscan]
            spots = [{'pos': newscan[i,:] } for i in range(newscan.shape[0])]
            self.sct.addPoints(spots)
        except Queue.Empty:
            pass

    def setdata(self, probdata, robotpose, newscan, resolution, ori_point):
        self.q.put((probdata, robotpose, newscan, resolution, ori_point))
        pass

if __name__ == "__main__":
    gui = LSLAMGUI()
    gui.start()
    print 'sample gui test'
    for i in range(1000):
        time.sleep(0.05)
        newscan = np.zeros((10,2))
        newscan.fill(0.1)
        gui.setdata(np.random.rand(80,80),np.random.rand(400,400), [0,0,i], newscan)

    
