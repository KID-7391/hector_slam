from gui import *
from GridMap import *
from SLAM import *
from data_wrapper.msg import scan_odom
import numpy as np
from math import sqrt, atan2, cos, sin, pi
import rospy
from geometry_msgs.msg import Twist
import time

max_speed = 0.3
max_turn = 0.2

class Controller:
    def __init__(self, k_rho=0.5, k_alpha=0.8, k_beta=-0.15, threshold=1.):
        assert k_rho >= 0 and k_beta <=0 and k_alpha >= k_rho, 'Bad parameters.'
        rospy.init_node('test_control', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        self.threshold = threshold
        self.is_navigating = False
        # self.K = np.array([
        #     [-k_rho, 0, 0],
        #     [0, -(k_alpha-k_rho), k_beta],
        #     [0, -k_rho, 0]
        # ])
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta

        # self.kp = [0.1, 0.1, 0.1]
        # self.ki = [0.1, 1, 0.1]


    def get_path(self, path):
        self.path = []
        for i in range(len(path)):
            if i == len(path) - 1:
                phi = self.path[i-1][2]
            else:
                phi = atan2(path[i+1, 1] - path[i, 1], path[i+1, 0] - path[i, 0])
            self.path.append([path[i, 0], path[i, 1], phi])
        

        self.path = np.array(self.path)

        np.savetxt('path.txt', self.path)
        
        return self.path

    def get_input(self, cur_pose_in_goal):
        ## stupid pi controller, which works like shit!!!
        # x, y, theta = e

        # try:
        #     last_v = self.last_v
        #     last_w = self.last_w
        #     last_e = self.last_e
        # except:
        #     self.last_v = 0
        #     self.last_w = 0
        #     self.last_e = e
        #     return np.array([0., 0.])
        
        # v = 0.*last_v + self.kp[0]*(e[0] - last_e[0]) + self.ki[0]*e[0]
        # w = 0.*last_w + self.kp[1]*(e[1] - last_e[1]) + self.ki[1]*e[1] + \
        #             self.kp[2]*(e[2] - last_e[2]) + self.ki[2]*e[2]

        # print e
        
        # return np.array([v, w])

        x, y = -cur_pose_in_goal[:2]
        theta = cur_pose_in_goal[2]

        # print x, y, theta

        if theta > pi:
            theta -= 2*pi
        elif theta < -pi:
            theta += 2*pi

        rho = sqrt(x**2 + y**2)
        alpha = -theta + atan2(y, x)

        if abs(y/x) < 1e-3:
            alpha = -theta

        if abs(y/x) > 1e3:
            if y >= 0:
                alpha = -theta + pi / 2.
            else:
                alpha = -theta - pi / 2.


        if alpha >= pi:
            alpha -= 2*pi
        elif alpha < -pi:
            alpha += 2*pi

        beta = -theta - alpha

        print theta, beta, alpha
        
        v = self.k_rho*rho
        w = self.k_alpha*alpha + self.k_beta*beta

        if rho < 0.1:
            w = rho / 0.1 * self.k_alpha*alpha + self.k_beta*beta


        return np.array([self.k_rho*rho, self.k_alpha*alpha + self.k_beta*beta])


    def forward(self):
        k = self.k
        pk = self.path[k]
        pose = self.slam.pose
        pose_mat = self.slam.pose2mat(pk)
        cur_pose_in_goal = self.slam.world2robot(pose[:2], pose_mat)
        


        theta = 1.*pose[2] - pk[2]
        if theta >= pi:
            theta = -2*pi + theta
        elif theta < -pi:
            theta = 2*pi + theta
        cur_pose_in_goal = np.insert(cur_pose_in_goal, 2, values=theta, axis=0)
        # print '*'*20
        # print pose, pk
        # print 'pose:', cur_pose_in_goal
        # print '*'*20

        # T_mat = np.array([
        #     [cos(pose[2]), -sin(pose[2]), 0],
        #     [sin(pose[2]), cos(pose[2]), 0],
        #     [0, 0, 1]
        # ])
        # e_pose = np.matmul(pk - pose, T_mat)

        v, w = self.get_input(e_pcur_pose_in_goal)
        self.publish(v, w)
    
    def publish(self, v, w):
        # if abs(w) > 0.1:
        #     v = 0
        # v = min(v, max_speed)
        # v = max(v, -max_speed)
        # w = min(w, max_turn)
        # w = max(w, -max_turn)

        k = 1.
        if abs(v) > max_speed:
            k = max_speed / abs(v)
        if abs(w) > max_turn:
            k = min(k, max_turn / abs(w))

        v *= k
        w *= k

        print 'v =',v , ', w =', w

        twist = Twist()
        twist.linear.x = v; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
        self.pub.publish(twist)

    def main_control(self):
        self.slam = SLAM(GridMap(), LSLAMGUI())
        rospy.Subscriber('scan_odom', scan_odom, self.Call, queue_size=1, buff_size=52428800)
        rospy.spin()

    def Call(self, msg):
        if self.slam.gui and self.slam.gui.state == -1:
            return 
        elif self.slam.gui and self.slam.gui.state == 0:
            self.slam.gui.start()
        elif self.slam.gui and self.slam.gui.state == 1:
            pass
        else:
            pose = self.slam.pose
            if self.is_navigating is False:
                self.is_navigating = True
                self.k = 0
                p_end = self.slam.init_pose[:2]
                p_start = self.slam.pose[:2]
                path = self.slam.gridmap.path_planning_map(p_start.copy(), p_end.copy())
                self.get_path(path)
            else:
                while self.k < len(self.path) and np.sqrt(np.sum(np.square(pose - self.path[self.k]))) < self.threshold:
                    self.k += 1
                # print 'k = ', self.k
                if self.k < len(self.path):
                    self.forward()

        self.slam.forward(msg)
