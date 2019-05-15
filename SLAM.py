
from GridMap import GridMap
import tf
import numpy as np
import time
from laser_geometry import LaserProjection
import cv2
import sensor_msgs.point_cloud2 as pc2
import ctypes
lib = ctypes.cdll.LoadLibrary('./c_extern/map_update.so')

## main class
class SLAM():
    def __init__(self, gridmap, gui):
        self.gridmap = gridmap
        self.map_ptr = self.gridmap.map.ctypes.data_as(ctypes.c_char_p)
        self.prob_ptr = self.gridmap.prob.ctypes.data_as(ctypes.c_char_p)
        self.dimx = int(self.gridmap.size[1])

        radar_angle = 0.
        radar_x = 0.
        radar_y = 0.
        scan_base = tf.transformations.euler_matrix(0., 0., radar_angle)
        scan_base[0, 3] = radar_x
        scan_base[1, 3] = radar_y
        self.scan_base = scan_base
        self.gui = gui

    def forward(self, msg=None):
        scan, odom = self.data_preprocess(msg)
        try:
            last_odom = self.last_odom
            self.last_odom = odom
        except:
            self.last_odom = odom
            self.pose = self.mat2pose(odom)
            self.init_pose = self.pose
            return
        
        pose_mat = self.motion_estimate(last_odom, odom)
        map_idx, scan_world = self.scan2world(scan, pose_mat)
        delta_pose = self.scan_match(map_idx, scan_world)
        self.pose = self.pose + delta_pose

        print self.pose

        self.map_update(map_idx)

        if self.gui:
           self.gui.setdata(self.gridmap.prob, self.pose, map_idx, self.gridmap.resolution, self.gridmap.ori_point)



    ## estimate motion with odom data
    def motion_estimate(self, last_odom, odom):
        last_odom_inv = np.linalg.inv(last_odom)
        pose_mat = self.pose2mat(self.pose)
        pose_mat = np.matmul(pose_mat, np.matmul(last_odom_inv, odom))
        self.pose = self.mat2pose(pose_mat)
        return pose_mat

    ## correct pose with radar data
    def scan_match(self, map_idx, scan_world):
        pose = self.pose
        n = len(map_idx)
        sinRot = np.sin(pose[2])
        cosRot = np.cos(pose[2])
        H = np.zeros((3, 3))
        dTr = np.zeros((3, 1))
        for i in range(n):
            M_p, dx, dy = self.gridmap.interpMapValueWithDerivatives(map_idx[i, :])
            curPoint = scan_world[i,:] * self.gridmap.resolution
            funVal = 1. - M_p
            dTr[0] += dx * funVal
            dTr[1] += dy * funVal
            dphi = (-sinRot*curPoint[0] - cosRot*curPoint[1]) * dx + \
                (cosRot*curPoint[0] - sinRot*curPoint[1]) * dy
            dTr[2] += dphi * funVal
            H[0, 0] += dx**2
            H[0, 1] += dx * dy
            H[0, 2] += dx * dphi
            H[1, 1] += dy**2
            H[1, 2] += dy * dphi
            H[2, 2] += dphi**2
        
        H[1, 0] = H[0, 1]
        H[2, 0] = H[0, 2]
        H[2, 1] = H[1, 2]

        if H[0, 0] != 0. and H[1, 1] != 0. and H[2, 2] != 0.:
            delta_pose = np.matmul(np.linalg.inv(H), dTr)
            r = self.gridmap.resolution
            delta_pose[2, 0] = min(delta_pose[2, 0], 0.2)
            delta_pose[2, 0] = max(delta_pose[2, 0], -0.2)
            return np.array([delta_pose[0,0] / r, delta_pose[1,0] / r, delta_pose[2, 0]])
        else:
            return np.zeros(3)

    ## update map (c implement)
    def map_update(self, map_idx):
        # pose = self.pose
        # for i in map_idx:
        #     self.gridmap.update_line(pose[:2], i, -0.1)
        #     self.gridmap.update((i + 0.5).astype(np.int), 0.9)
        map_idx_ptr = map_idx.ctypes.data_as(ctypes.c_char_p)
        x = ctypes.c_double(self.pose[0])
        y = ctypes.c_double(self.pose[1])
        lib.map_update(self.map_ptr, self.prob_ptr, map_idx_ptr, len(map_idx), x, y, self.dimx)

    def pose2mat(self, pose):
        mat = tf.transformations.euler_matrix(0., 0., pose[2])
        mat[:2, 3] = (pose[:2] - self.gridmap.ori_point).T / self.gridmap.resolution
        return mat

    def mat2pose(self, odom):
        _, __, phi = tf.transformations.euler_from_matrix(odom[:3, :3])
        m_pts = self.gridmap.world2map(np.array([[odom[0, 3], odom[1, 3]]]))
        return np.array([m_pts[0, 0], m_pts[0, 1], phi])

    ## transform scan from scan_base to world_base with pose matrix
    def scan2world(self, scan, pose_mat):
        scan_full = np.insert(scan, 2, values=0., axis=1)
        scan_full = np.insert(scan_full, 3, values=1., axis=1)
        scan_world = np.matmul(scan_full, self.scan_base.T)
        world_idx = np.matmul(scan_world, pose_mat.T)[:, :2]
        map_idx = self.gridmap.world2map(world_idx)
        return map_idx, scan_world[:, :2]

    ## transform point from world_base to robot_base with pose matrix
    def world2robot(self, p, pose_mat):
        p = (p - self.gridmap.ori_point) / self.gridmap.resolution
        p = np.insert(p, 2, values=0., axis=0)
        p = np.insert(p, 3, values=1., axis=0)
        w_p = np.matmul(p, np.linalg.inv(pose_mat).T)[:2]
        return w_p

    def data_preprocess(self, msg):
        laser_projector = LaserProjection()
        cloud = laser_projector.projectLaser(msg.scan)
        pts = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
        scan = np.array(pts)[:, :2]

        qx = msg.odom.pose.pose.orientation.x
        qy = msg.odom.pose.pose.orientation.y
        qz = msg.odom.pose.pose.orientation.z
        qw = msg.odom.pose.pose.orientation.w

        odom = tf.transformations.quaternion_matrix((qx,qy,qz,qw))
        odom[0,3] = msg.odom.pose.pose.position.x
        odom[1,3] = msg.odom.pose.pose.position.y
        odom[2,3] = msg.odom.pose.pose.position.z

        return scan, odom
