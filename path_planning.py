import numpy as np
import pickle as pkl
import cv2
from matplotlib import pyplot as plt
import ctypes
from scipy.interpolate import spline
lib = ctypes.cdll.LoadLibrary('./c_extern/find_path.so')

class PathPlanner:
    def __init__(self, prob):
        self.map, self.ori_box, self.ori_size = self.load_map(prob)

    def load_map(self, prob):
        print 'reading map'

        self.ori_prob = prob
        
        idx = np.where(prob != 0.5)
        x_min = max(0, np.min(idx[0]) - 20)
        x_max = min(prob.shape[0], np.max(idx[0]) + 20) 
        y_min = max(0, np.min(idx[1]) - 20)
        y_max = min(prob.shape[1], np.max(idx[1]) + 20)
        size = prob.shape
        prob = prob[x_min:x_max, y_min:y_max]
        prob[np.where(prob > 0.5)] = 1
        prob_tmp = prob.copy()
        prob_tmp[np.where(prob_tmp <= 0.51)] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
        dilation = cv2.dilate(prob_tmp, kernel, iterations=1)
        dilation = np.maximum(dilation, prob)

        # plt.subplot(1, 2, 1)
        # plt.imshow(prob_tmp)
        # plt.subplot(1, 2, 2)
        # plt.imshow(dilation)
        # plt.show()

        print 'done.'
        return dilation, np.array([x_min, y_min, x_max, y_max]), size

    def random_points_generate(self, map, n_pts):
        r = np.random.rand(n_pts, 2)
        r[:, 0] *= map.shape[0]
        r[:, 1] *= map.shape[1]
        r = r.astype(np.int)
        keep = []
        for p in r:
            if map[p[0], p[1]] < 0.5:
                keep.append(p)
        return keep

    def smooth(self, pts):
        new_pts = []
        n_spline = 5
        # M = np.array([
        #     [-1, 3, -3, 1],
        #     [3, -6, 3, 0],
        #     [-3, 0, 3, 0],
        #     [1, 4, 1, 0]
        # ], dtype=np.float)
        # while idx + 3 < len(pts):
        #     P = pts[idx:idx+4, :]
        #     idx += 1
        #     M_P = np.matmul(M, P)
        #     for i in range(n_spline):
        #         t = 1. * i / n_spline
        #         T = 1. / 6. * np.array([t**3, t**2, t, 1], dtype=np.float)
        #         new_pts.append(np.matmul(T, M_P))
        

        idx = 1
        while idx + 1 < len(pts):
            m1 = (pts[idx-1] + pts[idx]) / 2.
            m2 = (pts[idx] + pts[idx+1]) / 2.
            k = np.sum(np.square(pts[idx-1] - pts[idx])) / np.sum(np.square(pts[idx] - pts[idx+1]))
            m = (m1 + k*m2) / (1+k)
            m1 += pts[idx] - m
            m2 += pts[idx] - m

            if idx == 1:
                for i in range(1, n_spline):
                    t = 1. * i / n_spline
                    p = ((1-t)**2)*pts[0] + 2*t*(1-t)*m1 + (t**2)*pts[1]
                    new_pts.append(p)
            else:
                for i in range(1, n_spline):
                    t = 1. * i / n_spline
                    p = ((1-t)**3)*pts[idx-1] + 3*t*((1-t)**2)*last_m2 + 3*(t**2)*(1-t)*m1 + (t**3)*pts[idx]
                    new_pts.append(p)

            if idx == len(pts) - 2:
                for i in range(1, n_spline + 1):
                    t = 1. * i / n_spline
                    p = ((1-t)**2)*pts[idx] + 2*t*(1-t)*last_m2 + (t**2)*pts[idx+1]
                    new_pts.append(p)

            last_m2 = m2
            idx += 1

        return np.array(new_pts)

    def path_planning(self, p_start, p_end):
        print p_start, p_end
        map = self.map
        ori_box = self.ori_box
        ori_size = self.ori_size
        p_start -= ori_box[:2]
        p_end -= ori_box[:2]
        r = self.random_points_generate(map, 5000)
        r.append(p_start)
        r.append(p_end)

        # r = np.array([
        #     [400, 400],
        #     [410, 420],
        #     [420, 400],
        #     [450, 420]
        # ])

        r = np.array(r, dtype=np.int32)


        res = np.zeros(len(r), dtype=np.int32)

        map_ptr = map.ctypes.data_as(ctypes.c_char_p)
        r_ptr = r.ctypes.data_as(ctypes.c_char_p)
        res_ptr = res.ctypes.data_as(ctypes.c_char_p)
        n = lib.find_path(map_ptr, r_ptr, len(r), len(r) - 2, len(r) - 1, res_ptr, map.shape[1])
        res = res[:n]
        path = r[res]



        path_smooth = self.smooth(path)
        path_smooth = path_smooth + ori_box[:2]
        print path_smooth

        path += ori_box[:2]


        self.ori_prob = 255 - 255 * self.ori_prob
        for i in range(len(path)):
            cv2.circle(self.ori_prob, (int(path[i][1]), int(path[i][0])), 3, [173])
            
        for i in range(len(path_smooth) - 1):
            cv2.line(self.ori_prob, (int(path_smooth[i][1]), int(path_smooth[i][0])), (int(path_smooth[i+1][1]), int(path_smooth[i+1][0])), [64])

        cv2.circle(self.ori_prob, (int(path[0][1]), int(path[0][0])), 3, [32])
        cv2.circle(self.ori_prob, (int(path[-1][1]), int(path[-1][0])), 3, [32])
        plt.imshow(np.rot90(self.ori_prob))
        plt.show()


        return path_smooth

if __name__ == "__main__":
    with open('map.pkl', 'r') as f:
        res = pkl.load(f)
    prob = res['map']
    path_planner = PathPlanner(prob)
    path = path_planner.path_planning([400, 400], [850, 300])
    # print path

    # print len(path)

    # plt.imshow(prob)
    # plt.show()
