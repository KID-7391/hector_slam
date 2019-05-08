import numpy as np
import time

class GridMap:
    def __init__(self, size=[5000, 5000], resolution=40.):
        self.size = np.array(size, dtype=np.int32)
        self.resolution = resolution
        self.ori_point = np.array([size[0]//2, size[1]//2])
        self.map = np.zeros(size)
        ## prob = exp(map) / (1 + exp(map))
        self.prob = np.ones(size) * 0.5
    
    def world2map(self, w_pts):
        return w_pts * self.resolution + self.ori_point

    def map2world(self, m_pts):
        return (m_pts - self.ori_point) / self.resolution

    ## return [M(p), dM(p)/dx, dM(p)/dy]
    def interpMapValueWithDerivatives(self, p):
        factx = p[0] - float(int(p[0]))
        facty = p[1] - float(int(p[1]))
        p0 = self.prob[int(p[0]), int(p[1])]
        p1 = self.prob[int(p[0]) + 1, int(p[1])]
        p2 = self.prob[int(p[0]), int(p[1]) + 1]
        p3 = self.prob[int(p[0]) + 1, int(p[1]) + 1]

        M_p = (p0*(1-factx) + p1*factx) * (1-facty)+ \
            (p2*(1-factx) + p3*factx) * facty
        dx = (p1 - p0) * (1-facty) + (p3 - p2) * facty
        dy = (p2 - p0) * (1-factx) + (p3 - p1) * factx

        return [M_p, dx, dy]

    # ## get all points in the line
    # def get_line(self, start, end):
    #     x1 = int(start[0] + 0.5)
    #     y1 = int(start[1] + 0.5)
    #     x2 = int(end[0] + 0.5)
    #     y2 = int(end[1] + 0.5)
    #     dx = x2 - x1
    #     dy = y2 - y1
    #     k = -x1*y2 + x2*y1

    #     if dx == 0 and dy == 0:
    #         return np.array([[x1, y1]])

    #     pts = []
    #     if abs(dx) > abs(dy):
    #         if dx < 0:
    #             x1, y1, x2, y2 = x2, y2, x1, y1
    #             dx, dy = -dx, -dy
    #             k = -k
    #         for x in range(x1, x2 + 1):
    #             pts.append([x, int((x*dy + k)/dx + 0.5)])
    #     else: 
    #         if dy < 0:
    #             x1, y1, x2, y2 = x2, y2, x1, y1
    #             dx, dy = -dx, -dy
    #             k = -k
    #         for y in range(y1, y2 + 1):
    #             pts.append([int((y*dx - k)/dy + 0.5), y])
        
    #     return np.array(pts)

    # ## update all points in the line
    # def update_line(self, start, end, add):
    #     t0 = time.time()
    #     pts = self.get_line(start, end)
    #     t1 = time.time()
    #     print('get_line', t1 - t0)

    #     t0 = time.time()
    #     for p in pts:
    #         self.update(p, add)
    #     t1 = time.time()
    #     print('update', t1 - t0)

    # ## update a single point
    # def update(self, p, add):
    #     val = self.map[p[0], p[1]] + add
    #     ## avoid large float after exp
    #     if val > 50 or val < -50:
    #         return
    #     self.map[p[0], p[1]] = val
    #     exp_val = np.exp(val)
    #     self.prob[p[0], p[1]] = exp_val / (1 + exp_val)
        
