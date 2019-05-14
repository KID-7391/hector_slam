#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "map_update.hpp"

extern "C"
void map_update(double *map, double *prob, double *map_idx, int n,
             double px, double py, int dimx){
    for(int i = 0; i < n; i++){
        update_line(map, prob, px, py, map_idx[2*i], map_idx[2*i+1], -0.1, dimx);
        update_point(map, prob, (int)(map_idx[2*i] + 0.5), 
                (int)(map_idx[2*i+1] + 0.5), 0.9, dimx);
    }
}

void update_point(double *map, double *prob, int x, int y, double add, int dimx){
    int idx = x * dimx + y;
    double val = map[idx] + add;
    if(val > 50 || val < -50)return;
    map[idx] = val;
    double exp_val = exp(val);
    prob[idx] = exp_val / (1. + exp_val);
}

void update_line(double *map, double *prob, double startx, double starty, 
            double endx, double endy, double add, int dimx){
    int x1 = (int)(startx + 0.5), y1 = (int)(starty + 0.5);
    int x2 = (int)(endx + 0.5), y2 = (int)(endy + 0.5);
    double dx = endx - startx, dy = endy - starty;
    double k = -startx*endy + endx*starty;
    if(abs(dx) < 0.05 && abs(dy) < 0.05){
        update_point(map, prob, x1, y1, add, dimx);
        return ;
    }
    if(abs(dx) > abs(dy)){
        int s = min(x1, x2), e = max(x1, x2);
        for(int x = s; x <= e; x++){
            int y = 1.*(x*dy + k) / dx + 0.5;
            update_point(map, prob, x, y, add, dimx);
        }
    }else{
        int s = min(y1, y2), e = max(y1, y2);
        for(int y = s; y <= e; y++){
            int x = 1.*(y*dx - k) / dy + 0.5;
            update_point(map, prob, x, y, add, dimx);
        }
    }

}
