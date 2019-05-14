#ifndef FIND_PATH_HPP
#define FIND_PATH_HPP
#define MAX_EDGES 10
#define sq(x) ((x)*(x))

#include <vector>

struct Edge{
    int u, v;
    double w;
    Edge(int _u, int _v, int _w):u(_u), v(_v), w(_w){}
    bool operator < (const Edge &e){
        return w < e.w;
    }
};


bool check_line(double *map, int startx, int starty, int endx, int endy, int dimx);
inline double dist(double x1, double y1, double x2, double y2);
void build_graph(double *map, int *idx, int n_pts, std::vector<Edge> &edges, std::vector<int> G[], int dimx);

extern "C"
int find_path(double *map, int *idx, int n_pts, int s_idx, int e_idx, int *res, int dimx);

#endif