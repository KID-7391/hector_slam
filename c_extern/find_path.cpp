#include <cstdio>
#include <algorithm>
#include <queue>
#include <cmath>
#include "find_path.hpp"

bool check_line(double *map, int startx, int starty, int endx, int endy, int dimx){
    int x1 = (int)(startx + 0.5), y1 = (int)(starty + 0.5);
    int x2 = (int)(endx + 0.5), y2 = (int)(endy + 0.5);
    double dx = endx - startx, dy = endy - starty;
    double k = -startx*endy + endx*starty;
    if(dx < 0.05 && dy < 0.05){
        if(map[x1*dimx + y1] >= 0.5)return false;
        return true;
    }
    if(abs(dx) > abs(dy)){
        int s = std::min(x1, x2), e = std::max(x1, x2);
        for(int x = s; x <= e; x++){
            int y = 1.*(x*dy + k) / dx + 0.5;
            if(map[x*dimx + y] >= 0.5)return false;
        }
    }else{
        int s = std::min(y1, y2), e = std::max(y1, y2);
        for(int y = s; y <= e; y++){
            int x = 1.*(y*dx - k) / dy + 0.5;
            if(map[x*dimx + y] >= 0.5)return false;
        }
    }
    return true;
}

inline double dist(double x1, double y1, double x2, double y2){
    return sqrt(sq(x1-x2) + sq(y1-y2));
}

void build_graph(double *map, int *idx, int n_pts, std::vector<Edge> &edges, std::vector<int> G[], int dimx){
    std::vector<std::pair<double, int> > tmp;

    // for(int i = 0; i < n_pts; i++){
    //     printf("%d %d\n", idx[2*i], idx[2*i+1]);
    // }

    for(int i = 0; i < n_pts; i++){
        tmp.clear();
        for(int j = 0; j < n_pts; j++){
            if(i == j)continue;
            if(check_line(map, idx[2*i], idx[2*i+1], idx[2*j], idx[2*j+1], dimx))
                tmp.push_back(std::pair<double, int>(dist(idx[2*i], idx[2*i+1], idx[2*j], idx[2*j+1]) ,j));
        }
        std::sort(tmp.begin(), tmp.end());

        for(int j = 0; j < MAX_EDGES && j < tmp.size(); j++){
            G[i].push_back(edges.size());
            G[tmp[j].second].push_back(edges.size());
            edges.push_back(Edge(i, tmp[j].second, tmp[j].first));
        }
    }
}

extern "C"
int find_path(double *map, int *idx, int n_pts, int s_idx, int e_idx, int *res, int dimx){
    std::vector<Edge> *edges = new std::vector<Edge>;
    std::vector<int> *G = new std::vector<int>[n_pts];
    int *f = new int [n_pts];
    double *dis = new double [n_pts];
    for(int i = 0; i < n_pts; i++)dis[i] = 1e9 + 5, f[i] = -1;
    build_graph(map, idx, n_pts, *edges, G, dimx);
    // for(auto e:(*edges)){
    //     printf("%d %d %f\n", e.u, e.v, e.w);
    // }


    std::queue<std::pair<int, double> > q;
    q.push(std::pair<int, double>(s_idx, 0.));
    dis[s_idx] = 0;
    while(!q.empty()){
        auto p = q.front();q.pop();
        if(p.second > dis[p.first] + 1e-6)continue;
        int u = p.first;
        for(auto i:G[u]){
            Edge &e = (*edges)[i];
            if (dis[u] + e.w < dis[e.v]){
                f[e.v] = u;
                dis[e.v] = dis[u] + e.w;
                q.push(std::pair<int, double>(e.v, dis[e.v] ));
            }
        }
    }
    // printf("%d %d\n\n", s_idx, e_idx);
    // for(int i = 0; i < n_pts; i++){
    //     printf("%d %f %d\n", i, dis[i], f[i]);
    // }
    int p = e_idx, cnt = 0;
    while(cnt < 100 && p != s_idx){
        res[cnt++] = p;
        p = f[p];
    }
    res[cnt++] = s_idx;
    for(int i = 0; i < cnt / 2; i++){
        std::swap(res[i], res[cnt - i - 1]);
    }
    return cnt;
}