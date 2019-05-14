#ifndef MAP_UPDATE_H
#define MAP_UPDATE_H

#define max(x, y) ((x)>=(y)?(x):(y))
#define min(x, y) ((x)<=(y)?(x):(y))

extern "C"
void map_update(double *map, double *prob, double *map_idx, int n,
             double px, double py, int dimx);

void update_point(double *map, double *prob, int x, int y, double add, int dimx);

void update_line(double *map, double *prob, double startx, double starty, 
            double endx, double endy, double add, int dimx);

#endif