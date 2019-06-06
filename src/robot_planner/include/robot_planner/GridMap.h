#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <stdlib.h>
#include <math.h>
#include <assert.h>

typedef bool ** OccupancyGrid_t;
typedef bool OccupancyGrid;

namespace wheely_planner{
struct Index{
    Index(u_int _i,u_int _j)
    :i(_i),j(_j)
    {
    }
    u_int i,j;
};
class GridMap{
    public:
        GridMap(u_int32_t, float);
        ~GridMap();
        double ** getAdjacentCells(double x, double y);
        //Sets the occupancy status of the cell the point (x,y) is a part of 
        void setCellStatus(double x, double y,bool occupied);
    private:
        /*Computes the index into the grid map, i.e determines the grid in which (x,y) belongs to.
          the underlying (grid) map  is represented as follows, by using the grid's cells centers, and a bool.
          cell with center (0,0) = index i=0,j=0  
          for all points (x,y) map (x,y) to (Xc,Yc) the center of the grid they belong, then
          map negative Xc's (&Yc's) to even indexes, map positive Xc's (&Yc's) to odd indexes
        */
        Index* computeCellIndex(double x, double y);
        bool getCellStatus(double x, double y);
        
        u_int32_t mapSize;
        float cellSize; 
        OccupancyGrid_t grid;          /*is a multi-dimensional array of bools, each cell is represented by a center.*/
};
}
#endif