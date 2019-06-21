#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <functional>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include <fcntl.h>
#include <queue>
#include <limits.h>
#include <visualization_msgs/Marker.h>
#include <thread>
#include <unordered_map>
#include <boost/functional/hash/hash.hpp>

namespace wheely_planner{
#define EPSILON 0.001
#define THIS_ROS_PACKAGE_NAME "robot_planner"
#define SCALE_MAP_Z(z) new double[3] {-1,-1,z}
#define COLOR_DARK_BLUE new double[3]{0.1,0.1,0.25}
#define COLOR_LIGHT_BLUE new double[3]{0.85,0.85,1}
#define COLOR_DARK_RED new double[3]{0.25,0.1,0.1}

typedef struct Coordinate * Coordinate_t;
typedef bool ** OccupancyGrid_t;
typedef char *  MapFile;
typedef bool OccupancyGrid;

struct Index{
    Index(u_int _i,u_int _j):i(_i),j(_j),cx(0.0),cy(0.0)
    {
    }
    Index(u_int _i,u_int _j,double _cx,double _cy)
    :i(_i),j(_j),cx(_cx),cy(_cy)
    {
    }
    u_int i,j;      //Index to the grid map
    double cx,cy; //Centers

    bool operator==(Index const &other)const{
        auto in_x = fabs(cx - other.cx) <=EPSILON;
        auto in_y = fabs(cy - other.cy) <=EPSILON;
        return in_x && in_y;   
    }
};
struct Coordinate
{
    Coordinate(){}
    Coordinate(double _x,double _y):x(_x),y(_y){}
    double x;
    double y;
    bool operator==(Coordinate const &other)const
    {
        auto in_x = fabs(x - other.x) <=EPSILON;
        auto in_y = fabs(y - other.y) <=EPSILON;
        return in_x && in_y;
    } 
};
struct Cell{
    Cell(){};
    Cell(double x, double y):center(x,y){};
    Coordinate center;  //the cells center
    Cell *parent=nullptr;
    double f=std::numeric_limits<double>::infinity()/*Total Cost */,g=std::numeric_limits<double>::infinity()/* Path cost*/,h/*heuristic*/;
    bool operator==(Cell const &other)const
    {
        auto in_x = fabs(center.x - other.center.x) <=EPSILON;
        auto in_y = fabs(center.y - other.center.y) <=EPSILON;
        return in_x && in_y;
    }  
};
struct CellPriorityComp{
    bool operator()(Cell const &c1, Cell const &c2 ){
        //First compare by the total fun, but break ties by heuristic
        return c1.f > c2.f ||
        ( fabs(c1.f - c2.f ) < EPSILON && c1.h > c2.h);
    }
};

struct CellKeyHasher
{
    std::size_t operator()(const Cell& c)const
    {
        using boost::hash_value;
        using boost::hash_combine;
        int trunced_x = (int) trunc(c.center.x * 1000);
        int trunced_y = (int) trunc(c.center.y * 1000);
        std::size_t seed = (trunced_x) ^  (trunced_y);
        hash_combine(seed,hash_value(trunced_x));
        hash_combine(seed,hash_value(trunced_y));
        return seed;
    }
};
struct IndexKeyHasher
{
    std::size_t operator()(const Index& i)const
    {
        using boost::hash_value;
        using boost::hash_combine;
        int trunced_x = (int) trunc(i.cx * 1000);
        int trunced_y = (int) trunc(i.cy * 1000);
        std::size_t seed = (trunced_x) ^  (trunced_y);
        hash_combine(seed,hash_value(trunced_x));
        hash_combine(seed,hash_value(trunced_y));
        return seed;
    }
};
class GridMap{
    public:
        GridMap(size_t, float,double,ros::Publisher*);
        GridMap(MapFile map,double robo_radius,ros::Publisher*);
        GridMap(GridMap *);
        bool allocateGrid();
        ~GridMap();
        /*Computes the index into the grid map, i.e determines the grid in which (x,y) belongs to.
          the underlying (grid) map  is represented as follows, by using the grid's cells centers, and a bool.
          cell with center (0,0) = index i=0,j=0  
          for all points (x,y) map (x,y) to (Xc,Yc) the center of the grid they belong, then
          map negative Xc's (&Yc's) to even indexes, map positive Xc's (&Yc's) to odd indexes
        */
        Index* computeCellIndex(double x, double y);
        //Enlarges individual cells by robot.radius * k
        void computeCellCenter(Index* i);
        //Get the unoccupied adjacentCells of the cell the point (x,y) is part of
        std::vector<Cell> getAdjacentCells(Coordinate_t c);
        std::vector<Cell> getAdjacentCells_8(Coordinate_t c,bool includeOccupied=false);
        std::vector<Cell> getAdjacentCells_24(Coordinate_t c);
        std::queue<Coordinate> getNearestUnoccupied(Coordinate_t c);
        //Sets the occupancy status of the cell the point (x,y) is a part of 
        void setCellStatus(double x, double y,bool occupied=true);
        void saveMap(const std_msgs::StringConstPtr);
        //For Visualizing the map with Rviz
        void storePtForVis(Coordinate_t c , visualization_msgs::Marker* marker);
        void visualizeMap(double * rgb = new double[3]{0,1,0}, int id =0);
        void visualizeMapData(visualization_msgs::Marker* marker,int id,double *rgb, double * scale=NULL);
        void EnlargeObstacles(GridMap* dest);
        bool getCellStatus(Coordinate_t);
    private:
        void EnlargeObstacles();  
        void EnlargeObstacles(Coordinate,GridMap* dest);
        
        size_t mapSize;
        double cellSize,robot_radius; 
        OccupancyGrid_t grid = nullptr;          /*is a multi-dimensional array of bools, each cell is represented by a center.*/

        ros::Publisher* pub;
        visualization_msgs::Marker cells;
};

}
#endif