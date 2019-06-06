#include <robot_planner/GridMap.h>

namespace wheely_planner{

GridMap::GridMap(u_int32_t _mapSize, float _cellSize) :
mapSize(_mapSize) , cellSize(_cellSize){
    //Create a grid of mapSize * mapSize
    grid = (OccupancyGrid_t) malloc(mapSize * sizeof(OccupancyGrid));
    for (size_t i = 0; i < mapSize; i++)
    {
        grid[i] = (OccupancyGrid * ) calloc(mapSize, sizeof(OccupancyGrid));
    }
}

GridMap::~GridMap(){
    for (size_t i = 0; i < mapSize; i++)
    {
        free(grid[i]);
    }
    free(grid);
}   
double ** GridMap::getAdjacentCells(double x, double y){
    //compute the center of the cell
    u_int64_t kx = round(x/cellSize);       
    u_int64_t ky = round(y/cellSize);
    double cx  = kx * cellSize, cy = ky * cellSize;
    //Adjacent cells are (cx+-1,cy+-1)
    double ** adjacents = new double*[4];
    adjacents[0] = new double[2]{cx+cellSize,cy};
    adjacents[1] = new double[2]{cx-cellSize,cy};
    adjacents[2] = new double[2]{cx,cy+cellSize};
    adjacents[3] = new double[2]{cx,cy-cellSize};
    return adjacents;
}  
Index* GridMap::computeCellIndex(double x, double y){
    //Calculate the distance of the cell's center from origin
    u_int64_t kx = round(x/cellSize);       
    u_int64_t ky = round(y/cellSize);
    assert( kx >= mapSize && ky >= mapSize);
    //map to the grid, Center of cell would be located at (kx * cellSize, ky*cellSize)
    u_int i = kx <= 0 ? 2 * fabs(kx) : 2 * kx -1;   //Map negative to even index, positive to odd index
    u_int j = ky <= 0 ? 2 * fabs(ky) : 2 * ky -1;   //Map negative to even index, positive to odd index
    Index * index = new Index(i,j);
    return index;
}

void GridMap::setCellStatus(double x, double y,bool occupied){
    Index * index = computeCellIndex(x,y);
    grid[index->i][index->j]  = occupied;
    delete index;
}
bool GridMap::getCellStatus(double x, double y){
    Index * index = computeCellIndex(x,y);
    return grid[index->i][index->j];
}
}