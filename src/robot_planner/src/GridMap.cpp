#include <robot_planner/GridMap.h>

namespace wheely_planner{

//Needed to construct the map for the first time, new Environment
GridMap::GridMap(size_t _mapSize, float _cellSize) :
mapSize(_mapSize) , cellSize(_cellSize){
    //Create a grid of mapSize * mapSize
    allocateGrid();
}
//Construct a GridMap from an already constructed map
GridMap::GridMap(MapFile map,double robo_radius){
    std::string path = ros::package::getPath(THIS_ROS_PACKAGE_NAME);
    path.append("/map/");
    path.append(map);
    ROS_INFO("Opening Map File : %s", path.c_str());
    int fd = open(path.c_str(), O_RDONLY);
    if ( fd < 0 ){
        ROS_ERROR("Couldn't open Map,Opening Failed");
        return;
    }
    long int len = lseek(fd,0,SEEK_END);
    lseek(fd,0,SEEK_SET);
    if ( len < 0 ){ ROS_ERROR("Couldn't read Map,Opening Failed"); return;}
    char * mapData = (char * ) malloc( len );
    if (read(fd, mapData, len) < 0){
        ROS_ERROR("Couldn't read map Data.");
        return;
    }
    ROS_INFO("Read Map Data");

    //Extract header info
    if ( sscanf(mapData, "%zu\n%lf\n",&mapSize, &cellSize) == EOF) {ROS_INFO("Map File Bad Format.");return;}
    auto i =0;
    auto count =0 ;
    auto found = false;
    ROS_INFO("Extracted mapsize %zu, and cellSize %lf",mapSize, cellSize);

    while( count < len ){
        auto c = *mapData;
        if ( c == '\n'){
            i++;
        }
        if ( i == 2){
            mapData = mapData+1;
            found = true;
            break;
        }
        mapData++;
    }
    if (!found  ){
        ROS_ERROR("Map File Bad Format ");
        return;
    }
    if (!allocateGrid()){
        ROS_ERROR("Couldn't allocate Grid");
        return;
    }
    for (size_t i = 0; i < mapSize; i++)
    {
        memcpy(grid[i],mapData+(i*mapSize), mapSize);
    }
    ROS_INFO("Loaded Map %s", map);
    ROS_INFO("Enlargin Obstacles");
    EnlargeObstacles(robo_radius);
}

bool GridMap::allocateGrid(){
    grid = (OccupancyGrid_t) malloc(mapSize * sizeof(OccupancyGrid *));
    if ( !grid ){ROS_ERROR("Grid Aloccation failed"); return false;}
    for (size_t i = 0; i < mapSize; i++)
    {
        grid[i] = (OccupancyGrid * ) calloc(mapSize, sizeof(OccupancyGrid));
        if ( !grid[i]){ROS_ERROR("Grid Aloccation failed"); return false;}
    }
    return true;
}

GridMap::~GridMap(){
    ROS_INFO("In destructor");
    if(!grid) return;
    for (size_t i = 0; i < mapSize; i++)
    {
        free(grid[i]);
    }
    free(grid);
}   
std::vector<Cell> GridMap::getAdjacentCells(Coordinate_t c){
    //compute the center of the cell
    Index * index = computeCellIndex(c->x,c->y);
    std::vector<Cell> adjacents;
    if( !index ) return adjacents;
    double cx  = index->cx, cy = index->cy;
    delete index;
    double kx ,ky;
    double k_s[3] {cellSize,-cellSize,0.0};
    //Adjacent cells are (cx+-1,cy+-1)
    //Build an 8-connectivity node adjacency
    for (size_t i = 0; i < 4; i++)
    {
        kx = i<2 ? k_s[i%2] : 0.0;
        ky = i<2 ? 0.0 : k_s[i%2];
        Cell c(cx + kx, cy+ ky);
        if ( !getCellStatus(&c.center))
            adjacents.push_back(c);
    }
    return adjacents;
}  
std::vector<Cell> GridMap::getAdjacentCells_8(Coordinate_t c){
    Index * index = computeCellIndex(c->x,c->y);
    std::vector<Cell> adjacents;
    if( !index ) return adjacents;
    double cx  = index->cx, cy = index->cy;
    delete index;
    double kx = -cellSize,ky;
    double ky_s[3] {cellSize,-cellSize,0.0};
    //Adjacent cells are (cx+-1,cy+-1)
    //Build an 8-connectivity node adjacency
    for (size_t i = 0; i < 8; i++)
    {
        if( i % 3 == 0 ) kx *= -1;
        if( i == 6) kx =0;
        ky = ky_s[i%3];
        Cell c(cx + kx, cy+ ky);
        if ( !getCellStatus(&c.center))
            adjacents.push_back(c);
    }
    
    return adjacents;
}
Index* GridMap::computeCellIndex(double x, double y){
    //Calculate the distance of the cell's center from origin
    long int  kx = (long int) round(x/cellSize);       
    long int  ky = (long int) round(y/cellSize);
    // ROS_INFO("kX %ld KY %ld | x %f , y %f",kx,ky,x,y);
    //map to the grid, Center of cell would be located at (kx * cellSize, ky*cellSize)
    u_int i = kx <= 0 ? 2 * fabs(kx) : 2 * kx -1;   //Map negative to even index, positive to odd index
    u_int j = ky <= 0 ? 2 * fabs(ky) : 2 * ky -1;   //Map negative to even index, positive to odd index
    if( i >= mapSize && j >= mapSize){
        //Point doesn't exist on map
        return NULL;
    }
    double cx  = kx * cellSize, cy = ky * cellSize;
    Index * index = new Index(i,j,cx,cy);
    return index;
}
void GridMap::computeCellCenter(Index* i){
    double kx = i->i%2 == 0? -(double)i->i/2.0 : ((double)i->i + 1)/2.0;
    double ky = i->j%2 == 0? -(double)i->j/2.0 : ((double)i->j + 1)/2.0;
    i->cx = kx * cellSize;
    i->cy = ky * cellSize;
}
void GridMap::setCellStatus(double x, double y,bool occupied){
    Index * index = computeCellIndex(x,y);
    if(!grid || !index) return;
    grid[index->i][index->j]  = occupied;
    delete index;
}

bool GridMap::getCellStatus(Coordinate_t c){
    Index * index = computeCellIndex(c->x,c->y);
    if ( !index ) return true;  //Every point not on registered map is an obstacle
    auto i = index->i , j = index->j;
    delete index;
    if(!grid) return true; //No map so its safer to assume point is an obstacle.
    return grid[i][j];
}
void GridMap::EnlargeObstacles(double roboRadius){
    //Extract those deteted by sensor
    ROS_INFO("Enlarging Obstacles.");
    if ( !grid ){
        ROS_ERROR("Erorr! No map file Found.");
        return;
    }
    std::vector<Index*> occupiedGrids;
    Index * index ;
    for (size_t i = 0; i < mapSize; i++)
    {
        for (size_t j = 0; j < mapSize; j++)
        {
            if( grid[i][j]){
                //remember it 
                index = new Index(i,j);
                computeCellCenter(index); 
                occupiedGrids.push_back(index);
            }
        }
    }
    //Enlarge the original occupied cells by the robot's radiuus
    for (auto index : occupiedGrids)
    {
        auto c = Coordinate(index->cx,index->cy);
        EnlargeObstacles(&c, roboRadius);
        delete index;
    }
}
//Enlarge the obstacle located at the cell where c belongs
void GridMap::EnlargeObstacles(Coordinate_t c,double robot_radius){
    //Expand X by using breadth first search
    size_t ratio = std::ceil(robot_radius / cellSize);
    // ROS_INFO("CELL RATIO IS %ld,",ratio);
    std::queue<Coordinate_t> queue;
    queue.push(c);
    for (size_t i = 0; i <= ratio; i++)
    {
        if( queue.empty() ) break;
        c = queue.front();
        queue.pop();
        for (auto neighbors : getAdjacentCells_8(c)) //getAdjacentCells returns unoccupied adjacent cells
        {
            auto i = computeCellIndex(neighbors.center.x, neighbors.center.y);
            if( !i ) continue;
            if( !grid[i->i][i->j]) grid[i->i][i->j] = true;
            delete i;
            // ROS_INFO("Enlarging (%f,%f), Neighbors (%f,%f)",c->x,c->y,neighbors.center.x,neighbors.center.y);
            if( ratio != 1 ) queue.push(&neighbors.center);
        }
    }
}
void GridMap::storePtForVis(Coordinate_t c , visualization_msgs::Marker* marker){
        geometry_msgs::Point p;
        p.x = c->x;
        p.y = c->y;
        marker->points.push_back(p);
}
void GridMap::saveMap(const std_msgs::StringConstPtr str){
    if(!grid) return;
    std::string path = ros::package::getPath(THIS_ROS_PACKAGE_NAME);
    path.append("/map/");
    path.append(str->data.c_str());
    // char * name = (char * ) malloc( strlen(path) + strlen())
    int fd = open(path.c_str(), O_CREAT | O_RDWR |O_TRUNC, S_IWUSR | S_IRUSR | S_IROTH | S_IWOTH);
    if( fd < 0 ){
        ROS_ERROR("Couldn't save Map,Opening Failed");
        return;
    }
    //Save the mapSize,and cell size
    char header[16] = {'\0'};
    sprintf(header , "%zu\n%lf\n",mapSize, cellSize);
    write(fd, header, strlen(header));
    //Dump binary GridMap
    for (size_t i = 0; i < mapSize; i++)
    {
        if ( write(fd, grid[i],mapSize) < 0){
            ROS_ERROR("Couldn't save Map, Writing grid Failed.");
            return;
        }
    }
    ROS_INFO("Saved Map to file %s", str->data.c_str());
    close(fd);
    fflush(NULL);
}
void GridMap::visualizeMap(ros::Publisher *pub){
    if( !grid ){ 
        ROS_ERROR("No map found for visualization.");
        return;
    }
    for (int i = 0; i < mapSize; i++)
    {
        for(int j=0;j<mapSize;j++){
            if(grid[i][j])
            {
                double kx = i%2 == 0? -(double)i/2.0 : ((double)i + 1)/2.0;
                double ky = j%2 == 0? -(double)j/2.0 : ((double)j + 1)/2.0;
                geometry_msgs::Point pt;
                pt.x  = kx * cellSize;
                pt.y = ky * cellSize;
                pt.z = 0;
                cells.points.push_back(pt);
            }
        }
    }
    visualizeMapData(pub,&cells,0,new double[3]{0,1,0});
}

void GridMap::visualizeMapData(ros::Publisher *pub, visualization_msgs::Marker *marker,int id,double *rgb,double * scale){
    marker->header.frame_id = "/odom";
    marker->header.stamp = ros::Time();
    marker->ns = "wheely";
    marker->id = id;
    marker->type = visualization_msgs::Marker::CUBE_LIST;
    marker->action = visualization_msgs::Marker::ADD;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;
    marker->scale.x = scale != NULL ? (scale[0] != -1? scale[0]:cellSize) :cellSize;
    marker->scale.y = scale != NULL ? (scale[1] != -1? scale[1]:cellSize) :cellSize;
    marker->scale.z = scale != NULL ? (scale[2] != -1? scale[2]:cellSize) :cellSize;
    marker->color.a = 1.0; 
    marker->color.r = rgb[0];
    marker->color.g = rgb[1];
    marker->color.b = rgb[2];
    pub->publish(*marker);
    }
}
