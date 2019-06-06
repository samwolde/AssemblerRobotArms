#include <robot_planner/Slam.h>

namespace wheely_planner
{
    Slam::Slam(){
        gridMap = new GridMap(100,0.1);
    }
    bool Slam::UpdateMap(){

    }
} // namespace wheely_planner
