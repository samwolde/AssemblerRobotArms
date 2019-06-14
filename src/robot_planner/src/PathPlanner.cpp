#include <robot_planner/PathPlanner.h>

namespace wheely_planner
{
    PathPlanner::PathPlanner(ros::NodeHandlePtr rosNode,GridMap* grid, double kh ){
        goto_cl = rosNode->serviceClient<robot_lib::GoTo>("/wheely/nav/goto_srv");
        cmd_go_sub = rosNode->subscribe( "/wheely/clicked_pos",10,&PathPlanner::clicked_sub,this);
        gridMap = grid;
        this->kh = kh;
    }
    Cell PathPlanner::A_S_PlanPath(Coordinate from, Coordinate to){
        // rosNode->param<int>("kh",kh,1);
        int k_heuristics = kh;
        static double uniform_cost = 2;
        auto heuristics = [](Cell *c1,Cell *c2)->double{
            //The Eculidean distance
            return sqrt(pow(c1->center.x - c2->center.x,2) + pow(c1->center.y - c2->center.y,2));
        };
        explored_pts.points.clear();
        //Cells/Nodes in priority queue, priority == cells.f(n)
        std::priority_queue<Cell,std::vector<Cell>, CellPriorityComp> open;
        std::unordered_map<Cell,Cell,CellKeyHasher> encountered;
        //Identify the cell the goal belongs to 
        Index * i = gridMap->computeCellIndex(to.x,to.y);
        if( !i ) {
            ROS_ERROR("Goal (%f,%f) Not On the map, Can't navigate to it.", to.x,to.y);
            return Cell(0,0);
        }
        Cell goal(i->cx,i->cy);
        delete i;
        //Identify start as current pose, insert in priority_queue.
        i = gridMap->computeCellIndex(from.x,from.y);
        if( !i ) {
            ROS_ERROR("start (%f,%f) Not registered on the map,Aborting.", from.x,from.y);
            goal.parent = nullptr;
            return goal;
        }
        auto start = Cell(i->cx,i->cy);
        delete i;
        start.g =0;
        start.h = heuristics(&start, &goal);
        start.f  = start.g + start.h;
        open.push(start);
        encountered[start] = start;
        gridMap->storePtForVis(&start.center,&begin_end);//store the coorinates for visualization
        gridMap->storePtForVis(&goal.center,&begin_end);
        ROS_INFO("pOINTS ARE %f,%f -> %f, %f",start.center.x,start.center.y,goal.center.x,goal.center.y);
        ROS_INFO("Heuristics is %f, total f %f ,kh is %d", start.h, start.f,k_heuristics);
        auto found_path = false;
        auto nodes_visited = 0;
        while( !open.empty() ){
            auto c = open.top();
            open.pop();
            nodes_visited++;
            //For visualization
            gridMap->storePtForVis(&c.center,&explored_pts);
            if ( c == goal){
                //Found the min path 
                found_path = true;
                break;
            }
            //process adjacent cells that are open 
            std::vector<Cell> neighbors = gridMap->getAdjacentCells_24(&c.center);
            for (auto  neighbor: neighbors)
            {
                double n_cost = c.g + uniform_cost;
                //If we have found a better path to this node through c update nodes cost in queue
                if( encountered.find(neighbor) == encountered.end() || n_cost < encountered[neighbor].g ){
                    neighbor.g= n_cost; //update the better cost
                    neighbor.parent = new Cell(c);//update its parent to c
                    neighbor.f = n_cost + k_heuristics* heuristics(&neighbor, &goal);//update the priority
                    neighbor.h = k_heuristics*heuristics(&neighbor, &goal);//update the priority;
                    encountered[neighbor] =neighbor;
                    open.push(neighbor);//insert into priority queue
                }
            }
        }
        if(found_path ){
            ROS_INFO("Found Path from (%f, %f) -> (%f, %f)",start.center.x,start.center.y,goal.center.x,goal.center.y);
            ROS_INFO("Nodes Visited %d",nodes_visited);
            return encountered[goal];
        }
        goal.parent = nullptr;
        return goal;
    }

    std::stack<Cell*> PathPlanner::constructPath(Cell *node){
        std::stack<Cell*> path;
        if( !node  ){
            ROS_INFO("Not a valid path node is null");
            return path;
        }
        if(!node->parent){
            ROS_INFO("Not a valid path parent is null");
            return path;
        }
        path_found.points.clear();
        auto parent = node;
        while( parent != nullptr ){
            path.push(parent);
            gridMap->storePtForVis(&parent->center, &path_found);
            parent = parent->parent;
        }
        //pop the init node.
        path.pop();
        gridMap->visualizeMapData(&path_found,2,COLOR_LIGHT_BLUE,SCALE_MAP_Z(1));
        gridMap->visualizeMap();
        // pause();
        return path;
    }
    bool PathPlanner::FollowPath(std::stack<Cell*>* path){
        robot_lib::GoTo g;
        while( !path->empty() ){
            auto n = path->top();
            path->pop();
            geometry_msgs::Point pt;
            pt.x = n->center.x;
            pt.y = n->center.y;
            g.request.path.push_back(pt);
        }
        return goto_cl.call(g);
    }
    void PathPlanner::clicked_sub(geometry_msgs::PointConstPtr pt){
        // return;
        ROS_INFO("Calling A_S_PlanPath Sart %f,%f -> Goal  %f, %f...",this->x,this->y,pt->x, pt->y);
        auto goal = A_S_PlanPath(Coordinate(this->x,this->y),Coordinate(pt->x,pt->y));
        if ( !goal.parent ){
            ROS_INFO("Path Not Found!");
            return;
        }
        auto path = constructPath(&goal);
        // FollowPath(&path);
    }
} // namespace wheely_planner
