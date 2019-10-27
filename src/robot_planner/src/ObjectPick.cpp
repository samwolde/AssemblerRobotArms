#include <robot_planner/ObjectPick.h>

bool ObjectPick::alignNdGoSrv(robot_lib::PickObject::Request& req, robot_lib::PickObject::Response& res){
    wheely_planner::Coordinate_t c = new wheely_planner::Coordinate(req.distance[0],0);
    return (res.success = alignNdGo(c, req.angle[0]));
}
bool ObjectPick::moveFrwd(robot_lib::MoveFrwd::Request& req, robot_lib::MoveFrwd::Response& res){
    wheely_planner::Coordinate_t c = new wheely_planner::Coordinate(req.distance,0);
    return (res.success = alignNdGo(c, 0));
}
bool ObjectPick::alignNdGo(wheely_planner::Coordinate_t c, double angle){
    //Change the laser readings distance to the robot
    //Coordinate system
    Tf_From_Hand_Robo(c,angle);       
    //Change the robot coordinates to world coordinates.
    mapBuilder.Tf_From_Robo_World(c);

    geometry_msgs::Point pt;
    pt.x = c->x;
    pt.y = c->y;
    robot_lib::GoTo g;
    g.request.detectObstacles = true;
    g.request.path = std::vector<geometry_msgs::Point>{pt};
    return goto_cl.call(g);
}

void ObjectPick::Tf_From_Hand_Robo(wheely_planner::Coordinate_t c, double angle){
        //Translation Vector S_origin - Robo_Origin
        static Eigen::Affine3d Mt (Eigen::Translation3d(Eigen::Vector3d(0.6,0,0)) );
        //Rotation vector
        Eigen::Affine3d Mr(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

        Eigen::Vector3d co = (Mt * Mr) * Eigen::Vector3d(c->x,0 ,0);
        c->x = co.x();
        c->y = co.y();
}

