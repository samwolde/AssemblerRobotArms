#ifndef OBJECT_PICK_H
#define OBJECT_PICK_H

#include <robot_planner/MapBuilder.h>
#include <robot_lib/PickObject.h>
#include <robot_lib/MoveFrwd.h>

enum State {IN_PROGRESS, STALE};
class ObjectPick{
    private:
        // static State state;
        wheely_planner::MapBuilder& mapBuilder;
        ros::NodeHandlePtr rosNode;
        ros::ServiceServer goSrv,moveFrwdSrv;
        ros::ServiceClient goto_cl;
        double x,y;
    public:
        //Create service client,server and stuff..
        ObjectPick(wheely_planner::MapBuilder& mapBuilder,ros::NodeHandlePtr r)
        :mapBuilder(mapBuilder),rosNode(r)
        {
            // state = STALE;
            goSrv = rosNode->advertiseService("/wheely/object_pick/angle", &ObjectPick::alignNdGoSrv,this);
            moveFrwdSrv = rosNode->advertiseService("/wheely/object_pick/move_fwd", &ObjectPick::moveFrwd,this);
            goto_cl = rosNode->serviceClient<robot_lib::GoTo>("/wheely/nav/goto_srv");
        }
        /**
         * Pick object located at req.angle and req.distqance away, from current pose.
         * turn req.angle then
         * move straight req.distance away.
         */
        bool alignNdGoSrv(robot_lib::PickObject::Request& req, robot_lib::PickObject::Response& res);
        bool alignNdGo(wheely_planner::Coordinate_t c, double angle);
        /**
         * A varient of move forward, but this time, 
         * distance is given as an input not velocity.
         */
        bool moveFrwd(robot_lib::MoveFrwd::Request& req, robot_lib::MoveFrwd::Response& res);
        void Tf_From_Hand_Robo(wheely_planner::Coordinate_t c, double angle);
        void setXY(double _x, double _y){x=_x;y=_y;}

};

#endif