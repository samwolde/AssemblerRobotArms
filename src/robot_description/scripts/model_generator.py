from lib import * 
import math

PI = math.pi
default_inertial = [0.0005,0.0005,0.0005]
default_mass = 0.1
zero_pose = Pose(Location(0,0,0), Orientation(0,0,0))
width = 0.3
height = 2 * width
depth =  width/6

# The limit of the joint between Wheel and suspension
wheel_upper_limit,wheel_lower_limit = None, None
suspension_upper, suspension_lower = PI/3, -PI/3    #Only Turns 60 degrees

wheel_radius = width/4
wheel_length = wheel_radius/1.3

suspension_depth = wheel_radius * 1.3
suspension_rad = width/16

wheel_axel_radius = width/25
axel_length = .05

body_depth = suspension_depth + wheel_radius + depth/2 - 0.001

#Positions and Link of respective links

##Body LInk
pose_body = Pose(Location(0,0,body_depth),Orientation(0,0,0) )
body_link = RectangularLink("body_link",pose_body,default_mass,[width, height, depth],default_inertial )

##the front left suspension and wheel
susp_1_pose = [(width/2 - suspension_rad) , -(width-wheel_radius), wheel_radius + suspension_depth/2]

pose_susp_1 = Pose(Location(susp_1_pose[0],susp_1_pose[1],susp_1_pose[2]), Orientation( 0 ,0,0))
pose_wheel_1 = Pose(Location(susp_1_pose[0]+axel_length , susp_1_pose[1], wheel_radius), Orientation(0,rad(90),0))
pose_axel_1 = Pose(Location(susp_1_pose[0]+axel_length/2, susp_1_pose[1], wheel_radius), Orientation(0,rad(90),0))

susp_1 = CylindericalLink("susp_1",pose_susp_1,suspension_depth, default_mass, suspension_rad, default_inertial)
wheel_1 = CylindericalLink("wheel_1",pose_wheel_1, wheel_length, default_mass, wheel_radius, default_inertial)
wheel_axel_1 = CylindericalLink("axel_1",pose_axel_1,axel_length, default_mass, wheel_axel_radius, default_inertial)
## Fixed Joint between body and suspension 1
j_body_susp1_pose = Pose(Location(0, 0,suspension_depth/2 ),Orientation(0,0,0))
joint_sup1_body = Joint("susp1_body","fixed",j_body_susp1_pose,"susp_1","body_link")
## Fixed join between axel1 and susp1
pose_axelj = Pose(Location(0,0, -axel_length/2), Orientation(0,0,0))
joint_sup1_axel = Joint("susp_axel_1","fixed",pose_axelj, "axel_1","susp_1")
## Revolute Joint between wheel1 and suspension 1
pose_wheelj = Pose(Location(0,0, -wheel_length/2), Orientation(0,0,0))
joint_axel1_wheel = RevoluteJoint("axel_wheel_1",pose_wheelj, "wheel_1","axel_1",wheel_upper_limit,wheel_lower_limit,Orientation(1,0,0))



##Front right suspension and wheel pose
pose_susp_2 = Pose(Location(susp_1_pose[0],-susp_1_pose[1],susp_1_pose[2]), Orientation( 0 ,0,0))
pose_wheel_2 = Pose(Location(susp_1_pose[0]+axel_length , -susp_1_pose[1], wheel_radius), Orientation(0,rad(90),0))
pose_axel_2 = Pose(Location(susp_1_pose[0]+axel_length/2, -susp_1_pose[1], wheel_radius), Orientation(0,rad(90),0))

susp_2 = CylindericalLink("susp_2",pose_susp_2,suspension_depth, default_mass, suspension_rad, default_inertial)
wheel_2 = CylindericalLink("wheel_2",pose_wheel_2, wheel_length, default_mass, wheel_radius, default_inertial)
wheel_axel_2 = CylindericalLink("axel_2",pose_axel_2,axel_length, default_mass, wheel_axel_radius, default_inertial)
## Fixed Joint between suspension2 and body
j_body_susp2_pose =  Pose(Location(0, 0,suspension_depth/2 ),Orientation(0,0,0))
joint_sup2_body = Joint("susp2_body","fixed",j_body_susp2_pose,"susp_2","body_link")

## Fixed join between axel1 and susp1
pose_axelj = Pose(Location(0,0, -axel_length/2), Orientation(0,0,0))
joint_sup2_axel = Joint("susp_axel_2","fixed",pose_axelj, "axel_2","susp_2")
## Revolute Joint between wheel1 and suspension 1
pose_wheelj = Pose(Location(0,0, -wheel_length/2), Orientation(0,0,0))
joint_axel2_wheel = RevoluteJoint("axel_wheel_2",pose_wheelj, "wheel_2","axel_2",wheel_upper_limit,wheel_lower_limit,Orientation(1,0,0))



##Back left Suspension and wheel pose
susp_3_pose = [-susp_1_pose[0] , -susp_1_pose[1], susp_1_pose[2]]

pose_susp_3 = Pose(Location(susp_3_pose[0],susp_3_pose[1],susp_3_pose[2]), Orientation( 0 ,0,0))
pose_wheel_3 = Pose(Location(susp_3_pose[0]-axel_length , susp_3_pose[1], wheel_radius), Orientation(0,rad(90),0))
pose_axel_3 = Pose(Location(susp_3_pose[0]-axel_length/2, susp_3_pose[1], wheel_radius), Orientation(0,rad(90),0))

susp_3 = CylindericalLink("susp_3",pose_susp_3,suspension_depth, default_mass, suspension_rad, default_inertial)
wheel_3 = CylindericalLink("wheel_3",pose_wheel_3, wheel_length, default_mass, wheel_radius, default_inertial)
wheel_axel_3 = CylindericalLink("axel_3",pose_axel_3,axel_length, default_mass, wheel_axel_radius, default_inertial)

## Fixed Joint between body and suspension 4
j_body_susp3_pose = Pose(Location(0, 0,suspension_depth/2 ),Orientation(0,0,0))
joint_sup3_body = Joint("susp3_body","fixed",j_body_susp3_pose,"susp_3","body_link")
## Fixed join between axel1 and susp1
pose_axelj = Pose(Location(0,0, axel_length/2), Orientation(0,0,0))
joint_sup3_axel = Joint("susp_axel_3","fixed",pose_axelj, "axel_3","susp_3")
## Revolute Joint between wheel1 and suspension 1
pose_wheelj = Pose(Location(0,0, wheel_length/2), Orientation(0,0,0))
joint_axel3_wheel = RevoluteJoint("axel_wheel_3",pose_wheelj, "wheel_3","axel_3",wheel_upper_limit,wheel_lower_limit,Orientation(1,0,0))



##Back left Suspension and wheel pose
susp_4_pose = [-susp_1_pose[0] , susp_1_pose[1], susp_1_pose[2]]

pose_susp_4 = Pose(Location(susp_4_pose[0],susp_4_pose[1],susp_4_pose[2]), Orientation( 0 ,0,0))
pose_wheel_4 = Pose(Location(susp_4_pose[0]-axel_length , susp_4_pose[1], wheel_radius), Orientation(0,rad(90),0))
pose_axel_4 = Pose(Location(susp_4_pose[0]-axel_length/2, susp_4_pose[1], wheel_radius), Orientation(0,rad(90),0))

susp_4 = CylindericalLink("susp_4",pose_susp_4,suspension_depth, default_mass, suspension_rad, default_inertial)
wheel_4 = CylindericalLink("wheel_4",pose_wheel_4, wheel_length, default_mass, wheel_radius, default_inertial)
wheel_axel_4 = CylindericalLink("axel_4",pose_axel_4,axel_length, default_mass, wheel_axel_radius, default_inertial)

## Fixed Joint between body and suspension 4
j_body_susp4_pose = Pose(Location(0, 0,suspension_depth/2),Orientation(0,0,0))
joint_sup4_body = Joint("susp4_body","fixed",j_body_susp4_pose,"susp_4","body_link")
## Revolute Joint between wheel4 and suspension 4
pose_axelj = Pose(Location(0,0, axel_length/2), Orientation(0,0,0))
joint_sup4_axel = Joint("susp_axel_4","fixed",pose_axelj, "axel_4","susp_4")
## Revolute Joint between wheel1 and suspension 1
pose_wheelj = Pose(Location(0,0, wheel_length/2), Orientation(0,0,0))
joint_axel4_wheel = RevoluteJoint("axel_wheel_4",pose_wheelj, "wheel_4","axel_4",wheel_upper_limit,wheel_lower_limit,Orientation(1,0,0))


## create a plugin


plugin_wc = Plugin("test_plug","libwheel_plugin.so", 
{
    "robotNamespace":"wheely",
    "velPubTopic":"cmd_wheel",
    "odometrySubTopic":"odom",
    "velocity":0.2,
    #Tweak the below two parameteres if the turning angle overshoots.
    "angularVelocity":10,
    "kp":0.06
})

plugin = Plugin("skid_steer_controller", "libgazebo_ros_skid_steer_drive.so", 
{
    "alwaysOn": "true",
    "updateRate":1200,
    "robotNamespace":"wheely",
    "leftFrontJoint": "axel_wheel_4",
    "rightFrontJoint": "axel_wheel_1",
    "leftRearJoint":"axel_wheel_3",
    "rightRearJoint":"axel_wheel_2",
    "wheelSeparation": 0.891,#( (susp_1_pose[0]+axel_length) - (susp_4_pose[0]-axel_length) ) -wheel_length,
    "wheelDiameter": wheel_radius*2,
    "torque": 600,
    "commandTopic": "cmd_wheel",
    "odometryTopic":"odom",
    "odometryFrame":"odom",
    "broadcastTF":1,
    "robotBaseFrame": "body_link",
})

#Build The model
sdf = Sdf.createSdfDoc()
root = Sdf.getRootElement()
links_joints = [body_link,susp_1, wheel_1, joint_sup1_body,joint_sup1_axel,susp_2,wheel_2,joint_sup2_body,
joint_sup2_axel,susp_4,wheel_4,joint_sup4_body,joint_sup4_axel,
wheel_axel_1, joint_axel1_wheel, wheel_axel_2,joint_axel2_wheel,wheel_axel_3,joint_axel3_wheel,wheel_axel_4,joint_axel4_wheel,
susp_3,wheel_3,joint_sup3_body,joint_sup3_axel,plugin,plugin_wc]
model = Model("robot",links_joints)

#Append to root element and create sdf File.
root.appendChild(model)
Sdf.createSdfFile("../models/model.sdf")
