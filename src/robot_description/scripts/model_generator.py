from lib import * 
import math

PI = math.pi
default_inertial = [0.0005,0.0005,0.0005]
default_mass = 10
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

# masses (in kg)
body_mass   = default_mass
susp_mass   = default_mass/10
axel_mass   = default_mass/20
wheel_mass  = default_mass/10

body_inertial   = math_helper(Orientation(0,0,0)).inertial_rectangular(body_mass, width, height, depth)
susp_inertial   = math_helper(Orientation(0,0,0)).inertial_cylinderical(susp_mass, suspension_depth, suspension_rad)
axel_inertial   = math_helper(Orientation(0,rad(90),0)).inertial_cylinderical(axel_mass, axel_length, wheel_axel_radius)
wheel_inertial  = math_helper(Orientation(0,rad(90),0)).inertial_cylinderical(wheel_mass, wheel_length, wheel_radius)


#Positions and Link of respective links

##Body LInk
pose_body = Pose(Location(0,0,body_depth),Orientation(0,0,0) )
body_link = RectangularLink("body_link",pose_body, body_mass,[width, height, depth], body_inertial )

##the front left suspension and wheel
susp_1_pose = [(width/2 - suspension_rad) , -(width-wheel_radius), wheel_radius + suspension_depth/2]

pose_susp_1 = Pose(Location(susp_1_pose[0],susp_1_pose[1],susp_1_pose[2]), Orientation( 0 ,0,0))
pose_wheel_1 = Pose(Location(susp_1_pose[0]+axel_length , susp_1_pose[1], wheel_radius), Orientation(0,rad(90),0))
pose_axel_1 = Pose(Location(susp_1_pose[0]+axel_length/2, susp_1_pose[1], wheel_radius), Orientation(0,rad(90),0))

susp_1 = CylindericalLink("susp_1",pose_susp_1,suspension_depth, susp_mass, suspension_rad, susp_inertial)
wheel_1 = CylindericalLink("wheel_1",pose_wheel_1, wheel_length, wheel_mass, wheel_radius, wheel_inertial)
wheel_axel_1 = CylindericalLink("axel_1",pose_axel_1,axel_length, axel_mass, wheel_axel_radius, axel_inertial)
## Fixed Joint between body and suspension 1
j_body_susp1_pose = Pose(Location(0, 0,suspension_depth/2 ),Orientation(0,0,0))
joint_sup1_body = Joint("susp1_body","fixed",j_body_susp1_pose,"susp_1","body_link")
## Fixed join between axel1 and susp1
pose_axelj = Pose(Location(0,0, -axel_length/2), Orientation(0,0,0))
joint_sup1_axel = Joint("susp_axel_1","fixed",pose_axelj, "axel_1","susp_1")
## Revolute Joint between wheel1 and suspension 1
pose_wheelj = Pose(Location(0,0, -wheel_length/2), Orientation(0,-rad(90),0))
joint_axel1_wheel = RevoluteJoint("axel_wheel_1",pose_wheelj, "wheel_1","axel_1",wheel_upper_limit,wheel_lower_limit,Orientation(1,0,0))



##Front right suspension and wheel pose
pose_susp_2 = Pose(Location(susp_1_pose[0],-susp_1_pose[1],susp_1_pose[2]), Orientation( 0 ,0,0))
pose_wheel_2 = Pose(Location(susp_1_pose[0]+axel_length , -susp_1_pose[1], wheel_radius), Orientation(0,rad(90),0))
pose_axel_2 = Pose(Location(susp_1_pose[0]+axel_length/2, -susp_1_pose[1], wheel_radius), Orientation(0,rad(90),0))

susp_2 = CylindericalLink("susp_2",pose_susp_2,suspension_depth, susp_mass, suspension_rad, susp_inertial)
wheel_2 = CylindericalLink("wheel_2",pose_wheel_2, wheel_length, wheel_mass, wheel_radius, wheel_inertial)
wheel_axel_2 = CylindericalLink("axel_2",pose_axel_2,axel_length, axel_mass, wheel_axel_radius, axel_inertial)
## Fixed Joint between suspension2 and body
j_body_susp2_pose =  Pose(Location(0, 0,suspension_depth/2 ),Orientation(0,0,0))
joint_sup2_body = Joint("susp2_body","fixed",j_body_susp2_pose,"susp_2","body_link")

## Fixed join between axel1 and susp1
pose_axelj = Pose(Location(0,0, -axel_length/2), Orientation(0,0,0))
joint_sup2_axel = Joint("susp_axel_2","fixed",pose_axelj, "axel_2","susp_2")
## Revolute Joint between wheel1 and suspension 1
pose_wheelj = Pose(Location(0,0, -wheel_length/2), Orientation(0,-rad(90),0))
joint_axel2_wheel = RevoluteJoint("axel_wheel_2",pose_wheelj, "wheel_2","axel_2",wheel_upper_limit,wheel_lower_limit,Orientation(1,0,0))



##Back left Suspension and wheel pose
susp_3_pose = [-susp_1_pose[0] , -susp_1_pose[1], susp_1_pose[2]]

pose_susp_3 = Pose(Location(susp_3_pose[0],susp_3_pose[1],susp_3_pose[2]), Orientation( 0 ,0,0))
pose_wheel_3 = Pose(Location(susp_3_pose[0]-axel_length , susp_3_pose[1], wheel_radius), Orientation(0,rad(90),0))
pose_axel_3 = Pose(Location(susp_3_pose[0]-axel_length/2, susp_3_pose[1], wheel_radius), Orientation(0,rad(90),0))

susp_3 = CylindericalLink("susp_3",pose_susp_3,suspension_depth, susp_mass, suspension_rad, susp_inertial)
wheel_3 = CylindericalLink("wheel_3",pose_wheel_3, wheel_length, wheel_mass, wheel_radius, wheel_inertial)
wheel_axel_3 = CylindericalLink("axel_3",pose_axel_3,axel_length, axel_mass, wheel_axel_radius, axel_inertial)

## Fixed Joint between body and suspension 4
j_body_susp3_pose = Pose(Location(0, 0,suspension_depth/2 ),Orientation(0,0,0))
joint_sup3_body = Joint("susp3_body","fixed",j_body_susp3_pose,"susp_3","body_link")
## Fixed join between axel1 and susp1
pose_axelj = Pose(Location(0,0, axel_length/2), Orientation(0,0,0))
joint_sup3_axel = Joint("susp_axel_3","fixed",pose_axelj, "axel_3","susp_3")
## Revolute Joint between wheel1 and suspension 1
pose_wheelj = Pose(Location(0,0, wheel_length/2), Orientation(0,-rad(90),0))
joint_axel3_wheel = RevoluteJoint("axel_wheel_3",pose_wheelj, "wheel_3","axel_3",wheel_upper_limit,wheel_lower_limit,Orientation(1,0,0))



##Back left Suspension and wheel pose
susp_4_pose = [-susp_1_pose[0] , susp_1_pose[1], susp_1_pose[2]]

pose_susp_4 = Pose(Location(susp_4_pose[0],susp_4_pose[1],susp_4_pose[2]), Orientation( 0 ,0,0))
pose_wheel_4 = Pose(Location(susp_4_pose[0]-axel_length , susp_4_pose[1], wheel_radius), Orientation(0,rad(90),0))
pose_axel_4 = Pose(Location(susp_4_pose[0]-axel_length/2, susp_4_pose[1], wheel_radius), Orientation(0,rad(90),0))

susp_4 = CylindericalLink("susp_4",pose_susp_4,suspension_depth, susp_mass, suspension_rad, susp_inertial)
wheel_4 = CylindericalLink("wheel_4",pose_wheel_4, wheel_length, wheel_mass, wheel_radius, wheel_inertial)
wheel_axel_4 = CylindericalLink("axel_4",pose_axel_4,axel_length, axel_mass, wheel_axel_radius, axel_inertial)

## Fixed Joint between body and suspension 4
j_body_susp4_pose = Pose(Location(0, 0,suspension_depth/2),Orientation(0,0,0))
joint_sup4_body = Joint("susp4_body","fixed",j_body_susp4_pose,"susp_4","body_link")
## Revolute Joint between wheel4 and suspension 4
pose_axelj = Pose(Location(0,0, axel_length/2), Orientation(0,0,0))
joint_sup4_axel = Joint("susp_axel_4","fixed",pose_axelj, "axel_4","susp_4")
## Revolute Joint between wheel1 and suspension 1
pose_wheelj = Pose(Location(0,0, wheel_length/2), Orientation(0,-rad(90),0))
joint_axel4_wheel = RevoluteJoint("axel_wheel_4",pose_wheelj, "wheel_4","axel_4",wheel_upper_limit,wheel_lower_limit,Orientation(1,0,0))


ir_pose = Pose(Location(0,-height/2+0.02,body_depth+depth/2+0.03), Orientation(0,0,0) )
ir_sensor_link = CylindericalLink("IR_link", ir_pose,0.06,1e-5, 0.02,[1e-6,1e-6,1e-6])

ir_joint_pose = Pose(Location(0,0,-0.06/2), Orientation(0,0,0))
ir_joint = RevoluteJoint("IR_body_joint", ir_joint_pose,"IR_link","body_link",None,None,Orientation(0,0,1))

ir_hori_vert_param = {
    "samples":"1",
    "resolution":"1",
    "min_angle":0,
    "max_angle":0
}
ir_vert_param = {
    "samples":"1",
    "resolution":"1",
    "min_angle":"0",
    "max_angle":"0"
}
ir_plugin = Plugin("gazebo_ros_range", "libgazebo_ros_range.so", {
    "gaussianNoise":0.0,
    "alwaysOn":"true",
    "updateRate":50,
    "topicName":"wheely/sensor/ir",
    "frameName":"IR_link",
    "radiation":"infrared",
    "fov":"0.2967"
})
ir_sensor = IRSensor("ir_sensor",ir_hori_vert_param,ir_vert_param,{
    "min":"0.01",
    "max":"50",
    "resolution":"1"},Pose(Location(0,-0.02,0.03), Orientation(0,0,-PI/2)),ir_plugin
)
ir_joint.addOde()
ir_sensor_link.appendChild(ir_sensor)
ir_sensor_ctrl = Plugin("IR_sensor_ctrl","libIR_sensor.so",{})
## create a plugin


wheel_ctrl = Plugin("wheel_ctr","libwheel_plugin.so", 
{
    "prefix":"wheely/steering",
    "velPubTopic":"cmd_wheel",
    "odometrySubTopic":"odom",
    #Tweak the below parameteres if the turning angle overshoots.
    #Or the car is slowly turning.
    "kp":1.4,                   #Increase kp if car turn rate is slow, decrease if turning angle overshoots too often
    "ki":5.5,
    "kd":1.4,
    "dt":0.01,
    # turns within goal_radian +- turn_accuracy, higher accuracy higher turning time
    "turnAccuracy":0.01
})
# tsp_plugin = Plugin("test_tsp", "libtsp_plugin.so",
# {
#     "turnAccuracy":0.001,
#     "distanceAccuracy":0.5,
#     "kp":0.2
# })
skid_steer_ctrl = Plugin("skid_steer_controller", "libgazebo_ros_skid_steer_drive.so", 
{
    "alwaysOn": "true",
    "updateRate":1200,
    "robotNamespace":"wheely/steering",
    "leftFrontJoint": "axel_wheel_4",
    "rightFrontJoint": "axel_wheel_1",
    "leftRearJoint":"axel_wheel_3",
    "rightRearJoint":"axel_wheel_2",
    "wheelSeparation":( (susp_1_pose[0]+axel_length) - (susp_4_pose[0]-axel_length) ) -wheel_length,#0.891, 
    "wheelDiameter": wheel_radius*2,
    "torque": 100,
    "commandTopic": "cmd_wheel",
    "odometryTopic":"odom",
    "odometryFrame":"odom",
    "broadcastTF":1,
    "robotBaseFrame": "body_link",
})

# Create arm
# ***********************************************************************
# ***********************************************************************
ARM_1_LENGTH = width/2
ARM_2_LENGTH = width/2
ARM_RADIUS = width/30

ARM_BASE_RADIUS = width/10
ARM_BASE_LENGTH = width/20

ARM_BASE_TOP_LENGTH = width/10
ARM_BASE_TOP_RADIUS = width/30

arm_mass            = default_mass/10
arm_base_mass       = default_mass/10
arm_base_top_mass   = default_mass/20

arm_inertial            = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(arm_mass, ARM_1_LENGTH, ARM_RADIUS)
arm_base_inertial       = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(arm_base_mass, ARM_BASE_LENGTH, ARM_BASE_RADIUS)
arm_base_top_inertial   = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(arm_base_top_mass, ARM_BASE_TOP_LENGTH, ARM_BASE_TOP_RADIUS)


# Arm Base
# ----------------------------------------------
# Position
armBasePos = Pose(Location(body_link.loc.x, body_link.loc.y + height/2 - ARM_BASE_RADIUS, body_link.loc.z + depth/2), Orientation(0, 0, 0))

# Link
armBaseLink = CylindericalLink('arm_base', armBasePos, ARM_BASE_LENGTH, arm_base_mass, ARM_BASE_RADIUS, arm_base_inertial)

# Joint
bodyLink_armBase = Joint('bodyLink_armBase', 'fixed', Pose(), armBaseLink.name, body_link.name)

# ----------------------------------------------

# Arm base top
# ----------------------------------------------
# Position
armBaseTopPos = Pose(Location(armBasePos.loc.x, armBasePos.loc.y, armBasePos.loc.z + ARM_BASE_LENGTH/2 + ARM_BASE_TOP_LENGTH/2), Orientation(0,0,0))
armBaseTopJoiPos = Pose(Location(0, 0, -ARM_BASE_TOP_LENGTH/2), Orientation(0, 0, 0))

# Link
armBaseTopLink = CylindericalLink('arm_base_top', armBaseTopPos, ARM_BASE_TOP_LENGTH, arm_base_top_mass, ARM_BASE_TOP_RADIUS, arm_base_top_inertial)

# Joint
armBase_armBaseTop = RevoluteJoint('armBase_armBaseTop', armBaseTopJoiPos, armBaseTopLink.name, armBaseLink.name, None, None, Orientation(0, 0, 1))

# ----------------------------------------------

# Arm 1
# ----------------------------------------------
# Position
arm1Pos = Pose(Location(armBaseTopPos.loc.x, armBaseTopPos.loc.y, armBaseTopPos.loc.z + ARM_BASE_TOP_LENGTH/2 + ARM_1_LENGTH/2), Orientation(0,0,0))
arm1JoiPos = Pose(Location(0, 0, -ARM_1_LENGTH/2), Orientation(0, 0, 0))

# Link
arm1Link = CylindericalLink('arm1', arm1Pos, ARM_1_LENGTH, arm_mass, ARM_RADIUS, arm_inertial)

# Joint
armBaseTop_arm1 = RevoluteJoint('armBaseTop_arm1', arm1JoiPos, arm1Link.name, armBaseTopLink.name, PI, -PI, Orientation(1, 0, 0))

# ----------------------------------------------

# Arm 2
# ----------------------------------------------
# Position
arm2Pos = Pose(Location(arm1Pos.loc.x, arm1Pos.loc.y, arm1Pos.loc.z + ARM_1_LENGTH/2 + ARM_2_LENGTH/2), Orientation(0,0,0))
arm2JoiPos = Pose(Location(0, 0, -ARM_2_LENGTH/2), Orientation(0, 0, 0))

# Link
arm2Link = CylindericalLink('arm2', arm2Pos, ARM_2_LENGTH, arm_mass, ARM_RADIUS, arm_inertial)

# Joint
arm1_arm2 = RevoluteJoint('arm1_arm2', arm2JoiPos, arm2Link.name, arm1Link.name, PI, -PI, Orientation(1, 0, 0))
# ----------------------------------------------


# Plugin
# ----------------------------------------------
arm_control_plugin = Plugin("arm_controller","libarm_controller.so", {})
# ----------------------------------------------


# ************************************************************************
# ************************************************************************

#Gripper 

palm_radius     = width/20
finger_radius   = width/80
palm_length     = width/20
finger_length   = width/6
palm_mass       = default_mass/30
finger_mass     = default_mass/40
palm_inertial   = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(palm_mass, palm_length, palm_radius)
finger_inertial = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(finger_mass, finger_length, finger_radius)


palm_pos = Pose(Location(arm2Pos.loc.x, arm2Pos.loc.y, arm2Pos.loc.z + ARM_2_LENGTH/2 + palm_length/2), Orientation(0,0,0))

palm = CylindericalLink("palm", Pose( Location( palm_pos.loc.x, palm_pos.loc.y, palm_pos.loc.z ), Orientation( palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), palm_length, palm_mass, palm_radius, palm_inertial )

finger_one      = CylindericalLink("finger_one",    Pose( Location(palm_pos.loc.x, palm_pos.loc.y+(palm_radius/2), palm_pos.loc.z+(palm_length/2)+(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_two      = CylindericalLink("finger_two",    Pose( Location(palm_pos.loc.x, palm_pos.loc.y-(palm_radius/2), palm_pos.loc.z+(palm_length/2)+(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_three    = CylindericalLink("finger_three",  Pose( Location(palm_pos.loc.x+(palm_radius/2), palm_pos.loc.y, palm_pos.loc.z+(palm_length/2)+(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_four     = CylindericalLink("finger_four",   Pose( Location(palm_pos.loc.x-(palm_radius/2), palm_pos.loc.y, palm_pos.loc.z+(palm_length/2)+(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)

finger_one_tip  = CylindericalLinkWithSensor("finger_one_tip",   Pose( Location(palm_pos.loc.x, palm_pos.loc.y+(palm_radius/2), palm_pos.loc.z+(palm_length/2)+((3*finger_length)/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_two_tip  = CylindericalLinkWithSensor("finger_two_tip",   Pose( Location(palm_pos.loc.x, palm_pos.loc.y-(palm_radius/2), palm_pos.loc.z+(palm_length/2)+((3*finger_length)/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_three_tip= CylindericalLinkWithSensor("finger_three_tip", Pose( Location(palm_pos.loc.x+(palm_radius/2), palm_pos.loc.y, palm_pos.loc.z+(palm_length/2)+((3*finger_length)/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_four_tip = CylindericalLinkWithSensor("finger_four_tip",  Pose( Location(palm_pos.loc.x-(palm_radius/2), palm_pos.loc.y, palm_pos.loc.z+(palm_length/2)+((3*finger_length)/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)

palm_joint  = RevoluteJoint("palm_joint", Pose( Location(0, 0, -(palm_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "palm", arm2Link.name, PI, -PI, Orientation(0, 0, 1))

finger_one_joint    = RevoluteJoint("finger_one_joint",     Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_one",   "palm",  0,     PI/4, Orientation(1, 0, 0))
finger_two_joint    = RevoluteJoint("finger_two_joint",     Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_two",   "palm", -PI/4,  0,    Orientation(1, 0, 0))
finger_three_joint  = RevoluteJoint("finger_three_joint",   Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_three", "palm",  PI/4,  0,    Orientation(0, 1, 0))
finger_four_joint   = RevoluteJoint("finger_four_joint",    Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_four",  "palm",  0,    -PI/4, Orientation(0, 1, 0))

finger_one_tip_joint    = RevoluteJoint("finger_one_tip_joint",   Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_one_tip",   "finger_one",  -PI/2, 0,    Orientation(1, 0, 0))
finger_two_tip_joint    = RevoluteJoint("finger_two_tip_joint",   Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_two_tip",   "finger_two",   0,    PI/2, Orientation(1, 0, 0))
finger_three_tip_joint  = RevoluteJoint("finger_three_tip_joint", Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_three_tip", "finger_three", 0,   -PI/2, Orientation(0, 1, 0))
finger_four_tip_joint   = RevoluteJoint("finger_four_tip_joint",  Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_four_tip",  "finger_four",  PI/2, 0,    Orientation(0, 1, 0))

gripper_plugin = Plugin("gripper_plugin", "libgripper_plugin.so", {})

#Build The model
sdf = Sdf.createSdfDoc()
root = Sdf.getRootElement()
links_joints = [body_link,susp_1, wheel_1, joint_sup1_body,joint_sup1_axel,susp_2,wheel_2,joint_sup2_body,
joint_sup2_axel,susp_4,wheel_4,joint_sup4_body,joint_sup4_axel,
wheel_axel_1, joint_axel1_wheel, wheel_axel_2,joint_axel2_wheel,wheel_axel_3,joint_axel3_wheel,wheel_axel_4,joint_axel4_wheel,
susp_3,wheel_3,joint_sup3_body,joint_sup3_axel,
# Arm links and joints
# armBaseLink, bodyLink_armBase, armBaseTopLink, armBase_armBaseTop, arm1Link, armBaseTop_arm1, arm2Link, arm1_arm2,  
wheel_ctrl, skid_steer_ctrl,
# Gripper links and joints
# palm, palm_joint, finger_one, finger_one_joint, finger_two, finger_two_joint, finger_three, finger_three_joint, 
# finger_four, finger_four_joint, finger_one_tip, finger_one_tip_joint, finger_two_tip, finger_two_tip_joint, 
# finger_three_tip, finger_three_tip_joint, finger_four_tip, finger_four_tip_joint, 
# gripper_plugin, arm_control_plugin,
ir_sensor_link,ir_joint,ir_sensor_ctrl
]#,tsp_plugin]

model = Model("robot",links_joints)

#Append to root element and create sdf File.
root.appendChild(model)
Sdf.createSdfFile("../models/model.sdf")
