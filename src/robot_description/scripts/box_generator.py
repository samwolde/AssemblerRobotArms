from lib import *
import math

width = 0.1
height = 0.1
depth = 0.1
default_inertial = [0.0005, 0.0005, 0.0005]
body_mass = 0.1
body_depth = 0.1

links_joints = []

pose_body = Pose(Location(6.0, -4.6, 0.4), Orientation(0, 0, 0))
body_link = RectangularLink(
    "box1", pose_body, body_mass, [width, height, depth], default_inertial
)
links_joints.append(body_link)

pose_body = Pose(Location(4.7, -7.5, 1), Orientation(0, 0, 0))
body_link = RectangularLink(
    "box2", pose_body, body_mass, [width, height, depth], default_inertial
)
links_joints.append(body_link)

pose_body = Pose(Location(5.3, 6.1, 0), Orientation(0, 0, 0))
body_link = RectangularLink(
    "box3", pose_body, body_mass, [width, height, depth], default_inertial
)
links_joints.append(body_link)

pose_body = Pose(Location(-5.7, -4.0, 0.2), Orientation(0, 0, 0))
body_link = RectangularLink(
    "box5", pose_body, body_mass, [width, height, depth], default_inertial
)
links_joints.append(body_link)

pose_body = Pose(Location(-3.2, 5.2, 0.1), Orientation(0, 0, 0))
body_link = RectangularLink(
    "box4", pose_body, body_mass, [width, height, depth], default_inertial
)
links_joints.append(body_link)

# Build The model
sdf = Sdf.createSdfDoc()
root = Sdf.getRootElement()


model = Model("boxes", links_joints)

# Append to root element and create sdf File.
root.appendChild(model)
Sdf.createSdfFile("../models/boxes.sdf")
