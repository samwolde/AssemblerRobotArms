from lib import *
import random as rand

sdf  = Sdf.createSdfDoc()

class Prop:
    links_joints = []
    default_inertial = [0.0005, 0.0005, 0.0005]
    body_mass = 0.1
    body_depth = 0.1
    width = 0.25
    height = 0.25
    depth = 0.25
    locations = [[-13.802956,-1.684998],[-17.752222,-3.684369],[-23.848755,-1.587422],[-23.848755,-9.487605],[-10.633201,-9.195015],[-13.461333,-9.195015],[-18.191462,-7.049313],[-7.684233,-9.690165],[-5.093814,2.435014],[4.694657,-8.964105],[4.625290,6.112346],[4.416820,2.152175],[7.091980,6.781821],[14.245543,0.066458],[14.962285,4.533116]]
    colors = [[1,0,0,1],[0,1,0,1],[0,0,1,1],['0.96', '0.94', '0.26','1'],['0.96', '0.26', '0.90','1'],['0.26', '0.84', '0.96','1']]

class BoxCreator:
    def create(self, locations:list):
        i =1 
        for l in locations:
            pose_body = Pose(Location(l[0], l[1], 0), Orientation(0, 0, 0))
            r = rand.randint(0,1)
            c = rand.randint(0,len(Prop.colors)-1)
            # 0.1 0.1 0.1 1
            material = Material({
                "a":Prop.colors[c],
                "d":Prop.colors[c],
                "s":Prop.colors[c],
                "e":[0.1, 0.1, 0.1 ,1]
            })
            if r==0:
                link = RectangularLink(
                "box-"+str(i), pose_body, Prop.body_mass, [Prop.width, Prop.height, Prop.depth], Prop.default_inertial
                )
                link.vis_col.vis_col[0].appendChild(material)
            else:
                link = CylindericalLink(
                "box-"+str(i), pose_body,Prop.height/2, Prop.body_mass, Prop.depth/2, Prop.default_inertial
                )
                link.vis_col.vis_col[0].appendChild(material)
            Prop.links_joints.append(link)
            i+=1

#Create the boxes
bc = BoxCreator()
bc.create(Prop.locations)
root = Sdf.getRootElement()
model = Model("boxes-maze", Prop.links_joints)
root.appendChild(model)
Sdf.createSdfFile("../models/boxes-maze/boxes-maze.sdf")




