from lib import *

############ EXAMPLE USAGE###############
#create The sdf Document
sdf= Sdf.createSdfDoc()
root = Sdf.getRootElement()

#create a model
model = Model("model1", [])

#Two ways to create link 
#One is Create A generic Link 
link = Link("link-1",Pose(Location(2,3,4), Orientation(3,4,5)), "4", [1,2,3])

#create a Cylinderical collission and visual element
cylinder = CylinderColVis("cyl",0.5,0.9)
cylinder_vis = cylinder.col_vis[0]
cylinder_col = cylinder.col_vis[1]

#append these to the generic link
link.appendChild(cylinder_col)
link.appendChild(cylinder_vis)

#Or Create the specific link like the following
rectLink = RectangularLink("RectLink", Pose(Location(1,2,3), Orientation(1,2,3)),3,[1,2,3],[1,2,3])
cylidLink =CylindericalLink("CylLink", Pose(Location(1,2,3), Orientation(1,2,3)),3,4,4,[1,2,3]) 

#Creating a joint

joint = RevoluteJoint("joint1", Pose(Location(1,2,3), Orientation(1,2,3)), "RectLink", "CyLlink", 1,1,Orientation(1,2,3))
#append to the model
model.appendChild(link)
model.appendChild(rectLink)
model.appendChild(cylidLink)
model.appendChild(joint)


#append to the root sdf element
root.appendChild(model)
#create the file.
Sdf.createSdfFile("./generated.sdf")

