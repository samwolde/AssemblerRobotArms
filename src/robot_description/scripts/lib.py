import xml.dom.minidom as DOM
import math


'''
    This is the Sdf class, which represents the whole document.
    Contains the root element <sdf>.
'''
class Sdf():
    def __init__(self):
        raise Exception("Don't Use Constructor explicitly, Use createSdfDoc()!")
    
    @classmethod
    def createSdfDoc(cls,ver:str ="1.6" ) -> DOM.Document:
        if hasattr(cls, "_sdf_doc"):
            return cls._sdf_doc
        
        #Else create the SDF Document
        
        impl:DOM.DOMImplementation = DOM.getDOMImplementation()
        cls._sdf_doc:DOM.Document = impl.createDocument(None, "sdf",None)
        cls._root = cls._sdf_doc.documentElement
        cls._root.setAttribute("version",ver)
        cls._sdf_doc.appendChild(cls._root)
        
        return cls._sdf_doc

    @classmethod
    def getRootElement(cls) -> DOM.Element:
        return cls._root

    @classmethod
    def createSdfFile(cls,file:str) :
        f = open(file,'w')
        f.write(cls._sdf_doc.toprettyxml())

'''
    create a model with a name and a bunch of children, which include 
    multiple links,joint,sensors and maybe other models.
    Or
    create an empty model and populate it later.
'''
class Model(DOM.Element):
    def __init__(self,name:str, children=[]):
        super().__init__("model")
        self.sdf_doc = Sdf.createSdfDoc()
        self.name = name
        self.children = children
        self.buildModel()

    def buildModel(self):
        #assign name
        name_attr = self.sdf_doc.createAttribute("name")
        self.setAttributeNode(name_attr)
        self.setAttribute("name", self.name)
        for child in self.children:
            self.appendChild(child)

class Location:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def set_location(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Orientation:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def set_orientation(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
class Pose:
    def __init__(self, loc:Location=None, orie:Orientation=None):
        if loc is None:
            loc = Location(0,0,0)
        
        if orie is None:
            orie = Orientation(0, 0, 0)

        self.loc = loc
        self.orie = orie

#Generic Link class
class Link(DOM.Element):
    def __init__(self,name:str, pose:Pose,mass,inertial):
        super().__init__("link")
        
        self.name = name
        self.loc:Location = pose.loc
        self.sdf_doc = Sdf.createSdfDoc()
        self.orie = pose.orie
        self.mass = mass
        self.inertial = inertial
        self.build_link()
    
    def build_link(self):
        name_a = self.sdf_doc.createAttribute("name")
        self.setAttributeNode(name_a)
        self.setAttribute("name",self.name)
        pose = self.sdf_doc.createElement("pose")
        text = "{0} {1} {2} {3} {4} {5}".format(self.loc.x,self.loc.y,self.loc.z,self.orie.x,self.orie.y,self.orie.z)
        textNode = self.sdf_doc.createTextNode(text)
        pose.appendChild(textNode)
        self.appendChild(pose)
        self.appendChild(Inertial(self.mass, self.inertial))

#Create A rectangular link.
class RectangularLink(Link):

    def __init__(self, name, pose:Pose, mass,size, inertial):
        super().__init__(name,pose, mass, inertial)
        self.col_vis = RectangularColVis(name, size)
        self.appendChild(self.col_vis.col_vis[0])
        self.appendChild(self.col_vis.col_vis[1])


#Create A cylinderical link.
class CylindericalLink(Link):

    def __init__(self, name, pose:Pose,leng,mass,rad, inertial):
        super().__init__(name,pose, mass, inertial)
        self.col_vis = CylinderColVis(name, rad,leng)
        self.appendChild(self.col_vis.col_vis[0])
        self.appendChild(self.col_vis.col_vis[1])

class Plugin(DOM.Element):
    def __init__(self, name:str, filename:str,parameters:dict):
        super().__init__("plugin")
        sdf = Sdf.createSdfDoc()
        self.setAttributeNode(sdf.createAttribute("filename"))
        self.setAttributeNode(sdf.createAttribute("name"))
        self.setAttribute("filename", filename)
        self.setAttribute("name", name)

        for p in parameters:
            param = sdf.createElement(p)
            param.appendChild(sdf.createTextNode(str(parameters[p])))
            self.appendChild(param)


'''
    Generic Sensor Class
'''
class _Sensor(DOM.Element):

    def __init__(self, name:str, type:str,updateRate, visualize_, pose_p:Pose, plugin:Plugin):
        super().__init__("sensor")
        sdf_doc = Sdf.createSdfDoc()
        self.setAttributeNode(sdf_doc.createAttribute("name"))
        self.setAttribute("name",name)
        self.setAttributeNode(sdf_doc.createAttribute("type"))
        self.setAttribute("type",type)
        update_rate = sdf_doc.createElement("update_rate")
        update_rate.appendChild(sdf_doc.createTextNode(str(updateRate)))
        pose = sdf_doc.createElement("pose")
        text = "{0} {1} {2} {3} {4} {5}".format(pose_p.loc.x,pose_p.loc.y,pose_p.loc.z,pose_p.orie.x,pose_p.orie.y,pose_p.orie.z)
        textNode = sdf_doc.createTextNode(text)
        pose.appendChild(textNode)
        visualize = sdf_doc.createElement("visualize")

        visualize.appendChild(sdf_doc.createTextNode(str(visualize_)))
        self.appendChild(visualize)
        self.appendChild(update_rate)
        self.appendChild(pose)
        self.appendChild(plugin)
'''
    Infrared sensor
'''
class IRSensor(_Sensor):
    def __init__(self, name:str,hr:dict, vert:dict,range_:dict,pose:Pose, plugin:Plugin, visualize:str="false"):
        super().__init__(name,"ray", 50,visualize, pose, plugin)
        sdf_doc = Sdf.createSdfDoc()

        ray  = sdf_doc.createElement("ray")
        scan = sdf_doc.createElement("scan")
        hor = sdf_doc.createElement("horizontal")
        ver = sdf_doc.createElement("vertical")
        range_el = sdf_doc.createElement("range")
        noise = sdf_doc.createElement("noise")
        n_type = sdf_doc.createElement("type")
        n_mean = sdf_doc.createElement("mean")
        n_stddev = sdf_doc.createElement("stddev")
        n_type.appendChild(sdf_doc.createTextNode("gaussian"))
        n_mean.appendChild(sdf_doc.createTextNode("0.0"))
        n_stddev.appendChild(sdf_doc.createTextNode("0.01"))
        noise.appendChild(n_type)
        noise.appendChild(n_mean)
        noise.appendChild(n_stddev)
        
        scan.appendChild(hor)
        if ( vert != None):
            scan.appendChild(ver)

        ray.appendChild(scan)
        ray.appendChild(range_el)
        ray.appendChild(noise)

        self.appendChild(ray)
        for h in hr:
            t = sdf_doc.createElement(h)
            t.appendChild(sdf_doc.createTextNode(str(hr[h])))            
            hor.appendChild(t)
        if vert != None:
            for v in vert:
                t = sdf_doc.createElement(v)
                t.appendChild(sdf_doc.createTextNode(str(vert[v])))
                ver.appendChild(t)
        for r in range_:
            t = sdf_doc.createElement(r)
            t.appendChild(sdf_doc.createTextNode(str(range_[r])))
            range_el.appendChild(t)
        
        always_on = sdf_doc.createElement("always_on")
        always_on.appendChild(sdf_doc.createTextNode("true"))
        self.appendChild(always_on)

class CylindericalLinkWithSensor(CylindericalLink):

    def __init__(self, name, pose:Pose,leng,mass,rad, inertial):
        super().__init__(name, pose, leng, mass, rad, inertial)
        self.sensor = Sensor(name)
        self.appendChild(self.sensor)

class Sensor(DOM.Element):
    
    def __init__(self, name):
        super().__init__("sensor")
        sdf_doc = Sdf.createSdfDoc()

        always_on = sdf_doc.createElement("always_on")
        always_on.appendChild(sdf_doc.createTextNode(str("1")))
        contact = sdf_doc.createElement("contact")
        collision = sdf_doc.createElement("collision")
        collision.appendChild(sdf_doc.createTextNode(str("{0}_col".format(name))))
        update_rate = sdf_doc.createElement("update_rate")
        update_rate.appendChild(sdf_doc.createTextNode(str("30")))
        visualize = sdf_doc.createElement("visualize")
        visualize.appendChild(sdf_doc.createTextNode(str("true")))

        plugin = sdf_doc.createElement("plugin")
        plugin.setAttributeNode(sdf_doc.createAttribute("name"))
        plugin.setAttributeNode(sdf_doc.createAttribute("filename"))
        plugin.setAttribute("name", "my_plugin")
        plugin.setAttribute("filename", "libcontact_plugin.so")

        contact.appendChild(collision)
        self.appendChild(contact)
        self.appendChild(always_on)
        self.appendChild(update_rate)
        self.appendChild(visualize)
        self.appendChild(plugin)

        self.setAttributeNode(sdf_doc.createAttribute("name"))
        self.setAttributeNode(sdf_doc.createAttribute("type"))
        self.setAttribute("name","{0}_sensor".format(name))
        self.setAttribute("type","contact")
'''
    Create A Cylinderical Collision and Visual xml tags
    returns [collision, visual]
'''
class CylinderColVis():
    def __init__(self,name, radius, length):
        super().__init__()
        sdf = Sdf.createSdfDoc()
        col = sdf.createElement("collision")
        vis = sdf.createElement("visual")
        geo = sdf.createElement("geometry")
        cyl = sdf.createElement("cylinder")
        rad = sdf.createElement("radius")
        leng = sdf.createElement("length")

        rad.appendChild(sdf.createTextNode(str(radius)))
        leng.appendChild(sdf.createTextNode(str(length)))
        
        name_a = sdf.createAttribute("name")
        col.setAttributeNode(name_a)
        col.setAttribute("name",name+"_col")
        vis.setAttributeNode(sdf.createAttribute("name"))
        vis.setAttribute("name",name+"_vis")

        cyl.appendChild(rad)
        cyl.appendChild(leng)

        geo.appendChild(cyl)
        geo2 = geo.cloneNode(True)
        col.appendChild(geo)
        vis.appendChild(geo2)

        self.col_vis = [vis, col]

'''
    Create A rectangular Collision and Visual xml tags
    returns [collision, visual]
'''
class RectangularColVis():
    def __init__(self, name, size):
        sdf = Sdf.createSdfDoc()
        col = sdf.createElement("collision")
        vis = sdf.createElement("visual")
        geo = sdf.createElement("geometry")
        cyl = sdf.createElement("box")
        siz = sdf.createElement("size")

        siz.appendChild(sdf.createTextNode("{0} {1} {2}".format(size[0], size[1], size[2])))
        
        name_a = sdf.createAttribute("name")
        col.setAttributeNode(name_a)
        col.setAttribute("name",name+"_col")
        vis.setAttributeNode(sdf.createAttribute("name"))
        vis.setAttribute("name",name+"_vis")

        cyl.appendChild(siz)

        geo.appendChild(cyl)

        geo2 = geo.cloneNode(True)
        vis.appendChild(geo)
        col.appendChild(geo2)
        self.col_vis = [vis, col]


class Inertial(DOM.Element):
    def __init__(self, mass, inertial):
        super().__init__("inertial")
        self.sdf = Sdf.createSdfDoc()
        self.mass = self.sdf.createElement("mass")
        self.inertia = self.sdf.createElement("inertia")
        self.ixx = self.sdf.createElement("ixx")
        self.iyy = self.sdf.createElement("iyy")
        self.izz = self.sdf.createElement("izz")
        self.build_inertial(mass , inertial)

    def build_inertial(self,mass , inertial):
        sdf = self.sdf
        self.mass.appendChild(sdf.createTextNode(str(format(mass, '.8f'))))

        self.ixx.appendChild(sdf.createTextNode(str(format(inertial[0], '.8f'))))
        self.iyy.appendChild(sdf.createTextNode(str(format(inertial[1], '.8f'))))
        self.izz.appendChild(sdf.createTextNode(str(format(inertial[2], '.8f'))))

        self.inertia.appendChild(self.ixx)
        self.inertia.appendChild(self.iyy)
        self.inertia.appendChild(self.izz)
        self.appendChild(self.inertia)
        self.appendChild(self.mass)

#Generic Joint 
#TODO:
class Joint(DOM.Element):
    def __init__(self,name:str, type:str,  pose:Pose, child:str, parent:str):
        super().__init__("joint")
        sdf_doc = Sdf.createSdfDoc()
        self.loc = pose.loc
        self.orie =pose.orie
        self.pose = sdf_doc.createElement("pose")
        self.child  = sdf_doc.createElement("child")        
        self.parent  = sdf_doc.createElement("parent")        
        text = "{0} {1} {2} {3} {4} {5}".format(self.loc.x,self.loc.y,self.loc.z,self.orie.x,self.orie.y,self.orie.z)
        textNode = sdf_doc.createTextNode(text)
        self.pose.appendChild(textNode)
        self.child.appendChild(sdf_doc.createTextNode(child))
        self.parent.appendChild(sdf_doc.createTextNode(parent))
        
        self.appendChild(self.pose)
        self.appendChild(self.child)
        self.appendChild(self.parent)
        self.setAttributeNode(sdf_doc.createAttribute("name"))
        self.setAttributeNode(sdf_doc.createAttribute("type"))
        self.setAttribute("type",type)
        self.setAttribute("name",name)
    def addOde(self):
        sdf_doc = Sdf.createSdfDoc()
        physics = sdf_doc.createElement("physics")
        ode = sdf_doc.createElement("ode")
        cfm = sdf_doc.createElement("cfm_damping")
        imp = sdf_doc.createElement("implicit_spring_damper")
        physics.appendChild(ode)
        ode.appendChild(cfm)
        ode.appendChild(imp)

        cfm.appendChild(sdf_doc.createTextNode("1"))
        imp.appendChild(sdf_doc.createTextNode("1"))
        self.appendChild(physics)
class RevoluteJoint(Joint):
    def __init__(self, name:str, pose:Pose,child:str, parent:str, upper, lower, axis_orie:Orientation):
        super().__init__(name,"revolute", pose, child, parent)
        self.axis = Axis("0.05", "0.03", upper,lower, axis_orie)
        self.appendChild(self.axis)


class Axis(DOM.Element):
    def __init__(self, friction, damping, upper_s, lower_s, axis_orie:Orientation):
        super().__init__("axis")
        sdf_doc = Sdf.createSdfDoc()
        dynamics = sdf_doc.createElement("dynamics")
        friction_node = sdf_doc.createElement("friction")
        damping_node = sdf_doc.createElement("damping")
        xyz = sdf_doc.createElement("xyz")
        limit = sdf_doc.createElement("limit")
        upper = sdf_doc.createElement("upper")
        lower = sdf_doc.createElement("lower")

        friction_node.appendChild(sdf_doc.createTextNode(str(friction)))
        damping_node.appendChild(sdf_doc.createTextNode(str(damping)))
        xyz_t = sdf_doc.createTextNode("{0} {1} {2}".format(axis_orie.x,axis_orie.y,axis_orie.z))
        xyz.appendChild(xyz_t)
        upper.appendChild(sdf_doc.createTextNode(str(upper_s)))
        lower.appendChild(sdf_doc.createTextNode(str(lower_s)))
    
        dynamics.appendChild(friction_node)
        dynamics.appendChild(damping_node)
        limit.appendChild(upper)
        limit.appendChild(lower)

        self.appendChild(xyz)
        self.appendChild(dynamics)
        if(upper_s == None or lower_s == None):
            return
        self.appendChild(limit)

class math_helper():
    def __init__(self, orie:Orientation):
        self.orie = orie

    def inertial_cylinderical(self, mass:int, height:int, radius:int): 

        if(self.orie.y == 0):
            x = 1/12 * (mass * ( (3*(radius**2)) + (height**2))) 
            z = 1/2 * (mass * (radius**2))
            return [x, x, z]
        else:
            x = 1/2 * (mass * (radius**2))
            y = 1/12 * (mass * ( (3*(radius**2)) + (height**2))) 
            return [x, y, y]

    def inertial_rectangular(self, mass:int, width:int, height:int, depth:int):
        
        w = 1/12 * (mass * ( (height**2) + (depth**2))) 
        h = 1/12 * (mass * ( (width**2)  + (depth**2)))
        d = 1/12 * (mass * ( (height**2) + (width**2)))
        
        return [w, h, d]

#Generic Collision with different geometries
#TODO:
class Collision(DOM.Element):
    def __init__(self,name):
        pass

#Generic Visual with different geometries
#TODO:
class Visual(DOM.Element):
    pass

#Generic Geometry class
#TODO:
class Geometry(DOM.Element):
    pass

def degree(rad):
    return rad * 180.0 / math.pi

def rad(degree):
    return degree * math.pi / 180.0