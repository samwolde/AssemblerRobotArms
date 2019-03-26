import xml.dom.minidom as DOM


'''
    This is the Sdf class, which represents the whole document.
    Contains the root element <sdf>.
'''
class Sdf():
    def __init__(self):
        raise Exception("Don't Use Constructor explicitly, Use createSdfDoc()!")
    
    @classmethod
    def createSdfDoc(cls,ver:str ="1.4" ) -> DOM.Document:
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
    def __init__(self,name:str, children):
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
#Generic Link class
class Link(DOM.Element):
    def __init__(self,name:str, loc:Location, orie:Orientation,mass,inertial):
        super().__init__("link")
        
        self.name = name
        self.loc:Location = loc
        self.sdf_doc = Sdf.createSdfDoc()
        self.orie = orie
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

    def __init__(self, name, loc:Location, orie:Orientation, mass,size, inertial):
        super().__init__(name,loc, orie, mass, inertial)
        self.col_vis = RectangularColVis(name, size)
        self.appendChild(self.col_vis.col_vis[0])
        self.appendChild(self.col_vis.col_vis[1])


#Create A cylinderical link.
class CylindericalLink(Link):

    def __init__(self, name, loc:Location, orie:Orientation,leng,mass,rad, inertial):
        super().__init__(name,loc, orie, mass, inertial)
        self.col_vis = CylinderColVis(name, rad,leng)
        self.appendChild(self.col_vis.col_vis[0])
        self.appendChild(self.col_vis.col_vis[1])


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
        siz = sdf.createElement("length")

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
        self.mass.appendChild(sdf.createTextNode(str(mass)))
        self.ixx.appendChild(sdf.createTextNode(str(inertial[0])))
        self.iyy.appendChild(sdf.createTextNode(str(inertial[1])))
        self.izz.appendChild(sdf.createTextNode(str(inertial[2])))
        self.inertia.appendChild(self.ixx)
        self.inertia.appendChild(self.iyy)
        self.inertia.appendChild(self.izz)
        self.appendChild(self.inertia)
        self.appendChild(self.mass)

#Generic Joint 
#TODO:
class Joint:
    pass

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
