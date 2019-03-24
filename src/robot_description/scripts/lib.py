class Location:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def set_location(x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Orientation:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def set_orientation(x, y, z):
        self.x = x
        self.y = y
        self.z = z


class RectLink:

    def __init__(self, width, length, height, location=None, orientation=None):
        self.width = width
        self.length = length
        self.height = height
        if location == None:
            self.location = Location(0, 0, 0)
        else:
            self.location = location
        if orientation == None:
            self.orientation = Orientation(0, 0, 0)
        else:
            self.orientation = orientation
    
    def get_xml():
        return ""


class CylinerLink:

    def __init__(self, radius, height, location=None, orientation=None):
        self.radius = radius
        self.height = height
        if location == None:
            self.location = Location(0, 0, 0)
        else:
            self.location = location
        if orientation == None:
            self.orientation = Orientation(0, 0, 0)
        else:
            self.orientation = orientation


        def get_xml():
            return ""

        