import math

CONSTANT_PI = math.pi

deg_to_rad = lambda x : x*(CONSTANT_PI/180.)
rad_to_deg = lambda x : x*(180./CONSTANT_PI)


class Vector(object):
    """
        Vector with rectangular coordinates
    """
    x = None
    y = None

    M = None
    arg = None

    
    def __init__(self, x: float, y: float) -> None:
        """
            Constructor

            Params:
                x -> x position
                y -> y position
        """

        self.x = float(x)
        self.y = float(y)

        self.M = math.sqrt(x**2 + y**2)
        
        try:
            self.arg = math.atan((y/x))
        except ZeroDivisionError:
            self.arg = (-1.)*CONSTANT_PI/2

        if x < 0 and y >= 0:
            self.arg += deg_to_rad(180)
        elif x < 0 and y < 0:
            self.arg = (-1.)*(deg_to_rad(180) - self.arg)
        elif x > 0 and y < 0:
            self.arg *= -1.
    
    def __add__(self, vector2):
        """
            Addition (Special Method)
        """
        newX = self.x + vector2.get_X()
        newY = self.y + vector2.get_Y()

        return Vector(newX, newY)
    
    def __sub__(self, vector2):
        """
            Subtraction (Special Method)
        """
        newX = self.x - vector2.get_X()
        newY = self.y - vector2.get_Y()

        return Vector(newX, newY)
    
    def __repr__(self) -> str:
        """
            Print Format (Special Method)
        """
        return f"{self.x}i  {self.y}j -> {self.M}<{rad_to_deg(self.arg)}"
    

    def dot_product(self, vector2) -> float:
        """
            Dot product between this instance of a vector and another

            Params:
                vector2 -> teh vector to dot product

            Returns:
                The dot product of this vector and vector 2
        """
        return (self.x * vector2.get_X) + (self.y * vector2.get_Y)
    
    def get_X(self) -> float:
        """
            Returns vectors x value
        """
        return self.x
    
    def get_Y(self) -> float:
        """
            Returns vectors y value
        """
        return self.y
    
    def get_Magnitude(self) -> float:
        """
            Returns vectors magnitude
        """
        return self.M
    
    def get_Arg(self) -> float:
        """
            Returns vectors argument (-180 <= arg <= 180) (Rads)
        """
        return self.arg
    
class Vector_Polar(Vector):
    """
        Vector with polar coordinates
    """

    def __init__(self, M: float, arg: float) -> None:
        """
            Constructor 
        """
        x = math.cos(arg)*M
        y = math.sin(arg)*M
        super().__init__(x, y)
