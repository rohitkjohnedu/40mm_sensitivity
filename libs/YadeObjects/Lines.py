from yade import *
from yade.gridpfacet import *
# ------------------------------------------------------------------------ Point
class Point:
    """
    A simple point class
    """
    def __init__(self, pos):
        self.pos = Vector3(pos)
    
    # -------------------------------------------------------------- calc_DistanceTo
    def calc_DistanceTo(self, point):
        """
        Calculates the distance from self to the input point

        Parameters
        ----------
        point : List
            Point of interest

        Returns
        -------
        float
            Distance from self to point of interest
        """
        rel_pos = Vector3(point) - self.pos
        return rel_pos.norm()

    @property
    def vec(self):
        return self.pos

# ------------------------------------------------------------------------ Line
class Line:
    """
    A line class. Stores a line as a point and a direction
    """
    def __init__ (self, point, arg2):
        """
        Initialises the line class using (points, direction) or two points

        Parameters
        ----------
        point : Point
            Point on line
        arg2 : Point or Vector3
            Second point or direction of line
        """
        self.m_point = point.vec
        if isinstance(arg2, Point):
            self.m_point_2   = arg2.vec
            self.m_direction = (self.m_point - self.m_point_2).normalized()

        else:
            self.m_direction = Vector3(arg2).normalized()     

    # -------------------------------------------------------------- calc_DistanceToPoint
    def calc_DistanceToPoint(self, POI):
        a = self.m_point
        n = self.m_direction
        p = Vector3(POI)
        distance_vec = ((p - a) - (p - a).dot(n)*n)
        return distance_vec.norm()

    # -------------------------------------------------------------- calc_ClosestPointTo
    def calc_ClosestPointTo(self, POI):
        a = self.m_point
        n = self.m_direction
        p = Vector3(POI)

        closest_point = a + (p - a).dot(n)*n
        return closest_point
        
    @property
    def p(self):
        return self.m_point
        
    @property
    def n(self):
        return self.m_direction

# ------------------------------------------------------------------------ calc_DistanceBetween2Lines
def calc_DistanceBetween2Lines(l1, l2, tol = 1e-4):
    p1 = l1.m_point
    p2 = l2.m_point

    diff = 2

    while (diff > tol):
        p2  = l2.calc_ClosestPointTo(p1)
        dist_2_from_1 = (p2 - p1).norm()

        p1 = l1.calc_ClosestPointTo(p2)
        dist_1_from_2 = (p2 - p1).norm()

        diff = abs(dist_2_from_1 - dist_1_from_2)

    return min(dist_2_from_1, dist_1_from_2)