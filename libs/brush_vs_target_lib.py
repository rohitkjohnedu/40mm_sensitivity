from bisect import bisect_left
# --------------------------------------------------------------------------- getTargetDensityForMMOI
def getTargetDensityForMMOI(target_mass, target_side_length, pfacet_radius)->float:
    """
    Returns the density for the material which will be used for the pfacet spheres. 
    The density is calculated such that the MMOI of the target and the pfacet are the same.
    The mass must be manually set for the pfacet clump

    Args:
        target_mass (float): Mass of the target
        target_side_length (float): side length of the target
        pfacet_radius (float): the radius of the pfacet spheres

    Returns:
        float: the density of the pfacet spheres
    """
    f_mmoi             = target_mass * target_side_length**2.0 / 6.0
    f_half_side_length = target_side_length / 2.0
    f_calculated_mass  = f_mmoi / (
         28.0/5.0*pfacet_radius**2.0 + 
         20*f_half_side_length**2.0
        )

    f_volume           = 4.0 / 3.0 * 3.14159265 * pfacet_radius**3.0 
    f_density          = f_calculated_mass / f_volume

    # delete
    print("density : ", f_density)
    # / delete

    return f_density

# --------------------------------------------------------------------------- getTargetDensityForMMOI
def getCorrectGridNodeMassMMOI(target_mass, target_side_length)->float:
    """
        Returns the corrected mass and the mmoi for the gridNodes. yade has a wierd
    (and wrong?) method for assigning the mass and mmoi for the gridNodes

    Args:
        target_mass (float): Mass of the target
        target_side_length (float): side length of the target

    Returns:
        tuple: target gridNode mass, target gridNode MMOI
    """
    f_target_grid_mass = target_mass / 14.0
    f_target_mmoi      = target_mass * target_side_length**2.0 / 6.0
    f_half_side_length = target_side_length / 2.0
    f_target_grid_mmoi = (f_target_mmoi - 20.0*f_target_grid_mass*f_half_side_length**2) / 14.0

    return f_target_grid_mass, f_target_grid_mmoi

# --------------------------------------------------------------------------- getTargetDensityForMass
def getTargetDensityForMass(target_mass, target_side_length):
    """
    Returns the density for the material which will be used for the pfacet spheres. 
    The density is calculated such that the mass of the target and the pfacet are the same.
    The MMOI must be manually set for the pfacet clump

    Args:
        target_mass (float): Mass of the target
        target_side_length (float): side length of the target

    Returns:
        float: the density of the pfacet spheres
    """
    f_volume           = target_side_length**3.0 if target_side_length < 0.07 else target_side_length**2*0.06
    f_density          = target_mass / f_volume

    return f_density
    
# --------------------------------------------------------------------------- getAxleSphereParameters
def getAxleSphereParameters(axle_radius, axle_mass):
    """
    Calculates the radius and the density of the sphere that will represent the axle in the simulation

    Args:
        axle_radius (float): Radius of the axle
        axle_mass (float): Mass of the axle

    Returns:
        float: the radius of the sphere, the density of the sphere
    """
    f_axle_mmoi      = axle_mass * axle_radius ** 2.0 / 2.0

    # Since there will be two spheres for symmetry
    f_axle_mass = axle_mass / 2
    f_axle_mmoi = f_axle_mmoi / 2

    f_sphere_radius  = ((5.0 / 2.0) * (f_axle_mmoi / f_axle_mass))**0.5
    f_sphere_volume  = 4.0 / 3.0 * 3.14159265 * f_sphere_radius**3
    f_sphere_density = f_axle_mass / f_sphere_volume

    return  f_sphere_radius, f_sphere_density

# --------------------------------------------------------------------------- getHoleTargetParameters
def getHoleTargetParameters(target_mass, target_side_length, axle_radius):
    """
    Get the radius and the density of the spheres that will represent the axle in the simulation

    Args:
        target_mass (float): Mass of the target used to calculate the mass of the hole
        target_side_length (float): Side length of the target
        axle_radius (float): Radius of the axle 
    Returns:
        (float, float): radius of the sphere, density of the sphere
    """
    f_target_volume  = target_side_length**3.0 if target_side_length < 0.07 else target_side_length**2*0.06
    f_target_density = target_mass / f_target_volume
    f_hole_height    = target_side_length if target_side_length < 0.07 else 0.06
    f_hole_volume    = 3.14159265 * axle_radius**2 * f_hole_height

    f_hole_mass      = f_target_density * f_hole_volume
    f_hole_mmoi      = (f_hole_mass * axle_radius**2.0)/2.0


    f_sphere_radius  = ((5.0/2.0) * (f_hole_mmoi / f_hole_mass))**0.5
    f_sphere_volume  = 4.0 / 3.0 * 3.14159265 * f_sphere_radius**3 
    f_sphere_density = f_hole_mass / f_sphere_volume

    return  f_sphere_radius, f_sphere_density



#  --------------------------------------------------------------------------- take_closest
def getClosestFromOrderedList(my_list, my_number):
    """
    Assumes my_list is sorted. Returns closest value to my_number.

    If two numbers are equally close, return the smallest number.
    Taken from https://stackoverflow.com/questions/12141150/from-list-of-integers-get-number-closest-to-a-given-value/12141511#12141511
    

    Args:
        my_list (list): Input ordered list
        my_number (number): the number of interest

    Returns:
        [int, number]: [index of the number closest to my_number, number closest to my_number]
    """
    # get the position
    pos = bisect_left(my_list, my_number)
    if pos == 0:
        return 0, my_list[0]
    if pos == len(my_list):
        return -1, my_list[-1]

    # get the closest number, is it number at pos or pos - 1
    before = my_list[pos - 1]
    after  = my_list[pos]

    if after - my_number < my_number - before:
        return pos, after
    else:
        return pos - 1, before

#  --------------------------------------------------------------------------- take_closest
def getLastClosestFromOrderedList(my_list, my_number):
    """
    Assumes my_list is sorted. Returns closest value less than my_number.

    If two numbers are equally close, return the smallest number.
    Taken from https://stackoverflow.com/questions/12141150/from-list-of-integers-get-number-closest-to-a-given-value/12141511#12141511
    

    Args:
        my_list (list): Input ordered list
        my_number (number): the number of interest

    Returns:
        [int, number]: [index of the number closest to my_number, number closest to my_number]
    """
    # get the position
    pos = bisect_left(my_list, my_number)
    if pos == 0:
        return 0, my_list[0]
    if pos == len(my_list):
        return -1, my_list[-1]

    if my_list[pos] > my_number:
        pos = pos - 1

    # get the closest number, is it number at pos or pos - 1
    before = my_list[pos]
    return pos, before

# --------------------------------------------------------------------------- DataInterpolator
class DataInterpolator:
    """
    Used to interpolate a list based on an input number
    """
    def __init__(self, data_X, data_Y):
        """
        Initialises the DataInterpolator class using data points which will later be used for interpolation

        Args:
            data_X (list): x
            data_Y (list): y = f(x)
        """
        if len(data_X) != len(data_Y):
            print("[ERROR] DataInterpolator: size mismatch")

        self.x = data_X
        self.y = data_Y

    def out(self, datum_X):
        """
        Returns the interpolated f(x) value for given x from the data points.

        Args:
            datum_X (float): input x

        Returns:
            float: f(x)
        """
        pos, _   = getLastClosestFromOrderedList(self.x, datum_X)
        if pos < len(self.x ) - 1:
            datum_Y  = self._lerp(
                self.x[pos], self.x[pos + 1], 
                self.y[pos], self.y[pos + 1], 
                datum_X
                )

        else:
            datum_Y  = self._lerp(
                self.x[pos - 1], self.x[pos], 
                self.y[pos - 1], self.y[pos], 
                datum_X
                )

        return datum_Y

    def _lerp(self, x1, x2, y1, y2, datum_X):
        """
        Linear interpolation using two points (x1, y1) and (x2, y2) using input x

        Args:
            x1 (float): x1
            x2 (float): x2
            y1 (float): y1
            y2 (float): y2
            datum_X (float): input x

        Returns:
            float: output y
        """
        data_Y = y2 + (y1 - y2)*(datum_X - x2)/(x1 - x2)
        return data_Y