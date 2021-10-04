import math
from yade.gridpfacet import *
from math import *
import numpy as np
import itertools
import json
from json import JSONEncoder

# ---------------------------------------------------------------------------------------------- signum
def signum(x):
    """
    Returns +1 if x > 0, -1 if x < 0 and 0 if x = 0

    Args:
        x (float): input

    Returns:
        int: -1, 0 or +1
    """
    return -1*(x<0) + (x>1)
# ---------------------------------------------------------------------------------------------- VectorEncoder
class VectorEncoder(JSONEncoder):
    """
    This is used to serialise the Vector3 class so that json.dump can write this class
    """
    def default(self, o):
        return list(o)

# ---------------------------------------------------------------------------------------------- VectorEncoder
class ndarrayEncoder(JSONEncoder):
    """
    This is used to serialise the ndarray class so that json.dump can write this class
    """
    def default(self, o):
        return list(o)

# ---------------------------------------------------------------------------------------------- data
class data:
    pass

# ---------------------------------------------------------------------------------------------- lerp
def lerp(p1, p2, x):
    '''linear interpolation'''
    return (p2 - p1)*x + p1
    
# ---------------------------------------------------------------------------------------------- frange
def frange(min, max, step):
    ret_list = []
    a    = min
    while(a <= max):
        ret_list.append(a)
        a = a + step
    return ret_list

# ----------------------------------------------------------------------------------------------
def cross_prd(a,b):
    '''
    Returns the cross product of the vectors a and b
    '''
    a1,a2,a3 = a
    b1,b2,b3 = b

    c1 = a2*b3 - a3*b2
    c2 = a3*b1 - a1*b3
    c3 = a1*b2 - a2*b1

    c = [c1 ,c2, c3]
    return c

# ----------------------------------------------------------------------------------------------
def subtract(a,b):
    '''
    returns the difference of the vectors a and b
    '''
    a1,a2,a3 = a
    b1,b2,b3 = b

    c = [a1-b1, a2-b2, a3-b3]
    return c

# ----------------------------------------------------------------------------------------------
def body_angular_velocity_set(ang_vel, body_list, CM):
    '''
    Sets the initial angular velocity (ang_vel) of the pfacet body (body_list) about
    the centre point CM
    '''
    for i in body_list:
        r   = subtract(O.bodies[i].state.pos, CM)
        vel = cross_prd(r, ang_vel)
        O.bodies[i].state.vel    = vel
        O.bodies[i].state.angVel = ang_vel

# ----------------------------------------------------------------------------------------------
def body_angular_velocity_add(ang_vel, body_list, CM):
    '''
    Adds a value to the the initial angular velocity (ang_vel) of the pfacet body (body_list) about
    the centre point CM
    '''
    for i in body_list:
        r   = subtract(O.bodies[i].state.pos, CM)
        vel = cross_prd(r, ang_vel)
        O.bodies[i].state.vel    = O.bodies[i].state.vel    + vel
        O.bodies[i].state.angVel = O.bodies[i].state.angVel + ang_vel

# ----------------------------------------------------------------------------------------------
def body_linear_velocity_add(vel, body_list):
    '''
    Adds a value to the linear velocity of list of bodies
    '''
    for i in body_list:
        O.bodies[i].state.vel = O.bodies[i].state.vel + vel

# ----------------------------------------------------------------------------------------------
def body_linear_velocity_set(vel, body_list):
    '''
    Sets the linear velocity of list of bodies
    '''
    for i in body_list:
        O.bodies[i].state.vel = vel

# ----------------------------------------------------------------------------------------------
def linear_transform(M,x):
    '''
    Returns the product of the matrix M and the vector x
    '''
    x1, x2, x3 = x
    a, b, c    = M
    a1, a2, a3 = a
    b1, b2, b3 = b
    c1, c2, c3 = c

    res = [
        a1*x1 + a2*x2 + a3*x3,
        b1*x1 + b2*x2 + b3*x3,
        c1*x1 + c2*x2 + c3*x3
    ]

    return res

# ----------------------------------------------------------------------------------------------
def cuboid_gen(pos, l, b, h):
    points = []
    pos_x  = pos[0]
    pos_y  = pos[1]
    pos_z  = pos[2]


    points.append([pos_x - 0.5*l ,pos_y + 0.5*b ,pos_z - 0.5*h])
    points.append([pos_x - 0.5*l ,pos_y + 0.5*b ,pos_z + 0.5*h])
    points.append([pos_x - 0.5*l ,pos_y - 0.5*b ,pos_z + 0.5*h])
    points.append([pos_x + 0.5*l ,pos_y - 0.5*b ,pos_z - 0.5*h])
    points.append([pos_x + 0.5*l ,pos_y + 0.5*b ,pos_z - 0.5*h])
    points.append([pos_x + 0.5*l ,pos_y + 0.5*b ,pos_z + 0.5*h])
    points.append([pos_x + 0.5*l ,pos_y - 0.5*b ,pos_z + 0.5*h])
    points.append([pos_x - 0.5*l ,pos_y - 0.5*b ,pos_z - 0.5*h])

    with open('data.gts', 'w') as f:
        f.write('8 18 12 GtsSurface GtsFace GtsEdge GtsVertex\n')
        for i in points:
            f.write(str(i[0]) + ' ' + str(i[1]) + ' ' + str(i[2]) + '\n')

        f.write('1 2\n')
        f.write('3 1\n')
        f.write('2 3\n')
        f.write('4 5\n')
        f.write('1 4\n')
        f.write('5 1\n')
        f.write('5 6\n')
        f.write('5 2\n')
        f.write('6 2\n')
        f.write('7 5\n')
        f.write('7 4\n')
        f.write('4 8\n')
        f.write('4 3\n')
        f.write('3 8\n')
        f.write('7 6\n')
        f.write('3 7\n')
        f.write('8 1\n')
        f.write('2 7\n')
        f.write('1 2 3\n')
        f.write('4 5 6\n')
        f.write('7 8 9\n')
        f.write('4 10 11\n')
        f.write('12 13 14\n')
        f.write('15 10 7\n')
        f.write('16 13 11\n')
        f.write('17 5 12\n')
        f.write('1 8 6\n')
        f.write('16 18 3\n')
        f.write('14 2 17\n')
        f.write('9 18 15\n')

# ----------------------------------------------------------------------------------------------
def symmetric_cuboid_pfacet(CM, length, breadth, height, radius, int_mat, ext_mat, color = [0.5, 0.5, 0.5]):
    '''
    makes a symmetric cuboid using pfacet. The old gts makes a 
    assymetric pfacet. It is axis aligned
    '''
    l = length
    b = breadth
    h = height
    cm1, cm2, cm3 = CM # the centre of mass of the cube
    # coordinates of the vertices
    vertices = [
        [cm1 + l/2, cm2 + b/2, cm3 + h/2],
        [cm1 + l/2, cm2 - b/2, cm3 + h/2],
        [cm1 - l/2, cm2 - b/2, cm3 + h/2],
        [cm1 - l/2, cm2 + b/2, cm3 + h/2],
        [cm1 + l/2, cm2 + b/2, cm3 - h/2],
        [cm1 + l/2, cm2 - b/2, cm3 - h/2],
        [cm1 - l/2, cm2 - b/2, cm3 - h/2],
        [cm1 - l/2, cm2 + b/2, cm3 - h/2],
        [cm1 + l/2, cm2      , cm3      ],
        [cm1 - l/2, cm2      , cm3      ],
        [cm1      , cm2 + b/2, cm3      ],
        [cm1      , cm2 - b/2, cm3      ],
        [cm1      , cm2      , cm3 + h/2],
        [cm1      , cm2      , cm3 - h/2],        
    ]

    # connecting the vertices to form the edges
    edges    = [
        [1,2],[2,3],[3,4],[4,1],
        [1,5],[2,6],[3,7],[4,8],
        [5,6],[6,7],[7,8],[8,5],

        [13,1],[13,2],[13,3],[13,4],
        [14,5],[14,6],[14,7],[14,8],
        [11,1],[11,4],[11,8],[11,5],
        [12,2],[12,3],[12,7],[12,6],
        [9 ,1],[9 ,2],[9 ,6],[9 ,5],
        [10,3],[10,4],[10,8],[10,7],
    ]

    # connecting the vertices to form faces
    face     = [
        [13, 2, 1], [13, 3, 2], [13, 4, 3], [13, 1, 4],
        [14, 5, 6], [14, 6, 7], [14, 7, 8], [14, 8, 5],
        [11, 1, 5], [11, 5, 8], [11, 8, 4], [11, 4, 1],
        [12, 2, 3], [12, 3, 7], [12, 7, 6], [12, 6, 2],
        [9,  1, 2], [9,  2, 6], [9,  6, 5], [9,  5, 1],
        [10, 3, 4], [10, 4, 8], [10, 8, 7], [10, 7, 3],
    ]



    # generating the grid stuff
    nodesIds = []
    cylIds   = []
    pFacet   = []

    for i in face:
        pfacetCreator1(
            [
                vertices[i[0] - 1], 
                vertices[i[1] - 1], 
                vertices[i[2] - 1]
            ], 
            radius, 
            nodesIds = nodesIds, 
            cylIds   = cylIds, 
            pfIds    = pFacet, 
            wire     = False, 
            fixed    = False, 
            color    = color,
            materialNodes = int_mat, 
            material      = ext_mat, 
            )

    return nodesIds ,cylIds, pFacet
    # # generating the grid nodes
    # for i in vertices:
    #     nodesIds.append(
    #         O.bodies.append(
    #             gridNode(
    #                 i,
    #                 radius,
    #                 wire     = False,
    #                 fixed    = True,
    #                 material = material,
    #                 color    = color
    #                 )
    #         )
    #     )

    # # generating the pfacets
    # for i in edges:
    #     cylIds.append(
    #         O.bodies.append(
    #             gridConnection(
    #                 i[0],i[1],
    #                 radius,
    #                 color    = color,
    #                 material = 'gridNodeMat'
    #                 )
    #             )
    #     )

# ----------------------------------------------------------------------------------------------
def cylinder_pfacet(CM, cyl_rad, cyl_height, no, radius, int_mat, ext_mat, color = [0.5, 0.5, 0.5]):
    '''
    Returns a cylinder made of pfacet
    '''
    if no <= 1:
        print('cylinder_pfacet: no should be more than 1')
        return

    dtheta = 2*pi/(no)
    theta = frange(0,2*pi,dtheta)

    top_circle = []
    bot_circle = []
    mid_circle = []

    for j in range(no):
        i = theta[j]
        top_circle.append(
            [cyl_rad*cos(i), cyl_rad*sin(i), cyl_height/2]
        )

        bot_circle.append(
            [cyl_rad*cos(i), cyl_rad*sin(i), -cyl_height/2]
        )

    for j in range(no):
        if j < no-1:
            mid_circle.append([
                (top_circle[j+1][0] + top_circle[j][0])/2,
                (top_circle[j+1][1] + top_circle[j][1])/2,
                0,
            ])
        else:
            mid_circle.append([
                (top_circle[0][0] + top_circle[j][0])/2,
                (top_circle[0][1] + top_circle[j][1])/2,
                0,
            ])
        
# ----------------------------------------------------------------------------------------------
def brush_creator_scp(
    bristle_radius,
    bristle_lenght, 
    bristle_no_x, 
    bristle_no_y,  
    bristle_dx,
    bristle_dy,
    bristle_root_centre,
    bristle_segment_no,
    bristle_external_material,
    bristle_internal_material,
    brush_orientation = Quaternion((1,0,0),0), # This is axis(unit vector)-angle(in rads) notation
    root_fixed_Q = True, # If true the roots will be fixed to the ground
):
    '''
    Creates a square close packed brush and returns the ids of the root nodes
    and all objects. The bristles are alligned along the brush z axis
    '''

    bristle_segment_lenght = bristle_lenght/bristle_segment_no
    if (bristle_segment_lenght <= 2*bristle_radius):
        print('brush_creator_scp error: There is no gap between nodes. Segment length less than node diameter')
        return

    bristle_array = []
    bristle_node_Ids      = []
    bristle_cyl_Ids       = []
    bristle_root_Ids      = []

    all_bristle_root_ids      = []

    for i in range(bristle_no_x):
        bristle_array.append([])

        for j in range(bristle_no_y):
            bristle_array[-1].append([])

            for k in range(bristle_segment_no + 1):
                pos = (
                    i*bristle_dx - (bristle_no_x - 1)*bristle_dx/2,
                    j*bristle_dy - (bristle_no_y - 1)*bristle_dy/2,
                    bristle_lenght - k*bristle_segment_lenght
                    )

                pos = brush_orientation*pos
                pos = (
                    pos[0] + bristle_root_centre[0],
                    pos[1] + bristle_root_centre[1],
                    pos[2] + bristle_root_centre[2],
                )
                bristle_array[-1][-1].append(pos)

            cylinderConnection(
                bristle_array[-1][-1],
                radius = bristle_radius,
                nodesIds = bristle_node_Ids,
                cylIds   = bristle_cyl_Ids,
                extMaterial = bristle_external_material,
                intMaterial = bristle_internal_material,
                color       = (0.5,0.5,0.5),
            )
            bristle_root_Ids = bristle_node_Ids[-1]
            all_bristle_root_ids.append(bristle_root_Ids)            

            if (root_fixed_Q):
                O.bodies[bristle_root_Ids].state.blockedDOFs = 'xyzXYZ'

    if(root_fixed_Q):
        return bristle_cyl_Ids, bristle_node_Ids, all_bristle_root_ids # bristle_cyl_Ids, bristle_node_Ids

    else:
        return bristle_cyl_Ids, bristle_node_Ids, all_bristle_root_ids
           
# ----------------------------------------------------------------------------------------------
def cylindrical2cartesian(r, theta, z):
    x = r*cos(theta)
    y = r*sin(theta)

    return x, y, z

# ----------------------------------------------------------------------------------------------
def circular_brush_creator(    
    bristle_radius,
    bristle_no_z,
    bristle_no_theta,
    bristle_dz,
    bristle_segment_no,
    bristle_internal_material,
    bristle_external_material,
    brush_axis_centre_location,
    brush_inner_radius,
    brush_outer_radius,
    root_fixed_Q = True, # If true the roots will be fixed to the ground
    brush_angular_extend = 360, # A normal circular brush extends 360d, this can be used to define a smaller angle
    orientation = Quaternion((1,0,0),0) # the axis of the brush is oriented towards the brush Z axis
):
    '''
    Creates a circular brush and returns the ids
    '''

    bristle_segment_lenght = (brush_outer_radius - brush_inner_radius)/bristle_segment_no
    if (bristle_segment_lenght <= 2*bristle_radius):
        print('circular_brush_creator error: There is no gap between nodes. Segment length less than node diameter')
        return 

    bristle_node_Ids = []
    bristle_cyl_Ids  = []
    bristle_root_Ids = []

    all_bristle_node_Ids = []
    all_bristle_cyl_Ids  = []
    all_bristle_root_ids = []

    node_array = []
    brush_z_length = (bristle_no_z - 1)*bristle_dz

    if (bristle_no_theta > 1):
        bristle_dtheta = brush_angular_extend*(pi/180)/(bristle_no_theta)

    else:
        bristle_dtheta = 1

    for z in range(bristle_no_z):
        node_array.append([])

        for t in range(bristle_no_theta):
            node_array[-1].append([])

            for r in range(bristle_segment_no + 1):
                
                if (bristle_no_theta == 1):
                    theta_coordinate = 0

                else:
                    theta_coordinate = t*bristle_dtheta - brush_angular_extend/2

                r_coordinate = brush_outer_radius - r*bristle_segment_lenght
                z_coordinate = z*bristle_dz - brush_z_length/2

                x_coordinate, y_coordinate, z_coordinate = cylindrical2cartesian(
                    r_coordinate, theta_coordinate, z_coordinate
                )
                pos = (
                    x_coordinate, y_coordinate, z_coordinate
                )
                pos = orientation*pos
                pos = (
                    pos[0] + brush_axis_centre_location[0],
                    pos[1] + brush_axis_centre_location[1],
                    pos[2] + brush_axis_centre_location[2]
                )
                node_array[-1][-1].append(pos)

            cylinderConnection(
                node_array[-1][-1],
                radius   = bristle_radius,
                nodesIds = bristle_node_Ids,
                cylIds   = bristle_cyl_Ids,
                extMaterial = bristle_external_material,
                intMaterial = bristle_internal_material,
                color       = (0.5,0.5,0.5),
            )
            bristle_root_Ids = bristle_node_Ids[-1]
            all_bristle_root_ids.append(bristle_root_Ids)

            if root_fixed_Q:
                O.bodies[bristle_root_Ids].state.blockedDOFs = 'xyzXYZ'

    if(root_fixed_Q):
        return bristle_cyl_Ids, bristle_node_Ids

    else:
        return bristle_cyl_Ids, bristle_node_Ids, all_bristle_root_ids

            
# ----------------------------------------------------------------------------------------------
def fft_scaled(t, x):
    '''
    Returns the fft with the correct amplitude and the list of frequencies
    '''
    dt    = t[1] - t[0]
    fmax  = 1/(2*dt)
    n     = int(len(x)/2)

    f       = np.linspace(0, fmax, n)
    fft_x   = np.fft.fft(x) 
    fft_x_1   = fft_x[0:n]
    return f, 2*fft_x_1/len(x)

# ---------------------------------------------------------------------------------------------- brush_from_blender
def brush_from_blender(
    bristle_radius,
    bristle_lenght, 
    bristle_root_array,
    bristle_root_centre,
    bristle_segment_no,
    bristle_external_material,
    bristle_internal_material,
    bristle_root_arrar_scale = 1, # This scales the root grid
    bristle_axis             = "file", # if file, uses the normal data. Else use a quaeternion to define bristle diretion
    brush_orientation        = Quaternion((1,0,0),0), # This is axis(unit vector)-angle(in rads) notation
    root_fixed_Q             = True, # If true the roots will be fixed to the ground
):
    '''
    Creates a a brush based on the position of the roots. The root positions are defined using blender
    '''

    # initializing the arrays
    bristle_array         = []
    bristle_node_Ids      = []
    bristle_cyl_Ids       = []
    all_bristle_root_ids  = []

    # defines the length of each section
    bristle_segment_lenght = bristle_lenght/bristle_segment_no
    
    # interate for all points in the file
    for i in bristle_root_array:
        bristle_root_position = [
            bristle_root_arrar_scale*i[0], 
            bristle_root_arrar_scale*i[1], 
            bristle_root_arrar_scale*i[2]
            ]

        # gets the direction of the bristles either defined by normals or by the user
        if bristle_axis == "file":     
            bristle_root_normal   = [i[3], i[4], i[5]]

        elif bristle_axis == -1:
            bristle_root_normal   = [-i[3], -i[4], -i[5]]

        else:
            bristle_root_normal   = bristle_axis

        # This appended blank array will store the position of the nodes of a single bristle.
        # The array of array, bristle array, will store the all nodes of the brush, with each
        # sub array being a bristle
        bristle_array.append([])

        for k in range(bristle_segment_no + 1):
            pos = (
                    bristle_root_position[0] + (bristle_lenght - k*bristle_segment_lenght)*bristle_root_normal[0],
                    bristle_root_position[1] + (bristle_lenght - k*bristle_segment_lenght)*bristle_root_normal[1],
                    bristle_root_position[2] + (bristle_lenght - k*bristle_segment_lenght)*bristle_root_normal[2],
                )
            pos = brush_orientation*pos

            pos = (
                    pos[0] + bristle_root_centre[0],
                    pos[1] + bristle_root_centre[1],
                    pos[2] + bristle_root_centre[2],
                )
            
            # Appending the position of the node to the latest sub array
            bristle_array[-1].append(pos)

        cylinderConnection( 
            bristle_array[-1],
            radius   = bristle_radius,
            nodesIds = bristle_node_Ids,
            cylIds   = bristle_cyl_Ids,
            extMaterial = bristle_external_material,
            intMaterial = bristle_internal_material,
            color       = (0.5,0.5,0.5)
        )
        
        bristle_root_Id = bristle_node_Ids[-1]
        all_bristle_root_ids.append(bristle_root_Id)            

        if (root_fixed_Q):
            O.bodies[bristle_root_Id].state.blockedDOFs = 'xyzXYZ'

    if(root_fixed_Q):
        return bristle_cyl_Ids, bristle_node_Ids

    else:
        return bristle_cyl_Ids, bristle_node_Ids, all_bristle_root_ids

# ---------------------------------------------------------------------------------------------- L2_norm
def L2_norm(vector):
    mag = 0
    for i in vector:
        mag += i**2

    return mag**0.5

# ---------------------------------------------------------------------------------------------- normalise
def normalise(vector):
    mag = L2_norm(vector)
    if mag == 0:
        return [0,0,0]
    res = [i/mag for i in vector]
    return res

# ---------------------------------------------------------------------------------------------- constant array
def constant_Array(constant, length):
    res = []
    for i in range(length):
        res += [constant]
    return res

# ---------------------------------------------------------------------------------------------- conicalMaterial
def conicalMaterial(Material, radius_1, radius_2, no_of_nodes):
    '''
    Used for cylinder connections. Creates a list of materials who Youngs modulus varies such that the bending stiffness of the cylinder varies like it would for a frustom or a cylinder with increaing radius
    radius_1: Initial radius
    radius_2: Final radius
    Material: The base material. The material project at radius_1
    no_of_nodes: Number of nodes in the cylinder connections

    '''
    r2r1 = radius_2 / radius_1    
    if no_of_nodes < 2:
        raise Exception("conicalMaterial error: insufficient number of vertices")
        return

    radii_list  = [lerp(r2r1, 1, i/(no_of_nodes - 1)) for i in range(no_of_nodes)]

    mat_list    = []
    int_young   = O.materials[Material].young
    int_density = O.materials[Material].density
    int_poisson = O.materials[Material].poisson
    int_frict   = O.materials[Material].frictionAngle

    ## creating the materials. This is how we simulate the change in radius
    for i in range(no_of_nodes):
        mat_name = "int_mat" + str(i+1)
        mat_list.append(mat_name)
        O.materials.append(
            CohFrictMat(
                young   = int_young * (radii_list[i])**4,
                poisson = int_poisson,
                density = int_density,
                label   = mat_name,

                frictionAngle     = int_frict,
                momentRotationLaw = True,
                normalCohesion    = 1e40,
                shearCohesion     = 1e40,
            )
        )

    return mat_list


# ---------------------------------------------------------------------------------------------- conicalCylinderConnections
def conicalCylinderConnections(vertices,radius=0.2,nodesIds=[],cylIds=[],dynamic=None,fixed=False,wire=False,color=None,highlight=False,intMaterialList=-1,extMaterial=-1,mask=1):
    '''
    Used to model the bending properties of a conical cylinder (frustum). Although the radius remains constant, the variation in radius in terms of the change in bending stiffness is modelled by varying the youngs modulus.
    '''
    
    no_of_nodes =  len(vertices)
    no_of_mats  = len(intMaterialList)

    if no_of_nodes < 2:
        raise Exception("conicalCylinderConnections error: insufficient number of vertices")
        return
    if no_of_mats!= no_of_nodes:
        raise Exception("conicalCylinderConnections error: No of intMaterials do not match the number of nodes")
        return

	# create all gridNodes first
    nodesIdsCC=[]
    for i in range(no_of_nodes):
	    nodesIdsCC.append(
            O.bodies.append(
                gridNode(
                    vertices[i],
                    radius  = radius,
					dynamic = dynamic,
                    fixed = fixed,
                    wire  = wire,
                    color = color,
                    highlight = highlight,
                    material  = intMaterialList[i]
                )
            )
        )

    nodesIds.extend(nodesIdsCC)

	# now create connection between the gridNodes
    for i,j in zip( nodesIdsCC[:-1], nodesIdsCC[1:]):
	    cylIds.append( O.bodies.append( gridConnection(i,j,radius=radius,
						wire=wire,color=color,highlight=highlight,material=extMaterial,mask=mask,cellDist=None)) )


# ---------------------------------------------------------------------------------------------- hcp_brush
class hcp_brush():
    '''
    The class the create a brush in which the bristles follow a hexagonal close packing
    '''
    # -------------------------------------------------------------- constructor
    def __init__(
        self,
        bristle_radius = 1e-4, 
        bristle_length = 10e-3,
        no_of_segment  = 5, 
        bristle_external_material = 'default_ext_mat',
        bristle_internal_material = 'default_int_mat',        

        x_lenght = 10e-3, 
        y_length = 10e-3, 
        no_density = 1/(2e-3), 
        bristle_tip_spread_covariance = [[1e-8, 0],[0, 1e-8]],# 'default',
        clearance = 1e-5,
        brush_base_pos = [0,0,0],
        orientation = Quaternion((1,0,0),0), # This is axis(unit vector)-angle(in raself.ds) notation)

        root_fixed_Q = True, # If true the roots will be fixed to the ground
    ):
        self.bristle_radius = bristle_radius 
        self.bristle_length = bristle_length
        self.no_of_segment = no_of_segment 
        self.bristle_external_material = bristle_external_material
        self.bristle_internal_material = bristle_internal_material        

        self.x_lenght = x_lenght 
        self.y_length = y_length 
        self.no_density = no_density 
        self.covariance = bristle_tip_spread_covariance
        self.clearance  = clearance
        self.brush_base_pos = brush_base_pos
        self.orientation    = orientation
        self.root_fixed_Q   = root_fixed_Q

        self.dx = 1 / self.no_density
        self.dy = self.dx * (3**0.5 / 2)
        self.nx = int(math.floor(self.x_lenght / self.dx))
        self.ny = int(math.floor(self.y_length / self.dy))
        self.ds = self.bristle_length / self.no_of_segment

        if self.ds < 2.5*self.bristle_radius:
            print("[WARNING]: hcp_brush :: Too high number of segments per bristle")

        # Since the HCP has staggered arrangement, there can be a case where haveing self.nx for the staggered row will exceed the brush 
        # brush self.x_lenght. The follwing corrects this 
        if (self.nx + 0.5)*self.dx < self.x_lenght:
            self.nx_dash = self.nx
        else:
            self.nx_dash = self.nx - 1

        if bristle_external_material == 'default_ext_mat':
            O.materials.append(
                FrictMat(
                    young   = 1e8,
                    poisson = 0.3,
                    density = 1000,
                    label   = 'default_ext_mat',
                    frictionAngle = radians(30),
                )
            )

            O.materials.append(
                CohFrictMat(
                    young   = 1e8,
                    poisson = 0.3,
                    density = 1000,
                    label   = 'default_int_mat',

                    frictionAngle = radians(30),
                    momentRotationLaw = True,
                    normalCohesion    = 1e40,
                    shearCohesion     = 1e40,
                )
            )

    # -------------------------------------------------------------- generate_roots_pos_array
    def generate_roots_pos_array(self):
        '''
        Generating the root position array
        '''
        root_array_x = [self.dx*i for i in range(self.nx)]
        self.root_array   = []
        self.array_length = 0

        for j in range(self.ny):

            # if else used to create the staggered rows of hexagonal closed packing
            if j % 2 == 0:
                for i in range(self.nx):
                    self.root_array.append([root_array_x[i], j * self.dy, 0])
                    self.array_length += 1

            else:
                for i in range(self.nx_dash):
                    self.root_array.append([root_array_x[i] + self.dx/2, j * self.dy, 0])
                    self.array_length += 1

    # -------------------------------------------------------------- generate_bristle_direction_vector
    def generate_bristle_direction_vector(self):
        self.normals_array = constant_Array(0, self.array_length)
        mean = [0, 0]
        
        x_var, y_var = np.random.multivariate_normal(mean, self.covariance, self.array_length).T

        # The bristles are supposed to be within a space such that they do not collide with other bristles
        # This section checks if there could be potential for overlap and notfies the user
        # Calulate the maximum tip displacement of the tip. Here dispacement is the difference of the generated tip
        # from the ideal straight tip  
        max = 0
        for i in range(self.array_length):
            norm = L2_norm([x_var[i], y_var[i]])
            if max < norm:
                max = norm

        max_bristle_tip_disp = max #

        if max_bristle_tip_disp + self.bristle_radius + self.clearance/2 >= self.dx:
            print("[WARNING]: brush_HCP_creator :: Max tip displacement due to randomness greater than distance between two roots")

        # create the list of vectors
        for i in range(self.array_length):
            z = (self.bristle_length**2 - x_var[i]**2 - y_var[i]**2)**0.5

            if (x_var[i]**2 + y_var[i]**2)**0.5 > self.bristle_length:
                z = 0
            
            self.normals_array[i] = [ x_var[i], y_var[i], z ]
            self.normals_array[i] = normalise(self.normals_array[i])
    
    # -------------------------------------------------------------- generate_brislte_nodes_array
    def generate_brislte_nodes_array(self):
        self.bristle_node_list = []

        for i in range(self.array_length):
            nor      = self.normals_array[i]
            root_pos = self.root_array[i]

            self.bristle_node_list.append(
                [self.orientation * (
                    (self.bristle_length - self.ds * k) * nor[0] + root_pos[0] ,
                    (self.bristle_length - self.ds * k) * nor[1] + root_pos[1] ,
                    (self.bristle_length - self.ds * k) * nor[2] + root_pos[2] ,
                ) + Vector3([
                    self.brush_base_pos[0],
                    self.brush_base_pos[1],
                    self.brush_base_pos[2]
                ])for k in range(self.no_of_segment + 1)]
            ) 

    # -------------------------------------------------------------- generate_brush
    def brush_generator(self):
        '''
        Generates a brush from the bristle nodes list and returns the YADE body ids
        '''
        node_ids_array = []
        cyl_ids_array  = []
        root_node_ids_array = []

        for i in self.bristle_node_list:
            cylinderConnection( 
                i,
                radius   = self.bristle_radius,
                nodesIds = node_ids_array,
                cylIds   = cyl_ids_array,
                extMaterial = self.bristle_external_material,
                intMaterial = self.bristle_internal_material,
                color       = (0.5,0.5,0.5)
            )

            root_node_ids_array.append(node_ids_array[-1])

        if (self.root_fixed_Q):
            for i in root_node_ids_array:
                O.bodies[i].state.blockedDOFs = 'xyzXYZ'

        return node_ids_array, cyl_ids_array, root_node_ids_array

    # -------------------------------------------------------------- generate
    def generate(self):
        '''
        generate the brush
        '''
        self.generate_roots_pos_array()
        self.generate_bristle_direction_vector()
        self.generate_brislte_nodes_array()

        return self.brush_generator()

# ---------------------------------------------------------------------------------------------- conicalConnection
def conicalConnection(
    vertices,            radius_1  = 0.2,    radius_2    = 0.2,    nodesIds    = [],
    cylIds      = [],    dynamic   = None,   fixed       = False,  wire        = False,
    color       = None,  highlight = False,  intMaterial = -1,     extMaterial = -1,
    tipMaterial = -1,    mask      = 1
    ):
    """
    Create a chain of cylinders which forms a cone with given parameters. The cylinders (:yref:`GridConnection<GridConnection>`) and its corresponding nodes (yref:`GridNodes<GridNode>`) are automatically added to the simulation. The lists with nodes and cylinder ids will be updated automatically.

    :param [[Vector3]] vertices: coordinates of vertices to connect in the global coordinate system.
    
    See :yref:`yade.gridpfacet.cylinder` documentation for meaning of other parameters.

    Modified to be able to simulate a conical cylinder

    """
    # create all gridNodes first -------------------------------------------------------
    nodesIdsCC  = []
    numberNodes = len(vertices)
    radii = [lerp(radius_1, radius_2, i/(numberNodes - 1)) for i in range(numberNodes)]
    l     = 0 # dummy index
    # ----------------------------------------------------------------------------------
    # assigning the material of the tip. If it is user specified, then assigning it to the function member m_tipMaterial, else
    # assign the intMaterial to m_tipMaterial 
    if tipMaterial == -1:
        m_tipMaterial = intMaterial

    else:
        m_tipMaterial = tipMaterial
    # ----------------------------------------------------------------------------------

    # The tip node ------------------------------------------------
    i = vertices[0]
    nodesIdsCC.append(
        O.bodies.append(
            gridNode(
                i, radii[l], dynamic, fixed, wire, color, highlight, m_tipMaterial
            )
        ) 
    )
    l +=1

    # all other nodes ---------------------------------------------
    for i in vertices[1:]:
        nodesIdsCC.append(
            O.bodies.append(
                gridNode(
                    i, radii[l], dynamic, fixed, wire, color, highlight, intMaterial
                )
            ) 
        )
        l +=1

    nodesIds.extend(nodesIdsCC)
    # ----------------------------------------------------------------------------------
    
    # The cylinders must be generated based on the radius of the smaller node in the pair of nodes at the end of the cylinder. There are
    # two cases: the first is larger than second and first is smaller than the second. If tip is larger, then the radius of the first
    # cylinder is based on the second node. If tips is smaller, then the radius of the cylinder is based on the first node. 
    if radius_1 < radius_2:
        k = 0
    else:
        k = 1
    # ----------------------------------------------------------------------------------

    # now create connection between the gridNodes
    for i,j in zip( nodesIdsCC[:-1], nodesIdsCC[1:]):
        cylIds.append( 
            O.bodies.append( 
                gridConnection(
                    i,j,
                    radius = radii[k],
                    wire   = wire,
                    color  = color,
                    highlight = highlight,
                    material  = extMaterial,
                    mask      = mask,
                    cellDist  = None
                )
            ) 
        )
        k += 1

# ---------------------------------------------------------------------------------------------- ccp_brush
class ccp_brush():
    '''
    The class the create a brush in which the bristles follow a square close packing
    '''
    # -------------------------------------------------------------- constructor
    def __init__(
        self,
        bristle_radius   = 1e-4, 
        bristle_radius_2 = -1,
        bristle_length   = 10e-3,
        no_of_segment    = 5, 
        bristle_external_material = 'default_ext_mat',
        bristle_internal_material = 'default_int_mat',
        bristle_tip_material = -1,        

        x_lenght = 10e-3, 
        y_length = 10e-3, 
        x_no_density = 1/(2e-3), 
        y_no_density = 1/(2e-3), 
        bristle_tip_spread_covariance = [[1e-8, 0],[0, 1e-8]],# 'default',
        clearance = 1e-5,
        brush_base_pos = [0,0,0],
        orientation = Quaternion((1,0,0),0), # This is axis(unit vector)-angle(in raself.ds) notation)

        root_fixed_Q = True, # If true the roots will be fixed to the ground
    ):
        self.bristle_radius   = bristle_radius 
        self.bristle_radius_2 = bristle_radius_2
        self.bristle_length   = bristle_length
        self.no_of_segment    = no_of_segment 
        self.bristle_external_material = bristle_external_material
        self.bristle_internal_material = bristle_internal_material  
        self.bristle_tip_material      = bristle_tip_material

        self.x_lenght = x_lenght 
        self.y_length = y_length 
        self.x_no_density = x_no_density
        self.y_no_density = y_no_density 
        self.covariance = bristle_tip_spread_covariance
        self.clearance  = clearance
        self.brush_base_pos = brush_base_pos
        self.orientation    = orientation
        self.root_fixed_Q   = root_fixed_Q

        self.dx = 1 / self.x_no_density
        self.dy = 1 / self.y_no_density 
        self.nx = int(math.floor(self.x_lenght / self.dx))
        self.ny = int(math.floor(self.y_length / self.dy))
        self.ds = self.bristle_length / self.no_of_segment

        if self.ds < 2.5*self.bristle_radius:
            print("[WARNING]: hcp_brush :: Too high number of segments per bristle")

        if bristle_external_material == 'default_ext_mat':
            O.materials.append(
                FrictMat(
                    young   = 1e8,
                    poisson = 0.3,
                    density = 1000,
                    label   = 'default_ext_mat',
                    frictionAngle = radians(30),
                )
            )

            O.materials.append(
                CohFrictMat(
                    young   = 1e8,
                    poisson = 0.3,
                    density = 1000,
                    label   = 'default_int_mat',

                    frictionAngle = radians(30),
                    momentRotationLaw = True,
                    normalCohesion    = 1e40,
                    shearCohesion     = 1e40,
                )
            )

    # -------------------------------------------------------------- generate_roots_pos_array
    def generate_roots_pos_array(self):
        '''
        Generating the root position array
        '''
        # root_array_x = [self.dx*i for i in range(self.nx)]
        # self.root_array   = []
        # self.array_length = 0

        # for j in range(self.ny):
        #     for i in range(self.nx):
        #         self.root_array.append([
        #             root_array_x[i] - (self.nx - 1)*self.dx/2, 
        #             j * self.dy     - (self.nx - 1)*self.dx/2, 
        #             0
        #             ])
        #         self.array_length += 1
        # root_array_x = [self.dx*i for i in range(self.nx)]
        self.root_array   = []
        self.array_length = 0

        for j in range(self.ny):
            for i in range(self.nx):
                self.root_array.append([
                    i * self.dx     - (self.nx - 1)*self.dx/2, 
                    j * self.dy     - (self.ny - 1)*self.dy/2, 
                    0
                    ])
                self.array_length += 1
    # -------------------------------------------------------------- generate_bristle_direction_vector
    def generate_bristle_direction_vector(self):
        self.normals_array = constant_Array(0, self.array_length)
        mean = [0, 0]
        
        x_var, y_var = np.random.multivariate_normal(mean, self.covariance, self.array_length).T

        # The bristles are supposed to be within a space such that they do not collide with other bristles
        # This section checks if there could be potential for overlap and notfies the user
        # Calulate the maximum tip displacement of the tip. Here dispacement is the difference of the generated tip
        # from the ideal straight tip  
        max = 0
        for i in range(self.array_length):
            norm = L2_norm([x_var[i], y_var[i]])
            if max < norm:
                max = norm

        max_bristle_tip_disp = max #

        if max_bristle_tip_disp + self.bristle_radius + self.clearance/2 >= min(self.dx, self.dy):
            print("[WARNING]: brush_HCP_creator :: Max tip displacement due to randomness greater than distance between two roots")

        # create the list of vectors
        for i in range(self.array_length):
            z = (self.bristle_length**2 - x_var[i]**2 - y_var[i]**2)**0.5

            if (x_var[i]**2 + y_var[i]**2)**0.5 > self.bristle_length:
                z = 0
            
            self.normals_array[i] = [ x_var[i], y_var[i], z ]
            self.normals_array[i] = normalise(self.normals_array[i])
    
    # -------------------------------------------------------------- generate_brislte_nodes_array
    def generate_brislte_nodes_array(self):
        self.bristle_node_list = []

        for i in range(self.array_length):
            nor      = self.normals_array[i]
            root_pos = self.root_array[i]

            self.bristle_node_list.append(
                [self.orientation * (
                    (self.bristle_length - self.ds * k) * nor[0] + root_pos[0] ,
                    (self.bristle_length - self.ds * k) * nor[1] + root_pos[1] ,
                    (self.bristle_length - self.ds * k) * nor[2] + root_pos[2] ,
                ) + Vector3([
                    self.brush_base_pos[0],
                    self.brush_base_pos[1],
                    self.brush_base_pos[2]
                ])for k in range(self.no_of_segment + 1)]
            )

    # -------------------------------------------------------------- generate_brush
    def brush_generator(self):
        '''
        Generates a brush from the bristle nodes list and returns the YADE body ids
        '''
        node_ids_array = []
        cyl_ids_array  = []
        root_node_ids_array = []

        for i in self.bristle_node_list:
            conicalConnection( 
                i,
                radius_1  = self.bristle_radius,
                radius_2  = self.bristle_radius_2,
                nodesIds   = node_ids_array,
                cylIds     = cyl_ids_array,
                extMaterial = self.bristle_external_material,
                intMaterial = self.bristle_internal_material,
                tipMaterial = self.bristle_tip_material,
                color       = (0.5,0.5,0.5)
            )

            root_node_ids_array.append(node_ids_array[-1])

        if (self.root_fixed_Q):
            for i in root_node_ids_array:
                O.bodies[i].state.blockedDOFs = 'xyzXYZ'

        return node_ids_array, cyl_ids_array, root_node_ids_array

    # -------------------------------------------------------------- generate
    def generate(self):
        '''
        generates the brush
        '''
        self.generate_roots_pos_array()
        self.generate_bristle_direction_vector()
        self.generate_brislte_nodes_array()

        return self.brush_generator()

# ---------------------------------------------------------------------------------------------- symmetric_cuboid
class symmetric_cuboid():
    '''
    Creates a symmetric cuboid (the grids are symmetric. Stores the vertex and face information in class variables
    How to use:
        Instantiate. The vertex and the face list will be created.
        Call the translate and rotate method is you want to change the vertex list
    '''

    def __init__(self, 
    dimension = [10e-3, 10e-3, 10e-3],
    position = [0, 0, 0], 
    orientation = Quaternion((1,0,0),0),
    scale = 1
    ):
        self.length  = scale * dimension[0]
        self.breadth = scale * dimension[1]
        self.height  = scale * dimension[2]
        self.position = position

        self.orientation = orientation
        self._create_vertex_list()
        self._create_face_list()
        

    # -------------------------------------------------------------- create_vertex_list
    def _create_vertex_list(self):
        self.vertex_list = [
            [+ self.length/2, + self.breadth/2, + self.height/2],
            [+ self.length/2, - self.breadth/2, + self.height/2],
            [- self.length/2, - self.breadth/2, + self.height/2],
            [- self.length/2, + self.breadth/2, + self.height/2],
            [+ self.length/2, + self.breadth/2, - self.height/2],
            [+ self.length/2, - self.breadth/2, - self.height/2],
            [- self.length/2, - self.breadth/2, - self.height/2],
            [- self.length/2, + self.breadth/2, - self.height/2],
            [+ self.length/2,                0,               0],
            [- self.length/2,                0,               0],
            [              0, + self.breadth/2,               0],
            [              0, - self.breadth/2,               0],
            [              0,                0, + self.height/2],
            [              0,                0, - self.height/2],      
        ]

        # tranaform
        for i in range(len(self.vertex_list)):
            self.vertex_list[i] = self.orientation * self.vertex_list[i]

            self.vertex_list[i] = [
                self.vertex_list[i][0] + self.position[0],
                self.vertex_list[i][1] + self.position[1],
                self.vertex_list[i][2] + self.position[2]
            ]

    # -------------------------------------------------------------- create_face_list
    def _create_face_list(self):
        self.face_list = [
            [12, 1, 0],[12, 2, 1],[12, 3, 2],[12, 0, 3],
            [13, 4, 5],[13, 5, 6],[13, 6, 7],[13, 7, 4],
            [10, 0, 4],[10, 4, 7],[10, 7, 3],[10, 3, 0],
            [11, 1, 2],[11, 2, 6],[11, 6, 5],[11, 5, 1],
            [8,  0, 1],[8,  1, 5],[8,  5, 4],[8,  4, 0],
            [9,  2, 3],[9,  3, 7],[9,  7, 6],[9,  6, 2],
        ]

    # -------------------------------------------------------------- update
    # def update(self):
    #     '''
    #     When you update the object variables, update the vertex.
    #     '''
    #     self.create_vertex_list()

    # -------------------------------------------------------------- rotate
    def rotate(self, ori):
        new_vertex_list= []
        for i in self.vertex_list:
            new_vertex_list.append(
                ori * i
            )
        
        self.vertex_list = new_vertex_list
    # -------------------------------------------------------------- translate
    def translate(self, vec):
        new_vertex_list = []
        for i in self.vertex_list:
            new_vertex_list.append([
                i[0] + vec[0],
                i[1] + vec[1],
                i[2] + vec[2],
            ])

        self.vertex_list = new_vertex_list
        self.position[0] += vec[0]
        self.position[1] += vec[1]
        self.position[2] += vec[2]

# ---------------------------------------------------------------------------------------------- pfacet_from_vertex_face
class pfacet_from_vertex_face():
    '''
    Create a pfacet body and stores the ids in the instance. Constructed using list of vertices and faces
    '''

    # -------------------------------------------------------------- constructor
    def __init__(self, 
    vertex_list = [[1,0,0],[1,1,0],[0,1,0]], 
    faces_list  = [[0,1,2]], 
    grid_radius = 1e-3, 
    internal_material = 'default_int_mat', 
    external_material = 'default_ext_mat',
    color = (0.5,0.5,0.5),
    wire_Q = False,
    fixed_Q = False
    ):
        self.color   = color
        self.wire_Q  = wire_Q
        self.fixed_Q = fixed_Q

        self.vertex_list = vertex_list
        self.faces_list  = faces_list
        self.grid_radius = grid_radius

        self.internal_material = internal_material
        self.external_material = external_material        

        if external_material == 'default_ext_mat':
            O.materials.append(
                FrictMat(
                    young   = 1e8,
                    poisson = 0.3,
                    density = 1000,
                    label   = 'default_ext_mat',
                    frictionAngle = radians(30),
                )
            )

            O.materials.append(
                CohFrictMat(
                    young   = 1e8,
                    poisson = 0.3,
                    density = 1000,
                    label   = 'default_int_mat',

                    frictionAngle = radians(30),
                    momentRotationLaw = True,
                    normalCohesion    = 1e40,
                    shearCohesion     = 1e40,
                )
            )

        self.create_pfacet_body()

    # -------------------------------------------------------------- create_pfacet_body
    def create_pfacet_body(self):
        self.node_id_list   = []
        self.cyl_id_list    = []
        self.pfacet_id_list = []

        for i in self.faces_list:
            pfacetCreator1(
                [
                    self.vertex_list[i[0]], 
                    self.vertex_list[i[1]], 
                    self.vertex_list[i[2]]
                ], 
                radius = self.grid_radius, 
                nodesIds = self.node_id_list, 
                cylIds   = self.cyl_id_list, 
                pfIds    = self.pfacet_id_list, 
                wire     = self.wire_Q, 
                fixed    = self.fixed_Q, 
                color    = self.color,
                materialNodes = self.internal_material, 
                material      = self.external_material, 
                )

        self.ids = self.node_id_list + self.cyl_id_list + self.pfacet_id_list

# ---------------------------------------------------------------------------------------------- get_AABB
def get_AABB(vertex_list):
    '''
    Returns the AABB of a shape defined by the vertex list
    '''
    x_list = []
    y_list = []
    z_list = []

    for i in vertex_list:
        x_list.append(i[0])
        y_list.append(i[1])
        z_list.append(i[2])

    return [
        [min(x_list), max(x_list)],
        [min(y_list), max(y_list)],
        [min(z_list), max(z_list)]
    ]

# ---------------------------------------------------------------------------------------------- get_engines
def get_engines(dampingFraction = 0.0, CW_Q = False, angular_vel_orbit = 0.001133, gravity = (0.0, 0.0, 0.0), numerical_damping = 0.0, yade_version = 'official'):
    '''
    Returns the engines for a brush/target interaction in space
    '''
    if yade_version == 'official':
        # ------------------------------------------------------------------------------- official version of yade
        res = [
                ForceResetter(),

                InsertionSortCollider([
                    Bo1_GridConnection_Aabb(),
                    Bo1_PFacet_Aabb(),
                    Bo1_Sphere_Aabb(),
                ]),

                InteractionLoop(
                    [
                        Ig2_PFacet_PFacet_ScGeom(),
                        Ig2_GridConnection_GridConnection_GridCoGridCoGeom(),
                        Ig2_GridNode_GridNode_GridNodeGeom6D(),
                        Ig2_GridConnection_PFacet_ScGeom(),
                        Ig2_Sphere_PFacet_ScGridCoGeom(),
                    ],
                    [
                        Ip2_FrictMat_FrictMat_FrictPhys(),
                        Ip2_CohFrictMat_CohFrictMat_CohFrictPhys(
                            setCohesionNow = True, 
                            setCohesionOnNewContacts = False
                            ),
                    ],
                    [
                        Law2_GridCoGridCoGeom_FrictPhys_CundallStrack(),
                        Law2_ScGeom_FrictPhys_CundallStrack(),
                        Law2_ScGridCoGeom_FrictPhys_CundallStrack(),
                        Law2_ScGeom6D_CohFrictPhys_CohesionMoment(),
                    ],
                ),
                NewtonIntegrator(gravity = gravity, damping = numerical_damping)
            ] 

    else: 
        # ------------------------------------------------------------------------------- RKJ version
        res = [
            ForceResetter(),

            InsertionSortCollider([
                Bo1_GridConnection_Aabb(),
                Bo1_PFacet_Aabb(),
                Bo1_Sphere_Aabb(),
            ]),

            InteractionLoop(
                [
                    Ig2_PFacet_PFacet_ScGeom(),
                    Ig2_GridConnection_GridConnection_GridCoGridCoGeom(),
                    Ig2_GridNode_GridNode_GridNodeGeom6D(),
                    Ig2_GridConnection_PFacet_ScGeom(),
                    Ig2_Sphere_PFacet_ScGridCoGeom(),
                ],
                [
                    Ip2_FrictMat_FrictMat_FrictPhys(),
                    Ip2_CohFrictMat_CohFrictMat_CohFrictPhys(
                        setCohesionNow = True, 
                        setCohesionOnNewContacts = False
                        ),
                ],
                [
                    Law2_GridCoGridCoGeom_FrictPhys_CundallStrack(),
                    Law2_ScGeom_FrictPhys_CundallStrack(),
                    Law2_ScGridCoGeom_FrictPhys_CundallStrack(),
                    Law2_ScGeom6D_CohFrictPhys_CohesionMoment(
                        dampingFraction = dampingFraction
                        ),
                ],
            ),
        ]   

        if CW_Q:
            res.append(ClohessyWiltshireEngine(orbit_ang_vel = angular_vel_orbit))

        res.append(NewtonIntegrator(gravity = gravity, damping = numerical_damping),)

    return res

# ---------------------------------------------------------------------------------------------- add_brush_material
def add_brush_material(young, poisson, density, friction_Angle, ext_mat_name, int_mat_name, viscousDamping):
    O.materials.append(
        FrictMat(
            young   = young,
            poisson = poisson,
            density = density,
            label   = ext_mat_name,
            frictionAngle = friction_Angle,
        )
    )

    O.materials.append(
        CohFrictMat(
            young   = young,
            poisson = poisson,
            density = density,
            label   = int_mat_name,
            dampingFraction = viscousDamping,

            frictionAngle     = friction_Angle,
            momentRotationLaw = True,
            normalCohesion    = 1e40,
            shearCohesion     = 1e40,
        )
    )

# ---------------------------------------------------------------------------------------------- add_pfacet_material
def add_pfacet_material(young, poisson, density, friction_Angle, int_mat_name, ext_mat_name):
    O.materials.append(
        FrictMat(
            young   = young,
            poisson = poisson,
            density = density,
            label   = ext_mat_name,
            frictionAngle = friction_Angle,
        )
    )

    O.materials.append(
        CohFrictMat(
            young   = young,
            poisson = poisson,
            density = density,
            label   = int_mat_name,

            frictionAngle     = friction_Angle,
            normalCohesion    = 3e100,
            shearCohesion     = 3e100,
            momentRotationLaw = True,
        )
    )

# ---------------------------------------------------------------------------------------------- add_official_brush_material
def add_official_brush_material(young, poisson, density, friction_Angle, ext_mat_name, int_mat_name, friction_Angle_2 = -1):
    O.materials.append(
        FrictMat(
            young   = young,
            poisson = poisson,
            density = density,
            label   = ext_mat_name,
            frictionAngle = friction_Angle,
        )
    )

    if friction_Angle_2 < 0:
        friction_Angle_int = friction_Angle

    else:
        friction_Angle_int = friction_Angle_2

    O.materials.append(
        CohFrictMat(
            young   = young,
            poisson = poisson,
            density = density,
            label   = int_mat_name,

            frictionAngle     = friction_Angle_int,
            momentRotationLaw = True,
            normalCohesion    = 1e40,
            shearCohesion     = 1e40,
        )
    )

# ---------------------------------------------------------------------------------------------- add_official_pfacet_material
def add_official_pfacet_material(young, poisson, density, friction_Angle, int_mat_name, ext_mat_name):
    O.materials.append(
        FrictMat(
            young   = young,
            poisson = poisson,
            density = density,
            label   = ext_mat_name,
            frictionAngle = friction_Angle,
        )
    )

    O.materials.append(
        CohFrictMat(
            young   = young,
            poisson = poisson,
            density = density,
            label   = int_mat_name,

            frictionAngle     = friction_Angle,
            normalCohesion    = 3e100,
            shearCohesion     = 3e100,
            momentRotationLaw = True,
        )
    )

# ---------------------------------------------------------------------------------------------- add motion engines
def add_motion_engines(ids, linear_velocity, angular_velocity, center_of_rotation):
    '''
    Adds the engines which sets the initial velocities of a target. This is done because for some reason we cannot graph the velocities for non spherical objects by manually setting the velocities
    '''
    linear_velocity_magnitude = L2_norm(linear_velocity)
    linear_velocity_vector    = normalise(linear_velocity)

    angular_velocity_magnitude = L2_norm(angular_velocity)
    angular_velocity_vector    = normalise(angular_velocity)

    O.engines += [
        CombinedKinematicEngine(
            ids   = ids,
            label ='combined_motion_engine') + 
            
        TranslationEngine(
            translationAxis = linear_velocity_vector,
            velocity        = linear_velocity_magnitude,
            label           = 'translation_engine') + 
                
        RotationEngine(
            rotationAxis     = angular_velocity_vector, 
            angularVelocity  = angular_velocity_magnitude, 
            rotateAroundZero = True, 
            zeroPoint        = center_of_rotation,
            label            = 'rotation_engine'
            ),

        PyRunner(command = "motion_engine_stopper()", iterPeriod = 1, label = "stopper")
        
    ]

# ---------------------------------------------------------------------------------------------- motion_engine_stopper
def motion_engine_stopper():
    combined_motion_engine.dead = True
    translation_engine.dead = True
    rotation_engine.dead    = True
    stopper.iterPeriod      = int(1e14)
    print("Engines stopped")

# ---------------------------------------------------------------------------------------------- brush_from_blender
class blender_brush():
    '''
    Creates a brush using blender. Blender used to create the root positions in terms of vertices
    '''

    # -------------------------------------------------------------- constructor
    def __init__(
        self,    
        bristle_radius,
        bristle_length, 
        bristles_data_from_blender,
        bristle_segment_no,
        bristle_external_material,
        bristle_internal_material,
        brush_base_pos,
        bristle_tip_spread_covariance = [[0, 0, 0],[0, 0, 0],[0, 0, 0]],
        bristle_root_arrar_scale = 1, # This scales the root grid
        bristles_direction       = "file", # if file, uses the normal data. Else use a quaeternion to define bristle diretion
        orientation              = Quaternion((1,0,0),0), # This is axis(unit vector)-angle(in rads) notation
        root_fixed_Q             = True, # If true the roots will be fixed to the ground):
    ):

        self.bristle_radius = bristle_radius
        self.bristle_length = bristle_length
        self.bristles_data_from_blender  = bristles_data_from_blender
        self.no_of_segment       = bristle_segment_no
        self.bristle_external_material = bristle_external_material
        self.bristle_internal_material = bristle_internal_material
        self.bristle_root_arrar_scale  = bristle_root_arrar_scale
        self.bristles_direction        = bristles_direction            
        self.orientation       = orientation    
        self.root_fixed_Q      = root_fixed_Q
        self.covariance        = bristle_tip_spread_covariance
        self.brush_base_pos    = brush_base_pos
        

        self.array_length = len(bristles_data_from_blender)

    # -------------------------------------------------------------- generate_roots_pos_array
    def generate_roots_pos_array(self):
        # initializing the arrays
        self.bristle_array         = []
        self.bristle_node_Ids      = []
        self.bristle_cyl_Ids       = []
        self.bristle_root_ids      = []
        self.root_array = []

        self.ds = self.bristle_length / self.no_of_segment

        # Extracting the roots from the input data
        for i in self.bristles_data_from_blender:
            self.root_array += [[
                self.bristle_root_arrar_scale * i[0], 
                self.bristle_root_arrar_scale * i[1], 
                self.bristle_root_arrar_scale * i[2]
                ]]

    # -------------------------------------------------------------- generate_bristle_direction_vector
    def generate_bristle_direction_vector(self):
        mean = [0,0,0]
        x_var, y_var, z_var = np.random.multivariate_normal(mean, self.covariance, self.array_length).T
        self.normals_array  = []

        for i in range(len(self.bristles_data_from_blender)):
            # gets the direction of the bristles either defined by normals or by the user
            bristle_data = self.bristles_data_from_blender[i]

            if self.bristles_direction   == "file":     
                bristle_root_normal   = [ bristle_data[3],  bristle_data[4],  bristle_data[5]]

            elif self.bristles_direction == -1:
                bristle_root_normal   = [-bristle_data[3], -bristle_data[4], -bristle_data[5]]

            else:
                bristle_root_normal   = self.bristles_direction

            bristle_root_normal = [
                bristle_root_normal[0] + x_var[i],
                bristle_root_normal[1] + y_var[i],
                bristle_root_normal[2] + z_var[i],
            ]
            self.normals_array.append(bristle_root_normal)


    # -------------------------------------------------------------- generate_brislte_nodes_array
    def generate_brislte_nodes_array(self):
        self.bristle_node_list = []

        for i in range(self.array_length):
            nor      = self.normals_array[i]
            root_pos = self.root_array[i]

            self.bristle_node_list.append(
                [self.orientation * (
                    (self.bristle_length - self.ds * k) * nor[0] + root_pos[0] + self.brush_base_pos[0],
                    (self.bristle_length - self.ds * k) * nor[1] + root_pos[1] + self.brush_base_pos[1],
                    (self.bristle_length - self.ds * k) * nor[2] + root_pos[2] + self.brush_base_pos[2],
                ) for k in range(self.no_of_segment + 1)]
            )
        
    # -------------------------------------------------------------- generate_brush
    def brush_generator(self):
        '''
        Generates a brush from the bristle nodes list and returns the YADE body ids
        '''
        node_ids_array = []
        cyl_ids_array  = []
        root_node_ids_array = []

        for i in self.bristle_node_list:
            cylinderConnection( 
                i,
                radius   = self.bristle_radius,
                nodesIds = node_ids_array,
                cylIds   = cyl_ids_array,
                extMaterial = self.bristle_external_material,
                intMaterial = self.bristle_internal_material,
                color       = (0.5,0.5,0.5)
            )

            root_node_ids_array.append(node_ids_array[-1])

        if (self.root_fixed_Q):
            for i in root_node_ids_array:
                O.bodies[i].state.blockedDOFs = 'xyzXYZ'

        return node_ids_array, cyl_ids_array, root_node_ids_array

    # -------------------------------------------------------------- generate
    def generate(self):
        '''
        generate the brush
        '''
        self.generate_roots_pos_array()
        self.generate_bristle_direction_vector()
        self.generate_brislte_nodes_array()

        return self.brush_generator()

# -------------------------------------------------------------- constant_array_xd
def constant_array_xd(constant, dimension):
    '''
    returns a constant array fo size defined by dimension
    '''
    if len(dimension) == 1:
        return constant_Array(constant, dimension[0])

    inter_array = constant_array_xd(constant, dimension[1:])
    return constant_Array(inter_array, dimension[0])

# -------------------------------------------------------------- transpose
def transpose(matrix):
    return np.array(matrix).T.tolist()

# -------------------------------------------------------------- removed_list
def removed_list(array, index):
    '''
    Accepts a list and return a list with the ith element removed
    '''

    res_list = []
    length   = len(array)

    for i in range(index):
        res_list.append(array[i])

    for i in range(index + 1, length):
        res_list.append(array[i])

    return res_list 

# -------------------------------------------------------------- combinator
def combinator(parameter_list):
    '''
    Accepts a two dimensional array list. It is a list of list in which each list is a list of values for a paramter. This function returns all possible combination of parameters.
    '''

    if parameter_list[-1] == parameter_list[0]:
        return transpose(parameter_list)

    comboed_for_i1_to_n = combinator(parameter_list[1:])
    comboed_for_i_to_n  = []

    for i in parameter_list[0]:
        for j in comboed_for_i1_to_n:
            temp = [i] + j
            comboed_for_i_to_n.append(temp)

    return comboed_for_i_to_n

# -------------------------------------------------------------- sensitivity_list
def sensitivity_list(parameter_list):
    '''
    Returns a list of values for parameter by chaning the one parameter at a time. Parameters are in different rows
    '''

    no_of_parameters = len(parameter_list)
    default_value = []
    for i in parameter_list:
        default_value.append(i[0])

    sensitivity_array = [default_value[:]]

    for i in range(no_of_parameters):
        single_sim_values     = []
        for k in default_value:
            single_sim_values.append(k)

        for j in range(1,len(parameter_list[i])):
            single_sim_values[i] = parameter_list[i][j]
            sensitivity_array.append(single_sim_values[:])

    return sensitivity_array

# -------------------------------------------------------------- alphabet_Q
def alphabet_Q(char):
    return (char >= 'a' and char <= 'z') or (char >= 'A' and char <= 'Z')

# -------------------------------------------------------------- get_angular_vel
def get_angular_vel(linear_velocity, position, centre):
    if len(centre) is not 3:
        raise Exception("[ERROR] get_angular_vel: please input a 3 vector. Currect dim = ", len(centre))
    
    if len(position) is not len(centre):
        print("[ERROR] get_angular_vel: vector dimension mismatch. Len(position) = ", len(position), " Len(center) = ", len(centre))
        return
    
    if len(linear_velocity) is not len(centre):
        print("[ERROR] get_angular_vel: vector dimension mismatch. Len(linear_velocity) = ", len(linear_velocity), " Len(center) = ", len(centre))
        return

    relative_position     = [position[i] - centre[i] for i in range(len(centre))]    
    angular_velocity_dir  = cross_prd(relative_position,linear_velocity) 
    angular_velocity      = [i / (L2_norm(relative_position))**2 for i in angular_velocity_dir]

    return angular_velocity

# -------------------------------------------------------------- list_dot_product
def list_dot_product(list_1, list_2):
    if len(list_1) is not len(list_2):
        raise Exception("[ERROR] list_dot_product: list dimesions mismatch. Len(list_1) = ", len(list_1), " len(list_2) = ", len(list_2))
        return

    dot = 0
    for i in range(len(list_1)):
        dot += list_1[i] * list_2[i]

    return dot

# -------------------------------------------------------------- get_angular_vel_2
def get_angular_vel_2(pos, vel):
    pos_matrix = np.array(pos).T
    vel_matrix = np.array(vel).T

    omega = vel_matrix.dot(
        np.linalg.inv(pos_matrix)
    )

    w = [
        -omega[1,2],
        omega[0,2],
        -omega[0,1],
        ]
    return w


###-------------temp

def get_pfacet_angular_velocity(ids):
    pos_cm = [0,0,0]
    vel_cm = [0,0,0]
    no_bodies = len(ids)

    for i in ids:
        pos_cm[0] += O.bodies[i].state.pos[0]
        pos_cm[1] += O.bodies[i].state.pos[1]
        pos_cm[2] += O.bodies[i].state.pos[2]

        vel_cm[0] += O.bodies[i].state.vel[0]
        vel_cm[1] += O.bodies[i].state.vel[1]
        vel_cm[2] += O.bodies[i].state.vel[2]

    pos_cm = [pos_cm[i]/no_bodies for i in range(3)]
    vel_cm = [vel_cm[i]/no_bodies for i in range(3)]

    vertex_combos = [list(x) for x in itertools.combinations(ids, 3)]
    omega = [0,0,0]

    counter = 0
    for j in vertex_combos:
        vel_mat = []
        pos_mat = []
        for i in j:
            vel_mat.append([
                O.bodies[i].state.vel[0] - vel_cm[0],
                O.bodies[i].state.vel[1] - vel_cm[1],
                O.bodies[i].state.vel[2] - vel_cm[2],
            ])
            pos_mat.append([
                O.bodies[i].state.pos[0] - pos_cm[0], 
                O.bodies[i].state.pos[1] - pos_cm[1], 
                O.bodies[i].state.pos[2] - pos_cm[2], 
            ])

        if np.linalg.det(pos_mat) < 1e-5:
            continue

        omega_temp = get_angular_vel_2(pos_mat, vel_mat)
        omega = [omega[i] + omega_temp[i] for i in range(3)]
        counter += 1

    omega = [i/counter for i in omega]
    return omega
    ###-------------temp


# ---------------------------------------------------------------------------------------------- getParameters
def getParameters(vars_dict, additional_params_to_reject = []):
    """
     Generates the list of the parameters defined for this simulation.


    :param vars_dict: all variables in the simulation
    :type vars_dict: dict
    :param additional_params_to_reject: paramaters in the simulation that should not be recorded
    :type additional_params_to_reject: list
    :return: parameter dict
    :rtype: dict
    """           
    param_dict             = {}
    param_keys, param_vals = list(vars_dict.keys()), list(vars_dict.values())
    no_of_params           = len(param_vals)
    params_to_reject       = [
        'PYQT_VERSION_STR',
        "PYQT_VERSION_STR",
        "QT_VERSION_STR",
        "PYQT_VERSION",
        "QT_VERSION",
        "QWIDGETSIZE_MAX",
        "prefix",
        "suffix",
        "version",
        "debugbuild",
        "libPATH",
        "libDir",
        "prog",
        "pi",
        "e",
        "tau",
        "inf",
        "nan",
        "gui",
    ]

    params_to_reject += additional_params_to_reject

    for i in range(no_of_params):
        if (
            type(param_vals[i]) in [int, float, str, Vector3] and 
            param_keys[i]  not  in  params_to_reject and
            not param_keys[i].startswith("_")
        ):
            param_dict[param_keys[i]] = param_vals[i]
    return param_dict

    # ---------------------------------------------------------------------------------------------- getClosestNode
def getClosestNode(position, node_list):
    """
    Returns the ID of the node closest to the position from a list of nodes

    Args:
        position (Vector3): Required position
        node_list (list): list of node IDS

    Returns:
        int: ID of closest node
    """
    vec     = Vector3(position - O.bodies[node_list[0]].state.pos)
    dist    = vec.norm()
    req_node = 0

    for i in node_list:
        temp_vec  = Vector3(position - O.bodies[i].state.pos)
        temp_dist = temp_vec.norm()
        if temp_dist < dist:
            dist     = temp_dist
            req_node = i

    return req_node

        
# ---------------------------------------------------------------------------------------------- getRotMatAngle
def getRotMatAngle(rot_mat, ref_vec, ref_axis):    
    normalised_ref_vec = ref_vec.normalized()
    rotated_vec        = rot_mat * normalised_ref_vec  
    rel_vec            = rotated_vec - ref_vec

    # check if no rotation
    if rel_vec.norm() < 0.00001:
        print("[WARNING]: the matrix does not rotate the ref_vec")
        return 0.0

    perp_vec            = ref_axis.cross(ref_vec)
    normalised_perp_vec = perp_vec.normalized()

    x = rotated_vec.dot(normalised_ref_vec)
    y = rotated_vec.dot(normalised_perp_vec)

    angle = atan2(y, x)
    return angle


# --------------------------------------------------------------------------------------------- split_bySpaces
def split_bySpaces(input_string):
    """
    Uses spaces (' ') to split a string into a list of substrings

    Args:
        input_string (str): input string

    Returns:
        list: list of substrings
    """
    string = ['']
    for i in input_string:
        if i == ' ':
            string = string + ['']
            
        else:
            string[-1] = string[-1] + i

    if string[-1] == '':
        string = string[:-1]

    return string

# --------------------------------------------------------------------------------------------- GTSParser
class GTSParser:
    """
    Parses the GTS file and extracts the data pertaining to vertices, edges and faces
    """
    # ------------------------------------------------------------------
    def __init__(self, input_str):
        """
        Initialises and parses the dataset

        Args:
            input_str (str): The string data from GTS file
        """
        self.raw_data = input_str
        
        self._split_rawByLines()
        self._get_3DMetadata()
        self._get_vertices()
        self._get_edges()
        self._get_faces()

    # ------------------------------------------------------------------
    def _split_rawByLines(self):
        """
        Splits the GTS string into lines based on '\n' character. This is stored
        in list 
        """
        self._data_line_splitted = ['']
        for i in self.raw_data:
            if i == '\n':
                self._data_line_splitted = self._data_line_splitted + ['']
            else:
                self._data_line_splitted[-1] = self._data_line_splitted[-1] + i

    # ------------------------------------------------------------------
    def _get_3DMetadata(self):
        """
        Extracts the metadata (number of vertices, edges and faces) from the first line
        """
        self._metadata_string     = self._data_line_splitted[0]
        metadata_string_splitted = split_bySpaces(self._metadata_string)
        self._metadata = {}
        for i in range(3):
            self._metadata[metadata_string_splitted[-i-1]] = int(metadata_string_splitted[i])

    # ------------------------------------------------------------------
    def _get_vertices(self):
        """
        Extracts the vertices (Their location) and stores them in a variable
        """
        no_vertices = self._metadata['GtsVertex']
        start_idx   = 1
        vertex_data = self._data_line_splitted[start_idx:start_idx + no_vertices]
        vertices    = []
        for i in vertex_data:
            vertices = vertices + [[float(j) for j in split_bySpaces(i)]]

        self._vertices = vertices

    # ------------------------------------------------------------------
    def _get_edges(self):
        """
        Extracts the edges (the vertices which form them) and stores them in a variable
        """
        no_vertices = self._metadata['GtsVertex']
        no_edges    = self._metadata['GtsEdge']
        start_idx   = 1 + no_vertices
        edge_data   = self._data_line_splitted[start_idx:start_idx + no_edges]
        edges       = []

        for i in edge_data:
            edges = edges + [[int(j) for j in split_bySpaces(i)]]

        self._edges = edges

    # ------------------------------------------------------------------
    def _get_faces(self):
        """
        Extracts the faces (the edges which form them) and stores them in a variable
        """
        no_vertices = self._metadata['GtsVertex']
        no_edges    = self._metadata['GtsEdge']
        np_faces    = self._metadata['GtsFace']
        start_idx   = 1 + no_vertices + no_edges
        face_data   = self._data_line_splitted[start_idx:start_idx + np_faces]
        faces       = []
        for i in face_data:
            faces = faces + [[int(j) for j in split_bySpaces(i)]]

        self._faces = faces

    # ------------------------------------------------------------------
    @property
    def vertices(self):
        return self._vertices

    @property
    def edges(self):
        return self._edges

    @property
    def faces(self):
        return self._faces


# --------------------------------------------------------------------------------------------- GTSPfacet2
class GTSPfacet2:
    '''
    Creates a pfacet mesh from a GTS file
    '''
    def __init__(
        self,
        gts_file_path, 
        radius,
        shift,
        scale,
        wire,
        fixed,
        color,
        material_nodes,
        material_ext
        ):
        """
            Initialises and creates the pfacet
        """

        self._gts_file_path = gts_file_path
        self._radius   = radius
        self._shift    = Vector3(shift)
        self._scale    = scale
        self._wire_Q   = wire
        self._fixed_Q  = fixed
        self._color    = color
        self._material_nodes   = material_nodes
        self._material_ext     = material_ext

        self._read_file()
        self._parsed_gts = GTSParser(self._gts_data)
        self._generate_Nodes()
        self._generate_Edges()
        self._generate_Faces()


    # ------------------------------------------------------------------
    def _read_file(self):
        """
        Reads the gts into a variable
        """
        with open(self._gts_file_path, 'r') as file:
            self._gts_data = file.read()

    # ------------------------------------------------------------------
    def _generate_Nodes(self):
        """
        Generates the nodes from the vertices defined in the GTS file
        """
        self._node_ids = []
        self._node_id_to_gts_node_idx = {}

        # creating nodes
        for idx,i in enumerate(self._parsed_gts.vertices):
            self._node_ids.append( 
                O.bodies.append(
                    gridNode(
                        self._scale*Vector3(i) + self._shift,
                        self._radius,
                        # dynamic = None,
                        wire  = self._wire_Q,
                        fixed = self._fixed_Q,
                        color = self._color, 
                        highlight = False,
                        material  = self._material_nodes
                    )))

            # GTS vertex index -> ID  
            self._node_id_to_gts_node_idx[idx + 1] = self._node_ids[-1]

    # ------------------------------------------------------------------
    def _generate_Edges(self):
        """
        Generates the edges/GridConnections using the GTS file
        """
        self._cyl_ids = []
        self._edge_id_to_gts_edge_idx = {}

        # creating Edges
        for idx,k in enumerate(self._parsed_gts.edges): 
            id1 = self._node_id_to_gts_node_idx[k[0]]
            id2 = self._node_id_to_gts_node_idx[k[1]]
            self._cyl_ids.append(
                O.bodies.append(
                    gridConnection(
                        id1,id2,
                        self._radius,
                        wire  = self._wire_Q,
                        color = self._color, 
                        highlight = False,
                        material  = self._material_ext,
                        mask = -1
                )))
            # GTS edge index -> ID  
            self._edge_id_to_gts_edge_idx[idx + 1] = self._cyl_ids[-1]

    # ------------------------------------------------------------------
    def _generate_Faces(self):
        """
        Generates the pfacet faces defined the GTS file
        """
        self._pfacet_ids    = []
        self._face_id_to_gts_face_idx = {}

        # creating faces
        for idx, j in enumerate(self._parsed_gts.faces):
            id1 = self._edge_id_to_gts_edge_idx[j[0]]
            id2 = self._edge_id_to_gts_edge_idx[j[1]]
            id3 = self._edge_id_to_gts_edge_idx[j[2]]
            pfacetCreator4(
                id1,id2,id3,
                pfIds = self._pfacet_ids,
                wire  = self._wire_Q,
                fixed = self._fixed_Q,
                color = self._color, 
                material = self._material_ext,
                mask = -1
                )

            # GTS face index -> ID 
            self._face_id_to_gts_face_idx[idx + 1] = self._pfacet_ids[-1]

    # ------------------------------------------------------------------
    @property
    def node_ids(self):
        return self._node_ids

    @property
    def cyl_ids(self):
        return self._cyl_ids

    @property
    def face_ids(self):
        return self._pfacet_ids


# --------------------------------------------------------------------------------------------- create_GTSPfacet
def create_GTSPfacet(
        gts_file_path, 
        radius,
        shift,
        scale,
        wire,
        fixed,
        color,
        materialNodes,
        material
        ):
    """
    Creates a pfacet object from the input GTS file.

    Args:
        gts_file_path (str): The path to the GTS file
        radius (float):      Radius of the pfacet nodes.
        shift (Vector3):     The location of the pfacet object
        scale (Float):       The amout the object is to be scaled 
        wire (bool) : 
        fixed (bool):        Is it fixed
        color (list):        Color of the object
        materialNodes (str): The material name of the nodes
        material (str):      The material name of the pfacet and the grids

    Returns:
        (list, list, list): ids of the nodes, cylinder, pfacets
    """
    object = GTSPfacet2(
        gts_file_path, 
        radius,
        shift,
        scale,
        wire,
        fixed,
        color,
        materialNodes,
        material
        )

    return object.node_ids, object.cyl_ids, object.face_ids

# --------------------------------------------------------------------------------------------- Target
class Target:
    """
    Creates the target object and appends it into the simulation
    """
    # ----------------------------------------------------------------------- init
    def __init__(
        self,
        target_shape,
        p_radius,
        target_pos,
        target_young,
        target_poisson,
        target_friction,
        target_mass,
        target_pfacet_mass,
        target_side,
        axle_mass,
        axle_radius
        ):
        """
        Initialises the target object

        Args:
            target_shape (string): location of the target shape
            p_radius (float): radius of the pfacets
            target_pos (Vector3): Position of the target center of mass
            target_young (float): target young modulus
            target_poisson (float): target poisson ratio
            target_friction (float): target surface friction
            target_mass (float): Mass of the target cuboid (no including the hole)
            target_pfacet_mass (float): mass of the pfacet and gridconections = 0
            target_side (float): side length of the target center
            axle_mass (float): mass of the axle
            axle_radius (float): radius of the axle
        """


        self.m_target_shape    = target_shape
        self.m_p_radius        = p_radius
        self.m_target_pos      = target_pos
        self.m_target_young    = target_young
        self.m_target_poisson  = target_poisson
        self.m_target_friction = target_friction
        self.m_target_mass     = target_mass
        self.m_target_pfacet_mass = target_pfacet_mass
        self.m_target_side     = target_side
        self.m_axle_mass       = axle_mass
        self.m_axle_radius     = axle_radius

        self.m_target_ext_mat  = "Target_ext_mat"
        self.m_target_int_mat  = "Target_int_mat"
        self.m_target_hole_mat = "Target_hole_mat"
        self.m_target_axle_mat = "Target_axle_mat"

        # calculate the parameters
        self._calculateTargetMMOI()
        self._calculateTargetDensityForMMOI()
        self._calculateCorrectGridNodeMassMMOI()
        self._calculateAxleSphereParameters()
        self._calculateHoleTargetParameters()

        # create he materials
        self._createTargetNodeMaterial()
        self._createTargetPFacetMaterial()
        self._createTargetHoleMaterial()
        self._createTargetAxleMaterial()

        # create the bodies and set correct dynamics
        self._createTargetCuboid()
        self._setCurrectCuboidMass()
        self._createHole()
        self._createAxle()
        self._createClump()
        self._setCorrectPhysics()

    # ----------------------------------------------------------------------- _calculateAxleSphereParameters
    def _calculateAxleSphereParameters(self):
        """
        Calculates the radius and the density of the sphere that will represent the axle in the simulation
        """
        f_axle_mmoi      = self.m_axle_mass * self.m_axle_radius ** 2.0 / 2.0

        # Since there will be two spheres for symmetry
        f_axle_mass           = self.m_axle_mass / 2
        f_axle_mmoi           = f_axle_mmoi / 2
        self.m_axle_sphere_radius  = ((5.0 / 2.0) * (f_axle_mmoi / f_axle_mass))**0.5

        f_sphere_volume            = 4.0 / 3.0 * 3.14159265 * self.m_axle_sphere_radius**3
        self.m_axle_sphere_density = f_axle_mass / f_sphere_volume

    # ----------------------------------------------------------------------- _calculateCorrectGridNodeMassMMOI
    def _calculateCorrectGridNodeMassMMOI(self):
        """
            Returns the corrected mass and the mmoi for the gridNodes. yade has a wierd
        (and wrong?) method for assigning the mass and mmoi for the gridNodes
        """
        self.m_target_grid_mass = self.m_target_mass / 14.0
        f_target_mmoi           = self.m_target_MMOI[0]
        f_half_side_length      = self.m_target_side / 2.0
        self.m_target_grid_mmoi = (
            f_target_mmoi - 20.0*self.m_target_grid_mass*f_half_side_length**2
            ) / 14.0

    # ----------------------------------------------------------------------- _calculateHoleTargetParameters
    def _calculateHoleTargetParameters(self):
        """
        Get the radius and the density of the spheres that will represent the axle in the simulation
        """
        f_target_volume  = (
            self.m_target_side**3.0 if self.m_target_side < 0.07 else self.m_target_side**2*0.06
            )

        f_target_density = self.m_target_mass / f_target_volume
        f_hole_height    = self.m_target_side if self.m_target_side < 0.07 else 0.06
        f_hole_volume    = 3.14159265 * self.m_axle_radius**2 * f_hole_height

        f_hole_mass           = f_target_density * f_hole_volume
        f_hole_mmoi           = (f_hole_mass * self.m_axle_radius**2.0)/2.0
        self.m_hole_sphere_radius  = ((5.0/2.0) * (f_hole_mmoi / f_hole_mass))**0.5


        f_sphere_volume            = 4.0 / 3.0 * 3.14159265 * self.m_hole_sphere_radius**3 
        self.m_hole_sphere_density = f_hole_mass / f_sphere_volume

    # ----------------------------------------------------------------------- _calculateTargetDensityForMMOI
    def _calculateTargetDensityForMMOI(self):
        """
        Returns the density for the material which will be used for the pfacet spheres. 
        The density is calculated such that the MMOI of the target and the pfacet are the same.
        The mass must be manually set for the pfacet clump
        """
        f_mmoi             = self.m_target_MMOI[0]
        f_half_side_length = self.m_target_side / 2.0
        f_calculated_mass  = f_mmoi / (
            28.0/5.0*self.m_p_radius**2.0 + 
            20*f_half_side_length**2.0
            )

        f_volume               = 4.0 / 3.0 * 3.14159265 * self.m_p_radius**3.0 
        self.m_target_density  = f_calculated_mass / f_volume

    # ----------------------------------------------------------------------- _calculateTargetMMOI
    def _calculateTargetMMOI(self):
        """
        Calculate the inertia of the target body (cuboid)
        """
        f_mmoi = (self.m_target_mass * self.m_target_side**2)/6.0
        self.m_target_MMOI = Vector3([f_mmoi, f_mmoi, f_mmoi])

    # ----------------------------------------------------------------------- _createAxle
    def _createAxle(self):
        """
        Creates the axle objects: two spheres whose combined mass and inertia
        is equal to that of the axle
        """
        f_target_axle_offset  = Vector3([self.m_target_side, 0, 0])
        f_target_axle_object_1   = utils.sphere(
            Vector3(self.m_target_pos) + f_target_axle_offset,
            radius   = self.m_axle_sphere_radius,
            material = self.m_target_axle_mat 
        )
        self.m_target_axle_object_1_id =  O.bodies.append(f_target_axle_object_1)

        f_target_axle_object_2   = utils.sphere(
            Vector3(self.m_target_pos) -  f_target_axle_offset,
            radius   = self.m_axle_sphere_radius,
            material = self.m_target_axle_mat  
        )
        self.m_target_axle_object_2_id =  O.bodies.append(f_target_axle_object_2)

    # ----------------------------------------------------------------------- _createClump()
    def _createClump(self):
        """
        Creates a clump from the pfacet box, the hole body and the axle bodies
        """
        self.m_id_clump_objects = self.m_node_Ids + self.m_cyl_Ids + self.m_pf_Ids + [
            self.m_target_hole_sphere_id, 
            self.m_target_axle_object_1_id, 
            self.m_target_axle_object_2_id
            ]

        self.m_target_clump_id       = O.bodies.clump(self.m_id_clump_objects)
        self.m_target_moving_id_list = [self.m_target_clump_id ]

    # ----------------------------------------------------------------------- _createHole
    def _createHole(self):
        """
        Creates a sphere which has the mass (-ve mass) and inertia (-ve inertia)
        of the hole in the target cuboid
        """
        f_target_hole_sphere   = utils.sphere(
            Vector3(self.m_target_pos),
            radius   = self.m_hole_sphere_radius,
            material = self.m_target_hole_mat
        )
        self.m_target_hole_sphere_id =  O.bodies.append(f_target_hole_sphere)

    # ----------------------------------------------------------------------- _createTargetAxleMaterial
    def _createTargetAxleMaterial(self):
        """
        Creates the material for the bodies 
        which will represent the axle
        """
        O.materials.append(
            FrictMat(
                young   = 1e5,
                poisson = 0.3,
                density = self.m_axle_sphere_density,
                label   = self.m_target_axle_mat,
                frictionAngle = radians(0),
            )
        )

    # ----------------------------------------------------------------------- _createTargetCuboid
    def _createTargetCuboid(self):
        """
        Creates the pfacet body from a GTS file which 
        will represent the target cuboid
        """
        (
            self.m_node_Ids,
            self.m_cyl_Ids,
            self.m_pf_Ids,
        ) = create_GTSPfacet(
            self.m_target_shape,
            radius = self.m_p_radius,
            shift  = self.m_target_pos,
            scale  = 1,
            wire   = False,
            fixed  = False,
            color  = [0.1,0.5,0.1],
            materialNodes = self.m_target_int_mat,
            material      = self.m_target_ext_mat,
        )

    # ----------------------------------------------------------------------- _createTargetHoleMaterial
    def _createTargetHoleMaterial(self):
        """
        Creates the material for the body which will represent the
        hole in the target cuboid
        """
        self.m_target_hole_mat
        O.materials.append(
            FrictMat(
                young   = self.m_target_young,
                poisson = self.m_target_poisson,
                density = -self.m_hole_sphere_density, # -ve because it is a hole
                label   = self.m_target_hole_mat,
                frictionAngle = radians(0),
            )
        )

    # ----------------------------------------------------------------------- _createTargetNodeMaterial
    def _createTargetNodeMaterial(self):
        """
        Creates the material for the gridNodes which forms
        part of the pfacet clump which will represent the
        target cuboid
        """
        O.materials.append(
            CohFrictMat(
                young   = self.m_target_young,
                poisson = self.m_target_poisson,
                density = self.m_target_density,
                label   = self.m_target_int_mat,

                frictionAngle     = self.m_target_friction,
                normalCohesion    = 3e100,
                shearCohesion     = 3e100,
                momentRotationLaw = True,
            )
        )

    # ----------------------------------------------------------------------- _createTargetPFacetMaterial
    def _createTargetPFacetMaterial(self):
        """
        Creates the material for the pfacet and the
        gridConnections which will for the surface of
        the pfacet clump
        """
        O.materials.append(
            FrictMat(
                young   = self.m_target_young,
                poisson = self.m_target_poisson,
                density = 0.0,
                label   = self.m_target_ext_mat,
                frictionAngle = self.m_target_friction,
            )
        )

    # ----------------------------------------------------------------------- _setCorrectPhysics
    def _setCorrectPhysics(self):
        """
        Set the initial physics settings for the clump. The clump will be moved
        using the data from the experiments and not the dynamics of the body
        """
        # fixing the DOFS
        for i in self.m_target_moving_id_list:
            O.bodies[i].state.blockedDOFs = "xyzYZ"
            # the linear DOF along y and z while the constrained by the penalty force
        del i

        # The target should not be damped
        for i in self.m_id_clump_objects + self.m_target_moving_id_list:
            O.bodies[i].state.isDamped = False
            O.bodies[i].dynamic = False
        del i

    # ----------------------------------------------------------------------- _setCurrectCuboidMass
    def _setCurrectCuboidMass(self):
        """
        Sets the correct mass and mmoi to the pfacet and 
        gridConnections (0) and the gridNodes, such the clump 
        will have the mass and inertia of the target
        """
        for id in self.m_cyl_Ids + self.m_pf_Ids:
            O.bodies[id].state.mass = self.m_target_pfacet_mass
        del id

        for id in self.m_node_Ids:
            O.bodies[id].state.mass    = self.m_target_grid_mass
            O.bodies[id].state.inertia = Vector3([
                self.m_target_grid_mmoi, 
                self.m_target_grid_mmoi, 
                self.m_target_grid_mmoi
                ])

    # ----------------------------------------------------------------------- Properties
    @property
    def clump_id(self):
        return self.m_target_clump_id

    @property
    def moving_id_list(self):
        return self.m_target_moving_id_list

    @property
    def hole_sphere_id(self):
        return self.m_target_hole_sphere_id
# --------------------------------------------------------------------------------------------- tracker
class Tracker:
    """
    Tracks the orientation of the target and makes it smooth when the angle jumps from
    180 to -180 or vice versa
    """

    def __init__(self, ori, reference_vector, axis_vector):
        """
        Constructing tracker

        Args:
            ori (Quaternion): The reference orientation used to calc the relative orientation
            reference_vector (Vector3): The Vector which represents angle = 0
            axis_vector (Vector3): The axis about which the angle is measured
        """
        self.m_reference_ori    = ori
        self.m_reference_vector = reference_vector
        self.m_axis_vector      = axis_vector        
        self.m_ref_ori_mat      = self.m_reference_ori.toRotationMatrix()
        self.m_rotation_number    = 0

        self.m_angles = [0.0]
        
    
    def addOri(self, ori):
        """
        appends an orientation into the tracker

        Args:
            ori (Quaternion): Current orientation of the target
        """
        ori_mat     = ori.toRotationMatrix()
        rel_ori_mat = ori_mat * self.m_ref_ori_mat.inverse()
        angle       = getRotMatAngle(
            rel_ori_mat, 
            self.m_reference_vector, 
            self.m_axis_vector
            )*180/pi


        # correcting for the discontinuity at angle -180 to 180
        angle = angle + 360 * self.m_rotation_number
        if self.m_angles[-1] - angle > 300:
            self.m_rotation_number = self.m_rotation_number + 1
            angle = angle + 360

        elif self.m_angles[-1] - angle < -300:
            self.m_rotation_number = self.m_rotation_number - 1
            angle = angle - 360

        self.m_angles.append(angle )

    def getLatestAngle(self):
        """
        Returns the latest angle

        Returns:
            [float]: latest angle
        """
        return self.m_angles[-1]
