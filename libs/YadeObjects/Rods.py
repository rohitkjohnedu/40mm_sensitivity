from itertools import combinations
from yade import gridpfacet
from yade import *
from RKJ_utils import *
from YadeObjects.YadeObjects import *

# ------------------------------------------------------------------------ Setting up the logger 
import logging
from rich.logging import RichHandler

FORMAT = "%(message)s"
logging.basicConfig(
    level  = "NOTSET", 
    format = FORMAT, 
    datefmt  = "[%X]", 
    handlers = [RichHandler()]
)

log = logging.getLogger("rich")

# ------------------------------------------------------------------------ Rods
class Rod(YadeObjectList):
    """
    Creates a conical rod in yade and its associated data
    """
    def __init__(
        self, 

        rod_external_material = 'default_ext_mat',
        rod_internal_material = 'default_int_mat',
        rod_tip_material      = -1,   

        dynamic     = None,   
        fixed       = False,  
        wire        = False,
        rest_color  = None,  
        tip_color   = None,
        highlight = False,  
        mask      = 1,     

        root_fixed_Q = True
        ):
        
        """
        Initialises the Rod class of

        Args:
            rod_radius_root (float, optional)     : The root radius. Defaults to 1e-4.
            rod_radius_tip (float, optional)      : The tip radius. Defaults to -1.
            rod_external_material (str, optional) : The external material name. 
                                                   Defaults to 'default_ext_mat'.
            rod_internal_material (str, optional) : The internal material name. 
                                                   Defaults to 'default_int_mat'.
            rod_tip_material ((str, optionalint), optional): The tip material; 
                                                            -1 = internal material. 
                                                            Defaults to -1.
            root_fixed_Q (bool, optional)        : Whether is root is fixed. Defaults to True.
        """


        self.m_root_fixed_Q  = root_fixed_Q

        self.m_rod_tip_material      = rod_internal_material if rod_tip_material == -1 else rod_tip_material
        self.m_rod_external_material = rod_external_material
        self.m_rod_internal_material = rod_internal_material

        self.m_mask    = mask
        self.m_dynamic = dynamic 
        self.m_fixed   = fixed 
        self.m_wire    = wire 

        self.m_rest_color =  rest_color
        self.m_tip_color  =  tip_color
        self.m_highlight  = highlight

        self._generate_Rod()
        super().__init__()

    # -------------------------------------------------------------- _generate_Rod
    def _generate_Rod(self):
        self._create_DefaultMaterial()
        self._calc_Vertices()
        self._calc_radii()
        self._check_GridNodeCollision()
        self._create_GridNodes()
        self._create_GridConnection()
        self._assign_RodRootTipID()
        self._fix_root()

    # --------------------------------------------------------------_create_DefaultMaterial
    def _create_DefaultMaterial(self):
        """
        Creating the default Material
        """
        if self.m_rod_external_material == 'default_ext_mat':
            O.materials.append(
                FrictMat(
                    young   = 1e6,
                    poisson = 0.48,
                    density = 1250,
                    label   = 'default_ext_mat',
                    frictionAngle = radians(44),
                )
            )

        if self.m_rod_internal_material == 'default_int_mat':
            O.materials.append(
                CohFrictMat(
                    young   = 1e6,
                    poisson = 0.48,
                    density = 1250,
                    label   = 'default_int_mat',

                    frictionAngle     = radians(4),
                    momentRotationLaw = True,
                    normalCohesion    = 1e40,
                    shearCohesion     = 1e40,
                )
            )


    # -------------------------------------------------------------- _calc_Vertices
    @abstractmethod
    def _calc_Vertices(self):
        pass        

    # -------------------------------------------------------------- _calc_radii
    @abstractmethod
    def _calc_radii(self):
        pass
    # -------------------------------------------------------------- _calc_Vertices
    def _check_GridNodeCollision(self):
        """
        Checks if the distance between two vertices are less than the min
        diameter
        """
        for vec1, vec2 in combinations(self.m_vertices,2):
            rel_vec        = vec1 - vec2
            rel_vec_norm   = rel_vec.norm()
            smaller_radius = min(self.m_radii)
            if rel_vec_norm <= 2*smaller_radius:
                log.warning("Rod: Two nodes may be in contant")


    # -------------------------------------------------------------- _create_GridNodes
    def _create_GridNodes(self):
        """
            Create the gridnodes in YADE node ids to
            appropriate variables.
        """
        self.m_node_ids_array = []
        no_vertices = len(self.m_vertices)
        no_radii    = len(self.m_radii)
        tip_vertex  = self.m_vertices[-1]
        tip_radius  = self.m_radii[-1]

        # check if the number of radii and number of vertices are equal
        if no_vertices != no_radii:
            log.error("Rod class: Size of radii list and vertex list not same")

        # creating all nodes but the tip
        for pos, rad  in zip(self.m_vertices[:-1], self.m_radii[:-1]):
            self.m_node_ids_array.append(
                    O.bodies.append(
                        gridNode(
                            pos, 
                            rad, 
                            self.m_dynamic, 
                            self.m_fixed, 
                            self.m_wire, 
                            self.m_rest_color, 
                            self.m_highlight, 
                            self.m_rod_internal_material
                        )))

        # The tip has special attention. It could have different color
        #  or different material. This is handled below
        self.m_node_ids_array.append(
                O.bodies.append(
                    gridNode(
                        tip_vertex, 
                        tip_radius, 
                        self.m_dynamic, 
                        self.m_fixed, 
                        self.m_wire, 
                        self.m_tip_color, 
                        self.m_highlight, 
                        self.m_rod_tip_material
                    )))

    # -------------------------------------------------------------- _create_GridConnection
    def _create_GridConnection(self):
        """
            Create the gridConnections in YADE cyl ids to
            appropriate variables.
        """
        self.m_cyl_ids_array  = []

        for i,j in zip( self.m_node_ids_array[:-1], self.m_node_ids_array[1:]):
            rad_i  = O.bodies[i].shape.radius
            rad_j  = O.bodies[j].shape.radius
            radius = min(rad_i, rad_j)

            self.m_cyl_ids_array.append( 
                O.bodies.append( 
                    gridConnection(
                        i,j,
                        radius = radius,
                        wire   = self.m_wire, 
                        color  = self.m_rest_color, 
                        highlight = self.m_highlight, 
                        material  = self.m_rod_external_material,
                        mask      = self.m_mask,
                        cellDist  = None
                    )))


    # -------------------------------------------------------------- _assign_RodRootTipID
    def _assign_RodRootTipID(self):
        """
        Assigns the root and tip IDs to appropriate variables.
        """
        self.m_root_id = self.m_node_ids_array[ 0]
        self.m_tip_id  = self.m_node_ids_array[-1]

    # -------------------------------------------------------------- _fix_root
    def _fix_root(self):
        """
        Fixing the rod root is root_fixed_Q is true
        """
        if self.m_root_fixed_Q:
            O.bodies[self.m_root_id].state.blockedDOFs = "xyzXYZ"


    # -------------------------------------------------------------- properties
    @property
    def root_id(self):
        return self.m_root_id

    @property
    def tip_id(self):
        return self.m_tip_id

    @property
    def node_ids(self):
        return self.m_node_ids_array

    @property
    def cyl_ids(self):
        return self.m_cyl_ids_array

    # -------------------------------------------------------------- virtual methods
    def _generate_IDList(self):
        self.m_ids = self.m_node_ids_array + self.m_cyl_ids_array

    def _generate_StateList(self):
        self.m_states = {}
        for i in self.m_node_ids_array:
            self.m_states[i] = O.bodies[i].state
        del i

        for i in self.m_cyl_ids_array:
            self.m_states[i] = O.bodies[i].state

    def _generate_MaterialList(self):
        self.m_materials = {}
        for i in self.m_node_ids_array:
            self.m_materials[i] = O.bodies[i].material
        del i

        for i in self.m_cyl_ids_array:
            self.m_materials[i] = O.bodies[i].material

    def _generate_ClumpIDList(self):
        self.m_clump_ids = {}
        for i in self.m_node_ids_array:
            self.m_clump_ids[i] = O.bodies[i].clumpId
        del i

        for i in self.m_cyl_ids_array:
            self.m_clump_ids[i] = O.bodies[i].clumpId

# ------------------------------------------------------------------------ RadiusListRod
class RadiusListRod(Rod):
    def __init__(
        self, 
        radii = [1e-4, 1e-4], 
        **kwargs
        ):
        """
        Initialised the rod with node radii defined by user input list

        Parameters
        ----------
        radii : list, optional
            List of radii of the nodes, by default [1e-4, 1e-4]
        """

        self.temp_radii = radii
        super.__init__(**kwargs)

    def _calc_radii(self):
        """
        Assigns the user input radii list to the attribute m_radii
        """
        self.m_radii = self.temp_radii

# ------------------------------------------------------------------------ TaperedRod
class TaperedRod(Rod):
    def __init__(
        self, 
        rod_radius_root = 1e-4, 
        rod_radius_tip  = 1e-4,

        **kwargs
        ):
        """
        Initialises the TaperedRod class. The rod's radius decreases linearly with
        node index.

        Args:
            rod_radius_root (float, optional)      : The root radius. Defaults to 1e-4.
            rod_radius_tip (float, optional)       : The tip radius. Defaults to -1.
            rod_external_material (str, optional)  : The external material name. 
                                                   Defaults to 'default_ext_mat'.
            rod_internal_material (str, optional)  : The internal material name. 
                                                   Defaults to 'default_int_mat'.
            rod_tip_material ((str, optionalint), optional): The tip material; 
                                                            -1 = internal material. 
                                                            Defaults to -1.
            root_fixed_Q (bool, optional)        : Whether is root is fixed. Defaults to True.
        """


        self.m_rod_radius_root = rod_radius_root
        self.m_rod_radius_tip  = rod_radius_tip

        super().__init__(**kwargs)

    # -------------------------------------------------------------- _calc_radii
    def _calc_radii(self):
        no_vertices       = len(self.m_vertices )
        self.m_radii = [
            lerp(self.m_rod_radius_root, self.m_rod_radius_tip, i/(no_vertices - 1)) 
            for i in range(no_vertices)
            ]

# ------------------------------------------------------------------------ SimpleRod
class SimpleRod(Rod):
    """
    A simple rod is defined by the length, number of segments, root position
    and direction vector
    """
    def __init__(
        self, 
        rod_length    = 10e-3,
        no_of_segments = 5,     

        rod_root_position = [0,0,0],
        rod_direction     = [1,0,0],

        **kwargs
        ):
        """
        Initialises the SimpleRod class

        Args:
            rod_length (float, optional)         : The lenght of the rod. Defaults to 10e-3.
            no_of_segments (int, optional)        : The number of rod segments. Defaults to 5.
            rod_root_position (Vector3, optional): The position of the root. Defaults to [0,0,0].
            rod_direction (Vector3, optional)    : The direction of the rod. Defaults to [1,0,0].
        """

        self.m_rod_length        = rod_length
        self.m_no_of_segments     = no_of_segments
        self.m_rod_root_position = Vector3(rod_root_position)
        self.m_rod_direction     = Vector3(rod_direction)

        super().__init__(**kwargs)

    # -------------------------------------------------------------- _calc_Vertices
    def _calc_Vertices(self):
        """
        Calculate the locations of the rod nodes.
        """
        self.m_vertices           = []
        dl = self.m_rod_length/(self.m_no_of_segments - 1)

        self.m_vertices = [
            self.m_rod_root_position + i*dl*self.m_rod_direction 
            for i in range(self.m_no_of_segments)
            ]

# ------------------------------------------------------------------------ VertexListRod
class VertexListRod(Rod):
    """
    Abstract class for VertexListRod
    """
    def __init__(
        self, 
        vertices = [[0,0,0], [1e-3,0,0]],
        **kwargs
        ):
        """
        Initialises the Rod class of

        Args:
            vertices(list) : List of the vertices
        """
        self.temp_vertices     = vertices
        super().__init__(**kwargs)

    # -------------------------------------------------------------- _calc_Vertices
    def _calc_Vertices(self):
        """
        Calculate the locations of the rod nodes based on the
        vertices.
        """
        self.m_vertices  = [Vector3(i) for i in self.temp_vertices]

# ------------------------------------------------------------------------ SimpleTaperedRod
class SimpleTaperedRod(SimpleRod, TaperedRod):
    """
    A simple rod is defined by the length, number of segments, root position
    and direction vector
    """
    def __init__(
        self, 
        **kwargs
        ):
        super().__init__(**kwargs)

# ------------------------------------------------------------------------ VertexListTaperedRod
class VertexListTaperedRod(VertexListRod, TaperedRod):
    def __init__(
        self, 
        **kwargs
        ):
        """
        Constructs the VertexListTaperedRod
        """
        super().__init__(**kwargs)