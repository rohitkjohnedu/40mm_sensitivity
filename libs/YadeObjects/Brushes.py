from yade import *
from yade.gridpfacet import *
from itertools import combinations
from abc import ABC, abstractmethod
import numpy as np
from YadeObjects.Lines import *
from YadeObjects.Rods import *
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

# ------------------------------------------------------------------------ Brush
class Brush(YadeObjectList):
    """
    Abstract class for defining the features of a brush
    """
    def __init__(
            self, 

            external_material = 'default_ext_mat',
            internal_material = 'default_int_mat',
            tip_material = -1,        

            dynamic     = None,   
            fixed       = False,  
            wire        = False,
            rest_color  = None,  
            tip_color   = None,
            highlight = False,  
            mask      = 1,  

            scale       = 1,
            brush_pos   = [0,0,0],
            orientation = Quaternion((1,0,0),0),
            root_fixed_Q = True, # If true the roots will be fixed to the ground

            check_collision = True

        ):

        self.m_tip_material      = internal_material if tip_material == -1 else tip_material
        self.m_external_material = external_material
        self.m_internal_material = internal_material
        self.m_root_fixed_Q = root_fixed_Q

        self.m_mask    = mask
        self.m_dynamic = dynamic 
        self.m_fixed   = fixed 
        self.m_wire    = wire 

        self.m_rest_color =  rest_color
        self.m_tip_color  =  tip_color
        self.m_highlight  = highlight

        self._generate_BrushRootVertices()
        self._get_Radii()
        self._get_BrislteLine()
        self._add_BrislteNoise()
        if check_collision: self._check_BristleCollisions()
        self._apply_Translation(brush_pos)
        self._apply_Orientaion(orientation)
        self._apply_Scale(scale)
        self._create_Bristles()

        super().__init__()

    # -------------------------------------------------------------- abstract methods
    @abstractmethod
    def _generate_BrushRootVertices():
        pass

    @abstractmethod
    def _get_Radii():
        pass
                
    @abstractmethod
    def _get_BrislteLine():
        pass
        
    @abstractmethod
    def _add_BrislteNoise():
        pass

    @abstractmethod
    def _apply_Orientaion(orientation):
        pass
        
    @abstractmethod
    def _apply_Scale(scale):
        pass        

    @abstractmethod
    def _apply_Translation(brush_base_pos):
        pass

    @abstractmethod
    def _check_BristleCollisions():
        pass

    @abstractmethod
    def _create_Bristles():
        pass

    # -------------------------------------------------------------- virtual method defs
    def _generate_IDList(self):
        self.m_ids = []
        for i in self.m_bristles:
            ids = i.id_list
            self.m_ids = self.m_ids + ids

    def _generate_StateList(self):
        self.m_states = []
        for i in self.m_bristles:
            states = i.id_list
            self.m_states = self.m_states + states

    def _generate_ClumpIDList(self):
        self.m_clump_ids = []
        for i in self.m_bristles:
            clump_ids = i.id_list
            self.m_clump_ids = self.m_clump_ids + clump_ids

    def _generate_MaterialList(self):
        self.m_materials = []
        for i in self.m_bristles:
            materials = i.id_list
            self.m_materials = self.m_materials + materials

    # -------------------------------------------------------------- property
    @property
    def root_ids(self):
        ids = []
        for bristle in self.m_bristles:
            ids.append(bristle.root_id)

        return ids

# ------------------------------------------------------------------------ OrderedBrush
class OrderedSimpleBrush(Brush):
    """
    Abstract class which inherits the Brush class and adds the features
    of a brush with ordered root positions
    """
    def __init__(
            self,
            bristle_radius_tip  = 1e-4, 
            bristle_radius_root = 1e-4,
            bristle_length   = 10e-3,
            no_of_segments   = 5,  

            x_lenght = 10e-3, 
            y_length = 10e-3, 
            x_no_density = 1/(2e-3), 
            y_no_density = 1/(2e-3), 
            bristle_tip_spread_covariance = [[0, 0],[0, 0]],# 'default',
            clearance = 1e-6,

            **kwargs
        ):
        self.m_radius_tip  = bristle_radius_tip 
        self.m_radius_root = bristle_radius_root
        self.m_bristle_length   = bristle_length
        self.m_no_of_segments   = no_of_segments

        self.m_x_lenght = x_lenght
        self.m_y_length = y_length
        self.m_x_no_density = x_no_density
        self.m_y_no_density = y_no_density
        self.m_covar     = bristle_tip_spread_covariance
        self.m_clearance = clearance

        super().__init__(**kwargs)

    # -------------------------------------------------------------- _get_Radii
    def _get_Radii(self):
        """
        Generates an array with the tip and the rood radii
        """
        self.m_radii = [self.m_radius_root, self.m_radius_tip]

    # -------------------------------------------------------------- _get_BrislteLine
    def _get_BrislteLine(self):
        """
        Defines the bristle directions
        """
        self.m_bristle_directions = [Vector3([0,0,1]) for i in self.m_bristle_roots]

    
    # -------------------------------------------------------------- _add_BrislteNoise
    def _add_BrislteNoise(self):
        """
        Adds noise to the bristle directions
        """
        self.m_bristle_lines = []

        x_vars, y_vars = np.random.multivariate_normal(
                [0,0], 
                self.m_covar, 
                self.m_number_of_bristles
            ).T

        idx = 0
        for x_var, y_var, bristle_dir, bristle_root in zip(
            x_vars, 
            y_vars, 
            self.m_bristle_directions,
            self.m_bristle_roots
            ):

            noise_vec          = Vector3([x_var, y_var, 0])
            noised_bristle_dir = bristle_dir + noise_vec
            self.m_bristle_lines.append(
                Line(
                    Point(bristle_root),
                    noised_bristle_dir.normalized()
            ))
            self.m_bristle_directions[idx] = noised_bristle_dir.normalized()
            idx += 1

    
    # -------------------------------------------------------------- _check_BristleCollisions
    def _check_BristleCollisions(self):
        """
        Checks if any of the bristles are in contact
        """
        max_radius = max(self.m_radii)
        for l1, l2 in combinations(self.m_bristle_lines, 2):
            min_dist = calc_DistanceBetween2Lines(l1,l2)

            if min_dist < 2*min_radius:
                log.warning("Brush: Two bristles may be in contact")

    # -------------------------------------------------------------- _apply_Orientaion
    def _apply_Orientaion(self, orientation):
        """
        Rotates the brush

        Parameters
        ----------
        orientation : Quaternion    
            Orientation of the brush
        """
        self.m_ori = orientation
        for idx, i in enumerate(self.m_bristle_roots):
            self.m_bristle_roots[idx] = self.m_ori*i

        for idx, i in enumerate(self.m_bristle_directions):
            self.m_bristle_directions[idx] = self.m_ori*i

    # -------------------------------------------------------------- _apply_Scale
    def _apply_Scale(self, scale):
        """
        Scales the size of the brush. Does not affect the radius

        Parameters
        ----------
        scale : float   
            Amount to scale the brush
        """
        self.m_scale = scale
        for idx,i in enumerate(self.m_bristle_roots):
            self.m_bristle_roots[idx] = scale*i

    # -------------------------------------------------------------- _apply_Translation
    def _apply_Translation(self, translation):
        """
        Moves the brush

        Parameters
        ----------
        translation : Vector3
            Amount to move the brush
        """
        self.m_shift = Vector3(translation)
        for idx,i in enumerate(self.m_bristle_roots):
            self.m_bristle_roots[idx] = i + self.m_shift

    # -------------------------------------------------------------- _create_Bristles
    def _create_Bristles(self):
        """
        Creates the bristles using the SimpleTaperedRod class
        """
        self.m_bristles = []
        # for root_pos, dir in zip(self.m_bristle_roots, self.m_bristle_directions):
        for line in self.m_bristle_lines:
            root_pos = line.m_point
            dir      = line.m_direction 
            bristle = SimpleTaperedRod(
                rod_external_material = self.m_external_material,
                rod_internal_material = self.m_internal_material,
                rod_tip_material      = self.m_tip_material,   

                dynamic     = self.m_dynamic,   
                fixed       = self.m_fixed,  
                wire        = self.m_wire,
                rest_color  = self.m_rest_color,  
                tip_color   = self.m_tip_color,
                highlight   = self.m_highlight,  
                mask        = self.m_mask,     

                root_fixed_Q  = self.m_root_fixed_Q,
                rod_length    = self.m_bristle_length,
                no_of_segments = self.m_no_of_segments,     

                rod_root_position = root_pos,
                rod_direction     = dir,
                rod_radius_root   = self.m_radius_root, 
                rod_radius_tip    = self.m_radius_tip,
            )
        
            self.m_bristles.append(bristle)

# ------------------------------------------------------------------------ QuadGridBrush
class QuadGridBrush(OrderedSimpleBrush):
    def __init__(
            self,
            **kwargs
        ):
        super().__init__(**kwargs)

    # -------------------------------------------------------------- _generate_BrushRootVertices
    def _generate_BrushRootVertices(self):
        """
        Generates the positions of the bristle roots based on a quad grid
        """
        self.m_bristle_roots = []
        self.m_x_bristle_nos = int(math.floor(self.m_x_lenght * self.m_x_no_density))
        self.m_y_bristle_nos = int(math.floor(self.m_y_length * self.m_y_no_density))
        dx = 1/self.m_x_no_density
        dy = 1/self.m_y_no_density

        for i in range(self.m_x_bristle_nos):
            for j in range(self.m_y_bristle_nos):
                root = Vector3([
                    i * dx     - (self.m_x_bristle_nos - 1)*dx/2, 
                    j * dy     - (self.m_y_bristle_nos - 1)*dy/2, 
                    0
                ])

                self.m_bristle_roots.append(root)

        self.m_number_of_bristles = len(self.m_bristle_roots)


# ------------------------------------------------------------------------ TriGridBrush
class TriGridBrush(OrderedSimpleBrush):
    def __init__(
            self,
            **kwargs
        ):
        super().__init__(**kwargs)

    # -------------------------------------------------------------- _generate_BrushRootVertices
    def _generate_BrushRootVertices(self):
        '''
        Generats the position of the bristle roots based on a tiangular grid pattern
        '''

        self.m_bristle_roots = []
        self.m_x_bristle_nos = int(math.floor(self.m_x_lenght * self.m_x_no_density))
        self.m_y_bristle_nos = int(math.floor(self.m_y_length * self.m_y_no_density))

        dx = 1/self.m_x_no_density
        dy = 1/self.m_y_no_density

        # m_x_bristle_nos_dash is the number of bristles in the alternating rows
        # Depending on the brush x_lenght, it can be equal to or one less than
        # m_x_bristle_nos
        if (self.m_x_bristle_nos + 0.5)*dx < self.m_x_lenght:
            self.m_x_bristle_nos_dash = self.m_x_bristle_nos
        else:
            self.m_x_bristle_nos_dash = self.m_x_bristle_nos - 1

        # Generating arrays which store the x and y coordinates
        root_array_x = [
            (i*dx - (self.m_x_bristle_nos - 1)*dx/2) 
            for i in range(self.m_x_bristle_nos)
            ]
        
        root_array_y = [
            j*dy - (self.m_y_bristle_nos - 1)*dy/2
            for j in range(self.m_y_bristle_nos)
            ]

        # Looping to get the root positions
        for j in range(self.m_y_bristle_nos):
            # if else used to create the staggered rows of hexagonal closed packing
            if j % 2 == 0:
                for i in range(self.m_x_bristle_nos):
                    self.m_bristle_roots.append(Vector3([
                        root_array_x[i], 
                        root_array_y[j], 
                        0
                        ]))

            else:
                for i in range(self.m_x_bristle_nos_dash):
                    self.m_bristle_roots.append(Vector3([
                        root_array_x[i] + dx/2, 
                        root_array_y[j], 
                        0
                        ]))

        self.m_number_of_bristles = len(self.m_bristle_roots)

        