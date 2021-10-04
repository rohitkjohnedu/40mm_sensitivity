from abc import ABC, abstractmethod
from yade import *
 
# ------------------------------------------------------------------------ YadeObject
class YadeObject(ABC):
    """
    This is the abstract class for all yade object class
    """

    @abstractmethod
    def _assign_ID(self):
        """
        This assigns the object ID to the m_id variable. Makes sure
        this attribute is set.
        """
        pass

    @property
    def id(self):
        return self.m_id

    @property
    def state(self):
        return O.bodies[self.id].state

    @property
    def material(self):
        return O.bodies[self.id].material

    @property
    def clumpId(self):
        return O.bodies[self.id].clumpId

# ------------------------------------------------------------------------ YadeObjectList
class YadeObjectList(ABC):
    """
    This is the abstract class for all list of objects
    E.g. brush
    """

    def __init__(self):
        self._generate_IDList()
        self._generate_StateList()
        self._generate_ClumpIDList()
        self._generate_MaterialList()

    @abstractmethod
    def _generate_IDList(self):
        pass

    @abstractmethod
    def _generate_StateList(self):
        pass

    @abstractmethod
    def _generate_ClumpIDList(self):
        pass

    abstractmethod
    def _generate_MaterialList(self):
        pass

    @property
    def id_list(self):
        return self.m_ids

    @property
    def state_list(self):
        return self.m_states

    @property
    def material_list(self):
        return self.m_materials

    @property
    def clumpId_list(self):
        return self.m_clump_ids
    