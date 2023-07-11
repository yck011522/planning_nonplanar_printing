from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation, Vector, Translation
from compas_fab.robots import Configuration

from compas.robots import ToolModel
from compas.datastructures import Mesh
from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles
from compas.files import URDF
from compas.robots import MeshDescriptor
from compas_fab.robots import Configuration

from npp import ToolExport

class PrintingTool (ToolModel):
    def __init__(self,
                 name,
                 tool_coordinate_frame,     #type: Frame # The frame that connects to the tool
                 collision_meshes = None,     #type: list[Mesh]
                 visual_mesheses = None,     #type: list[Mesh]
                 link_name=None
                 ):

        # Call Tool init
        super(PrintingTool, self).__init__(visual_mesheses, tool_coordinate_frame, collision_meshes, name=name, link_name=link_name)

        self.collision_meshes = collision_meshes
        self.visual_mesheses = visual_mesheses

    # --------------------------------------------------------------
    # Convienience functions
    # --------------------------------------------------------------

    @property
    def t_tcf_from_t0cf(self):
        return self.t_t0cf_from_tcf.inverse()

    @property
    def t_t0cf_from_tcf(self):
        return Transformation.from_frame(self.tool_coordinate_frame)
    
    # --------------------------------------------------------------
    # The collision mesh is stored in a Link object
    # --------------------------------------------------------------

    @property
    def collision_meshes(self):
        # type: () -> list[Mesh]
        """List of collision meshes"""
        tool_base_link = self.get_link_by_name('tool_base')
        return [collision_object.geometry.geo for collision_object in tool_base_link.collision]

    @collision_meshes.setter
    def collision_meshes(self, collision_meshes):
        """This can be either one or a list of meshes"""
        from compas.robots import Axis, Joint, Link
        if collision_meshes is not None:
            # Collision mesh stored in both visual and collision objects
            gripper_base = self.add_link('tool_base', visual_meshes = collision_meshes, collision_mesheses = collision_meshes)
            self.root = gripper_base
            self._create(self.root, Transformation())

    def get_base_link_name(self):
        return 'tool_base'