import os
from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles
from compas.robots import MeshDescriptor
from compas.files import URDF

class ToolExport(object):

    def __init__(self, tool_name, package_dir):
        self.tool_name = tool_name
        self.package_dir = package_dir

    # --------------------------------------------------------
    # URDF export/import
    # @property
    # def urdf_folder_path(self):
    #     return os.path.join(self.package_dir, self.tool_name, 'urdf')
    
    # @property
    # def urdf_path(self):
    #     return os.path.join(self.urdf_folder_path, self.tool_name + '.urdf')
    
    # @property
    # def visual_mesh_folder_path(self):
    #     robot_filepath = os.path.join(self.package_dir, self.tool_name, 'mesh', 'visual')
    #     return robot_filepath
    
    # @property
    # def collision_mesh_folder_path(self):
    #     robot_filepath = os.path.join(self.package_dir, self.tool_name, 'mesh', 'collision')
    #     return robot_filepath
    
    # def _export_shape(self, shape, folder_path, mesh_name, triangulize=True):
    #     """Export a single element to a mesh file and return the address"""
    #     assert isinstance(shape, MeshDescriptor)

    #     mesh = shape.geometry.copy()
    #     if triangulize:
    #         mesh_quads_to_triangles(mesh)

    #     shape.filename = mesh_name  # str(mesh.guid)
    #     try:
    #         sub_path = os.path.join(folder_path, mesh_name + '.stl')
    #         mesh.to_stl(os.path.join(self.package_dir, sub_path), binary=True)
    #     except:
    #         sub_path = os.path.join(folder_path, mesh_name + '.obj')
    #         mesh.to_obj(os.path.join(self.package_dir, sub_path))
        
    #     return 'package://' + sub_path.replace('\\', '/')

    # def save_as_urdf(self, scale=1.0, triangulize=True):
    #     # modified from: https://github.com/compas-dev/compas_fab/blob/6e68dbd7440fa68a58606bb9100495583bc79980/src/compas_fab/backends/pybullet/client.py#L235
    #     package_name = self.name or str(self.guid)
    #     self.ensure_geometry()

    #     visual_mesh_subdir = os.path.join(package_name, 'meshes', 'visual')
    #     collision_mesh_subdir = os.path.join(package_name, 'meshes', 'collision')
        
    #     """Make sure the folders exist"""
    #     os.makedirs(self.package_dir, exist_ok=True)
    #     os.makedirs(self.urdf_folder_path, exist_ok=True)
    #     os.makedirs(self.visual_mesh_folder_path, exist_ok=True)
    #     os.makedirs(self.collision_mesh_folder_path, exist_ok=True)

    #     # TODO this alter the object!
    #     self.scale(scale)
    #     # * write meshes to cache
    #     address_dict = {}
    #     mesh_scale = '{} {} {}'.format(scale, scale, scale)
    #     for link_id, link in enumerate(self.links):
    #         for v_eid, element in enumerate(link.visual):
    #             mesh_name = 'L{}_visual_{}'.format(link_id, v_eid)
    #             assert mesh_name not in address_dict
    #             shape = element.geometry.shape
    #             address_dict[mesh_name] = self._export_shape(shape, self.visual_mesh_folder_path, mesh_name, triangulize)
                
    #         for c_eid, element in enumerate(link.collision):
    #             self._export_element(element, collision_mesh_subdir, 'L{}_collision_{}'.format(link_id, c_eid),
    #                                  address_dict, triangulize)

    #     # * create urdf with new mesh locations
    #     urdf = URDF.from_robot(self)
    #     meshes = list(urdf.xml.root.iter('mesh'))
    #     for mesh in meshes:
    #         filename = mesh.attrib['filename']
    #         # TODO why sometimes filename is ''?
    #         if filename in address_dict:
    #             mesh.attrib['filename'] = address_dict[filename]
    #             mesh.attrib['scale'] = mesh_scale
    #     # write urdf
    #     urdf.to_file(self.urdf_path, prettify=True)

