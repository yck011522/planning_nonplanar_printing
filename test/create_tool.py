# Testing the creation of PrintingTool Class
#
import os, json
from compas.robots import ToolModel
from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.files import OBJ
from compas.data import DataEncoder

urdf_package_path = os.path.join('test_result', 'tool', 'create_tool')
os.makedirs(urdf_package_path, exist_ok=True)


# Load collision meshes from obj files
collision_meshes = []
visual_mesheses = []
collision_meshes.append(Mesh.from_obj('tool/otto1/collision_1.obj'))
collision_meshes.append(Mesh.from_obj('tool/otto1/collision_2.obj'))
collision_meshes.append(Mesh.from_obj('tool/otto1/collision_3.obj'))
collision_meshes.append(Mesh.from_obj('tool/otto1/collision_4.obj'))
visual_mesheses.append(Mesh.from_obj('tool/otto1/visual_1.obj'))

# Create a ToolModel with multiple collision mesh
tool = ToolModel(visual_mesheses,
                 Frame.worldXY(),
                 collision_meshes=collision_meshes,
                 name="otto",
                 link_name=None)

print(tool)

for link in tool.links:
    link_name = link.name

    
    os.makedirs(os.path.join(urdf_package_path, tool.name, 'meshes', 'visual'),exist_ok=True)
    for i, visual in enumerate(link.visual):
        mesh_descriptor = visual.geometry.shape
        # Change the filename of the visual mesh to be exported
        file_path = [tool.name, 'meshes', 'visual', link_name + '_%i.obj' % i]
        mesh_descriptor.filename = 'package://' + '/'.join(file_path)
        # Export the visual mesh to obj file
        obj = OBJ(os.path.join(urdf_package_path, *file_path))
        obj.write(mesh_descriptor.meshes)

    os.makedirs(os.path.join(urdf_package_path, tool.name, 'meshes', 'collision'),exist_ok=True)
    for i, visual in enumerate(link.collision):
        mesh_descriptor = visual.geometry.shape
        # Change the filename of the visual mesh to be exported
        file_path = [tool.name, 'meshes', 'collision', link_name + '_%i.obj' % i]
        mesh_descriptor.filename = 'package://' + '/'.join(file_path)
        # Export the visual mesh to obj file
        obj = OBJ(os.path.join(urdf_package_path, *file_path))
        obj.write(mesh_descriptor.meshes)

# Create folder for tool export if it didn't exist
os.makedirs(urdf_package_path, exist_ok=True)
tool.to_urdf_file(os.path.join(urdf_package_path, 'otto1.urdf'), True)

# Save tool to json
with open(os.path.join(urdf_package_path, 'otto1.json'), 'w') as f:
    json.dump(tool,cls=DataEncoder, fp=f, indent=2)