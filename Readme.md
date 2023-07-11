Codebase for planning non-planar robotic 3D printing trajectories

### Install libraries

Run this the following in terminal from the root folder of this repo. All libraries are installed from source (in the [editable mode](https://pip.pypa.io/en/stable/reference/pip_install/#install-editable)).



```bash
# install `integral_timber_joints` from source 
pip install -e .

# Run the following code add the python library paths to Rhino / Grasshopper:
python -m compas_rhino.install -p compas compas_fab compas_ghpython compas_rhino npp
```

Note that the following forked libraries are installed from the submodule directories. They are specific forks of the original repository and should be installed locally.
```bash
# The following do not have to be run manually.
# pip install -e .\external\compas
# pip install -e .\external\compas_fab
# pip install -e .\external\compas_fab_pychoreo
```

## General Setup Steps

1. **Setup robot definitions (default location is `.\robot` folder)**

   Robots can be defined as URDF packages that consist of URDF, SRDF, collision meshes and visual meshes.  

2. **Setup tool definitions (default location is `.\tool` folder)**

   Tools can be defined using the Tool Model class in compas_fab. It is similar to a robot definition in the sense that it consists of collision meshes and visual meshes. 

   It can also consist of multiple kinematic joints, similar to the robot, visual and collision meshes. In addition, it consist of a tool tip transformation `ToolModel.frame`, it represents the printing tip of the tool relative to the tool0 frame (a.k.a the base of the tool). By convention, the Z-axis of the printing tool should point away from the workpiece and towards the robot (typically upwards).

   The procedure to define a tool can be performed in Rhino to 3D model the tool, and using Grasshopper or RhinoPython to access the `compas.robot.ToolModel` class.

3. **Encode designs**

   Assuming you have a single continuous print path as input. You will need to specify the path as a `npp.PrintPath` object, which is essentially a list of [`compas.geometry.Frames`]. For planar prints, the normal vectors are pointing upwards (Z positive). However for non-planar prints, the normal vectors may be pointing in a different orientations.

   The X direction of the frame can be specified, depending on the solving mode that is defined later, it may or may not be respected.   

   If there are multiple print paths that are discontinuous in-between them. It is best to specify them as a list of `npp.PrintPath` objects.

4. **Define robotic movement** 

   The `PrintPath` object can be directly converted to a `npp.PrintMovement` object. After which it is possible to specify different planning parameters and outcomes. This includes:

   - Plan with or without checking for collisions
   - Plan with or without respect for the X/Y direction of the frames
   - Plan with or without respect to printing speed and max robot joint velocity.

   This step (and together with the next step) can be completed in Grasshopper or in VSCode.

5. **Specify lead-in and lead-out movement**

   It is common to include lead-in and lead-out movements before and after the print path. 

   - At the beginning, this should at least include a `npp.FreeMovement` to go from the previous location (typically a fixed starting config) to the start location, and to turn on the extruder with `npp.ExtruderOn`.
   - At the end, it should at least turn off the extruder with `npp.ExtruderOff`, and possibly go back to a preprogrammed position (can also be the starting configuration).
   - You can also introduce a short segment of `npp.LinearMovement` at the beginning or the end if necessary.

   Notice all of the robotic movements mentioned above can be provide with the following optional parameters.

   - Kinematic Chain (a list of`ToolModel`, by default is no tool)

   - Fixed Target Configuration (optional, for Linear and Free Movement only)

   

6. **Preplanning check**

   In general it is beneficial to check whether the movements are defined correctly. One method is to perform an IK check on all the target positions to make sure that they can be 

7. **Plan trajectories**

   After all the movements (robotic and tool movements) are created, it is stored in a `npp.TaskPlan` object. Note that it is necessary to specify a starting robot configuration at the beginning of the `TaskPlan`, in many cases, the 'zero configuration' of the robot can be used if it does not cause a collision.

   After the task plan is defined, you can leave it to the computer to do the rest of the planning. Note that motion planning is a stochastic process and it may take a long time to find a viable plan. There is also no guarantee that is a valid solution. Therefore, a timer is used to keep track of planning time (do not confuse with robot execution time), if there are no solutions within a given time, the planner should give up. In practice, it is best to get a feeling for how long it takes to plan a simple motion and to set the time-out accordingly.

   

## Debug

If planning fails to find a solution, there are some steps that can be used to identify the problem. There are three typical scenarios:

1. Some movements are not defined correctly (e.g. some targets in collision)
2. Consecutive motions causes their neighbors to fail. 
3. Some movements are over-constrained along its path and the robot kinematics (often 6 or 7 DOF) cannot follow the specified path while respecting the constrains. The following motions are listed in increasing difficulty: 
   1. Free Movement (unconstrained along path)
   2. Print Movement (4 DOF constrained along path)
   3. Linear Movement (Highly constrained - 6 DOF constrained along path)
4. Some movements are really difficult (e.g. narrow path problems)

The debug process should start by identifying if your motions belongs to one of the three problems:

1. Perform pre-planning check to make sure movement targets are not already in collision. 
2. Reduce the number of movements in the TaskPlan and try to have that working first. Progressively add more movements and identify what addition causes problem.
3. Try to turn off collision checking during planning. 

## Collision Geometry

A general notes for collision geometry belonging to the robot (in the URDF package), the Tool (defined as `ToolModel` class), and the static environment.

For **visual meshes**, you can model whatever pleases you. They are not used for collision detection and it is possible to detail them enough for visualization and rendering purpose. (Note that very detailed meshes can still slow sown visualization, aka. playback) 

The most simple method to create the **collision geometries** is to create low-poly meshes.  Ensure the following:

- Triangulate the meshes before using them for collision purposes
- Low-Poly enough to allow fast Collision Checks. 
  - 50 to 500 faces per mesh for each link is reasonable
  - Most static collision geometry should be represented by simple box-like meshes.
- The collision meshes are physically larger than the object they protect. A small offset from the actual object is desirable if the physical object or the robot is not accurate. The amount of offset can be equal to the combined maximum uncertainty of the robot position, and the object geometry.



## Defining Movements 

### PrintMovement

This is a movement from wherever the robot was previously, to a list of Cartesian Target (Frame), In general only the tool tip point and the normal axis is respected.  

### LinearMovement

This is a movement from wherever the robot was previously, to one single Cartesian Target (Frame), linear 

### FreeMovment
