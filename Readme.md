Codebase for planning non-planar robotic 3D printing trajectories

### Install libraries

Run this the following in terminal from the root folder of this repo. All libraries are installed from source (in the [editable mode](https://pip.pypa.io/en/stable/reference/pip_install/#install-editable)).

```bash
# Install the submodule libraries from source
pip install -e .\external\compas
pip install -e .\external\compas_fab
pip install -e .\external\compas_fab_pychoreo

# install `integral_timber_joints` from source 
pip install -e .

# Run the following code add the python library paths to Rhino / Grasshopper:
python -m compas_rhino.install -p compas compas_fab compas_ghpython compas_rhino npp
```
