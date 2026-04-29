# URDF → MuJoCo XML Conversion Guide
### When and how to use `<freejoint>` for humanoid/biped robots

---

## Why Convert?

URDF (Unified Robot Description Format) is the ROS standard. MuJoCo uses MJCF (`.xml`). Tools like IsaacLab and GMR run inside MuJoCo, so URDF must be converted before they can simulate the robot.

---

## Step 1: Basic Conversion Command

MuJoCo's Python API can load URDF directly and save it as MJCF:

```python
import mujoco

# Load URDF (MuJoCo parses it)
m = mujoco.MjModel.from_xml_path('/path/to/robot.urdf')

# Save as MJCF format
mujoco.mj_saveLastXML('/path/to/robot.xml', m)

print(f"DOF: {m.nv}, Bodies: {m.nbody}")
```

**Run this from the directory containing the URDF and meshes** so MuJoCo can resolve mesh references.

---

## Step 2: The Mesh Path Problem

### Symptom
```
ValueError: Error: Error opening file 'left_knee_link.STL'
```

### Cause
URDF files often have **absolute mesh paths**:
```xml
<mesh filename="/home/user/robot/meshes/pelvis.STL" />
```
But MuJoCo searches relative to the XML file's directory, not absolute paths.

### Fix
Make mesh paths relative by stripping the directory prefix:
```bash
sed 's|filename="/abs/path/to/meshes/|filename="|g' \
    original.urdf > fixed.urdf
```
Then place the STL files in the **same folder** as the URDF before conversion.

---

## Step 3: The Critical `<freejoint>` Problem

### Symptom
After conversion, your robot's root link disappears as a named body:

```
mink.exceptions.InvalidFrame: body 'pelvis' does not exist in the model.
Available body names: ['world', 'left_hip_pitch_link', ...]
                       ↑
                       Pelvis should be here but isn't!
```

### Cause
In a URDF, the root link has no parent joint. MuJoCo handles this by **merging the root link's geometry into `<worldbody>`** — the root link disappears as an addressable body.

```xml
<!-- After URDF→MJCF conversion -->
<worldbody>
  <geom type="mesh" mesh="pelvis"/>          ← pelvis geometry, but no <body> name!
  <body name="left_hip_pitch_link" ...>      ← children attach directly to world
    ...
  </body>
</worldbody>
```

This is **fine for fixed-base robots** (factory arms bolted to a table). But for **humanoid/biped robots that need to move around**, you must add a `<freejoint>` so the root body can translate and rotate freely in 3D space.

---

## When to Use `<freejoint>`

### ✅ USE IT for:
- **Humanoid robots** (walking, running, jumping)
- **Biped/quadruped legged robots**
- **Mobile manipulators**
- Any robot whose **base moves in the world** (not bolted down)

### ❌ DON'T USE IT for:
- **Fixed-base manipulators** (industrial arms, table-top robots)
- **Anything mounted to a wall/ceiling/table**
- **Wheeled robots with wheel joints** (wheels handle motion)

**Rule of thumb:** If your robot needs to fall under gravity or walk around, add a freejoint.

---

## How to Add `<freejoint>` After Conversion

After running the URDF → MJCF conversion, manually edit the XML:

### Before (broken — pelvis merged into worldbody):
```xml
<worldbody>
    <geom type="mesh" rgba="0.75 0.75 0.75 1" mesh="pelvis"/>
    <geom pos="0.04 0 0.05" type="mesh" mesh="imu_in_pelvis"/>
    <body name="left_hip_pitch_link" pos="0 0.13 -0.10">
      <joint name="left_hip_pitch_joint" axis="0 1 0" range="-3.14 2.5"/>
      ...
    </body>
    <body name="right_hip_pitch_link" ...>
      ...
    </body>
</worldbody>
```

### After (fixed — pelvis as a body with freejoint):
```xml
<worldbody>
  <body name="pelvis" pos="0 0 0.96">             ← NEW: wrap with body, set spawn height
    <freejoint name="pelvis"/>                    ← NEW: 6-DOF free motion
    <geom type="mesh" rgba="0.75 0.75 0.75 1" mesh="pelvis"/>
    <geom pos="0.04 0 0.05" type="mesh" mesh="imu_in_pelvis"/>
    <body name="left_hip_pitch_link" pos="0 0.13 -0.10">
      <joint name="left_hip_pitch_joint" axis="0 1 0" range="-3.14 2.5"/>
      ...
    </body>
    <body name="right_hip_pitch_link" ...>
      ...
    </body>
  </body>                                          ← NEW: close pelvis body
</worldbody>
```

### Three things to do:
1. Add `<body name="ROOT_NAME" pos="0 0 HEIGHT">` after `<worldbody>`
2. Add `<freejoint name="ROOT_NAME"/>` as the first child
3. Add a closing `</body>` before `</worldbody>` to close the wrapper

---

## What `pos="0 0 HEIGHT"` Does

This is the **initial spawn position** of the robot in world coordinates. Setting it correctly prevents the robot from spawning with feet through the ground.

### How to find the right height

```python
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path('robot.xml')
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

# Find lowest mesh vertex with all joints at zero
lowest_z = float('inf')
for i in range(m.ngeom):
    if m.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH:
        continue
    mesh_id = m.geom_dataid[i]
    verts = m.mesh_vert[m.mesh_vertadr[mesh_id]:
                        m.mesh_vertadr[mesh_id]+m.mesh_vertnum[mesh_id]]
    world_verts = verts @ d.geom_xmat[i].reshape(3,3).T + d.geom_xpos[i]
    lowest_z = min(lowest_z, world_verts[:,2].min())

print(f"Spawn height: pos=\"0 0 {-lowest_z + 0.02:.4f}\"")  # +2cm clearance
```

For our `long_metal_biped` this gave `pos="0 0 0.96"` (root at 96 cm = leg length + 2 cm clearance).

---

## What `<freejoint>` Adds Mechanically

A freejoint adds **6 degrees of freedom** to your robot's root body:
- 3 translation (x, y, z)
- 3 rotation (roll, pitch, yaw)

These DOFs are exposed in the simulator state:
```
qpos[0:3]   → root translation (x, y, z)
qpos[3:7]   → root rotation as quaternion (w, x, y, z)
qpos[7:]    → joint angles (your actual robot DOFs)
```

So if your robot has 12 joints, with a freejoint the total state vector has **18 DOFs** (6 freejoint + 12 joint).

---

## Verification After Adding Freejoint

```python
import mujoco
m = mujoco.MjModel.from_xml_path('robot.xml')

bodies = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
          for i in range(m.nbody)]
print('pelvis present:', 'pelvis' in bodies)  # should be True
print('Total DOF:', m.nv)                      # should be num_joints + 6
```

Expected output for our 12-joint biped:
```
pelvis present: True
Total DOF: 18
```

---

## Complete Example Workflow

```bash
# 1. Setup folder with URDF + meshes
mkdir robot_assets
cp /original/path/robot.urdf robot_assets/
cp /original/path/meshes/*.STL robot_assets/

# 2. Fix mesh paths to relative
sed -i 's|filename="/original/path/meshes/|filename="|g' robot_assets/robot.urdf

# 3. Convert URDF → MJCF
cd robot_assets
python -c "
import mujoco
m = mujoco.MjModel.from_xml_path('robot.urdf')
mujoco.mj_saveLastXML('robot.xml', m)
"

# 4. (Manual) Edit robot.xml to wrap root in <body> with <freejoint>
# 5. (Manual) Compute and set the spawn height (pos="0 0 X")
# 6. Verify

python -c "
import mujoco
m = mujoco.MjModel.from_xml_path('robot.xml')
print('DOF:', m.nv, 'Bodies:', m.nbody)
"
```

---

## Common Errors

| Error | Cause | Fix |
|---|---|---|
| `Error opening file '...STL'` | Absolute mesh paths in URDF | Strip path with `sed`, place STLs alongside XML |
| `body 'X' does not exist` | Root link merged into worldbody | Add `<body>` + `<freejoint>` wrapper |
| Robot falls through ground at start | `pos="0 0 0"` (default) | Compute correct spawn height (above lowest mesh) |
| Robot frozen in place | Forgot `<freejoint>` | Add it inside the root body |
| Two `<freejoint>` warnings | Added freejoint twice | Each body can have only ONE freejoint |
| `nv` is exactly num_joints | No freejoint added | Add one — `nv` should be `num_joints + 6` |

---

## TL;DR

1. **Convert URDF to MJCF** with `mujoco.mj_saveLastXML()`.
2. **Fix mesh paths** to be relative, place STL files next to the XML.
3. **For floating-base robots** (humanoids, bipeds): manually add `<body>` + `<freejoint>` around the root link, set `pos="0 0 HEIGHT"`.
4. **For fixed-base robots** (arms): no freejoint needed.
5. **Verify**: `pelvis` (or your root) should be a named body, total DOF = num_joints + 6 (with freejoint) or num_joints (without).
