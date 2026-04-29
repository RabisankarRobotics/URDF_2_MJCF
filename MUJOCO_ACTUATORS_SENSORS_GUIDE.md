# MuJoCo Actuators & Sensors — When and How to Add
### Companion to URDF_TO_MUJOCO_GUIDE.md

---

## Quick Decision Matrix

| Your use case | Need actuators in XML? | Need sensors in XML? |
|---|---|---|
| **GMR retargeting** | ❌ No | ❌ No |
| **mink IK solver** | ❌ No | ❌ No |
| **IsaacLab / IsaacSim RL training** | ❌ No (added via `ArticulationCfg`) | ❌ No (added via `SensorCfg`) |
| **Raw MuJoCo simulation** (mj_step) | ✅ Yes | ✅ Yes (if reading sensors) |
| **mujoco-py / dm_control RL** | ✅ Yes | ✅ Yes |
| **Standalone MuJoCo viewer with control** | ✅ Yes | ✅ Optional |
| **Sim-to-sim validation** | ✅ Yes | ✅ Yes |

**Rule of thumb:** If you're using a high-level framework (IsaacLab, GMR) that wraps MuJoCo → it adds actuators/sensors for you. If you're talking to MuJoCo directly via `mj_step()` → you must add them in the XML.

---

## Why Our Files Don't Have Actuators or Sensors

Our `long_metal_biped.xml` and `tahiti_b1_metal.xml` were used for:
1. **GMR retargeting** — pure kinematics (no physics control needed)
2. **IsaacLab training** — actuators come from `ArticulationCfg.actuators` in `merai_humanoid_assets.py`

For both use cases, the XML only needs **joints** (which MuJoCo creates from URDF automatically). No actuators or sensors required.

---

## What Are Actuators?

In MuJoCo, an **actuator** defines HOW a joint is driven:
- **`motor`** — applies raw torque/force
- **`position`** — drives joint to a target position (PD controller)
- **`velocity`** — drives joint to a target velocity
- **`general`** — fully customizable

Without an actuator, a joint is **passive** — it can move freely under gravity/contact but cannot be controlled.

### Why URDF doesn't have actuators
URDF describes the **structure** (links + joints + limits). Control is left to the framework using the URDF (ROS controllers, MoveIt, etc.). MuJoCo follows this convention — it makes joints when converting URDF, but you must add actuators yourself.

---

## When to Add Actuators

### ✅ ADD them when:
- You'll run `mujoco.mj_step(m, d)` directly to simulate
- You need to control joints from Python (e.g., write `data.ctrl[:] = action`)
- You're doing sim-to-sim validation
- You want to deploy a trained policy via raw MuJoCo

### ❌ SKIP them when:
- You use IsaacLab → it adds them via `ArticulationCfg.actuators = {...}`
- You use GMR → it only needs kinematics
- You only render or do IK (no physics control)

---

## How to Add Actuators (When Needed)

Place an `<actuator>` block at the top level of the XML, AFTER `<worldbody>`:

```xml
<mujoco model="long_metal_biped">
  <compiler angle="radian"/>
  
  <asset>...</asset>
  <worldbody>...</worldbody>
  
  <actuator>
    <!-- One actuator per joint you want to control -->
    <position name="left_hip_pitch_actuator"
              joint="left_hip_pitch_joint"
              kp="250" kv="15"
              ctrlrange="-3.14 2.5"
              forcerange="-85 85"/>
    
    <position name="left_hip_roll_actuator"
              joint="left_hip_roll_joint"
              kp="250" kv="15"
              ctrlrange="-0.43 3.14"
              forcerange="-85 85"/>
    
    <!-- ... one per joint ... -->
    
    <!-- For ankle joints: lower stiffness -->
    <position name="left_ankle_pitch_actuator"
              joint="left_ankle_pitch_joint"
              kp="80" kv="10"
              ctrlrange="-0.87 0.52"
              forcerange="-20 20"/>
  </actuator>
</mujoco>
```

### Actuator parameters explained

| Parameter | Meaning | Typical for biped legs |
|---|---|---|
| `kp` | Position gain (stiffness) | 250 (hips/knees), 80 (ankles) |
| `kv` | Velocity gain (damping) | 15 (hips/knees), 10 (ankles) |
| `ctrlrange` | Min/max command value | Match joint range |
| `forcerange` | Max torque output | 85 Nm (hips), 20 Nm (ankles) |
| `joint` | Joint this actuator drives | Joint name from `<worldbody>` |

### Mapping to IsaacLab equivalent

If you've configured actuators in IsaacLab via `DelayedPDActuatorCfg`, the XML version mirrors them:

```python
# IsaacLab (used in your project)
"legs": DelayedPDActuatorCfg(
    joint_names_expr=[".*_hip_.*", ".*_knee_.*"],
    effort_limit=320,        # → forcerange in XML
    velocity_limit=13.0,
    stiffness=250,           # → kp in XML
    damping=15,              # → kv in XML
)
```

```xml
<!-- Equivalent XML actuator -->
<position joint="left_hip_pitch_joint" kp="250" kv="15"
          forcerange="-320 320"/>
```

---

## What Are Sensors?

**Sensors** in MuJoCo expose simulated measurements that you can read from `data.sensordata`:
- **IMU** (orientation, acceleration, angular velocity)
- **Force/torque** at joints or body sites
- **Contact sensors** (foot touching ground)
- **Joint position/velocity readings** with noise simulation

### Why URDF rarely has sensors
URDF was designed before sim-based RL was common. Most URDFs have NO sensor definitions. Sensors get added in the simulator config or XML.

---

## When to Add Sensors

### ✅ ADD them when:
- You need simulated IMU readings in raw MuJoCo
- You need contact force at feet
- You're doing sim-to-real with sensor noise simulation
- You want privileged sensor data for the critic in asymmetric RL

### ❌ SKIP them when:
- IsaacLab provides them via `ContactSensorCfg`, `ImuCfg`, etc.
- You only need joint pos/vel (read directly from `data.qpos`, `data.qvel`)
- You're doing pure kinematics (GMR)

---

## How to Add Sensors (When Needed)

Sensors require **sites** (named reference points on bodies) or directly attach to joints/bodies. Add `<sensor>` block at top level:

```xml
<mujoco>
  <worldbody>
    <body name="pelvis">
      <freejoint name="pelvis"/>
      
      <!-- Site for IMU placement -->
      <site name="imu_site" pos="0 0 0" size="0.01"/>
      
      <geom .../>
      <body name="left_ankle_roll_link">
        <!-- Site for foot contact -->
        <site name="left_foot" pos="0 0 -0.05" size="0.05" rgba="0 1 0 0.3"/>
        ...
      </body>
    </body>
  </worldbody>

  <sensor>
    <!-- IMU readings -->
    <framequat name="orientation" objtype="site" objname="imu_site"/>
    <gyro name="angular_vel" site="imu_site"/>
    <accelerometer name="linear_acc" site="imu_site"/>
    
    <!-- Foot contact -->
    <touch name="left_foot_touch" site="left_foot"/>
    <force name="left_foot_force" site="left_foot"/>
    
    <!-- Joint readings -->
    <jointpos name="left_hip_pitch_pos" joint="left_hip_pitch_joint"/>
    <jointvel name="left_hip_pitch_vel" joint="left_hip_pitch_joint"/>
  </sensor>
</mujoco>
```

### Common sensor types

| Tag | Returns | Use for |
|---|---|---|
| `<framequat>` | Body orientation (quaternion) | Robot upright check |
| `<gyro>` | Angular velocity | Orientation tracking |
| `<accelerometer>` | Linear acceleration | Step detection, sim IMU |
| `<touch>` | Contact normal force | Foot-on-ground detection |
| `<force>` | 3D force vector | Contact dynamics |
| `<jointpos>` | Joint angle (with noise option) | Encoder sim |
| `<jointvel>` | Joint angular velocity | Tachometer sim |

### Reading sensor data

```python
import mujoco
m = mujoco.MjModel.from_xml_path('robot.xml')
d = mujoco.MjData(m)
mujoco.mj_step(m, d)

# All sensor readings concatenated in data.sensordata
print(d.sensordata)

# Get specific sensor by name
sensor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "left_foot_touch")
adr = m.sensor_adr[sensor_id]
dim = m.sensor_dim[sensor_id]
foot_force = d.sensordata[adr:adr+dim]
```

---

## IsaacLab Equivalents (For Reference)

Since you'll mostly work in IsaacLab, here's how to add what XML actuators/sensors would do:

### Actuators (in `ArticulationCfg`)
```python
robot_cfg = ArticulationCfg(
    spawn=...,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_.*", ".*_knee_.*"],
            stiffness=250,
            damping=15,
        ),
    },
)
```

### Contact sensor (in `SceneCfg`)
```python
contact_forces = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/robot/.*_ankle_roll_link",
    history_length=3,
    track_air_time=True,
)
```

### IMU sensor
```python
imu = ImuCfg(
    prim_path="{ENV_REGEX_NS}/robot/base_link",
    update_period=0.0,
)
```

The IsaacLab approach is cleaner for RL because everything is configured in Python. You'd only edit the XML directly if you're moving to a simulator that doesn't provide these abstractions.

---

## Summary Decision Tree

```
Are you using IsaacLab/IsaacSim?
├── YES → Don't touch the XML
│         Add actuators in ArticulationCfg.actuators
│         Add sensors in SceneCfg (ContactSensorCfg, ImuCfg, etc.)
│
└── NO → Are you using raw MuJoCo (mj_step)?
         ├── YES → Add actuators to XML (required for control)
         │         Add sensors to XML (required for sensor reading)
         │
         └── NO → (using GMR / mink / pure visualization)
                  → Skip both — kinematics only
```

---

## When You Might Need to Edit the XML for Real

Even with IsaacLab, there are a few cases where you'd add actuators/sensors directly to the XML:

1. **Sim-to-sim validation** — train in IsaacLab, deploy in raw MuJoCo for testing
2. **MuJoCo MJX training** (alternative to IsaacLab) — needs actuators in XML
3. **Custom physics tests** — debugging robot dynamics outside IsaacLab
4. **Real robot control via ROS-MuJoCo bridge** — needs actuators

For the work in this project (GMR + IsaacLab), neither actuators nor sensors are needed in the XML.

---

## TL;DR

- **Actuators** = how joints are driven. Required for raw MuJoCo control, NOT required for IsaacLab (configured via Python) or GMR (kinematics only).
- **Sensors** = simulated measurement outputs. Required for raw MuJoCo if reading sim sensors, NOT required for IsaacLab (configured via SensorCfg) or GMR.
- **For our project:** Skip both — we use IsaacLab for control and GMR for kinematics. The XML stays minimal.
- **If you ever need them:** add `<actuator>` and `<sensor>` blocks at the top level of the XML, after `<worldbody>`.
