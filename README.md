# eYRC 2026–27 — Strata Cobot

**Team:** eYRC #4376  
**Author:** Saksham Kumar Aggarwal  
**Theme:** Strata Cobot (SC)  
**Platform:** Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic

This repository is my working archive for the **e-Yantra Robotics Competition 2026–27 — Strata Cobot**. It is intended to preserve the implementation, reasoning, debugging notes, and important competition requirements so the work remains useful as a reference after the competition.

---


## Task 1B — UR7e Cartesian Waypoints

### Result

**Task 1B completed: 40/40.**

- **AWR: 35/35** — all 5 waypoints reached in the required order.
- **Bonus B: 5/5** — every waypoint was reached within **0.03 m**.
- Each waypoint was held for the required **2 seconds**.
- The final run completed without a protective stop.

### Objective

Task 1B requires the UR7e TCP to move through five Cartesian waypoints expressed in `base_link`, in the specified order, and hold at each waypoint for at least two seconds.

The required waypoints are:

```text
1. (-0.4085, -0.5379, 0.1967)
2. (-0.8000, -0.0005, 0.3967)
3. (-0.7430,  0.5280, 0.1967)
4. (-0.4097,  0.5280, 0.1967)
5. (-0.0763,  0.5280, 0.1967)
```

The final implementation uses:

- `/joint_states` for joint feedback
- `/tcp_pose_raw` for Cartesian TCP feedback
- `/delta_joint_cmds` for initial joint-space unfolding
- `/delta_twist_cmds` for Cartesian waypoint motion
- `/ur_arm_controller/switch_controller` to switch between joint and Cartesian servoing

### Motion Strategy

The default UR7e starting configuration is relatively folded, with the elbow close to its lower joint limit. Sending Cartesian commands immediately from this configuration caused the controller to approach a singular configuration during development.

The final solution therefore uses a two-phase motion strategy:

```text
Default folded pose
        │
        ▼
Joint-space unfolding
        │
        ▼
Open / safer joint configuration
        │
        ▼
Switch to Cartesian twist control
        │
        ▼
WP1 → hold 2 s
        │
        ▼
WP2 → hold 2 s
        │
        ▼
WP3 → hold 2 s
        │
        ▼
WP4 → hold 2 s
        │
        ▼
WP5 → hold 2 s
        │
        ▼
Complete
```

The unfolding target is:

```text
shoulder_pan_joint   =  0.00 rad
shoulder_lift_joint  = -1.20 rad
elbow_joint          = -1.60 rad
wrist_1_joint        = -1.90 rad
wrist_2_joint        =  1.57 rad
wrist_3_joint        =  1.57 rad
```

Joint velocities are limited during this phase before switching to the Cartesian controller.

### Cartesian Control

The final controller uses **position-only Cartesian velocity commands**.

For the current TCP position `P` and waypoint `P_target`:

```text
error = P_target - P
```

The error is converted directly into a Cartesian linear velocity and capped at:

```text
0.025 m/s
```

No angular velocity is commanded:

```text
angular.x = 0
angular.y = 0
angular.z = 0
```

This is intentional because Task 1B evaluates the TCP position rather than requiring a specific TCP orientation. Avoiding unnecessary orientation constraints also helped prevent the Cartesian controller from driving the elbow toward a singular configuration.

### Waypoint Detection and Hold

A waypoint is considered reached when the Euclidean Cartesian distance is:

```text
distance <= 0.030 m
```

At that point the node:

1. Publishes zero Cartesian velocity.
2. Records the hold start time.
3. Maintains zero velocity for **2 seconds**.
4. Advances to the next waypoint.

The 3 cm threshold also satisfies the Task 1B bonus requirement, which was achieved for all five waypoints in the final evaluator run.

### Important Debugging Lessons

The main failure during development was a UR7e singularity caused by the Cartesian controller driving the elbow close to zero while approaching the first waypoint.

A second issue was an intermediate high-Z approach path. The controller could reach the waypoint's position but the staged path logic could become stuck when the current TCP height was already above the artificial approach height.

The final solution removed that unnecessary intermediate path and instead performs:

```text
unfold → direct Cartesian waypoint → hold → next waypoint
```

This produced the successful **40/40** evaluator result.

### Final Task 1B File

```text
task1B.py
```

The cleaned submission version follows the Strata Cobot coding standards while preserving the motion logic used by the successful evaluator run.

### Running Task 1B

Launch the simulation:

```bash
ros2 launch eyantra_kepler_colony task1b.launch.py
```

Build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select algorithms
source install/setup.bash
```

Run the node:

```bash
ros2 run algorithms task1B.py
```

For the evaluator:

```bash
cd ~/ros2_ws/src
./eyrc-sc-evaluator --task 1B --team-id 4376
```

Wait for:

```text
Ready
Start your node now.
```

Then start the solution node.

The evaluator generates a signed `result.zip`. It must be preserved exactly as produced.

## Task 1A — Ore Perception

### Result

**Task 1A completed: 20/20 + 2/2 bonus.**

The solution detects all six ores in the simulated conveyor scene, estimates their 3D positions using the RealSense RGB-D camera, transforms those positions into `base_link`, and continuously publishes the required TF frames.

### Objective

Task 1A requires the node to:

1. Detect six ores in the camera image:
   - 2 × Azurite — blue
   - 2 × Malachite — green
   - 2 × Vanadinite — orange
2. Estimate the 3D position of every ore.
3. Publish one TF transform per ore with `base_link` as the parent.
4. Keep publishing the settled transforms continuously for the duration of the evaluator run.
5. Produce an annotated detection image containing all six ores with labels matching the published TF child-frame names.

Required child frames:

```text
azurite_ore_1
azurite_ore_2
malachite_ore_1
malachite_ore_2
vanadinite_ore_1
vanadinite_ore_2
```

The six ore positions are randomized between launches, so the solution must **detect them dynamically rather than hardcoding coordinates**.

---

## 1. System Architecture

The complete Task 1A pipeline is:

```text
                 RealSense Camera
                       │
          ┌────────────┴────────────┐
          │                         │
      RGB image                Aligned depth
          │                         │
          ▼                         ▼
     HSV thresholding          Depth patch
          │                    + median filter
          ▼                         │
      Contours                      │
          │                         │
          ▼                         │
     Ore centroids                  │
          │                         │
          └──────────┬──────────────┘
                     │
                     ▼
              Pixel + depth +
             Camera intrinsics
                     │
                     ▼
          3D point in camera frame
                     │
                     ▼
                    TF2
                     │
                     ▼
            3D point in base_link
                     │
                     ▼
        TransformStamped published
                     │
                     ▼
                   /tf
```

The important conceptual separation is:

- **OpenCV** answers: *Where are the ores in the image?*
- **Depth + camera intrinsics** answer: *Where is that pixel in 3D relative to the camera?*
- **TF2** answers: *Where is that same point relative to `base_link`?*
- **ROS 2 TF broadcasting** makes the answer available to the rest of the system.

---

## 2. Camera Topics

Task 1A uses the following topics:

| Topic | Message | Purpose |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB image used for colour segmentation |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Depth aligned to the RGB image |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsic parameters |

The camera image is 640 × 480 in the Task 1A simulation.

The colour and depth images must be aligned because the centroid `(u, v)` obtained from the RGB image is used to index the corresponding depth pixel.

---

## 3. Image Processing

### Why HSV?

RGB values mix colour and brightness. HSV separates the hue component from saturation and value, making simple colour segmentation more practical for the three ore colours.

The implementation converts the BGR image to HSV and applies `cv2.inRange()` for each ore type.

### Current thresholds

```python
colour_ranges = {
    'azurite_ore': (
        np.array([100, 80, 50]),
        np.array([130, 255, 255]),
        200
    ),
    'malachite_ore': (
        np.array([60, 150, 80]),
        np.array([90, 255, 255]),
        150
    ),
    'vanadinite_ore': (
        np.array([5, 220, 200]),
        np.array([15, 255, 200]),
        150
    )
}
```

The final detector:

1. Converts BGR → HSV.
2. Creates a binary mask for each colour.
3. Finds contours using `cv2.findContours()`.
4. Rejects very small contours using an area threshold.
5. Keeps the two largest valid candidates of each ore type.
6. Calculates the centroid using image moments.
7. Draws the detected boundary and centre for debugging/submission output.

### Why morphology was not kept

Morphological operations were tested during development, but they distorted the ore masks enough that they were counterproductive for this particular simulated scene. The simpler HSV masks were stable enough without them, so the final implementation does not depend on morphology.

---

## 4. Stable Ore IDs

There are two ores of every type, so detecting `azurite_ore` is not enough. The node must consistently decide which physical ore is `_1` and which is `_2`.

The implementation keeps two previous pixel-space positions for every ore type:

```text
azurite_ore   → [previous position of #1, previous position of #2]
malachite_ore → [previous position of #1, previous position of #2]
vanadinite_ore → [previous position of #1, previous position of #2]
```

New detections are associated with the nearest existing detection **in pixel space**.

This is important because the Task 1A instructions explicitly require stable identities and caution against matching detections using depth. The numbering itself can be arbitrary; consistency is what matters.

---

## 5. Depth Processing

The RealSense Task 1A depth stream is:

```text
Encoding: 32FC1
Type: float32
Units: metres
```

It represents **z-depth**, i.e. perpendicular distance from the camera plane, rather than distance measured along the viewing ray.

Depth pixels can be invalid or non-finite. Therefore, the implementation does not blindly use one depth pixel.

For a centroid `(cX, cY)`, it samples a 9 × 9 window:

```python
patch = self.depth_image[cY-4:cY+5, cX-4:cX+5]
```

and selects finite, positive values before taking the median.

Conceptually:

```text
single pixel depth
       ↓
   noisy / invalid
       ↓
small neighbourhood
       ↓
finite + positive values
       ↓
      median
       ↓
 stable Z estimate
```

The median is useful because depth edges can be noisy and individual pixels can contain invalid values.

---

## 6. Pixel → 3D Camera Coordinates

Once the centroid `(u, v)` and depth `Z` are known, the pinhole camera model gives the 3D point in the camera optical frame.

Using the intrinsics from `CameraInfo`:

```text
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
Z = depth
```

Where:

- `fx`, `fy` = focal lengths in pixels
- `cx`, `cy` = principal point
- `(u, v)` = image pixel
- `Z` = depth in metres

The important camera optical-frame convention is:

```text
+X → right
+Y → down
+Z → forward
```

The final implementation reads `fx`, `fy`, `cx`, and `cy` from `/camera/camera/color/camera_info` rather than hardcoding them.

Previously observed simulation values were approximately:

```text
fx = 457.650
fy = 457.650
cx = 320.0
cy = 240.0
```

These are useful for understanding/debugging, but the node obtains the actual values from `CameraInfo` at runtime.

---

## 7. TF2: Camera Frame → `base_link`

The 3D point initially exists in:

```text
camera_color_optical_frame
```

but the evaluator expects the ore transform to be relative to:

```text
base_link
```

The solution therefore creates a `PointStamped`, assigns the camera optical frame, looks up the TF2 transform, and uses `do_transform_point()`.

Conceptually:

```text
P_camera
   │
   │  TF2
   ▼
P_base_link
```

Mathematically this is:

```text
P_B = ᴮT_A P_A
```

where `A` is the camera optical frame and `B` is `base_link`.

### Observed camera → base transform

During development, the stable transform was approximately:

```text
Translation:
[-1.152, -0.020, 1.238]

Quaternion [x, y, z, w]:
[-0.659, 0.660, -0.255, 0.254]

RPY:
[-137.800°, 0°, -90.091°]
```

Approximate homogeneous matrix:

```text
[-0.002  -0.741   0.672  -1.152]
[-1.000   0.001  -0.001  -0.020]
[-0.000  -0.672  -0.741   1.238]
[ 0       0       0        1    ]
```

**Important:** this transform is not hardcoded into the solution. TF2 provides the transform from the live simulation tree.

This was one of the most important lessons from Task 1A: **do not manually rebuild a TF tree when ROS is already providing the required relationship.**

---

## 8. Ore Transform Publication

Every ore is published as a `TransformStamped` with:

```text
Parent: base_link
Child:
    azurite_ore_1
    azurite_ore_2
    malachite_ore_1
    malachite_ore_2
    vanadinite_ore_1
    vanadinite_ore_2
```

Only translation is relevant to scoring.

The rotation is nevertheless required to be a valid quaternion. The implementation therefore publishes the identity rotation:

```python
transform.transform.rotation.w = 1.0
```

with `x = y = z = 0`.

Transforms are broadcast **continuously**, not just once.

---

## 9. Top Face vs. Physical Centre

One of the easiest details to misunderstand is the evaluator's reference point.

The ore is approximately:

```text
Width:  101.6 mm
Height: 76.2 mm
```

The evaluator scores the transform against the **centre of the ore's top face**.

The physical centre is 38.1 mm below the top face.

The Task 1A pass band is:

```text
≤ 0.05 m straight-line 3D error
```

and the bonus band is:

```text
≤ 0.03 m
```

Since the RealSense depth measurement is taken from the visible top face, the correct approach for the high-accuracy solution is to use that top-face position directly rather than subtracting the ore height.

This is a good example of why understanding what a sensor actually measures matters more than blindly applying a geometric correction.

---

## 10. Task 1A Scoring

The official scoring formula is:

```text
Task 1A Marks = (CP × 3) + B
```

For the main score:

- The node must publish a TF transform for each ore.
- `base_link` must be the parent frame.
- The transform must be within 0.05 m of the ore's top-face centre in straight-line 3D distance.
- Each correctly located ore is worth 3 marks.
- There are six ores, giving a maximum of 18 marks from correct positions.
- The frame name is part of the requirement; an incorrectly named frame is not counted.

The remaining 2 marks are available through the Task 1A bonus criterion.

---

## 11. Detection Image Requirement

The submission requires one annotated OpenCV image containing **all six ores at once**.

Each ore must have:

- its detected boundary drawn, and
- a label exactly matching the transform child frame.

Required filename for team 4376:

```text
SC#4376_task1A_detection.png
```

The code writes the annotated image using:

```python
cv2.imwrite('SC#4376_task1A_detection.png', annotated_frame)
```

---

## 12. Running Task 1A

### Launch the simulation

```bash
ros2 launch eyantra_kepler_colony task1a.launch.py
```

Wait for Gazebo to open and the ores to appear.

### Start the evaluator

The evaluator must be started before the solution node, and the node should only be started **after the evaluator prints `Ready`**. Anything published before that point is not part of the scored run.

### Run the node

The final node is:

```text
Task_1A/task1A.py
```

When running through the competition workspace, make sure the workspace is built and sourced so that `ros2 run` executes the current installed copy.

A common ROS 2 workflow is:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

After modifying a node:

```bash
colcon build --packages-select algorithms
source install/setup.bash
```

Then verify the executable with:

```bash
ros2 pkg executables algorithms
```

---

## 13. Official Task 1A Submission Package

The competition submission for Task 1A consists of three files at the top level of the archive:

```text
SC#4376_task1A.zip
├── result.zip
├── task1A.py
└── SC#4376_task1A_detection.png
```

`result.zip` is generated by the evaluator and must be preserved exactly as produced.

The GitHub repository is a **development/reference archive**, not a replacement for the official submission ZIP.

---

## 14. Coding Standards Used

The final `task1A.py` was cleaned according to the Strata Cobot coding standards.

Important conventions applied:

- File header contains Team ID, Theme, Author List, Filename, Functions, and Global variables.
- Module-level functions are listed in `Functions`.
- Node classes have a class-level docstring describing their purpose and ROS interfaces.
- Functions and methods document:
  - Purpose
  - Input Arguments
  - Returns
  - Example call
- Important variables have descriptive comments.
- Non-obvious implementation steps have short explanatory comments.
- The final documentation cleanup was verified without changing the executable AST of the working algorithm.

---

## 15. What I Actually Learned from Task 1A

### Computer vision

- Why HSV is useful for colour segmentation.
- How `cv2.inRange()` produces binary masks.
- How contours represent connected regions.
- How contour area filtering removes noise.
- How image moments give an object's centroid.
- Why morphological operations are not automatically beneficial.

### RGB-D perception

- Why RGB and depth must be aligned.
- Why one depth pixel is unreliable.
- How a small median depth window improves robustness.
- Why invalid/NaN depth values must be filtered.
- The difference between z-depth and distance along a ray.

### Camera geometry

- How a 2D pixel becomes a 3D point when depth is known.
- The role of focal length and principal point.
- The meaning of the camera optical-frame axes.
- Why CameraInfo should provide the intrinsics instead of hardcoding them.

### Coordinate transforms

- The meaning of a homogeneous transform.
- The difference between a point's coordinates and the frame in which those coordinates are expressed.
- How TF2 maintains the transform tree.
- How to transform a `PointStamped` between frames.
- Why the final evaluator frame matters.

### ROS 2

- Creating subscribers for camera streams.
- Using `CvBridge` to convert ROS images into OpenCV arrays.
- Reading `CameraInfo`.
- Creating a TF2 buffer/listener.
- Looking up transforms.
- Broadcasting `TransformStamped` messages.
- Running periodic processing with a ROS 2 timer.
- Understanding the difference between source files and the installed copy launched by `ros2 run`.

### Robotics software engineering

The biggest lesson was that the final algorithm is a chain of relatively small transformations:

```text
Image
→ Detection
→ Pixel coordinate
→ Depth
→ Camera-frame 3D point
→ TF2 transformation
→ base_link position
→ Named TF frame
```

When debugging, each stage can be tested independently instead of treating the entire robot perception system as one giant black box.

---

## 16. Debugging Lessons

### Do not start by changing everything

When the system fails, isolate the stage that is wrong:

```text
Are the ores visible?
        ↓
Are the colour masks correct?
        ↓
Are the contours correct?
        ↓
Are the centroids correct?
        ↓
Is depth valid at those pixels?
        ↓
Is the camera-frame 3D point correct?
        ↓
Does TF2 return the expected transform?
        ↓
Is the base_link position sensible?
        ↓
Is the published frame named correctly?
```

### Validate coordinates numerically

Visual debugging is useful, but printing actual coordinates exposed mistakes much faster than looking at an image alone.

### Keep evaluator requirements in mind

A mathematically correct solution can still fail if it violates an interface requirement such as:

- wrong parent frame,
- wrong child-frame name,
- wrong detection-image filename,
- publishing only once,
- starting before the evaluator is ready,
- using hardcoded ore locations.

---

## 17. Final Task 1A File

```text
Task_1A/
└── task1A.py
```

This is the cleaned version of the solution used for the successful Task 1A run.

The executable algorithm was kept unchanged during the final coding-standard cleanup; only documentation/comments/header information were improved.

---

## 18. Useful References

Official e-Yantra Strata Cobot resources:

- [eYRC Strata Cobot repository](https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot)
- [e-Yantra](https://www.e-yantra.org/)
- [Task 1A Instructions](https://portal.e-yantra.org/courses/theme_sc/stage_1/task_1/task_1a/instruction/)
- [Task 1A Submission](https://portal.e-yantra.org/courses/theme_sc/stage_1/task_1/task_1a/submission/)
- [From Pixels to a Pose](https://portal.e-yantra.org/courses/theme_sc/learnings/image_processing/pose_estimation/)
- [Image Processing Basics](https://portal.e-yantra.org/courses/theme_sc/learnings/image_processing/image_processing/)
- [Transforms and tf2](https://portal.e-yantra.org/courses/theme_sc/learnings/ros/tf2/)
- [The RealSense Camera](https://portal.e-yantra.org/courses/theme_sc/learnings/hardware/realsense/)
- [Writing a ROS 2 Node in Python](https://portal.e-yantra.org/courses/theme_sc/learnings/ros/writing_a_node/)
- [Coding Standards](https://portal.e-yantra.org/courses/theme_sc/learnings/python/coding_standards/)

---

## Next Tasks

Task 1 consists of three subtasks:

| Subtask | Objective |
|---|---|
| **1A** | Detect the six ores and publish their positions as TF2 transforms |
| **1B** | Move the UR7e end effector through the specified waypoints |
| **1C** | Navigate the eBot through the required task |

Task 1A is complete. This repository will be extended with the subsequent task implementations and notes as the competition progresses.


====================================================================================================================================================================================================================


# e-Yantra Robotics Competition 2026-27 — StrataCobot (SC)

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic

## 1. Install the task dependencies

From this directory:

```bash
./requirements.sh
```

It installs what the task needs on top of an existing ROS 2 Jazzy. It does not install
ROS itself.

## 2. Build the workspace

```bash
cd ..
colcon build
source install/setup.bash
```

Add the `source` line to your `~/.bashrc`.

## 3. Start the simulation

Each task has its own launch file. Leave it running in its own terminal.

```bash
ros2 launch eyantra_kepler_colony task0.launch.py    # Task 0 — the whole arena
ros2 launch eyantra_kepler_colony task1a.launch.py   # Task 1A — ore perception
ros2 launch eyantra_kepler_colony task1b.launch.py   # Task 1B — arm waypoints
ros2 launch eyantra_kepler_colony task1c.launch.py   # Task 1C — eBot navigation
```

None of them starts a solution node. That part is the task.

Add `rviz:=false` to any of them to start without RViz.

## 4. Write and run your node

Your code goes in the **`algorithms`** package. Copy the boilerplate for your subtask into
the matching folder and rename it to whatever you like:

```bash
cd algorithms
cp boilerplate/task1b_boilerplate.py scripts/task1b/arm_waypoints.py
chmod +x scripts/task1b/arm_waypoints.py
```

Add that path to the `SCRIPTS` list in `algorithms/setup.py`. **The entry is a path from
the package root, not a bare file name:**

```python
SCRIPTS = [
    'scripts/task1b/arm_waypoints.py',
]
```

Build, source, and run it. The executable name is the **file name, `.py` included**:

```bash
cd ..
colcon build --packages-select algorithms
source install/setup.bash
ros2 run algorithms arm_waypoints.py
```

Three things that catch people out:

* A file is not installed until it is in `SCRIPTS` **and** you have built again.
* Without `chmod +x` it installs fine and then refuses to start.
* `ros2 run` starts the **installed** copy, so rebuild and re-source after every edit.

`ros2 pkg executables algorithms` lists what is actually installed.

The topics and services your node works with are on the task page.

## 5. Run the evaluator

In a second terminal, with the simulation still running and the workspace sourced:

```bash
./eyrc-sc-evaluator --task 1B --team-id 1455
```

Both arguments can be left out and it will ask for them. It checks your setup, prints
`Ready`, and records your run. **Start your node only after `Ready`.** Press `q` when your
run has finished (`Ctrl-C` if you are not on an interactive terminal).

It writes a **`result.zip`** into a folder named for your team and subtask. Do not unpack
it, rebuild it or rename anything inside it — it is signed, and editing it makes it
unreadable.

## 6. Submit

Copy your node into the folder the evaluator wrote, **renamed as the task page says**,
compress the contents together, and upload that one archive.

```
SC#1455_task1B.zip
  ├── result.zip          from the evaluator, untouched
  └── task1B.py           your node, renamed
```

**Take the archive name, the file names and the full list of what goes in from the task
page** — it differs per subtask, and some subtasks ask for more than these two files.

---

The UR7e description in `ur_description` is modified by e-Yantra from the upstream
Universal Robots ROS 2 description; see `ur_description/LICENSE`.
