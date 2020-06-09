## Spot Mini Mini

### Kinematics and Gait:

Pybullet Environment and body manipulation with leg IK from: https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot

![SIK](bullet/media/spot_rpy.gif)

Open-Loop Gait using 12-Point Bezier Curves based on: https://dspace.mit.edu/handle/1721.1/98270

Forward and Lateral Motion:

![SLAT0](bullet/media/spot_lat_logic.gif)
![SLAT1](bullet/media/spot_lat_demo.gif)


Yaw logic based on 4-wheel steering car: http://www.inase.org/library/2014/santorini/bypaper/ROBCIRC/ROBCIRC-54.pdf

![SYAW0](bullet/media/spot_yaw_logic.gif)
![SYAW1](bullet/media/spot_yaw_demo.gif)


### Controls
Assuming you have a Logitech Joystick:

`A`: switch between stepping and RPY

`X`: E-STOP (engage and disengage)

**Stepping Mode**:

* `Right Stick Up/Down`: Step Length
* `Right Stick Left/Right`: Lateral Fraction
* `Left Stick Up/Down`: Robot Height
* `Left Stick Left/Right`: Yaw Rate
* `Arrow Pad Up/Down` (DISCRETE): Step Height
* `Arrow Pad Left/Right` (DISCRETE): Step Depth
* `Bottom Right/Left Bumpers`: Step Velocity (modulate)
* `Top Right/Left Bumpers`: reset all to default

**Viewing Mode**:

* `Right Stick Up/Down`: Pitch
* `Right Stick Left/Right`: Roll
* `Left Stick Up/Down`: Robot Height
* `Left Stick Left/Right`: Yaw

Changing `Step Velocity` while moving forward:

![SVMOD](mini_ros/media/stepvel_mod.gif)

### Reinforcement Learning
I've found that the Bezier Curve gait lends itself well to optimization via RL. Notice that the open-loop forward command drifts significantly over time:

![DRIFT](bullet/media/spot_drift.gif)

With a one-dimensional action space [`Yaw Rate`], and a 16-dimensional observation space [`IMU Inputs` (8), `Leg Phases` (4), `Leg Contacts` (4)], an `Augmented Random Search` agent (linear) was able to correct the trajectory after 299 epochs:

![NODRIFT](bullet/media/spot_no_drift.gif)

Here is the policy output for this demo. It's clearly biased on one end to account for Spot's drift:

![NODRIFTPOL](bullet/media/spot_no_drift_action.png)


