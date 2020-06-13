## Spot Mini Mini

### Kinematics and Gait:

Pybullet Environment and body manipulation with leg IK from: https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot

![SIK](spot_bullet/media/spot_rpy.gif)

Open-Loop Gait using 12-Point Bezier Curves based on: https://dspace.mit.edu/handle/1721.1/98270

Forward and Lateral Motion:

![SLAT0](spot_bullet/media/spot_lat_logic.gif)
![SLAT1](spot_bullet/media/spot_lat_demo.gif)


Yaw logic based on 4-wheel steering car: http://www.inase.org/library/2014/santorini/bypaper/ROBCIRC/ROBCIRC-54.pdf

![SYAW0](spot_bullet/media/spot_yaw_logic.gif)
![SYAW1](spot_bullet/media/spot_yaw_demo.gif)


### Controls
Assuming you have a Logitech Gamepad F310:

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

Changing `Step Length` while moving forward:

![SVMOD](mini_ros/media/steplen_mod.gif)

Yaw In Place: Slightly push the `Right Stick` forward while pushing the `Left Stick` maximally in either direction:

![SVMOD](mini_ros/media/yaw_in_place.gif)



### Reinforcement Learning
I've found that the Bezier Curve gait lends itself well to optimization via RL. Notice that the open-loop forward command drifts significantly over time (rougly 1m per 2m forward):

![DRIFT](spot_bullet/media/spot_drift.gif)

With a one-dimensional action space [`Yaw Rate`], and a 16-dimensional observation space [`IMU Inputs` (8), `Leg Phases` (4), `Leg Contacts` (4)], an `Augmented Random Search` agent (linear) was able to correct the trajectory after 299 epochs:

![NODRIFT](spot_bullet/media/spot_no_drift.gif)

Here is the policy output for this demo. It's clearly biased on one end to account for Spot's drift:

![NODRIFTPOL](spot_bullet/media/spot_no_drift_action.png)

### How To Run

#### Control:
First, you're going to need a joystick (okay, not really, but it's more fun if you have one).

**Setting Up The Joystick:**
* Get Number (you will see something like jsX): `ls /dev/input/`
* Make available to ROS: `sudo chmod a+rw /dev/input/jsX`
* Make sure `<param name="dev" type="string" value="/dev/input/jsX"/>` matches your setup in the launchfile

Then simply: `roslaunch mini_ros spot_move.launch`

You can ignore this msg: `[ERROR] [1591631380.406690714]: Couldn't open joystick force feedback!` It just means your controller is missing some functionality, but this package doesn't use it.

If you don't have a joystick, go to `spot_bullet/src` and do `./env_tester.py`. A Pybullet sim will open up for you with the same controls you would have on the joystick, except each is on its own scrollbar.

#### Reinforcement Learning
Go to `spot_bullet/src` and do `./spot_ars_eval.py`. When prompted, enter `299`. That's the best policy I have. Although, I have since modified the Bezier gait generator, so you might want to `git revert` to this commit: `96e2fb948947bcac2720e3ac01c65c19edbf308e`.



