## Simulation

This section contains information on the most up-to-date simulation solutions.

### Quickstart PyBullet

I have deployed a 12-point Bezier Curve gait to make Spot walk:

![PyBullet](../spot_bullet/media/spot-mini-mini.gif)

This example can be found in this [repository](https://github.com/moribots/spot_mini_mini). You can optionally use a Game Pad:
```
pip3 install numpy
pip3 install pybullet
pip3 install gym

cd spot_bullet/src

./env_tester.py
```

**Optional Arguments**

```
-h, --help          show this help message and exit
-hf, --HeightField  Use HeightField
-r, --DebugRack     Put Spot on an Elevated Rack
-p, --DebugPath     Draw Spot's Foot Path
-ay, --AutoYaw      Automatically Adjust Spot's Yaw
```

If you decide to use a controller, you can achieve some fairly fluid motions!

Changing Step Length:

![PyBullet](../mini_ros/media/steplen_mod.gif)

Yaw in Place:

![PyBullet](../mini_ros/media/yaw_in_place.gif)

### Kinematics

In this [repository](https://github.com/moribots/spot_mini_mini), there is a working IK solver for both Spot's legs and its body:

![IK](../spot_bullet/media/spot_rpy.gif)

