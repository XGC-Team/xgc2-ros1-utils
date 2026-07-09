# XGC2 ROS1 Utils

`ros1_utils` collects the small ROS1-coupled helper surface shared by active
XGC2 controllers and estimators.

The supported include namespace is:

- `ros1_utils/*`

The package intentionally does not own controller runtime scheduling,
calibration, VRPN routing, or math/filter algorithms. Those responsibilities
belong to the controller, estimator, simulator bridge, and `xgc2_math` products.

Install:

```bash
sudo apt update
sudo apt install ros-melodic-xgc2-ros1-utils
```
