Ball Detector
================================

## Introduction

CHRISLab code for demonstrating simple color-based and blob-based ball detection.

Includes some basic [FlexBE] state implementations for integrating with the FlexBE Behavior Engine.

## Operation
---------

 `ros2 launch simple_ball_detector <launch file>.launch.py`

 * For testing based on fake detection messages, see `fake_ball_detector.launch.py` and associated scripts
 * For balls in simulation or real hardware, use `ball_detector.launch.py`
   * Use `usb_cam.launch.py` for hardware, or
   * See [chris_world_models] for adding balls to simulation

------

[FlexBE]: https://github.com/FlexBE
[chris_world_models]: https://github.com/CNURobotics/chris_world_models
