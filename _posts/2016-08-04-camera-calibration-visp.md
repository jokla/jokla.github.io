---
title:  "Intrinsic camera calibration for Nao/Romeo/Pepper with Visp"
excerpt: "Intrinsic camera calibration for Nao/Romeo/Pepper with Visp."
modified: 2016-08-04
categories: 
  - Robotics
tags:
  - ViSP
---

We will show here how to estimate the [camera intrinsic parameters](http://ksimek.github.io/2013/08/13/intrinsic/) for the robot Nao, Romeo or Pepper, using the [ViSP camera calibration tool](http://visp-doc.inria.fr/doxygen/visp-2.8.0/tutorial-calibration.html).

First of all we need to have 'ViSP',`visp_naoqi` and the C++ SDK from Softbank. You can follow [this guide](http://giovanniclaudio.com/robotics/visp_naoqi/). 

Once everything is working we can run the program to estimate the parameters:  

* Go to the build folder of `visp_naoqi` via terminal  
* Run the program `camera_calibration`:  

  ```
  Usage: ./sdk/bin/camera_calibration  [ --config <configuration file>.cfg] [--ip <robot address>] [--port <port robot>] [--cam camera_number] [--name camera_name] [--vga] [--help]
  ```

  Here the explanation of the options:

  * [ `--config <configuration file>.cfg`]  The path to a configuration file where we define the kind of pattern we are using ( size of the grid and  the dimension  of the circle/square). You can find two examples here:(default-chessboard.cfg)[https://github.com/lagadic/visp_naoqi/blob/master/tools/calibration/default-chessboard.cfg] or (default-circles.cfg)[https://github.com/lagadic/visp_naoqi/blob/master/tools/calibration/default-circles.cfg]  
  * [`--ip <robot address>`] Se the IP of the robot.  
  * [`--port <port robot>`] Se the port of the robot: default 9559.  
  * [`--cam camera_number`] Choose the camera you want to use. For Pepper and Nao 0 = TopCamera, 1 = BottomCamera.  
  * [`--name camera_name`] Set the name of the camera.  
  * [`--vga`] if you want to set the camera at the resolution of 640x480. Default resolution: 320x240  
  Example:

  ```
  $ ./sdk/bin/camera_calibration --config /udd/gclaudio/romeo/cpp/workspace/visp_naoqi/tools/calibration/default-circles.cfg --cam 0 --name cameraTopPepper --ip 131.254.10.126
  ```

