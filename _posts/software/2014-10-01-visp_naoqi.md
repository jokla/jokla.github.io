---
layout: post-sw
title: "VispNaoqi"
author_sw: G. Claudio, F. Spindler
website: http://jokla.github.io/visp_naoqi/
source: https://github.com/lagadic/visp_naoqi
excerpt: "Brigde over ViSP and Aldebaran SDK."
maintainer: G. Claudio, F. Spindler 
license: BSD
videos: [TgpLfgFSca8]
categories: "software"
tags: [Romeo, Aldebaran]
---
ViSPNaoqi is a bridge between ViSP and the Aldebaran SDK C++, the library that contains all the tools to manage  the robot. ViSPNaoqi is used mainly to grab images from Romeo, estimate the intrinsic and extrinsic camera parameters, control the robot in velocity and get the actual articular Jacobians of the robot.

### VispNaoqi grabber class:
 * Acquisition images from the Robot, convertion in OpenCv and Visp format, visualization.
 * Algoritm to estimate and save instrinsic and extrinsic parameters of the camera.

### VispNaoqi robot class:
 * Initialize connection with the robot.
 * Conversion from Visp to Naoqi data and vice-versa.
 * Kinematic model of the robot using  Metapod from LAAS.
 * Velocity controller.
