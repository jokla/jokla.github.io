---
layout: post-sw
title: "VispNaoqi"
author_sw: G. Claudio, F. Spindler
website: http://jokla.github.io/visp_naoqi/
source: https://github.com/lagadic/visp_naoqi
excerpt: "Brigde over ViSP and Aldebaran SDK."
maintainer: G. Claudio, F. Spindler 
license: BSD
videos: [kz1Ob0Ks554]
categories: "software"
tags: [Romeo, Aldebaran, ROS]
---
The library is composed mainly by two classes:
* VispNaoqi grabber:
 * Acquisition images from the Robot, convertion in OpenCv and Visp format, visualization.
 * Algoritm to estimate and save instrinsic camera parameters and extrinsic parameters.
* VispNaoqi robot:
 * Kinematic model of the robot using  Metapod from LAAS.
 * Velocity controller.
