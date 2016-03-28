---
layout: post
title: Speech recognition using ROS and Pocketsphinx
excerpt: " Speech recognition using ROS and Pocketsphinx"
modified: 2016-03-28
tags: [robotics,howto]
comments: true
image:
  feature: sample-image-1.jpg
  credit:
  creditlink:
---

<section id="table-of-contents" class="toc">
  <header>
    <h3>Overview</h3>
  </header>
<div id="drawer" markdown="1">
*  Auto generated table of contents
{:toc}
</div>
</section><!-- /#table-of-contents -->

## Introduction
How to install pocketsphinx on ROS Indigo (Ubuntu 14.04).


## Install Dependecies
* `$ sudo apt-get install gstreamer0.10-pocketsphinx`
* `$ sudo apt-get install python-gst0.10`
* `$ sudo apt-get install gstreamer0.10-gconf`

## Clone and Build Pocketsphinx
* Clone the repository in your catkin src folder:
    * `$ git clone https://github.com/mikeferguson/pocketsphinx.git`
* Launch the catkin_make command from the catkin workspace folder

## Test Pocketsphinx
* Now we can start the speech recognizer:
    * `$ roslaunch pocketsphinx turtlebot_voice_cmd.launch`
* These are the basic commands can be recognized (see file voice_cmd.corpus in the folder demo):

``` text

forward
left
right
back
backward
stop
move forward
move right
move left
move back
move backward
halt
half speed
full speed

```

* Open another terminal to check if the words are recognized:
* `$rostopic echo /recognizer/output`

``` text
jokla@Dell-PC:~/catkin_ws/src$ rostopic echo /recognizer/output 
data: back
---
data: speed
---
data: move right
---
data: move back
---
data: back
---
data: left
---
data: speed
---
data: left
---
data: move right
---
data: stop
---
data: move right
---
data: move left
---
data: full speed
```

* To each vocal command corresponds a twist command. For example, this is the twist corresponding to the command "back":

``` shell
linear: 
  x: -0.4
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0`
```

## Command Turtlebot robot in Gazebo using voice commands
* Install [turtlebot_simulator](http://wiki.ros.org/turtlebot_simulator)
    * `$ sudo apt-get install ros-indigo-turtlebot-simulator`

* Open the launch file `turtlebot_voice_cmd` and remap the name of the topic cmd_vel to  `/mobile_base/commands/velocity`

``` xml
<node name="voice_cmd_vel" pkg="pocketsphinx" type="voice_cmd_vel.py" output="screen">
    <remap from="cmd_vel" to="/mobile_base/commands/velocity"/>
 </node>`
```
 
In this way the node pocketsphinx publishes the velocities in the topic `/mobile_base/commands/velocity` and gazebo subscribes to it.

* Launch simulation (click [here](http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Gazebo%20Bringup%20Guide) for more info). Note that Gazebo may update its model database when it is started for the first time. This may take a few minutes.
    * `$ roslaunch turtlebot_gazebo turtlebot_world.launch` 
* Now we can start the speech recognizer:
    * `$ roslaunch pocketsphinx turtlebot_voice_cmd.launch`
* You should be able to control Turtlebot using your voice.


