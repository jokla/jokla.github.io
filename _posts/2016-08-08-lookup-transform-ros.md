---
title:  "How to compute frame transformations with Tf (ROS) "
excerpt: "using the function lookupTransform"
modified: 2016-08-08
categories: 
  - Robotics
tags:
  - ROS
---


Here a script to compute the transformation between two frames published in /tf in ROS.
You can [donwload](https://gist.github.com/jokla/2630f8dbbd4391d91ab28b2f3f76801a/raw/fbf02a7ec13c4d7f61e9aea86719f866f18f539b/computeTF2frames.py) the python script, modify the name of the topics and run it with
`python computeTF2frames.py`

The result:

```
Translation:  (0.06290424180516997, -0.004345408291908189, 1.1515618071559173)
Rotation:  (-0.5041371308002005, 0.5088056423602352, -0.49116582568647843, 0.4957002151791443)

```

{% gist 2630f8dbbd4391d91ab28b2f3f76801a %}
