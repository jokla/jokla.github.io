---
title: "Giovanni Claudio"
layout: splash
excerpt: "Robotics engineer specialized in sensor-based robot control, visual servoing, computer vision and machine learning."
embedded: <iframe src='//padlet.com/embed/vv562ia8ablc' frameborder='0' width='100%' height='400px' style='padding:0;margin:0;border:none'></iframe>
header:
  overlay_color: "#000"
  overlay_filter: "0.7"
  overlay_image: /work/bk.jpg
  cta_label: "Download my CV"
  cta_url: "/share/Claudio_CV.pdf"
  #caption: "Photo credit: [**Unsplash**](https://unsplash.com)"
permalink: /work/
intro: 
- excerpt: "I am currently a R&D Robotics Engineer working in the [Lagadic](http://www.irisa.fr/lagadic/) group at [INRIA](http://www.inria.fr/en/) in Rennes."
feature_row:
  - image_path: /work/lagadic_2016.jpg
    alt: "Lagadic"
    title: "My current lab"
    excerpt: 'Research activities of the [Lagadic](http://www.irisa.fr/lagadic/) team deal with robot vision, visual servoing, real-time visual tracking and SLAM for applications in localization, manipulation, navigation, medical robotics and augmented reality. Visual servoing consists in using the information provided by a vision sensor to control the movements of a dynamic system. Such systems are usually robot arms, or mobile robots, but can also be virtual robots, or even a virtual camera.'
    url: "http://www.irisa.fr/lagadic/"
    btn_label: "Read More"
    #btn_class: "btn--inverse"
job:
  - image_path: /work/giovanni.claudio.romeo.jpg
    alt: "My job"
    title: "My job"
    excerpt: 'In November 2013 I joined the team [Lagadic](http://www.irisa.fr/lagadic/)  in INRIA Rennes led by [François Chaumette](http://www.irisa.fr/lagadic/team/Francois.Chaumette-eng.html) with the role of R&D Robotics Engineer.

My goal is to make robots smarter, helping them to perceive and understand our world and to take action autonomously.

I work on real-time detection, localization, pose estimation and tracking of different kinds of targets and I implemented state-of-the-art visual servoing algorithms that significantly improved the robustness and accuracy of several types of robots (mobile, humanoid, industrial robots and drones).  To validate these approaches, I created numerous demonstrations using 2D and RGB-D cameras, radars and microphones. 

I also worked on improving the perception and motion of the humanoid robots Romeo and Pepper. These robots can now track a target with their gaze, detect and follow a person, detect and grasp objects, deliver them to a human, manipulate them using two hands simultaneously and open a door.

I have contributed to the development of a framework based on ROS, MATLAB/Simulink, and V-REP, for a fast prototyping of robot control algorithms. This system allows testing sensor-based control algorithms before on simulated robots in V-REP and later on the real robots, with a few changes. 

In the last years, I supervised several student internships and I am participating as a mentor in the Google Summer of Code. I also published scientific articles at IEEE Robotics and Automation Letters (RA-L),  ICRA’17 and Humanoids’16.
'
    #url: "#test-link"
    #btn_label: "Read More"
    #btn_class: "btn--inverse"

background:
  - image_path: /work/ecn.jpg
    alt: "My background"
    title: "My background"
    excerpt: 'I graduated in [Computer Engineering](http://www.ingegneriainformatica.dibris.unige.it/) in Genoa (Italy) and later I obtained a double degree: Master in Robotics Engineering (University of Genoa) and  [Master ARIA](http://masteraria.irccyn.ec-nantes.fr/index.php/presentationaria-en) in Advanced Robotics (École Centrale de Nantes). 
I also did an internship at [IRCCYN](http://www.irccyn.ec-nantes.fr/en/) on "Pose and velocity estimation for high-speed robot control" (using a vision system) under the supervision of Philippe Martinet.
The goal of the thesis was to develop a method able to estimate the pose and the velocity of a high-speed parallel robot at a very high frequency (1 kHz- 2 kHz). My work was part of the French ANR [Project ARROW](http://www.irccyn.ec-nantes.fr/~briot/ANR_ARROW.html):
Design of Accurate and Fast Robots with Large Operational Workspaces'
    #url: "http://www.irisa.fr/lagadic/"
    #btn_label: "Read More"
    #btn_class: "btn--inverse"


videos1:
  - video_link: SZxp6BJgBUg
    alt: "Matlab Ros Bridge"
    #title: "Matlab Ros Bridge"
    #excerpt: "Matlab Ros Bridge"
  - video_link: vfdq9UK5SkU
    alt: "PBVS Pioneer"
  - video_link: n3UmgCXw5lY
    #title: "PBVS Pioneer Real"

videos2:
  - video_link: TgpLfgFSca8
    alt: "RomeoGrasping"
    #title: "Matlab Ros Bridge"
  - video_link: -wIzJ2Ckifg
    alt: "Romeo maze"
  - video_link: ajZpUOC5ERM
    #title: "PBVS Pioneer Real"

videos3:
  - video_link: 1--VjdDlFg4
  - video_link: qotsmwXmTUY
  - video_link: bf2GvyEKQQo

videos4:
  - video_link: o22hN7YmVd4
  - video_link: 4B_aEEaosyw
  - video_link: QDmDY5koKIE

---

{% include feature_row id="intro" type="center" %}

{% include feature_row type="right"%}

{% include feature_row id="job" type="left"%}

{% include feature_row id="background" type="right"%}

# Interfacing Matlab with V-REP using ROS (2013)
{% include feature_row id="videos1" %}

# Some demos with Romeo (2014-2015)
{% include feature_row id="videos2" %}

# Whole-body control and door opening demo with Romeo (2016)
{% include feature_row id="videos3" %}

# Vision-based demos with Pepper (2016-2017)
{% include feature_row id="videos4" %}

