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
permalink: /work
intro: 
- excerpt: "I am currently a R&D Robotics Engineer working in the [Lagadic](http://www.irisa.fr/lagadic/) group at [INRIA](http://www.inria.fr/en/) in Rennes."
feature_row:
  - image_path: /work/lagadic_2016.jpg
    alt: "Lagadic"
    title: "My current lab"
    excerpt: 'Research activities of the [Lagadic](http://www.irisa.fr/lagadic/) team deals with robot vision, visual servoing, real time visual tracking and SLAM for applications in localization, manipulation, navigation, medical robotics and augmented reality. Visual servoing consists in using the information provided by a vision sensor to control the movements of a dynamic system. Such systems are usually robot arms, or mobile robots, but can also be virtual robots, or even a virtual camera.'
    url: "http://www.irisa.fr/lagadic/"
    btn_label: "Read More"
    #btn_class: "btn--inverse"
job:
  - image_path: /work/giovanni.claudio.romeo.jpg
    alt: "My job"
    title: "My job"
    excerpt: 'In November 2013 I joined the team [Lagadic](http://www.irisa.fr/lagadic/)  in INRIA Rennes leaded by [François Chaumette](http://www.irisa.fr/lagadic/team/Francois.Chaumette-eng.html) with the role of R&D Robotics Engineer.
My principal task is to design, develop, integrate and test sensor-based control algorithms for robots.


I have experience with mobile robots (Pioneer P3-DX, Thymio), industrial robots (Adept Viper s650), quadcopters, humanoid robots (Nao, Romeo) and haptic interfaces. 
As my first project I created a framework for development of control algorithms for robotic platforms based on ROS, Matlab/Simulink and V-REP.


From September 2014 I am in charge of the humanoid robot Romeo from Aldebaran. I developed a framework for visual servo control and created several applications on objects detection, tracking and grasping, two handed-manipulation and human-robot interaction.


In these years I have also contribuited to the library ViSP, created several ROS packages, organized several demo presentations for the public and supervised some internship students.'
    #url: "#test-link"
    #btn_label: "Read More"
    #btn_class: "btn--inverse"

background:
  - image_path: /work/ecn.jpg
    alt: "My background"
    title: "My background"
    excerpt: 'I graduated in [Computer Engineering](http://www.ingegneriainformatica.dibris.unige.it/) in Genoa (Italy) and later I obtained a double degree: Master in Robotics Engineering (University of Genoa) and  [Master ARIA](http://masteraria.irccyn.ec-nantes.fr/index.php/presentationaria-en) in Advanced Robotics (École Centrale de Nantes). 
I also did an internship at [IRCCYN](http://www.irccyn.ec-nantes.fr/en/) on "Pose and velocity estimation for high speed robot control" (using a vision system) under the supervision of Philippe Martinet.
The goal of the thesis was to develop a method able to estimate the pose and the velocity of a high speed parallel robot at a very high frequency (1 kHz- 2 kHz).'
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
---

{% include feature_row id="intro" type="center" %}

{% include feature_row type="right"%}

{% include feature_row id="job" type="left"%}

{% include feature_row id="background" type="right"%}

# Interfacing Matlab with V-REP using ROS (2013)
{% include feature_row id="videos1" %}

# Some demos with Romeo (2014-2015)
{% include feature_row id="videos2" %}