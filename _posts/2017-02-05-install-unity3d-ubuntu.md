---
title:  "Install Unity in Ubuntu 14.04"
excerpt: "Install Unity in Ubuntu 14.04"
categories: 
  - Self-Driving Car
tags:
- Simulation
---

{% include toc title="table of contents" icon="file-text" %}

I wanted to test on my laptop this nice project created with Unity3D by [tawnkramer](https://github.com/tawnkramer):
{% include video id="e0AFMilaeMI" provider="youtube" %}
Unity3D is used to simulate a self-driving car. A neural network based on the Nvidia's paper ["End to End Learning for Self-Driving Cars"](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf) is trained to drive the car down a randomly generated road. You can find the source code [here](https://github.com/tawnkramer/sdsandbox).



## Download Unity for Linux

Donwload the last version [here](https://forum.unity3d.com/threads/unity-on-linux-release-notes-and-known-issues.350256/). I downloaded the debian package: [Unity 5.6.0xb3Linux](http://beta.unity3d.com/download/35e1927e3b6b/public_download.html).

## Installation
You can install the .deb package via the Ubuntu Software Center and is expected to work on installations of Ubuntu 12.04 or newer.

## Fix black screen launching the application
Initially Unity3D was not able to start properly, I was only getting a grey screen. [This solution](https://forum.unity3d.com/threads/dark-grey-screen-fix.448936/), posted by malyzeli, solved the problem.