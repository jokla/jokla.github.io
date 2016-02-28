---
layout: post
title: Install Symoro on Ubuntu 14.04
excerpt: "How to install Symoro on Ubuntu 14.04"
modified: 2013-05-31
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

[Symoro+](https://github.com/symoro/symoro) is an open-source version written in Python of SYMORO (SYmbolic MOdeling of RObots software). Both are developped in [IRCCyN](www.irccyn.ec-nantes.fr/) - Ecole Centrale de Nantes. 

## Install Dependecies
To run Symoro you will need:

* python (>= 2.7, 3.* is not supported)
* sympy (== 0.7.3)
* numpy (>= 1.6.1)
* wxPython (>= 2.8.12)
* OpenGL (>= 3.0.1b2)

### Install sympy

We need to install the version 0.7.3. We can download it from [here](https://github.com/sympy/sympy/releases/tag/sympy-0.7.3). Download the version sympy-0.7.3.tar.gz, that works with Python 2.5, 2.6, and 2.7.

Now open a terminal, go to the folder where you downloaded sympy-0.7.3.tar and extract the files:


`tar -xvzf sympy-0.7.3.tar.gz`


`cd sympy-0.7.3/`


Now we can go in the folder and install it:


`sudo python setup.py install`


To execute all tests, run:


`./setup.py test`



### Install numpy

Check if it already installed. Otherwise you can install it cloning from the repository or typing:


`sudo apt-get install python-numpy`


[See here](http://www.scipy.org/scipylib/download.html) for more information.

### Install wxPython

In order to install wxPython I used [Anaconda](http://docs.continuum.io/anaconda/).

Download it from [here](http://continuum.io/downloads) and once the download is finished:

`bash <downloaded_file_name>`

for example in my case was: 

`bash Anaconda-2.0.1-Linux-x86_64.sh`

Now we can install wxPython:

`conda install -c asmeurer wxpython`




### Check OpenGL

To know your OpenGL version in Ubuntu:

Install Glxinfo

`sudo apt-get install mesa-utils`

`glxinfo | grep "OpenGL version"`

You will see the version of OpenGL:

`OpenGL version string: 3.0 Mesa 10.1.3`


## Install Symoro

Now you can get and install Symoro. Follow the instruction you can find [here](https://github.com/symoro/symoro/wiki/Setup).

