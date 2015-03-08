---
layout: post
title: Visp_naoqi Guide
excerpt: "How to use the bridge Visp_naoqi."
modified: 2014-09-29
tags: [naoqui]
comments: true
image:
  feature: texture-feature-05.jpg
  credit: Texture Lovers
  creditlink: http://texturelovers.com
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

# Install visp_naoqi bridge
* Visp_naoqi is a C++ library, a bridge over ViSP and NaoQi. It is available on GitHub http://www.github.com/lagadic/visp_naoqi. You can find a description of the software [here](http://jokla.me/software/visp_naoqi/).

## Prerequisites 
* Install ViSP from source (See ViSP tutorials [here](http://www.irisa.fr/lagadic/visp/publication.html#started))
* Install and configure Naoqi C++ SDK (You can follow [this guide](http://jokla.me/install-sdk-c-naoqi/))
* Install Metapod library: 
  * Clone the repository of [Metapod library](https://github.com/laas/metapod) :
  ` clone --recursive https://github.com/laas/metapod.git`
  * Compile it (you will need  liburdfdom or liburdf, see [here](https://github.com/laas/metapod)):
  `mkdir _build`
  `cd _build`
  `cmake -DBUILD_METAPODFROMURDF=ON ..`
  `make install`




## Clone and build visp_naoqi

`$ cd ~/romeo/workspace`
`$ git clone http://www.github.com/lagadic/visp_naoqi.git`
`$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release`
`$ qibuild make -c toolchain_romeo`


## Known issues

### 1) System libraries conflict:
`$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release`

{% highlight Bash shell scripts %}


CMake Warning at /udd/fspindle/.local/share/cmake/qibuild/target.cmake:85
(add_executable):
Cannot generate a safe runtime search path for target image_viewer_opencv
because files in some directories may conflict with libraries in implicit
directories:

runtime library [libz.so.1] in /usr/lib/x86_64-linux-gnu may be hidden by
files in:
      /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib
      

{% endhighlight %}


In that case, backup /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib
    and remove /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib/libz.so.*
    
With the `naoqi-sdk-2.3.0.14-linux64` we have another conflict with [libusb-1.0.so.0]. Remove libusb-1.* in /local/soft/naoqi/naoqi-sdk-2.3.0.14-linux64/lib
    
### 2) Macro names must be identifiers:

`$ qibuild make -c toolchain_romeo`

{% highlight bash %}
     ...
[ 20%] Building CXX object CMakeFiles/visp_naoqi.dir/src/grabber/vpNaoqiGrabber.cpp.o
<command-line>:0:1: error: macro names must be identifiers
<command-line>:0:1: error: macro names must be identifiers
<command-line>:0:1: error: macro names must be identifiers
      
{% endhighlight %}
 
Edit /local/soft/ViSP/ViSP-build-release/VISPConfig.cmake to replace
    
`SET(VISP_DEFINITIONS "-DVP_TRACE;-DVP_DEBUG;-DUNIX")`

with:

`SET(VISP_DEFINITIONS "VP_TRACE;VP_DEBUG;UNIX")`
	
### 3) Conflicts with boost:

`$ qibuild make -c toolchain_romeo`

{% highlight bash %}    
    Linking CXX executable sdk/bin/image_viewer_opencv
    /usr/bin/ld: warning: libboost_system.so.1.55.0, needed by   
    /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib/libqitype.so, 
    may conflict with libboost_system.so.1.54.0
{% endhighlight %}

In that case, you have to build again ViSP turning Ogre support off:
`$ cd <ViSP build folder>`
`$ cmake -DUSE_OGRE=OFF <path to ViSP source code>`
`$ make -j8`


## Get an image from the robot with ViSP (Lab)

* You can find an example in `/local/soft/romeo/cpp/workspace/getimage`. This project creates two executables, getimages_visp (use ViSP) and getimages (use OpenCV).
* C++ NAOqi SDK provides some OpenCV libraries. However, on Ubuntu, these libraries have been built without GTK support for portability reasons. 
* To use the OpenCV of the system we deleted the OpenCV libraries in the Naoqui C++SDK (see [here](https://community.aldebaran-robotics.com/doc/1-14/dev/cpp/examples/vision/opencv.html#removing-opencv-from-the-naoqi-sdk)).    
* To use ViSP in the project you have to modify the CMakeLists.txt adding the following lines:
{% highlight CMake %}

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include_directories(${VISP_INCLUDE_DIRS})
  link_libraries(${VISP_LIBRARIES})
endif(VISP_FOUND)

{% endhighlight %}
* To avoid conflict with the boost library we re-bulit ViSP without OGRE. 
* You can configure and build the project with:
`$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release/`
`$ qibuild make -c toolchain_romeo`
* Now you can run the application specifing the IP of the robot:
`$ ./build-toolchain_romeo/sdk/bin/getimages_visp 198.18.0.1`

NB: Remember to run before:
`$ sudo ip route (see internal documentation)...`

## Romeo's Camera Control
* [Camera parameters](file:///local/soft/naoqi/naoqi-sdk-2.1.0.19-linux64/doc/naoqi/vision/alvideodevice-api.html#cameraparameter)

## Sensor LOG 

### Record position and velocity Romeo Joints

* Go via terminal in `/local/soft/romeo/python/from-aldebaran/module_sensor_log`
* Run the sensor log:
`$ python sensorlog.py --ip 198.18.0.1 --port 9559`
* The sensor log will start to record all the position of the joints. To stop it press Ctrl+C.
* A new txt file will be created in the directory (ex. `sensorlog.txt_romeo_1412082576.txt`)
* Now we can plot the data using the script plotData.py:
`$ python plotData.py --fileName sensorlog.txt_romeo_1412082576.txt --jointName RShoulderPitch --listPlot q dq`
* With `--listPlot` you can give one or more arguments among:  `q", "dq", "torque", "plot2d", "plot3d"`

## Velocity controller in joint space:
* You can find a python script in `/local/soft/romeo/python/Vel_Ctrl` named `motion_setVelocity.py`:
`$ python motion_setVelocity.py --ip 198.18.0.1 --jointNameList NeckYaw --jointVelocityList +0.5`


## Add Metapod library (VispNaoqi)
* Clone the repository of [Metapod library](https://github.com/laas/metapod) :
` clone --recursive https://github.com/laas/metapod.git`
* Compile it (you will need  liburdfdom or liburdf, see [here](https://github.com/laas/metapod)):
`mkdir _build`
`cd _build`
`cmake -DBUILD_METAPODFROMURDF=ON ..`
`make install`
* Copy the file `metapod/cmakeutils.cmake` in the vispnaoqi folder.
* Go in the vispNaoqi


## Demo grap_tea_mtb

`./sdk/bin/grasp_tea_mbt --model /udd/gclaudio/romeo/cpp/workspace/romeo_tk/demos/grasping/teabox --learn`
`~/romeo/cpp/workspace/romeo_tk/build-toolchain_romeo$ ./sdk/bin/grasp_tea_mbt` 
`--model /udd/gclaudio/romeo/cpp/workspace/romeo_tk/demos/grasping/teabox`
`--haar ../demos/face_tracking/haarcascade_frontalface_alt.xml`
