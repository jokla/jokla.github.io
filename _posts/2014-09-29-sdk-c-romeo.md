---
layout: post
title: SDK C++ for Romeo
excerpt: "How to use the SDK C++ for Romeo."
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

#Install SDK C++ for Romeo

## Prerequisites

###Installation IDE 
QT Creator is the IDE recommended by Aldebaran.

* Download the installer available [here](http://qt-project.org/downloads#qt-creator). In my case the file is named `qt-opensource-linux-x64-1.6.0-5-online.run`.
* Go in the folder where you downloaded the installer of qt-creator and give execute permission with:  
`$ chmod a+x qt-opensource-linux-x64-1.6.0-5-online.run`   
* Run the installer:  
`$ ./qt-opensource-linux-x64-1.6.0-5-online.run`  
 
### Download software

#### C++ SDK and Cross Toolchain
Download the following packages [here](https://community.aldebaran-robotics.com/resources/):

* C++ SDK 2.1 Linux 64 (or newier version)
* Cross Toolchain 2.1 Linux 32 (or newier version)

### Creation Devtools and workspace folders
* Let's create now some folders useful for the development with the SDK:  
`$ mkdir -p ~/romeo/{devtools,workspace} `   

NB: This is just a suggestion, you can manage this folders as you prefer.

* Now we can extract the C++ SDK and Cross Toolchain in the devtools folder. Go via terminal in the folder where you downloaded the tools and run:  
`$ tar -zxvf naoqi-sdk-2.1.0.19-linux64.tar.gz -C ~/romeo/devtools/`  
`$ unzip linux32-atom-pub.zip -d ~/romeo/devtools/`


#### Qibuild
* Open a terminal and install Qibuild with [pip](https://pip.pypa.io/en/latest/installing.html#install-pip):
`$ pip install qibuild --user` 
* Now we add the installation location of Qibuild in the PATH. Open the file bashrc: `$ gedit ~/.bashrc` and in the end of the file add:  
 `export PATH=${PATH}:${HOME}/.local/bin`  
* Open a new terminal and check if Qibuild is correctly installed:  
`$ qibuild --version`
* Now we have to create a qibuild “worktree”. This path will be the root from where qiBuild searches to find the sources of your projects. We can use the folder we created before: `~/romeo/workspace`.  
`$ cd ~/romeo/workspace`
And digit:  
`$ qibuild init`
* Now we can run: 
`$ qibuild config --wizard`   
A file will be generated in ~/.config/qi/qibuild.xml. It is shared by all the worktrees you will create. You will be asked to choose a CMake generator, select Unix Makefiles, and to choose a IDE, choose QtCreator (or another if you use a different IDE).
* We can create, configure and build a new project called "foo":
`$ qisrc create foo`   
 New project initialized in /home/jokla/romeo/workspace/foo 
`$ qibuild configure foo` 
{% highlight bash %}

Current build worktree: /home/jokla/romeo/workspace 
Build type: Debug 
* (1/1) Configuring foo 
-- The C compiler identification is GNU 4.8.2
-- The CXX compiler identification is GNU 4.8.2
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Using qibuild 3.6.2
-- Binary: foo
-- Binary: test_foo
-- Configuring done
-- Generating done
-- Build files have been written to: /home/jokla/romeo/workspace/foo/build-sys-linux-x86_64

{% endhighlight %}



`$ qibuild make foo`
{% highlight bash %}

Current build worktree: /home/jokla/romeo/workspace 
Build type: Debug 
* (1/1) Building foo 
Scanning dependencies of target foo
[ 50%] Building CXX object CMakeFiles/foo.dir/main.cpp.o
Linking CXX executable sdk/bin/foo
[ 50%] Built target foo
Scanning dependencies of target test_foo
[100%] Building CXX object CMakeFiles/test_foo.dir/test.cpp.o
Linking CXX executable sdk/bin/test_foo
[100%] Built target test_foo

{% endhighlight %}
* We can run the executable of the project "foo":  
`$ cd ~/romeo/workspace/foo/build-sys-linux-x86_64/sdk/bin/foo'  
You should see:  
 
`Hello, world`

References: [link1]( https://community.aldebaran-robotics.com/doc/1-14/dev/cpp/tutos/using_qibuild.html#cpp-tutos-using-qibuild), [link2](https://community.aldebaran-robotics.com/doc/qibuild/beginner/qibuild/aldebaran.html)
 

## Using qibuild with Aldebaran C++ SDKs 
* Open a terminal and digit:  
`$ cd ~/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/doc/dev/cpp/examples`  
`$ qibuild init --interactive`
* Now we say to Qibuild to use a toolchain:  
`$ qitoolchain create toolchain_romeo /home/jokla/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/toolchain.xml --default`

NB: Instead of `toolchain_romeo` you can choose the name that you want. You can create also different toolchains.
  
* Now we can configure and build the examples:  
`$ cd core/helloworld/`  
`$ qibuild configure -c toolchain_romeo`  
`$ qibuild make -c toolchain_romeo`

* You can also build in release mode:  
`$ qibuild configure --release <project_name>`  
`$ qibuild make --release <project_name>`

## Install visp_naoqi bridge
* A C++ library that bridges ViSP and NaoQi is available on GitHub http://www.github.com/lagadic/visp_naoqi. In that project you will find some examples that allows to acquire and display images from Romeo, but also examples that show how to move the joints.
* To get and build this project install first ViSP and run:
`$ cd ~/romeo/workspace`
`$ git clone http://www.github.com/lagadic/visp_naoqi.git`
`$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release`
`$ qibuild make -c toolchain_romeo`


* Known issues
  * System libraries conflict:
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
    
  * macro names must be identifiers
  ` $ qibuild make -c toolchain_romeo`

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
	
  * conflicts with boost
    $ qibuild make -c toolchain_romeo
    Linking CXX executable sdk/bin/image_viewer_opencv
    /usr/bin/ld: warning: libboost_system.so.1.55.0, needed by   
    /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib/libqitype.so, 
    may conflict with libboost_system.so.1.54.0

    In that case, you have to build again ViSP turning Ogre support off
    $ cd <ViSP build folder>
    $ cmake -DUSE_OGRE=OFF <path to ViSP source code>
    $ make -j8


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

# ROS and ROMEO

List of packages:
*  [ros-aldebaran/romeo_robot](https://github.com/ros-aldebaran/romeo_robot) 
*  [ros-nao/nao_sensors](https://github.com/ros-nao/nao_sensors)
*  [ros-aldebaran/romeo_moveit_config](https://github.com/ros-aldebaran/romeo_moveit_config)

## Step 1: Controller Romeo and  joint state publisher:
* Install the package [ros_control](http://wiki.ros.org/ros_control):
  `sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers`
* Set AL_DIR to the path to naoqiSDK-c++ on your computer:
  * `gedit ~/.bashrc`
  * Add the following line setting the correct path to your naoqi-sdk-c++:
  `export AL_DIR=/local/soft/naoqi/naoqi-sdk-2.1.0.19-linux64`
* Clone the repository [ros-aldebaran/romeo_robot](https://github.com/ros-aldebaran/romeo_robot) in your `catkin_ws`
* Go via terminal in your `catkin_ws` and build the packages with `catkin_make`
* Source your workspace:
 `source devel/setup.bash`
* Now you can run:
`roslaunch romeo_dcm_bringup romeo_dcm_bringup_remote.launch`

## Step 2: Use Moveit 
* Follow the Step 1
* Install Moveit in Indigo:
  * Open synaptic package manager and search for `moveit` 
  * Install the packages you find (I don't know exactly with packages are required so I installed all of them, except those with `config` )
* Clone in your repository [ros-aldebaran/romeo_moveit_config](https://github.com/ros-aldebaran/romeo_moveit_config)
* Go via terminal in your `catkin_ws` and build the packages with `catkin_make`
* Source your workspace:
 `source devel/setup.bash`
* Now we can run the `dcm_bringup`
  `roslaunch romeo_dcm_bringup romeo_dcm_bringup_remote.launch`
* Wait until `romeo_dcm_bringup` node is ready, then run MoveIt:
 `roslaunch romeo_moveit_config moveit_planner_romeo.launch`


## Calibration camera
Calibrate the camera: 
`rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.025 --no-service-check`
`image:=/nao_camera/image_raw camera:=/nao_camera`

http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

To converver ini to yaml:

http://wiki.ros.org/camera_calibration_parsers
