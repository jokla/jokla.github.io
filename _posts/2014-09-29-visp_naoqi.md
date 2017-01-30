---
title:  "Visp_naoqi Guide"
excerpt: "How to use the bridge Visp_naoqi."
modified: 2016-06-03
categories: 
  - Robotics
tags:
  - Romeo
  - ViSP
---


<!-- {% include toc title="table of contents" icon="file-text" %} -->


 Visp_naoqi is a C++ library, a bridge over ViSP and NaoQi. It is available on GitHub [http://www.github.com/lagadic/visp_naoqi](http://www.github.com/lagadic/visp_naoqi). You can find a description of the software [here](http://jokla.me/software/visp_naoqi/).

# Install visp_naoqi bridge

## Prerequisites 
* Install ViSP from source (See ViSP tutorials [here](http://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html))
* Install and configure Naoqi C++ SDK (You can follow [this guide](http://jokla.me/robotics/install-sdk-c-naoqi/))
* Install Metapod library: 
  * Clone the repository of [Metapod library](https://github.com/laas/metapod) :
  `clone --recursive https://github.com/laas/metapod.git`
  * Compile it (you will need  liburdfdom or liburdf, see [here](https://github.com/laas/metapod)):

```shell
  $ mkdir _build
  $ cd _build
  $ cmake -DBUILD_METAPODFROMURDF=ON ..
  $ sudo make install
```

* If you are using `naoqi-sdk-2.3.0.14-linux64` (or newer) :
  * Rename
  `naoqi-sdk-2.3.0.14-linux64/bin/metapodfromurdf`
   to
   `/naoqi-sdk-2.3.0.14-linux64/bin/metapodfromurdf_aldebaran`
  * Open the file `/naoqi-sdk-2.3.0.14-linux64/include/qi/type/detail/object.hxx` and rename two times `None`  to `None_` (lines 197,212)

## Clone and build visp_naoqi

```shell
$ cd ~/romeo/workspace
$ git clone http://www.github.com/lagadic/visp_naoqi.git
$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release
$ qibuild make -c toolchain_romeo
```

## Known issues

### 1) System libraries conflict:
`$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release`

``` shell
CMake Warning at /udd/fspindle/.local/share/cmake/qibuild/target.cmake:85
(add_executable):
Cannot generate a safe runtime search path for target image_viewer_opencv
because files in some directories may conflict with libraries in implicit
directories:

runtime library [libz.so.1] in /usr/lib/x86_64-linux-gnu may be hidden by
files in:
      /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib
```

In that case, backup `/local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib`
    and remove  the files`/naoqi-sdk-2.1.0.19-linux64/lib/libz.so.*`

```shell
  $ cd /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib
  $ rm libz.so.*
```
    
With the `naoqi-sdk-2.3.0.14-linux64` we have another conflict with `[libusb-1.0.so.0]`. Remove  the files `libusb-1.*` in `/naoqi-sdk-2.3.0.14-linux64/lib`:

```shell
  $ cd /local/soft/romeo/devtools/naoqi-sdk-2.3.0.14-linux64/lib
  $ rm libusb-1.*
```
    
### 2) Macro names must be identifiers:

`$ qibuild make -c toolchain_romeo`

```
     ...
[ 20%] Building CXX object CMakeFiles/visp_naoqi.dir/src/grabber/vpNaoqiGrabber.cpp.o
<command-line>:0:1: error: macro names must be identifiers
<command-line>:0:1: error: macro names must be identifiers
<command-line>:0:1: error: macro names must be identifiers
```
 
Edit `/ViSP/ViSP-build-release/VISPConfig.cmake` to replace
    
`SET(VISP_DEFINITIONS "-DVP_TRACE;-DVP_DEBUG;-DUNIX")`

with:

`SET(VISP_DEFINITIONS "VP_TRACE;VP_DEBUG;UNIX")`
	
### 3) Conflicts with boost:

`$ qibuild make -c toolchain_romeo`

```
    Linking CXX executable sdk/bin/image_viewer_opencv
    /usr/bin/ld: warning: libboost_system.so.1.55.0, needed by   
    /local/soft/romeo/devtools/naoqi-sdk-2.1.0.19-linux64/lib/libqitype.so, 
    may conflict with libboost_system.so.1.54.0
```

In that case, you have to build again ViSP turning Ogre support off:
`$ cd <ViSP build folder>`
`$ cmake -DUSE_OGRE=OFF <path to ViSP source code>`
`$ make -j8`

### 4)  metapod/algos/djac.hh: No such file or directory
* Open the file `CMakeLists.txt` of METAPOD and add the line:
  `include/${PROJECT_NAME}/algos/djac.hh`
in `SET(${PROJECT_NAME}_ALGOS_HEADERS` after `include/${PROJECT_NAME}/algos/jac_point_chain.hh`
* Build and install Metapod again

### 5) ‘RotationMatrix’ is not a member of ‘metapod::Spatial’
If you have this error:
```
error: ‘RotationMatrix’ is not a member of ‘metapod::Spatial’
error: wrong number of template arguments (2, should be 3)
error: template argument 3 is invalid
error: ‘Joint’ does not name a type Joint joint;
```

* Rename
 `naoqi-sdk-2.3.0.14-linux64/bin/metapodfromurdf`
  to
 `/naoqi-sdk-2.3.0.14-linux64/bin/metapodfromurdf_aldebaran`
In this way CMake will find the right version of `metapodfromurdf` (in /usr/local/bin/metapodfromurdf).

## Test model URDF for Metapod
 In order to create a Metapod model it is necessary to symplify the original URDF model. To test it go to the folder `/metapod/_build/metapodfromurdf` (`_build` could have a different name, depending on what you choosed) and run:
 
`metapodfromurdf pepper.urdf --name pepper --directory pepper`
 

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


