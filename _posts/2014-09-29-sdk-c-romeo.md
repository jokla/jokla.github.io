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

* Download the installer avaible [here](http://qt-project.org/downloads#qt-creator). In my case the file is named `qt-opensource-linux-x64-1.6.0-5-online.run`.
* Go in the folder where you dowloaded the installer of qt-creator and give execute permission with:  
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

NB: This is just a suggestion, you can magane this folders as you prefer.

* Now we can extract the C++ SDK and Cross Toolchain in the devtools folder. Go via terminal in the folder where you downloaded the tools and run:  
`$ tar -zxvf naoqi-sdk-2.1.0.19-linux64.tar.gz -C ~/romeo/devtools/`  
`$ unzip linux32-atom-pub.zip -d ~/romeo/devtools/`


#### Qibuild
* Open a terminal and install Qibuild with [pip](https://pip.pypa.io/en/latest/installing.html#install-pip):
`$ pip install qibuild --user` 
* Now we add the installation location of Qibuild in the PATH. Open the file bashrc: `$ gedit ~/.bashrc` and in the end of the file add:  
 `export PATH=${PATH}:${HOME}/.local/bin`  
* Open a new terminal and check if Qibuil is correctly installed:  
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
#container {
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
}
{% endhighlight %}



`$ qibuild make foo`
{% highlight bash %}
#container {
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
}
{% endhighlight %}
* We can run the executable of the project "foo":  
`$ cd ~/romeo/workspace/foo/build-sys-linux-x86_64/sdk/bin/foo'  
You should see:  
 
`Hello, world`



 https://community.aldebaran-robotics.com/doc/1-14/dev/cpp/tutos/using_qibuild.html#cpp-tutos-using-qibuild
 
 
 https://community.aldebaran-robotics.com/doc/qibuild/beginner/qibuild/aldebaran.html
 

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

* You can also build in releas mode:  
  `$ qibuild configure --release <project_name>`  
`$ qibuild make --release <project_name>`

## Get an image from the robot with ViSP (Lab)

* You can find an example in `/local/soft/romeo/cpp/workspace/getimage`. This project creates two executables, getimages_visp (use ViSP) and getimages (use OpenCV).
* C++ NAOqi SDK provides some OpenCV libraries. However, on Ubuntu, these libraries have been built without GTK support for portability reasons. 
* To use the OpenCV of the system we deleted the OpenCV libraries in the Naoqui C++SDK (see [here](https://community.aldebaran-robotics.com/doc/1-14/dev/cpp/examples/vision/opencv.html#removing-opencv-from-the-naoqi-sdk)).    
* To use ViSP in the project you have to modify the CMakeLists.txt adding the following lines:
{% highlight CMake %}
#container {
find_package(VISP REQUIRED)
if(VISP_FOUND)
  add_definitions(${VISP_DEFINITIONS})
  include_directories(${VISP_INCLUDE_DIRS})
  link_directories(${VISP_LIBRARY_DIRS})
  link_libraries(${VISP_LIBRARIES})
endif(VISP_FOUND)
}
{% endhighlight %}
* You can configure and build the project with:
`$ qibuild configure -c toolchain_romeo -DVISP_DIR=/local/soft/ViSP/ViSP-build-release/`
`$ qibuild make -c toolchain_romeo`
* Now you can run the application specifing the IP of the robot:
`$ ./build-toolchain_romeo/sdk/bin/getimages_visp 198.18.0.1`

NB: Rember to run before:
`$ sudo ip route add 198.18.0.0/24 via 131.254.13.37`

