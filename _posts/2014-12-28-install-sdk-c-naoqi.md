---
layout: post
title: Naoqi C++ SDK Installation
excerpt: "How to install the SDK C++ for Romeo and Nao."
modified: 2016-02-14
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

#Install SDK C++ for Romeo and Nao

## Prerequisites

### Installation IDE 
QT Creator is the IDE recommended by Aldebaran.

* Download the installer available [here](http://qt-project.org/downloads#qt-creator). In my case the file is named `qt-opensource-linux-x64-1.6.0-5-online.run`.
* Go in the folder where you downloaded the installer of qt-creator and give execute permission with:  
`$ chmod a+x qt-opensource-linux-x64-1.6.0-5-online.run`   
* Run the installer:  
`$ ./qt-opensource-linux-x64-1.6.0-5-online.run`  
 
### Download software

#### C++ SDK and Cross Toolchain
Download the following packages [here](https://community.aldebaran-robotics.com/resources/):

* C++ SDK 2.3 Linux 64 (or newier version)


### Creation Devtools and workspace folders
* Let's create now some folders useful for the development with the SDK:  
`$ mkdir -p ~/romeo/{devtools,workspace} `   

NB: This is just a suggestion, you can manage these folders as you prefer.

* Now we can extract the C++ SDK and Cross Toolchain in the devtools folder. Go via terminal in the folder where you downloaded the tools and run:  
`$ tar -zxvf naoqi-sdk-2.3.0.14-linux64.tar.gz -C ~/romeo/devtools/`  



#### Qibuild
* Open a terminal and install Qibuild with [pip](https://pip.pypa.io/en/latest/installing.html#install-pip):
`$ pip install qibuild`
* If you don't have pip installed you can install it with:
`$ sudo apt-get install python-pip` 
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
`:: Please choose a generator:
> 1 (Unix Makefiles)
:: Please choose an IDE
> 2 (QtCreator)
:: Do you want to use qtcreator from /usr/bin/qtcreator?
> Y (Yes)
:: Found a worktree in /udd/fspindle/soft/romeo/workspace_gantry
:: Do you want to configure settings for this worktree? (y/N)
> y
:: Do you want to use a unique build dir? (mandatory when using Eclipse) (y/N)
> N`
  * If you see a message like "CMake not found" probably you have to install CMake:
* `sudo apt-get update &&  sudo apt-get install cmake `
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
* Now we need to create a toolchain (change with your path to the file toolchain.xml you want to use. You will find it in the naoqi-sdk folder):  
`$ qitoolchain create toolchain_romeo /home/jokla/romeo/devtools/naoqi-sdk-2.3.0.14-linux64/toolchain.xml --default`
NB: Instead of `toolchain_romeo` you can choose the name that you want. You can create also different toolchains.
* If you have a new version of qibuild the procedure is slightly different:
`$ qitoolchain create toolchain_romeo /local/soft/naoqi-sdk/naoqi-sdk-2.3.0.14-linux64/toolchain.xml`
`$ qibuild add-config toolchain_romeo -t toolchain_romeo --defaul`

* Optional Test: Open a terminal and digit:  
`$ cd ~/romeo/devtools/naoqi-sdk-2.3.0.14-linux64/doc/dev/cpp/examples`  
`$ qibuild init --interactive`  
* Now we can configure and build the examples:  
`$ cd core/helloworld/`  
`$ qibuild configure -c toolchain_romeo`  
`$ qibuild make -c toolchain_romeo`

* You can also build in release mode:  
`$ qibuild configure --release <project_name>`  
`$ qibuild make --release <project_name>`
