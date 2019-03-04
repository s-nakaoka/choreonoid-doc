Teleoperation sample using ROS
==============================

Here, we will introduce a sample using ROS to perform teleoperation. Like the :doc:`teleoperation-rtm` , it is to allow  :doc:`simulation-samples`  to be operated remotely.

.. contents::
   :local:

.. highlight:: sh

Installing ROS
--------------

If you have not yet installed ROS, install it by following the instructions at  `ROS.org <http://wiki.ros.org>`_ - `ROS/Installation <http://wiki.ros.org/ROS/Installation>`_ .

Regarding the version of ROS, check the operation of Kinetic on Ubuntu 16.04 or Melodic on Ubuntu 18.04.

In the case of Ubuntu 16.04, install it as follows. ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
 sudo apt-get update
 sudo apt-get install ros-kinetic-desktop-full
 sudo rosdep init
 rosdep update
 echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc

When using Choreonoid with ROS, use the new version of the build tool Catkin  ( `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_）. This can be installed using the command: ::

 sudo apt install python-catkin-tools

.

Creating a Catkin workspace
---------------------------

We will create a Catkin workspace for Choreonoid.

A workspace is usually created in the home directory. The workspace is usually named catkin_ws. You can change this name, but if you do so, be sure to change all instances of catkin_ws in the explanation below to the alternative name you have used.

First, create an empty workspace. ::

 mkdir catkin_ws
 cd catkin_ws
 mkdir src
 catkin init

Adding package source
---------------------

With the src directory of the workspace you created, clone the Choreonoid-related source repositories. ::

 cd src
 git clone https://github.com/s-nakaoka/choreonoid.git
 git clone https://github.com/s-nakaoka/choreonoid_rosplugin.git
 git clone https://github.com/s-nakaoka/choreonoid_ros_samples.git
 git clone https://github.com/s-nakaoka/choreonoid_joy.git

Then, each repository will support the following software.

* choreonoid: Choreonoid itself
* choreonoid_rosplugin: ROS Plugin in order to use ROS in Choreonoid
* choreonoid_ros_samples: samples that use ROS in Choreonoid
* choreonoid_joy: ROS node in order to use the joystick (gamepad) in Choreonoid mapping

These are the sources for the packages to be used in the workspace. Keep the contents of each repository as up to date as possible.

There are tools, such as  `wstool <http://wiki.ros.org/wstool>`_  and `vcstool <https://github.com/dirk-thomas/vcstool>`_   that can be used for things like updating multiple repositories at once. Personally, I think that vcstool has the better usability. If you are going to use vcstool, you can execute the following command: ::

 sudo apt install python3-vcstool

to install it.

Execute: ::

 vcs help

to see how to use it.

For example, if you want to use git pull for all repositories, you should execute: ::

 vcs pull

.

.. _teleoperation_ros_build_packages:

Building the package
--------------------

We will build the package on the workspace.

If you have not yet done the usual :ref:`wrs2018_install_choreonoid` , execute the Choreonoid package installation script, just in case. Go to the Choreonoid source directory, and execute: ::

 misc/script/install-requisites-ubuntu-16.04.sh

. (In the case of Ubuntu 18.04, run install-requisites-ubuntu-18.04.sh)

It should have been solved originally with the dependency package information for Catkin. But it is possible that it is not complete yet, so it is probably a good idea to do this, just in case. (This is not necessary if Choreonoid is already installed.)

.. note:: The usual  :ref:`wrs2018_install_choreonoid`  is the installation method when ROS is not used. If ROS is used, it is okay to install using the method on this page. However, since they differ in terms of the directory structure and whether or not the executable file is in your PSAATH, please take that into account when trying out the samples introduced thus far.

Next, we will configure the CMake options. As shown in the  :ref:`wrs2018_install_choreonoid` section, several optional Choreonoid features are required when executing the WRS2018 simulation, and these were enabled using the CMake options. Specifically, the options are:

* BUILD_WRS2018
* BUILD_AGX_DYNAMICS_PLUGIN
* BUILD_AGX_BODYEXTENSION_PLUGIN
* BUILD_SCENE_EFFECTS_PLUGIN
* BUILD_MULTICOPTER_PLUGIN
* BUILD_MULTICOPTER_SAMPLES
* ENABLE_CORBA
* BUILD_CORBA_PLUGIN
* BUILD_OPENRTM_PLUGIN
* BUILD_OPENRTM_SAMPLE
* BUILD_COMPETITION_PLUGIN

.

Another point to note is that ROS Kinetic uses Python version 2.7, but Choreonoid uses Python 3 by default. In this case, it seems they may fail, perhaps because of a conflict between the shared libraries of Python versions 2 and 3. Therefore, also set the following CMake options.

* If USE_PYTHON3: is set to ON, use Python3. if it is set to OFF, use Python2.

This must be set to OFF in ROS Kinetic. ROS melodic tries to use Python3, so leave this option at its default ON setting.

If building on Catkin, these options are configured as part of the workspace settings. Specifically, by giving the -cmake-args option to catkin config, it is configured as follows: ::

 catkin config --cmake-args -DBUILD_WRS2018=ON -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON -DBUILD_OPENRTM_SAMPLES=ON -DBUILD_COMPETITION_PLUGIN=ON -DUSE_PYTHON3=OFF

. (In Melodic, delete the final -DUSE_PYTHON3=OFF.)

After configuring the settings, execute the command ::

 catkin config

and the workspace settings will be displayed. If the following are displayed ::

 Additional CMake Args:  -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON
 -DBUILD_COMPETITION_PLUGIN=ON -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON
 -DBUILD_OPENRTM_SAMPLES=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DUSE_PYTHON3=OFF -DBUILD_WRS2018=ON

everything is okay.

.. note:: If the settings are configured in this way, these options will be enabled for all packages in the workspace, and options may be unintentionally enabled in other packages. However, Catkin does not have the function to allow CMake options to be set separately for each package ( `it has been postponed even though there is a demand <https://github.com/catkin/catkin_tools/issues/205>`_ ）, so this situation is unavoidable.

When the settings are complete, let’s perform the build. If the directory is within the workspace, the build is executed using the following command: ::

 catkin build

Refer to  `Catkin Command Line Tools manual <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ for details about the build process.

If the build is successful, the message ::

 [build] Summary: All 4 packages succeeded!

will be displayed.

When the build is performed, a file named setup.bash will be created in the devel workspace directory. The settings described in this script are required when executing packages in the workspace, so they should be run by default. Usually, you add the following description ::

 source $HOME/catkin_ws/devel/setup.bash

to the .bashrc file in the home directory.

When you do so, this file is executed automatically when the terminal is launched, and the settings are imported.

Since these settings are not yet imported during the initial build, restart the terminal or input the above command directly from the command line to update the settings.

.. note:: Be careful when running Catkin's configuration script because it may affect the running of any Choreonoid installed separately outside the Catkin environment. This is caused by the fact that Catkin's configuration script adds the devel/lib directory of the Catkin workspace to the path of the shared library (this path is added to the environment variable LD_LIBRARY_PATH). With this setting, Choreonoid running outside the Catkin environment may import the shared library of the Choreonoid generated in Catkin. In that case, if there are differences in the source code version or the build settings, Choreonoid may not work properly or may fail. In other words, builds performed in different environments cannot be mixed. In order to avoid this problem, disable the Catkin configuration script when running Choreonoid outside of Catkin. (A mechanism called RPATH is used by default in Choreonoid, which should prevent this problem. But it does not seem to work properly sometimes, depending on the environment.)


Running Choreonoid
------------------

First, if the ROS Master is not running, launch it. ::

 roscore

If the build was performed in the Catkin workspace, the path to the executable file is added to PATH by the above setup.bash script. Therefore, if you simply enter choreonoid anywhere in the directory, Choreonoid will launch. ::

 choreonoid

Move to the src/choreonoid/samaple/WRS2018/script directory in the workspace, execute a command such as ::

 choreonoid T1-AizuSpiderSS.py

and you can execute the :doc:`simulation-samples` .

Running the teleoperation samples
---------------------------------

The teleoperation samples using ROS are available, and the file names are the same as those for the samples introduced in the :doc:`simulation-samples`  section, but with the suffix -ROS appended.

At the present time, the following projects are available.

* T1-AizuSpiderSA-ROS.py
* T1-AizuSpiderSS-ROS.py
* T1-DoubleArmV7A-ROS.py
* T1-DoubleArmV7S-ROS.py

In the same way as described in the :doc:`simulation-samples` section, import one of the above projects. For example, from the Choreonoid source directory, it will be something like ::

 bin/choreonoid sample/WRS2018/script/T1-AizuSpiderSA-ROS.py

.

Teleoperation nodes and tools must also be launched. First, in order to operate using the gamepad, connect the gamepad and them run the choreonoid_jy package as follows: ::

 rosrun choreonoid_joy node

This will deliver the state of the gamepad as a topic.

This functions in the same way as the ROS joy package, but it differs in that the axis and button mapping is the Choreonoid standard. If it is a supported gamepad, the axis and button mapping will be the same regardless of the model. Choreonoid samples are made using this mapping, so it is best to use this choreonoid_joy when moving them.

Next, we will enable display of the camera image. There are various ways of doing this, but here we will use the rqt_image_view tool. Launch it using the following command: ::

 rosrun rqt_image_view rqt_image_view

In the upper left corner of this tool there is a combo box where you can specify which topic's image data to display, so use it to specify the camera image you want to display. For the Aizu Spider, select /AizuSpider/FRONT_CAMERA/image.

This completes the preparations. Launch the simulation in Choreonoid. If that goes well, the Aizu Spider camera image will be displayed in rqt_image_view. And you will be able to operate the robot with the gamepad.

You can also execute the DoubleArmV7 sample in the same way. For the DoubleArmV7, select /DoubleArmV7/FRAME_FRONT_CAMERA/image as the camera image topic.

.. note:: With this sample, only the camera image corresponding to the above topic is simulated. if you want to also simulate other camera images, refer to the :doc:`../simulation/vision-simulation` section and configure the GLVisionSimulator item. However, if you increase the number of target cameras for simulation, it may cause the overall simulation to slow down.

Remote communication using 2 PCs
--------------------------------

Even when using ROS, it is of course possible to have separate PCs for simulation and operation.

In that case, launch the Choreonoid simulation project on the simulation PC and the choreonoid_joy node and rqt_image_view on the teleoperation PC.

In order that the ROS node can communicate between the two PCs, you need to use a shared ROS Master.

Briefly, decide the host (PC) where you will install the ROS Master and launch roscore there. Then on the other PC, set your own IP address as the ROS_IP environment variable and the master address as ROS_ MASTER_URI.

For example, if the configuration is as follows:

* Set the simulation PC as the Master;
* IP address of the simulation PC: 192.168.0.10;
* IP address of the operation PC: 192.168.0.20,

launch roscore on the simulation PC, and on the operation PC, execute the following commands: ::

 export ROS_IP=192.168.0.20
 export ROS_MASTER_URI=http://192.168.0.10:11311

. (If the address can be deducted from the host name, it can be set as the host name instead of the IP address.)

When you’ve finished configuring the settings, launch the simulation in Choreonoid on the simulation PC. When you do so, the camera image will be displayed in rqt_image_view on the teleoperation PC and you should be able to operate the robot using the gamepad connected to the teleoperation PC.




