Structure of the simulation environment
=======================================

.. contents::
   :local:

.. highlight:: sh

Preparing the simulation PC
---------------------------

First, prepare the simulation PC and install Choreonoid.

Refer to the information in the  :ref:`wrs2018_overview_simulator`  section when preparing the OS and specifications of the PC you will be using.

It is assumed that the OS is Ubuntu 16.04 64-bit. The ISO image of it can be downloaded from the `this page <http://releases.ubuntu.com/xenial/>`_  . 

Note that we have tested this sample as working even with Ubuntu 18.04. However, as of December 25, 2018, since a version of OpenRTM (version 1.2.0) compatible with Ubuntu 18.04 has not been released, samples that use OpenRTM cannot be executed. Samples using ROS work even with 18.04.

.. note:: Use a native install of Ubuntu. It’s not that it won’t work on a virtual machine, but the simulation may be slow or other problems may occur. If you really want to try it out on a virtual machine, refer to the section on `building an Ubuntu 16.04 virtual machine using VMWare <http://choreonoid.org/ja/workshop/vmware.html>`_ . However, the WRS2018 sample simulation is not guaranteed to work properly.


Installing Git
--------------

You will need a Git of the version control system in order to proceed with the work described below. If you don’t have it installed yet, use the following command to install it. ::

 sudo apt install git

.. _wrs2018_install_agx:

Installing AGX Dynamics
-----------------------

If you have an AGX Dynamics license, you should install AGX Dynamics in advance. Download the package for the corresponding Ubuntu version (usually x64, Ubuntu 16.04) from the AGX Dynamics download site indicated by the vendor. Also, if you have been provided with a USB dongle, insert it into the PC.

Once the package has downloaded, install it by following the instructions for :doc:`../agxdynamics/install/install-agx-ubuntu` .

If you don’t have an AGX Dynamics license, skip this task.

.. _wrs2018_install_openrtm:

Installing OpenRTM-aist
-----------------------

To run the :doc:`teleoperation-rtm` , follow the instructions in  :doc:`../openrtm/install` starting by    :ref:`openrtmplugin_install_openrtm`   . Also, configure the  :ref:`openrtmplugin_setup_corba`.

Be sure to also  :ref:`openrtmplugin_patch_for_version112`.

.. _wrs2018_install_choreonoid:

Installing Choreonoid
---------------------

Following the `Choreonoid latest version (development version) manual  <../index.html>`_  section on `Building and installing from source code (Ubuntu Linux version) <../install/build-ubuntu.html>`_, install the latest `development version  <../install/build-ubuntu.html#id4>`_ of Choreonoid.

Refer to the above document for more details about the installation and run the command below for Ubuntu 16.04.

First, get the Choreonoid source code from the Git repository. ::

 git clone https://github.com/s-nakaoka/choreonoid.git

Move to the directory where the source code is saved. ::

 cd choreonoid

Install the dependency packages. ::

 misc/script/install-requisites-ubuntu-16.04.sh

(In the case of Ubuntu 18.04, run install-requisites-ubuntu-18.04.sh)

Configure build settings with CMake If you are using only the default features of Choreonoid, run the command ::

 cmake .

.

However, in order to execute the WRS2018 sample, the following options must also be enabled (ON).

* WRS2018 sample

 * BUILD_WRS2018

* If you are using AGX Dynamics

 * BUILD_AGX_DYNAMICS_PLUGIN
 * BUILD_AGX_BODYEXTENSION_PLUGIN

* When reproducing smoke and flames

 * BUILD_SCENE_EFFECTS_PLUGIN

* When using the multicopter

 * BUILD_MULTICOPTER_PLUGIN
 * BUILD_MULTICOPTER_SAMPLES

* When using OpenRTM

 * ENABLE_CORBA
 * BUILD_CORBA_PLUGIN
 * BUILD_OPENRTM_PLUGIN
 * BUILD_OPENRTM_SAMPLES

* Competition plugin (used mainly by the judges)

 * BUILD_COMPETITION_PLUGIN

You can set these options interactively using the ccmake command, but you can also give the cmake command the -D option. For example, to set BUILD_SCENE_EFFECTS_PLUGIN to ON, input the following. ::

 cmake -DBUILD_SCENE_EFFECTS_PLUGIN=ON

This option can be added multiple times. If you want to enable all the above options, input the following. ::

 cmake -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON -DBUILD_OPENRTM_SAMPLES=ON 

If you do not have AGX Dynamics or OpenRTM installed, remove the corresponding options from the above command line parameters and execute it.

Next, perform the build with the make command. ::

 make

If you are using a multi-core CPU, it is a good idea to parallelize the build by adding the -j option to the make command. For example, as follows. ::

 make -j 8

In this case, up to 8 build processes will be run simultaneously. It’s a good idea to input this if the CPU has 4 cores and 8 threads. Usually, specify the number of logical cores in the CPU.

Even after installation, you can always use the latest version of Choreonoid by executing the following commands in the source directory where the above operation was done. ::

 git pull
 make -j 8

Please note that development of Choreonoid will continue for the time being until close to the opening of the competition. On this basis, we expect to continue making preparations while occasionally updating to the latest version. If you come across any bugs, please get in touch with our :doc:`support` .


Rendering-related settings
--------------------------


When installing Choreonoid, if it is possible to :ref:`build_ubuntu_gpu_driver` , be sure to do so. Also, for  :doc:`../install/setup-renderer` , switch to the GLSL rendering engine, if possible. As the WRS2018 simulation requires high-level rendering ability, these settings are indispensable for a complete simulation.

Also, it is probably a good idea to apply the  :ref:`build_ubuntu_qt_style` .

Preparing the gamepad
---------------------

With this sample, you can operate the robot using a gamepad. To do so, prepare a gamepad and connect it to a PC.

For details about what gamepads can be used, refer to the  :doc:`../simulation/tank-tutorial/index` section on  :ref:`simulation-tank-tutorial-gamepad` . We recommend the  `DUALSHOCK4 <http://www.playstation.com/en-us/explore/accessories/gaming-controllers/dualshock-4/>`_ controller for PlayStation 4. The DUALSHOCK4 can be used wirelessly using a  `USB wireless adapter <https://support.playstation.com/s/article/DUALSHOCK-4-USB-Wireless-Adapter?language=en_US>`_ .
