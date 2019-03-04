
Building the Bullet plugin
==========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>


.. contents:: 目次
   :local:


Installing Bullet Physics Library
---------------------------------

Bullet Physics Library needs to be installed in order to use the Bullet plugin.

First, get the source code for Bullet Physics Library.

Open the `Bullet Physics Library <http://bulletphysics.org>`_ site , click Download in the top left, and you will be taken to the site where the Bullet source is stored. Once there, click **Source code (zip)** or **Source code (tar.gz)** to download the compressed file in either format and extract it in a suitable location.

The current version tested as working is bullet 2.87.

There should be a file named README.md in the directory where you extracted the file. This file describes the installation process, so basically you should follow these instructions.  There are slight variations between different versions of Bullet, but it seems to be possible to use CMake with recent versions, so you can install it in the same way as building Choreonoid. We will explain the installation process for version 2.87.

On Ubuntu
~~~~~~~~~
You will use CMake. It is used when installing Choreonoid, so it should be already installed.

Go to the directory where you extracted the file and execute the following command: ::

 ./build_cmake_pybullet_double.sh

This will execute from CMake through the build.

Go to the newly created directory named build_cmake. ::

 make install

Execute this command to install. If you are specifying the installation destination in build_cmake, it is: ::

 ccmake .

This will launch CMake, so specify the installation location in the **CMAKE_INSTALL_PREFIX** item and execute **make install**. You can change other options here, too. Make sure that **USE_DOUBLE_PRECISION** is ON.

On Windows
~~~~~~~~~~

n the README it says to execute build_visual_studio_vr_pybullet_double.bat, so you can use this, but here I would prefer to use the more familiar CMake.

First, go to the directory where Bullet was extracted and create a directory named build_cmake.

Launch the CMake GUI and enter the path to the directory where Bullet was extracted in the “Where is the source code” field and the path to the newly created build_cmake directory in the “Where to build the binaries” field.

Set the following options to ON:

* **BUILD_EXTRAS**
* **INSTALL_EXTRA_LIBS**
* **INSTALL_LIBS**
* **USE_DOUBLE_PRECISION**
* **USE_MSVC_RUNTIME_LIBRARY_DLL**

Setting the following options to OFF is also the safest approach.

* All of **BUILD_XXX_DEMOS**
* **BUILD_BULLET3**
* **BUILD_PYBULLET**
* **BUILD_PYBULLET＿XXX**
* **BUILD_UNIT_TESTS**

Set **CMAKE_BUILD_TYPE** to Release and the installation destination to **CMAKE_INSTALL_PREFIX**.

Proceed by clicking the **Configure** and **Generate** buttons as in the section  :ref:`build-windows-cmake`  in Choreonoid.

.. note:: The CMake options may vary depending on the version of Bullet. Remember that the explanation here is for an example of a working version.

Open the Visual Studio solution file that should have been created in the build_cmake directory.

Confirm that **Release** and **x64** are displayed on the screen in the same way as  :ref:`build-windows-visualstudio`  in Choreonoid, and execute **Build Solution** and **INSTALL**.

Building plugins
----------------

In the CMake build settings for Choreonoid, set the **BUILD_BULLET_PLUGIN** flag to **ON** and specify the directory in which the Bullet library is installed with **BULLET_DIR**.

Execution of simulation
-----------------------

Simulation using the Bullet plugin is the same as when using  :ref:`other physics simulators <simulation_creation_and_configuration_of_simulator_item>` . It can be executed by generating the simulator item BulletSimulator and allocating it as a child item of the world item.

