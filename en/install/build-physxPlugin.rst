
Building the PhysX plugin
=========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>


.. contents:: 目次
   :local:


Installing PhysX
----------------

The PhysX library needs to be installed in order to use the PhysX plugin. We are currently testing operation with version 3.4.

First, get the source code. The PhysX source code is managed on GitHub, but to access it you need to follow the procedure described on the `PhysX Source on GitHub <https://developer.nvidia.com/physx-source-github>`_ page. The procedure is:

1. create an account on developer.nvidia.com;
2. login to that account and register your GitHub account;
3. follow the instructions in the invitation email you will receive from GitHub

. Links for creating an account, etc. are on the above page, so follow those instructions.

Once you are able to access the source for NVIDIAGameWorks/PhysX-3.4, get it by executing git clone.

On Ubuntu
~~~~~~~~~

Go to the **PhysX-3.4/PhysX_3.4/Source/compiler/linux64** directory, where you saved PhysX. ::

  make release

Execute this command to create the library.

On Windows
~~~~~~~~~~

Open the solution file named PhysX.sln in the **PhysX-3.4/PhysX_3.4/Source/compiler/vc14win64** directory, where you saved PhysX.

.. figure:: images/PhysXVC1.png

Right-click on the LowLevel project as shown in the screenshot to display the menu, and select Properties.

When the Properties dialog box opens, change the setting for Code Generation - Runtime Library to Multi-threaded DLL (/ MD).

.. figure:: images/PhysXVC2.png

Make the same change to all the projects.

Then, build the solution.

Building plugins
----------------

In the CMake build settings for Choreonoid, set the **BUILD_PHYSX_PLUGIN** flag to **ON** and set **PHYSX_DIR** to the **PhysX-3.4/PhysX_3.4** directory where you saved PhysX.

Execution of simulation
-----------------------

Simulation using the PhysX plugin is the same as when using :ref:`other physics simulators <simulation_creation_and_configuration_of_simulator_item>` . It can be executed by generating the simulator item PhysXSimulator and allocating it as a child item of the world item.


