Building the Roki plugin
========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>


.. contents:: 目次
   :local:

.. highlight:: YAML

Installing RoKi
---------------

RoKi (Robot Kinetics Library) needs to be installed in order to use the Roki plugin. RoKi only works on Linux.

First, install CURE.

On the `RoKi website  <http://www.mi.ams.eng.osaka-u.ac.jp/open-e.html>`_ , go to the CURE page, and download and extract the file named cure-1.0.0-beta5.tgz.

There is a file named README and you should basically follow the instructions contained in it.

.. note:: By default, the installation destination is **home/usr**. The following explanation assumes that this is the installation location. If you want to change the installation destination, edit the config file and change the following details as appropriate.
 
First, create the installation destination folder, and then create subfolders named bin and lib inside it. Launch Terminal and execute the following in the home directory. ::

 mkdir usr
 cd usr
 mkdir bin
 mkdir lib

Add this directory to the environment variables PATH and LD_LIBRARY_PATH. Using a text editor, open the file named .profile in the home directory. Paste the following at the end of the file and save it: ::

 export PATH=$PATH:$HOME/usr/bin
 export LD_LIBRARY_PATH=$HOME/usr/lib:$LD_LIBRARY_PATH

In order for these settings to take effect, execute the command: ::

 source .profile
 
. Go to the directory where the CURE source code was extracted and execute the following  ::

 make
 make install
 
If this is successful, you should have cure-config under home/usr/bin, and libcure.so under home/usr/lib.

Next, go to the ZM page, and download and extract the file. Go to the directory where you extracted the file and execute make and make install.

Install Zeo in the same way as RoKi.

Building plugins
----------------

In the CMake build settings for Choreonoid, set the **BUILD_ROKI_PLUGIN** flag to ON and specify the directory in which RoKi is installed with **ROKI_DIR**.

Execution of simulation
-----------------------

Simulation using the Roki plugin is the same as when using :ref:`other physics simulators <simulation_creation_and_configuration_of_simulator_item>` . It can be executed by generating the simulator item RokiSimulator and allocating it as a child item of the world item.

RokiSimulator item properties
-----------------------------

The RokiSimulator item has the following properties

.. list-table:: Roki plugin properties
 :widths: 15,60
 :header-rows: 1

 * - Property
   - Details
 * - staticfriction
   - Static friction
 * - kineticfriction
   - Kinetic friction
 * - contactType
   - Select “rigid” or “elastic” as the type of contact between objects
 * - solverType
   - Select “Vert” or “Volume”
 * - compensation
   - Enabled when contact parameter contactType is “rigid” and useContactFile is “false”
 * - relaxation
   - Enabled when contact parameter contactType is “rigid” and useContactFile is “false”
 * - elasticity
   - Enabled when contact parameter contactType is “elastic” and useContactFile is “false”
 * - viscosity
   - Enabled when contact parameter contactType is “elastic” and useContactFile is “false”
 * - useContactFile
   - Whether or not to read the contact parameter settings from file
 * - contactFileName
   - The name of the contact parameter settings file. Enabled when useContactFile is “true”

Refer to the `RoKi website <http://www.mi.ams.eng.osaka-u.ac.jp/open-j.html>`_ for details about parameters. 

Joint dynamics simulation
-------------------------

Joint dynamics simulation is possible with RoKi. This sample project is RokiArm2Dof.cnoid.

The joint dynamics parameters are described in the model file named arm_2dof.body. In this model, the same parameters are applied to the two joints, so we will use the  :ref:`body-file-reference-link-node`  **import**. Refer to the  :ref:`modelfile_yaml_alias`  section for an explanation of the alias function. If different parameters are to be set for each joint, they are entered directly in the Link node. ::

 actuator1: &actuator1
   rotorInertia: 1.65e-6
   gearRatio: 120.0
   gearInertia: 5.38e-6
   motorAdmittance: 0.42373
   motorConstant: 2.58e-2
   motorMinVoltage: -24.0
   motorMaxVoltage: 24.0
   jointStiffness: 0.0
   jointViscosity: 2.2
   jointFriction: 4.32
   jointStaticFriction: 4.92
  
 links:
    .......
   -
     name: Joint1
      .......
     import: *actuator1
      .......
   -
     name: Joint2
      .......
     import: *actuator1
      .......
      
The joint parameters are as follows:

.. list-table:: 
 :widths: 15,40
 :header-rows: 1

 * - Parameter
   - Details
 * - motorconstant
   - The motor constant (torque constant)
 * - admitance
   - The admittance between the terminals (the reciprocal of the impedance between the terminals)
 * - minvoltage
   - The minimum voltage
 * - maxvoltage
   - The maximum voltage
 * - inertia
   - The motor’s moment of inertia
 * - gearinertia
   - The reduction gear moment of inertia
 * - ratio
   - The reduction ratio
 * - stiff
   - The joint’s stiffness coefficient
 * - viscos
   - The joint’s viscosity coefficient
 * - coulomb
   - The joint’s dryness coefficient (dynamic friction torque)
 * - staticfriction
   - The maximum static friction torque

Simulation of destruction
-------------------------

In RoKi, simulation of fracture is possible by describing the location where the fracture occurs as a joint in the model file. This sample project is RokiBreakWall.cnoid.

The fracture model is described in breakWall.body. Define the location where the fracture occurs as a joint and set the joint type to “free”. Then, in order of force and then torque, describe the norm threshold of force and torque where fracture occurs in the **break** parameter. ::

 links :
   -
    name: BASE
    jointType: fixed
     ................
    elements:
      Shape:
        geometry: { type: Box, size: [ 0.099, 0.049, 0.099 ] }
   -
    name: link1
    parent: BASE
    translation : [ 0, 0, 0.05 ]
    jointType: free
      .............
    break: [ 200.0, 200.0 ]
      .............
    elements:
      Shape:
        geometry: { type: Box, size: [ 0.099, 0.049, 0.099 ] }
  -
    name: link2
    parent: link1
    translation : [ 0, 0, 0.1 ]
    jointType: free
      .............
    break: [ 10.0, 10.0 ]
      .............
      
So that objects don’t slip through each other after fracture, the self-collision detection property of the breakWall model item needs be set to true. But this means that autointerference also occurs before the fracture. In order to avoid this, in the breakWall model the geometry of the links is set so as to form a slight gap between the links.

Set the “Output all link positions and orientations” property of the RokiSimulation item to true.

