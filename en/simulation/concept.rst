
Basic Concept
===============

.. contents::
   :local:
   :depth: 1


Simulation Function
----------------------

Choreonoid is equipped with the simulation function, which can be used as a robot simulator. The function simulate how a robot or an environment object moves though calculations and presents the result in animation by 3DCG or output as data. Using this function, it becomes possible to perform hardware design and software-related verifications as well as practise operations or use for training.

The simulation targets are the "body models", which were introduced in :doc:`../handling-models/index` . By importing body models as body items and make them belong to a virtual world as world items, it will be possible to make simulations targeting this virtual world.


Physical Calculation Engine
-----------------------------

The core part of the simulator is referred to as "Physical Calculation Engine", which is the part that calculates how an object moves physically. Various algorithms for physical calculations are devised and many different physical calculation engines, which are the implementations of such algorithms, have been developed. Models and physical phenomena that can be simulated depend on the engine and the simulation characteristics including the accuracy, the stability and the calculation speed also varies depending on the engine. So, it is important to use a proper engine for the target and the purpose of the simulation. From this point of view, Choreonoid is so designed that various physical calculation engines can be used.


.. _simulation_simulator_item:

Simulator Items
--------------------

In Choreonoid, a physical calculation engine is represented as a project item called a "simulator item". It is an item to incorporate physical calculation engines to the simulation function of Choreonoid and is equipped with APIs, which is the base for this purpose. What will be actually used is an item that inherits a simulator item and a corresponding simulator item is defined for each physical calculation engine. In concrete terms, the following simulator items are available:

* **AIST Simulator Item**

 It is a standard simulator item of Choreonoid, which makes simulations using its unique physical calculation engine.

* **ODE Simulator Item**, **Bullet Simulator Item**, **PhysX Simulator Item**

 The are simulator items that utilise `Open Dynamics Engine (ODE) <http://www.ode.org/>`_ , `Bullet Physics Library <http://bulletphysics.org>`_ and `PhysX <https://developer.nvidia.com/gameworks-physx-overview>`_ respectively, which are physical calculation engines that can be used externally as a library. They become available by installing the corresponding library and building ODE plug-in, Bullet plug-in or PhysX plug-in.

.. note:: For physical calculations, it is necessary to detect a collision that occurs between the objects to be simulated. Normally, the collision detector that performs such operations is also included in a physical calculation engine. On the other hand, as was described in  :doc:`../handling-models/collision-detection` under :doc:`../handling-models/index`, the function to detect a collision is prepared as a basic function of Choreonoid and various collision detectors are available there. (:ref:`handling-models_switch-collision-detector`) With some simulator items, any collision detector of the collision detection function as the basic function can be used.

.. _simulation_subsimulator:

Sub-simulator
----------------

A physical calculation is basically performed by a simulator item, but "sub-simulators" are also available as an item that can realise diversified simulation functions supplementary thereto.

For example, you may simulate the camera or the distance sensor function mounted in a robot and retrieve a camera image or a distance image even during the simulation. As a sub-simulator that adds this function, :ref:`simulation-gl-vision-simulator` item is available. This simulator simulates the sensor outputs by performing internally a drawing process similar to 3DCG view from the perspectives of a camera or a distance sensor. In contrast to a "physical calculation engine", such a sub-simulator can be called as a "visual information calculation engine". This function is not dependent on a physical calculation algorithm and can be used in combination with any simulator item.

Sub-simulators can realise many other different functions in the framework where the situation of a virtual world is monitored so that the corresponding outputs can be provided or the virtual work can be modified.

Controller
------------

To make a robot motion, a programme to control it is required. Such a programme is called a "Controller". In a simulation, a controller to make a robot motion is required. In general, the same controller is used for both a real robot and a simulated robot. By doing so, we aim to carry out the development and the verification of a controller efficiently on the simulation. Also, by doing so, it will be possible for users to practise the operation and manoeuvre of the robot system developed on the simulator.

Anyway, a controller is required to make a robot motion and also is a main element of a simulation.

.. _simulation-concept-controller-item:

Controller Item
--------------------

In the simulation function of Choreonoid, a controller is represented as a project item called a "controller item". Actually, the controller body implemented separately from the controller item is operated using the item model that inherits the controller item. There can be various formats of controller body. However, as long as a controller items that supports a certain format is prepared, the controller in that format can be used. For example, for "RT Component", which is a software component of RT middleware, the corresponding controller item "Body RTC Item" can be used.

How to use controller items will be described in :doc:`howto-use-controller` .


Output and Input between Robot and Controller
-----------------------------------------------

What is necessary first for a controller to control a robot is to input and output different data with the robot. That is to say, the controller retrieves the status of the robot or its environment from the input from the different sensors mounted to the robot first and then it outputs the instruction value decided as a result of the control calculation based on the input to the actuator, etc. of the robot.

In concrete, the following elements can be the actual input:

* Joint angle of rotary joint
* Joint translation of linear motion joint
* Force sensor
* Acceleration sensor
* Angular acceleration sensor
* Camera image
* Range sensor distance image

The following elements are the output targets:

* Torque at rotary joint
* Force at linear motion joint
* Operation instruction (on/off) of various devices (including the light)

You may well regard a controller item as something that defines the interface for input/output.

The actual input and output methods will be described under :doc:`howto-implement-controller` .

Utilisation of Plug-in
-------------------------

It is possible to add an inheriting item model to a simulator item, a sub-simulator item and a controller item using a plug-in. Using a plug-in,

* Addition of a physical calculation engine available;
* Expansion of a simulation function; and/or
* Addition of a supportable controller format

can be supported. In other words, Choreonoid is a platform on which the simulation function per se can be expanded.

.. See :doc:`../plugin-development/ode-plugin` :doc:`../plugin-development/index` for how to implement a simulator item.

