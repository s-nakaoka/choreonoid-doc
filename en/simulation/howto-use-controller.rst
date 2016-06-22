
Introduction of Controller
==========================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents::
   :local:

.. highlight:: cpp

Introduction of Controller
----------------------------

It is necessary to introduce a controller to a simulation project to control a robot in the simulation. The basic flow of introduction is as follows:

1. Select a controller item type to be used.
2. Prepare a controller itself that complies with the controller item type.
3. Create a controller item and allocate it as a child item of the body item.
4. Set the controller itself to the controller item.

Here is a concrete explanation about the above operations:

Preparation of Simulation Project
----------------------------------

First, according to the process described in :doc:`simulation-project` , let's create a project with other elements than the controller prepared.

This time, a model that can be the control target of the controller is prepared. In the :ref:`bodymodel_samplemodels` , there is a robot model called "SR1". Let's use it. Load "mode/SR1/SR1.yaml" in share directory.

In addition, incorporate the floor model, the simulator item and the world item that unifies them and make the following project structure: ::

 [ ] - World
 [/]   + SR1
 [/]   + Floor
 [ ]   + AISTSimulator

.. images/controller-project1.png

The robot will be displayed as follows in the scene view:

.. image:: images/controller-scene1.png

SR1 model is a multi-link model having 29 joints and can make different poses by moving its joints. By default, it stands erect, but you can start simulation from any other pose as the initial status. See :doc:`../handling-models/index` - :doc:`../handling-models/pose-editing` and :doc:`../handling-models/legged-model`  for how to edit the pose.

Don't forget :ref:`simulation-time-step` , too. This time, too, set it 1000 [fps] for the time being.

In Case without Controller
----------------------------

To make clear the effect of the controller, let's do simulation as it is without a controller. What does become of the robot when the simulation starts?

.. image:: images/nocontroller-falldown.png

As shown in the figure, it falls down immediately after the start of the simulation.

Since there is no controller, no command is sent to the joints. So, no joint torque takes place and the robot is so weak that it cannot just stand. On the other hand, the gravity work by default, so all the links of the robot fall down according to the gravity.

In this way, a robot cannot even stand still without a controller. So what we need first for a robot is a controller. To make it walk and work, we need a certain level of controller that can support such actions.

Selection of Controller Item Type
----------------------------------

In Choreonoid, controllers are introduced by "Controller Item". A controller item is an abstract item type that defines the base of outputs and inputs. Actually, an item type that inherits this is used. Generally speaking, the controller itself is implemented separately from the controller item. So we have to prepare it as well.

This mechanism allows the controller itself to have any implementation format. In fact, there are various controller formats for robot. Some of them are unique to each robot and others are created according to the specification of middleware such as OpenRTM and ROS. A controller item works as a mediator between its respective implementation format and the virtual robot on Choreonoid. By employing a controller item that can support the controller itself that you want to use, the controller itself is introduced.

For this purpose, it is necessary to have the controller items that can support the controller format to be used. Currently, the following types of controller items are included in the main part of Choreonoid:

* **Simple Controller Item**

 It is a controller item that supports "Simple Controller" format, which is a unique controller implementation format. This format is designed focusing on the simplicity of the controller implementation for the purpose to implement mainly samples. However, the versatility is not well focused and is not assumed to be applied to an actual robot system. Simple controller items are available with "SimpleController-Plugin", which is introduced by default.

* **Body RTC Item**

 A Body RTC item is a controller item that enables co-operating with "OpenRTM", which is middleware for robot. By using this, it becomes possible to control a virtual robot using "RT-Component", which is a component of OpenRTM. BodyRTC items are available by introducing "OpenRTM-Plugin".

* **OpenHRP Controller Item**

 An OpenHRP controller item is a controller item that supports the controller format of "OpenHRP", which is a robot simulator. In fact, there is "OpenHRP3.0 controller item" that supports OpenHRP version 3.0 and "OpenHRP3.1 controller item" that supports "OpenHRP3.1 controller item". By introducing the respective version of OpenHRP-Plugin", they become available respectively. These controller items are prepared to support OpenHRP assets. You need not use controller of this format now.
 
To use a controller that no existing controller item can support, it will be necessary to develop a new Choreonoid plugin that can provide such a controller item.

As for ROS, which have been used a lot recently, the development of the controller items that can support ROS are ongoing.

.. note:: As a method of introducing a controller, you can implement the controller itself as it is as the controller item inheriting type. With this method, you can directly use the native API that accesses to the virtual robot, so the flexibility and the efficiency of the controller can be maximized. However, the controller cannot be used in Choreonoid only and it is not easy to implement it as a plugin. So, this method is generally not used.

Preparation of Controller Itself
-----------------------------------

Prepare a controller itself that complies with the selected controller item type. You may use an existing controller, but you can also develop a new one if necessary.

This time, let's use a simple controller item. In this case, we prepare the controller itself implemented with the simple controller format. In the simple controller format, the control codes are implemented by defining in a C++ class that inherits "SimpleController" class and overriding some virtual functions. What is compiled and made into a shared library file (.so) or a dynamic link library file (.DLL) is the controller itself. See :doc:`howto-implement-controller`  for detail.

.. _simulation-create-controller-item:

Creation of Controller Item
-------------------------------

Select the controller item type to be used from "File" in Main Menu - "New" and create a controller item. Place the created item as a child item of the body item to be controlled. You can create a control item by selecting the body item in advance, or you can drag the created item to this position. This item placement is necessary for the system to identify the control target of the controller item.

In the example this time, we create a simple controller item by selecting "SimpleController" under "New" menu and allocate it under SR1 item as illustrated below: ::

 [ ] - World
 [/]   + SR1
 [ ]     + SimpleController
 [/]   + Floor
 [ ]   + AISTSimulator


.. images/controller-project2.png

.. note:: To use simple controller item, the CMake option when building Choreonoid "BUILD_SIMPLE_CONTROLLER_PLUGIN" must be set to ON. This setting is ON by default.

.. _simulation-set-controller-to-controller-item:

Setting of Controller Itself
-------------------------------

Set the controller itself to the controller item.

In case of a simple controller item, this can be done by specifying the file name of the controller itself in the property "controller module".

Note that, when you omit the directory name and specify the file name only, the file will be searched from the standard directory of the system. The standard directory is "simplecontroller" under the "plugin directory", which was introduced in :doc:`../install/directories` . Therefore, by storing the file of the controller itself, you can specify the controller only with the file name. Also, the extensions like ".so" and ".DLL" can be omitted. Omission of extensions will make the project available in any OS.

For example, let's set "SR1MinimumController", which is a sample of a simple controller targeting SR1 model. It's a very simple controller that only maintains the current posture of a robot. First, verify that the file of this controller is stored in the system's standard directory and then configure the "controller module" of the simple controller item with "SR1MinimumController".

.. note:: The sample controller is created if the CMake option when building Choreonoid "BUILD_SIMPLE_CONTROLLER_PLUGIN" is set to ON. (This setting is ON by default.)

.. note:: How to set the controller itself is specifically different from a controller item type to another. Based on the basic flow of introduction of a controller stated in this section, read the document of the controller item to be used actually and configure it. In case of a Body RTC item, for example, you can configure the controller by combining multiple RT components, but you cannot realize it by just specifying one file name of the controller but a more complicated configuration is required.

Execution of Simulation
---------------------------

The above configuration done, execute the simulation. If the configuration is successful, the robot can maintain its posture without falling down this time. This is because the torque order to maintain the posture is output to each of the joints by PD control code implemented in "SR1MinmumController".

If it is not successful, check the message view. If there is a problem in the configuration or the operation of the controller, a message informing this may be output when the simulation is started.

As for a body item in which only one controller is configured, :ref:`simulation-result-item-output` is not a child item of the body item but a child item of the controller item. This is to make the item tree easy to read but the result of replay or other operations is not particularly different in case without controller.

Other controller samples are available: See  :ref:`basics_sample_project` and try other samples, too. For the samples targeting SR1 model, we have projects like "SR1Walk.cnoid", which makes the robot walk and "SR1Liftup.cnoid", which makes the robot lift up a box. You can verify how the robot acts differently depending on the controllers.
