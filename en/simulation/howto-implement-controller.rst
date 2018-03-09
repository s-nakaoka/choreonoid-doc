
Controller implementation
=================================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents:: 
   :local:

.. highlight:: cpp

Controller implementation
---------------------------------

Here we describe the basics of implementing a controller in your workflow.

A controller generally performs the three core functions below. These are executed as a repeating control loop.

1. Robot state input
2. Control calculations
3. Outputting commands to the robot

These processes can be managed through standalone controllers for each, or through the combination of multiple software components. We referred to “control calculations” above as a single concept, but this includes a wide range of detection and movement computations and may also include input/output designed for entities other than robots. In the context of robots, however, it is best to think of the controller as ultimately carrying out the three processes above.

This lets us interpret the controller as a software module equipped with the interfaces to perform those three processes. The actual API varies by controller format, but the core aspects are the same.

Below, we utilize the SR1MinimumController discussed in the section on :doc:`howto-use-controller`  as an example. The controller is structured as a “simple controller” used for sample purposes in Choreonoid. The control routines are limited to maintaining the orientation of the robot by use of PD control of the joints. The controller is written in C++.

When developing your own controller, you can refer to this sample and its core functionality while restructuring it in your preferred format and with your preferred control routines. Generally speaking, the development of robot controllers requires a range of knowledge and expertise in control processes, programming, hardware, and more. The majority of these competencies are outside of the scope of this manual. You should pursue each area as needed for your particular use case.


Sample controller source code
----------------------------------

Below you will find the SR1MinimumController source code. You will find this source code in the file SR1MinimumController, located in the /sample/SimpleController/ path of the Choreonoid source directory. ::

 #include <cnoid/SimpleController>
 #include <vector>
 
 using namespace cnoid;
 
 const double pgain[] = {
     8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
     3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
     8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
     3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
     8000.0, 8000.0, 8000.0 };
     
 const double dgain[] = {
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0 };

 class SR1MinimumController : public SimpleController
 {
     BodyPtr ioBody;
     double dt;
     std::vector<double> qref;
     std::vector<double> qold;

 public:

     virtual bool initialize(SimpleControllerIO* io) override
     {
	 ioBody = io->body();
	 dt = io->timeStep();

         for(int i=0; i < ioBody->numJoints(); ++i){
             Link* joint = ioBody->joint(i);
	     joint->setActuationMode(Link::JOINT_TORQUE);
	     io->enableIO(joint);
	     qref.push_back(joint->q());
	 }
	 qold = qref;

	 return true;
     }

     virtual bool control() override
     {
	 for(int i=0; i < ioBody->numJoints(); ++i){
	     Link* joint = ioBody->joint(i);
	     double q = joint->q();
	     double dq = (q - qold[i]) / dt;
	     double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
	     qold[i] = q;
	     joint->u() = u;
	 }
	 return true;
     }
 };

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

This controller is provided as a sample with Choreonoid; by default, it is configured to build simultaneously with Choreonoid. (It is enough for the CMake settings to specify **BUILD_SIMPLE_CONTROLLER_SAMPLES** as “ON.”)

For instructions on how to separately implement the SimpleController in a standalone fashion from the sample, please refer to the section on :doc:`howto-build-controller` .

The SimpleController Class
--------------------------------

Controllers in the SimpleController format are implemented by inheriting the SimpleController class. This class can be used per the below: ::

 #include <cnoid/SimpleController>

This includes the cnoid/SimpleController header.

Generally speaking, this class is defined as follows: ::

 class SimpleController
 {
 public:
     virtual bool initialize(SimpleControllerIO* io) = 0;
     virtual bool control() = 0;
 };

By overriding the inherited class with this virtual function, we specify the controller process. The contents of each function are given below.

* **virtual bool initialize(SimpleControllerIO\* io)**

 This function initializes the controller. Use the io argument to poll objects and data implicated in the control process.

* **virtual bool control()**

 This function handles controller input, control, and output. When being controlled, this function is executed as an ongoing control loop.

If you define a class to inherit SimpleController, you must define a factory function for it. It is fine to use a macro as below to achieve this: ::

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

This allows the common library files (dynamic links) compiled from source to be usable as an actual controller in the form of a SimpleController item.

.. _simulator-simple-controller-io:

IO objects
--------------

The SimpleControllerIO object passed as an io argument to the above initialize function is an object that contains the requisite information for I/O between the controller and robot. Below, we refer to this object as an “IO object.”

This class inherits the ControllerIO class. Some of the functions defined in the ControllerIO class are below; these can be used to implement the controller as you see fit.

* **Body\* body()**

 Returns the body object used for input/output.

* **std::string optionString() const**

 Returns the option string given to the controller.

* **std::vector<std::string> options() const**

 Returns a space-delimited breakdown of the option string.

* **std::ostream& os() const**

 Returns an output stream of messages output from the controller.

* **double timeStep() const**

 Returns the time step. Given in seconds.
 
* **double currentTime() const**

 Returns the current time. Given in seconds. At the start of the simulation, the time is 0.

.. _simulator-io-by-body-object:

Input/output using body objects
-----------------------------------------

The SimpleController allows for input and output via Body objects. The Body object is a Choreonoid internal expression for :doc:`../handling-models/bodymodel` and an instance of the Body class defined in C++. The Body class is a data structure used to store the robot model and its state, so it can be used to store values like joint angle, torque, sensor status, and other data implicated in input/output. This is why the SimpleController allows for input and output via Body objects. These Body objects can be obtained via the body function of the IO object.


Link objects
~~~~~~~~~~~~~~~~

Body objects are expressed as a Link class object representing the individual components (rigid bodies) making up the model. These objects contain information pertaining to joints. (See :ref:`model_structure` ）. Link objects can be obtained via the Body class functions below.

* **int numJoints() const**

 Returns the number of joints in the model.

* **Link\* joint(int id)**

 Returns the Link object corresponding to the joint ID.
  
* **Link\* link(const std::string& name)**

 Returns the Link object with the name given for the name variable.
 
The below member functions (state variables) can be used to access the joint state values for the Link object polled. (These member functions return a reference to the corresponding variable and can be used to substitute a value.) 

* **double& q()**

 Returns a reference to a joint displacement value. Works with JOINT_ANGLE, JOINT_DISPLACEMENT. Units are [rad] or [m].

* **double& dq()**

 Returns a reference to a joint velocity value. Works with JOINT_VELOCITY. Units are [rad/s] or [m/s].
* **double& ddq()**

 Returns a reference to a joint velocity value. Works with JOINT_ACCELERATION. Units are [rad/s^2] or [m/s^2].
 
* **double& u()**

 Returns a reference to a joint torque (translation) value. Works with JOINT_TORQUE and JOINT_FORCE. Units are [N, m] or [N].

SimpleController generally uses the above state variables to handle input/output for each joint. In other words, when taking input, it reads the variable value; when giving output, it writes the corresponding variable value.

However, which value is treated as an actuator command value and which is read as input varies by the type of actuator and control method.

.. _simulation-implement-controller-actuation-mode:

ActuationMode
~~~~~~~~~~~~~~~~~~~~~~~~

ActuationMode is the basic concept implicated in joint output. It is used to determine which state variable to use as a command value when driving joints. The below symbols are defined in the Link class for this mode.

.. list-table:: **Link::ActuationMode enumeration symbols**
 :widths: 20,60,20
 :header-rows: 1

 * - Symbol
   - Details
   - State variable
 * - **NO_ACTUATION**
   - No actuation/drive. The joints operate freely.
   - 
 * - **JOINT_EFFORT**
   - A command value is used to assign force and torque to the joint.
   - Link::u()
 * - **JOINT_FORCE**
   - Same as JOINT_EFFORT. Defined for prismatic joints.
   - Link::u()
 * - **JOINT_TORQUE**
   - Same as JOINT_EFFORT. Defined for rotating joints.
   - Link::u()
 * - **JOINT_DISPLACEMENT**
   - A command value for joint displacement (joint angle and joint translation position).
   - Link::q()
 * - **JOINT_ANGLE**
   - Same as JOINT_DISPLACEMENT. Defined for rotating joints.
   - Link::q()
 * - **JOINT_VELOCITY**
   - A command value for joint velocity and offset speed.
   - Link::dq()
 * - **JOINT_SURFACE_VELOCITY**
   - A command value for relative velocity on the intersection of the link surface and environment. This is used for simplified crawler and conveyor belt simulations. For details, see the section on :doc:`pseudo-continuous-track` .
   - Link::dq()

ActuationMode references and configures the following Link class functions.

* **ActuationMode actuationMode() const**

 Returns the currently set ActuationMode.

* **void setActuationMode(ActuationMode mode)**

 Configures the ActuationMode.

Enabling I/O
~~~~~~~~~~~~~~~~

IO objects are used to configure which state variables to use as input/output to/from the controller. To do so, the SimpleControllerIO class defines the following functions.

* **void enableInput(Link\* link)**

 Enables input to the state quantity controller for the link specified with the “link” attribute. Works with corresponding state quantities for the ActuationMode set for the link.

* **void enableInput(Link\* link, int stateTypes)**

 Using the link given with “link,” stateTypes enables passing input on status quantity to the controller.

* **void enableOutput(Link\* link)**

 Using the link given with “link,” enables output of status quantity from the controller. The intended output target is the state quantity corresponding to the ActuationMode set for the link.

* **void enableIO(Link\* link)**

 Enables input/output of status quantities for the link given with “link.” The intended output target is the state quantity corresponding to the ActuationMode set for the link.
 
.. note:: SimpleControllerIO includes definitions for functions like setLinkInput, setJointInput, setLinkOutput, and setJointOutput. These are the functions used in Choreonoid version 1.5 and prior; starting in 1.6, they have been replaced by the aforementioned enableIO, enableInput, and enableOutput. You should use the latter functions when using versions 1.6 and later.

The values given to StateTypes in the enableInput function are the below symbols defined for SimpleControllerIO.

.. list-table::
 :widths: 20,60,20
 :header-rows: 1

 * - Symbol
   - Details
   - State variable
 * - JOINT_DISPLACEMENT
   - Joint displacement
   - Link::q()
 * - JOINT_ANGLE
   - Same as JOINT_DISPLACEMENT. Defined for rotating joints.
   - Link::q()
 * - JOINT_VELOCITY
   - Joint speed (angular velocity)
   - Link::dq()
 * - JOINT_ACCELERATION
   - Joint acceleration (angular acceleration)
   - Link::ddq()
 * - JOINT_EFFORT
   - The joint translation force or torque.
   - Link::u()
 * - JOINT_TORQUE
   - Same as JOINT_EFFORT. Defined for rotating joints.
   - Link::u()
 * - JOINT_FORCE
   - Same as JOINT_EFFORT. Defined for prismatic joints.
   - Link::u()
   
To specify multiple elements, separate them with the bitwise operator “|”. For example, using ::

 JOINT_DISPLACEMENT | JOINT_VELOCITY

lets you specify both the joint displacement and velocity.

The ActuationMode available for use varies based on the simulator item (≒ physics engine) type and configuration. The majority of simulator items support JOINT_EFFORT. Combining this with JOINT_DISPLACEMENT input let you perform PD control.

The ActuationMode set for the Link object generally takes the following types of input.

.. list-table::
 :widths: 50,25,25
 :header-rows: 1

 * - ActuationMode
   - Input
   - Output
 * - JOINT_EFFORT
   - Link::q()
   - Link::u()
 * - JOINT_DISPLACEMENT
   - N/A
   - Link::q()
 * - JOINT_VELOCITY
   - Link::q()
   - Link::dq()

However, using enableInput to pass a stateTypes parameter lets you freely input a state quantity of your choice.

.. note:: You can also use the **LINK_POSITION** symbol against direct input/output on the position and orientation of a link in 3D space. We go into this in later detail in the section on  :ref:`simulation-implement-controller-link-position` .

The initialization process
--------------------------------

The initialize function inherits the SimpleController class and is used to initialize the controller.

In the sample, first use ::

 ioBody = io->body();

to poll the I/O body object and store it in the member variable ioBody. This lets you use the object in a different function within the controller.

Similarly, the time step (delta time) used for control calculations is: ::

 dt = io->timeStep();

This stores values in a member variable (dt).

Next, we use the following for statement to initiate a loop on all of the robot’s joints and initialize them. ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     ...
 }

The line below in the loop polls the link object for the nth (i) joint and sets it to the joint variable. ::

 Link* joint = ioBody->joint(i);

.

Next, use ::

 joint->setActuationMode(Link::JOINT_TORQUE);

to configure the ActuationMode for this joint. Here we use Link::JOINT_TORQUE to set a command value for joint torque. Also, using ::

 io->enableIO(joint);

enables input/output for the joint. Since the ActuationMode is set to JOINT_TORQUE, the output is joint torque and the input is joint angle. This lets you achieve PD control.

Next,  ::

 qref.push_back(joint->q());

is used to store the vector variable for the joint angle when the robot is in its default state. This also uses PD control. This concludes the for loop used for the joints.

Next,  ::

 qold = qref;

is used to initialize the qold variable to the same value as qref. This variable is used in PD control to reference the joint angle one step prior.

Returning “true” as the return value against the initialize function conveys to the simulator that the initialization succeeded.

Control loops
-------------------

Next, we give the class inheriting SimpleController a control loop in its control function.

As with initializing, use the for statement below: ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     Link* joint = ioBody->joint(i);
     ...
 }

This performs control calculations against all joints. This code is used for operations on each joint.

First, we input the current joint angle. ::

 double q = joint->q();

PD control is used to calculate the command value for joint torque. The difference from the last joint angle in the control loop is used to calculate the current joint angular velocity. ::

 double dq = (q - qold[i]) / dt;

The goal of the control operation here is to maintain the initial orientation and stance of the model; the joint angle target is the initial joint angle, and the velocity is 0 (still), with a torque command value calculated to that end. ::

 double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];

The pgain and dgain array set at the beginning of the source code are used to extract gain values for each joint. Gain values must be adjusted for each model; in the interest of time, we will omit an explanation of that here.

Save the joint angle as the qold variable for use in calculation later. ::

 qold[i] = q;

This exports a command value on calculated torque. This allows you to control the joint and maintain its initial angle. ::

 joint->u() = u;

All of the above apply to joints and ensure that the orientation and stance of the entire robot is maintained.

Lastly, when the control function returns true, this conveys to the simulator that the control has been inherited. This allows the control function to be continuously called in a loop.

.. _simulation-device:

Device I/O
----------------------

Devices explained
~~~~~~~~~~~~~~~~~~~~~~~~~~

Thus far, we have handled input/output of status quantity implicated in joints, such as with joint angle and torque. By contrast, there are also input/output elements that are separate from joints. These are defined as “devices” in Choreonoid and form constituent elements of Body models.

Examples of devices include:

* Power sensors, velocity sensors, angular velocity sensors (rate gyros)
* Cameras and laser range sensors

among others. These are generally used as sensors for input.

In addition, external (outside world) outputs can include:

* Lights
* Speakers
* Displays

and other devices. (Speakers and display are listed only as examples and not actually implemented at this time.)

When developing actual controllers, input and output must be handled with respect to these numerous devices. To do so, you must ascertain:

* How the device is defined in the model
* How to access a given device in the controller format

.

.. _simulation-device-object:

Device objects
~~~~~~~~~~~~~~~~~~~~

Choreonoid’s Body models express device information in the form of Device objects. These are instances that inherit the properties of Device classes; Device objects are defined for each device type. The default devices available include: ::

 + Device
   + ForceSensor (force sensor)
   + RateGyroSensor (angular velocity sensor)
   + AccelerationSensor (accelerometer)
   + Camera (camera）
     + RangeCamera (camera + distance image sensor）
   + RangeSensor (range sensor）
   + Light
     + PointLight (point light source）
     + SpotLight (spot light）

Device information in robots is generally described in model files. Standard format model files involve :ref:`body-file-reference-devices` , as discussed in the :doc:`../handling-models/modelfile/yaml-reference` .

As with the Body and Link objects, SimpleController conducts input/output on devices as-is using the Device object, an internal expression of Choreonoid.

The Device objects used by the SR1 model we reference in this section are as follows:

.. tabularcolumns:: |p{3.5cm}|p{3.5cm}|p{6.0}|

.. list-table::
 :widths: 30,30,40
 :header-rows: 1

 * - Name
   - The type of device.
   - Details
 * - WaistAccelSensor
   - AccelerationSensor
   - Accelerometer installed on the waist link
 * - WaistGyro
   - RateGyroSensor
   - Gyro installed on the waist link
 * - LeftCamera
   - RangeCamera
   - Distance imaging sensor for the left eye of the camera
 * - RightCamera
   - RangeCamera
   - Distance imaging sensor for the right eye of the camera
 * - LeftAnkleForceSensor
   - ForceSensor
   - Force sensor installed on the left ankle
 * - RightAnkleForceSensor
   - ForceSensor
   - Force sensor installed on the right ankle


Polling device objects
~~~~~~~~~~~~~~~~~~~~~~~~~~

Device objects are polled by using the below functions against Body objects.

* **int numDevices() const**

 Returns the number of devices.

* **Device\* device(int i) const**

 Returns the nth (i) device. Device order follows the order of notation within the model file.

* **const DeviceList<>& devices() const**

 Returns a list of all devices.

* **template<class DeviceType> DeviceList<DeviceType> devices() const**

 Returns a list of all devices of the type given.

* **template<class DeviceType> DeviceType\* findDevice(const std::string& name) const**

 If there is a Device of the type and name given, returns it.

To poll a specific type of device, use DeviceList, a template class. DeviceList is an array that contains the device objects specified. Its constructor and extraction operator (<<) can be used to extract the specific object from the list and exclude others. For instance, if you want to pull the force sensor belonging to “ioBody,” a Body object, use: ::

 DeviceList<ForceSensor> forceSensors(ioBody->devices());

Or, you could add the below against an existing list: ::

 forceSensors << ioBody->devices();

.

DeviceList contains functions and operators similar to std::vector. For example, use: ::

 for(size_t i=0; i < forceSensors.size(); ++i){
     ForceSensor* forceSensor = forceSensor[i];
     ...
 }

to access each object.

Using the findDevice function lets you specify a device type and name and poll it. For example, the SR1 model has a WaistAccelSensor equipped on the waist link. This is an accelerometer. To poll it, you would use the below against the Body object: ::

 AccelerationSensor* accelSensor =
     ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");

.

.. _simulation-implement-controller-device-io:

I/O methods
~~~~~~~~~~~~~~~~~~~

Input and output via a Device object is done as follows:

* **Input**

 Invoke the below function against the SimpleController IO object:

 * **void enableInput(Device\* device)**

 This lets you enable input to the device. You can then use member variables for the corresponding Device object to poll values.

* **Output**

 After using the member function for the corresponding Device object, set a value and then invoke the following function for the Device object:

 * **void notifyStateChange()**

 This will update the device status on the simulator.
 
To do the above, you must know the class definitions used by the device in question. For example, AccelerationSensor, an accelerometer class, includes the dv() member function used to access its state. This returns a three-dimensional vector result for the acceleration.

Accelerometer input on the SR1 model functions as follows. In the initialize function for the controller, use the below: ::

 AccelerationSensor* accelSensor =
     ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
 io->enableInput(accelSensor);

This will enable input to the accelSensor. Next, in the section referencing the accelerometer value within the control function, use the below: ::

 Vector3 dv = waistAccelSensor->dv();

to poll it.

Similarly, you can use the corresponding member functions for ForceSensor and RateGyroSensor to handle state input.

When using cameras, range sensors, and other visual sensors, you must configure your settings accordingly. This is explained in the section on  :doc:`vision-simulation` .

For output to a device, see the sample for TankJoystickLight.cnoid, which handles on/off functionality for the light.

.. _simulation-implement-controller-link-position:

Input/output of link position and orientation
-----------------------------------------------------

Other targets of controller input/output are link position and orientation. This refers not to the joint angle, but rather to the position and orientation of the link itself as a rigid body in the global coordinate system. This value ordinarily cannot carry out input/output against a robot device. For robots not fixed to a point in the space, obtaining the exact position and orientation of a specific link (provided you are not using super accurate motion capture) is difficult. Furthermore, it is physically impossible to directly change this position and orientation of a link through controller output. However, the above can be achieved in a simulation, so the system includes input/output of this value for such use.

To do so, specify **LINK_POSITION** as a symbol of state quantity. To generate output, give the setActuationMode function of the Link object the following: **Link::LINK_POSITION**. Then, use the enableIO function or enableOutput function of the IO object to enable output. For input, use the enableInput function for the IO object and set **SimpleControllerIO::JOINT_POSITION**.

The position and orientation of Link objects is stored as a Position value. This is a customized “Transform” version of the Eigen matrix and vector library used to implement Choreonoid. It generally works by storing a converted array of 3D homogeneous coordinates. This value can be accessed by using the below Link class functions, among others.

* **Position& T(), Position& position()**

 Returns a reference to the Position value for the position and orientation.

* **Position::TranslationPart translation()**

 Returns a 3D vector for the position.

* **void setTranslation(const Eigen::MatrixBase<Derived>& p)**
   
 Sets the position element. You can use a 3D vector format equivalent to that used by Eigen for the argument.

* **Position::LinearPart rotation()**

 Returns a 3x3 array for the orientation (rotation) element.。

* **setRotation(const Eigen::MatrixBase<Derived>& R)**

 Sets the orientation (rotation). You can use a 3x3 array equivalent to that used by Eigen for the argument. 

* **setRotation(const Eigen::AngleAxis<T>& a)**

 Sets the orientation (rotation). The argument is the AngleAxis format used by Eigen to describe rotational axis and angle of rotation. 
 
As an example, when entering the root link position, you could use the below for the controller initialize function: ::

 io->enableInput(io->body()->rootLink(), LINK_POSITION);

For the control function, using ::

 Position T = io->body()->rootLink()->position();
 Vector3 p = T.translation();
 Matrix3 R = T.rotation();

would obtain the root link position and orientation.

A simulator supporting output of link position and orientation is needed here, which is a special use case. For example, the AIST simulator item allows for changing the dynamics mode to kinematics, with no dynamics calculations performed in the simulation; instead, only the position and orientation given are reproduced. In this case, outputting the position and orientation of the robot’s root link will navigate the root link to that point. If you output the joint angle, it will reproduce the orientation based on the forward kinematics from the root link.

Other samples
--------------------
 
Choreonoid includes a variety of other controllers besides the SR1MinimumController. You can find :ref:`basics_sample_project` that make use of these, so please have a look.
