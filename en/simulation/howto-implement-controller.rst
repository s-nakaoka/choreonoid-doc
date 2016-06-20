
Implementation of Controller
============================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents::
   :local:

.. highlight:: cpp


Implementation of Controller
----------------------------

The basiscs of how to implement a controller are described below:

What a controller does is basically the following three things and it executes them repeatedly as "control loop".

1. To input the status of the robot;
2. To make control calculations; and
3. To output commands to the robot

These processes are executed by a single controller or in combination with multiple software components. A process where "control calculations" are bundled actually involves diversified processes like different recognitions and motion plans and may include inputs/outputs for something else than the robot. However, from the robot perspective, what a controller does can be sorted out to the above-mentioned three processes.

From this perspective, a controller is a software module having interfaces that handle the above three actions. The actual API for this varies among the controller formats but the essential part is identical.

Here, an explanation is provided using "SR1MinimumController" sample, which was also used in :doc:`howto-use-controller` . The format of the controller is the "simple controller" format designed for the samples in Choreonoid and its content of control is just to maintain the robot's posture. The implementation language used is C++.

When actually developing a controller, the basics that were provided using this sample can be replaced with the desired controller format and the content of control. Generally speaking, diversified knowledge and skills related to control, programming, hardware, etc. are required to develop a controller for a robot. Many of those skills are out of scope of this manual. Study the knowledge and the skills required separately.


Source Code of Sample Controller
--------------------------------

First, the source code of SR1MinimumController is as follows: This source code is a file called "SR1MinimumController.cpp" under the directory "sample/SimpleController" of Choreonoid sources. ::

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
 
     virtual bool initialize(SimpleControllerIO* io)
     {
         ioBody = io->body();
         dt = io->timeStep();

         io->setJointInput(JOINT_ANGLE);
         io->setJointOutput(JOINT_TORQUE);
 
         for(int i=0; i < ioBody->numJoints(); ++i){
             qref.push_back(ioBody->joint(i)->q());
         }
         qold = qref;
 
         return true;
     }
 
     virtual bool control()
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

As for compile, it is described in: ::

 add_cnoid_simple_controller(SR1MinimumController SR1MinimumController.cpp)

in CMakeList.txt unde the same directory. See "src/SimpleControllerPlugin/library/CMakeLists.txt" for detail of this function. Basically, it is OK to link with the library "CnoidSimplerController". (In case of Linux, the file name of the library will be "libCnoidCimpleController.so".

SimpleController Class
----------------------

A controller of simple controller format is implemented by inheriting SimpleController class. This class becomes available by including cnoid/SimpleController header. ::

 #include <cnoid/SimpleController>

The basic part of this class is defined as follows: ::

 class SimpleController
 {
 public:
     virtual bool initialize(SimpleControllerIO* io);
     virtual bool control() = 0;
 };

Processing details of the controller are implemented by overriding the follwoing virtual functions in a inherited class:

* **virtual bool initialize(SimpleControllerIO\* io)**

 Initialize the controller. The io parameter provides the objects and information used for control.

* **virtual bool control()**

 Perform input, control and output processes of the control. This function will be executed repeatedly as a control loop under control.

Once you define a class inheriting the SimpleController class, you need to define its factory function. You can do it using a macro as follows: ::

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

With this factory function, the shared (dynamic link) library file compiled from this source becomes available from a simple controller item.

SimpleControllerIO Object
-------------------------

A SimpleControllerIO object, which is passed as a parameter to the above 'initialize' function, handles the information used for input/output (I/O) between the controller and the robot. The following functions are available wtih this object:

* **Body\* body()**

 It returns a Body object to be used for I/O.

* **void setJointInput(int stateTypes)**

 It specifies the types of joint state values that are input to the controller.

* **void setJointOutput(int stateTypes)**

 It specifies the types of joint state values that are output from the controller.
 
* **double timeStep() const**

 It returns the time step of the control. The above control function is called repeatedly under control with this time interval.

* **std::ostream\& os() const**

 It returns an output stream to output a text. By outputting to this stream, a text message can be displayed on the message view of Choreonoid.


This object is called 'io object' in the following sections.


I/O using a Body Object
-----------------------

The simple controller inputs and outputs via a "Body item" returned by ioBody(). A Body object is an internal expression of Choreonoid of :doc:`../handling-models/bodymodel`, and an instance of "Body class" defined in C++. Since a Body class has data structure storing the status of the body model, elements like joint angle, torque and sensor status subject to output can of course be stored. The simple controller inputs and outputs via this Body class object. The Body object for this purpose is obtained by the io objet's body function.

.. note:: A Body class has various information and functions related to the body model, so it is an over-qualified class for I/O only. This type of class is not usually used for an I/O interface. Generally, a data structure optimised for exchanging only I/O elements is used. So, please be reminded of this point when you apply the description of this section to other controller formats. For example, RT component of OpenRTM normally uses "data port" interface for I/O by data type.

The elements of the robot state handled as I/O data are specified by using the setJointInput and setJointOutput functions of the io object. These functions specify input data types and output data types, respectively. The following symbols are used for specifying the data types:

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - Symbol
   - Data
 * - JOINT_ANGLE
   - Joint angle values
 * - JOINT_DISPLACEMENT
   - joint dispacment values
 * - JOINT_VELOCITY
   - joint velocity values
 * - JOINT_ACCELERATION
   - joint acceleration values
 * - JOINT_TORQUE
   - joint torque values
 * - JOINT_FORCE
   - joint force values

.. note:: What JOINT_ANGLE specifies in the simulator is same as that of JOINT_DISPLACEMENT. Theare may be revolute joints and prismatic joints, and the two symbols are defined to be able to express both the joint types. JOINT_TORQUE and JOINT_FORCE are defined for the same reason.

When you specify more than one element type, enumerates the corresponding symbols with the bit operator '|'. For example, ::

 JOINT_ANGLE | JOINT_VELOCITY

specifies both the joint angles and joint velocities.
	  
The above elements are actually contained in an object of the 'Link' class, which models a link of a robot. A link object of each joint can be retrieved from the Body object using the following member function:

* **int numJoints() const**

 It returns the number of the joints owned by the Body object.

* **Link\* joint(int id)**

 It returns the Link object that corresponds to the joint index.

For the Link object retrieved, it is possible to access to the joint state values using the following member function.

* **double& q()**

 It returns the reference to the joint angle (displacement) value. The value corresponds to JOINT_ANGLE or JOINT_DISPLACEMENT. The unit is [rad] or [m].

* **double& dq()**

 It returns the reference to the joint velocity value. The value corresponds to JOINT_VELOCITY. The unit is [rad/s] or [m/s].

* **double& ddq()**

 It returns the reference to the joint acceleration value. The value corresponds to JOINT_ACCELERATION. The unit is [rad/s^2] or [m/s^2].

* **double& u()**

 It returns the reference to the joint torque value. The value corresponds to JOINT_TORQUE or JOINT_FORCE. The unit is [N･m] or [N].

As each of these member functions returns the reference to its corresponding variable, you can substitute another value. Outputing values from the controller is done in that way.

The element types that can actually be used for I/O depend on the type and configuration of a simulator item. Most simulator items support the input of joint angle (displacement) values and the output of joint torque (force) values, and they make it possible to perform basic PD control. In that case, for each Link object of the joints, the joint angle (displacement) value is input by reading the q() value, and the value is then used in the calculation of the PD control, and the resulting joint torque (force) value is set to u() to output it to the robot.

In fact, any element types can be input to a controller in most simulator items. All the element values are contained in the internal physics calculation process, and the simulator can pass the values to the controller as its input values. However, it is not applied to a real robot. In a real robot, inputting a joint displacement requires an encoder at the joint, and inputting a joint torque requires a torque sensor at as well. Joint velocity values and joint accleration values are usually obtained by differentiating the joint displacement values.

With regard to output, the element types other than the joint torque (force) can only be output in limited situations. An example of such situations is the 'High-gain dynamics' mode of the AIST simulator. When you set it to the 'Dynamics mode' property of an AIST simulator item, it accepts joint displacement, joint velocity, and joint acceleration values as the output values from the controller. In this case, the motion of the robot is calculated so that the given joint posture can be achieved. Note that, however, this function would not be available for the real robot.

Initialization Process
----------------------

The initialization of a controller is done by overriding the 'initialize' function in a SimpleController inheriting class.

In the sample, the Body object used for I/O is obtained with: ::

 ioBody = io->body();

. This object will be accessed repeatedly, so it is stored in ioBody variable for efficiency and descriptive simplification for use.

Similarly, the time step value is stored in dt variable with: ::

 dt = io->timeStep();

for control calculation.

Next, ::

 io->setJointInput(JOINT_ANGLE);
 io->setJointOutput(JOINT_TORQUE);

specifies the element types used for I/O. Here the joint angles are specified as the input values and the joint torques are specified as the output values. You only have to do this configuration once in the initialization. ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     qref.push_back(ioBody->joint(i)->q());
 }
 qold = qref;

This sets the robot's joint angles when initialized (when the simulation is started) to a variable called qref where the target joint angles are stored. qold is a variable in which the joint angles one step before are stored and this will also be used for control calculation. qold is initialized to the identical value to qref.

Here, the statement ::

 ioBody->joint(i)->q()

inputs the joint angle of the i-th angle.

By returning true in the end, it informs the simulator of the successful initialization.

Control Loop
------------

A control loop is implemented in the 'control' function of the inheriting class.

In the sample, control calculation is performed with: ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     ...
 }

for all the joints. The content of this is the processing code.

First, with: ::

 Link* joint = ioBody->joint(i);

the Link object corresponding to the i-th joint is obtained.

Next, input the current joint angle: ::

 double q = joint->q();

Calculate the order value of the joint torque by PD control.  First, calculate the joint angular velocity from the difference from the previous joint angle in the control loop. ::

 double dq = (q - qold[i]) / dt;

Since the purpose of the control is to maintain the initial posture, calculate the torque order value with the joint angle being the initial joint angle and the angular velocity being 0 (state of rest) as a goal. ::

 double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];

The gain values are obtained from the pgain and dgain arrays defined in the beginning of the source code. The gain values require tuning for each model, but how to tune them is omitted here.

Save the joint angle in qold variable for next calculation. ::

 qold[i] = q;

Output the calculated torque value. By this, the joint angle can be controlled so that the initial joint angle can be maintained. ::

 joint->u() = u;

When the above process is applied to all the joints, the total posture of the robot can be maintained.

Finally, the control function informs the simulator of the continuation of the control by returning true. As a result, the control function is called repeatedly.

Devices
-------

In the above example, the joint angle was input and the joint torque was output. In other words, the I/O is made to the devices like an encoder and en actuator that are equipped in the joint.

There are many other different devices as the target of I/O. For example, the followings are the target of inputs as a sensor like an encoder:

* Force sensor, acceleration sensor and angular velocity sensor (rate gyro)
* Camera and laser range finder
* Microphone

and other devices.

The followings are the target of outputs and work to the external world as an actuator:

* Speaker
* Display
* Light

and other devices.

In the actual controller development, it is necessary to input/output for these diversified devices. To do so,

* it is necessary to understand how the devices are defined in the model, and
* how to access the specified devices in the controller format to be used

.. _simulation-device-object:

.

Device Objects
--------------

In a Body model of Choreonoid, the device information is represented as "Device" objects. It is an instance of the class that inherits the Device class and a different type is defined for each different device type. The device types defined as standard are as follows: ::

 + Device
   + ForceSensor 
   + RateGyroSensor  (angular velocity sensor)
   + AccelerationSensor 
   + Camera 
     + RangeCamera (camera + distance image sensor)
   + RangeSensor 
   + Light
     + PointLight 
     + SpotLight 

The information on the devices installed in a robot is usually described in a model file. For a model file in OpenHRP format, :ref:`oepnrhp_modelfile_sensors` in :doc:`../handling-models/modelfile/modelfile-openhrp` is used to describe the devices.

In a simple controller, similarly to a Body object, Device objects, which are internal expressions of Choreonoid, are used as they are to the devices for input and output. A Device object can be retrieved from a Body object using the following function:

* **int numDevices() const**

 It returns the number of the devices.

* **Device\* device(int i) const**

 It returns the i-th device. The order of the devices are the order described in the model file.

* **const DeviceList<>& devices() const**

 It returns the list of the devices.

* **template<class DeviceType> DeviceList<DeviceType> devices() const**

 It returns the list of the devices of the type specified.

* **template<class DeviceType> DeviceType\* findDevice(const std::string& name) const**

 It returns any device having the type and the name specified.

Use a template class DeviceList to get the devices of a specific type. DeviceList is an array that stores the device objects of the type specified and it allows extracting only the corresponding type using its constructor, the extraction operator (<<), etc. from DeviceList having other types. For example, if you want to retrieve the force sensor owned by the Body object ioBody, type: ::

 DeviceList<ForceSensor> forceSensors(ioBody->devices());

or add it to the existing list as follows: ::

 forceSensors << ioBody->devices();

DeviceList has functions and operators similar to std::vector. For example, with the following: ::

 for(size_t i=0; i < forceSensors.size(); ++i){
     ForceSensor* forceSensor = forceSensor[i];
     ...
 }

you can access to each device objects.

By using the 'findDevice' function, you can identify a device with its type and name and get it. For example, SR1 model has an acceleration sensor called "WaistAccelSensor" mounted in the waist link. You can type as follows ::

 AccelerationSensor* waistAccelSensor =
     ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");

to the Body object, then you can get it.

The devices that SR1 model has are as follows:

.. tabularcolumns:: |p{3.0cm}|p{3.0cm}|p{6.0}|

.. list-table::
 :widths: 25,25,50
 :header-rows: 1

 * - Name
   - Type of device
   - Description
 * - WaistAccelSensor
   - AccelerationSensor
   - Acceleration sensor mounted in the waist link
 * - WaistGyro
   - RateGyroSensor
   - Gyro mounted in the waist link
 * - LeftCamera
   - RangeCamera
   - Distance image sensor corresponding to the left eye
 * - RightCamera
   - RangeCamera
   - Distance image sensor corresponding to the right eye
 * - LeftAnkleForceSensor
   - ForceSensor
   - Force sensor mounted in the left ankle
 * - RightAnkleForceSensor
   - ForceSensor
   - Force sensor mounted in the right ankle


I/O for Devices
---------------

The I/O for a Device object is performed in the following way:

* **Input**

 Obtain the value using the member function of the corresponding Device object.

* **Output**

 Update the value using the member function of the corresponding Device object and run "notifyStateChange()" function of the Device object.

To do so, you must know the class definition of the device to be used. For example, for "AccelerationSensor", which is the class of an acceleration sensor, there is a member function "dv" to access to its state. This function returns three-dimension vector in Vector3 type.

Thus, the acceleration of the acceleration sensor waistAccelSensor can be obtained as follows: ::

 Vector3 dv = waistAccelSensor->dv();

Similarly, it is possible to input the state using the relevant member function for ForceSensor and RateGyroSensor, too.

Use of visual sensors like a camera or a range sensor requires some preparation. This will be described in :doc:`vision-simulation`.

For output to a device, see the sample "TankJoystickLight.cnoid", which turns on and off the light.

I/O of Link Positions
---------------------

In addition to the I/O elements described above, link positions can be the I/O element. Here, 'link position' is not the joint angle of the corresponding link, but the position and orientation of the link as a rigid body in the global coordinate. It is usually impossible to input/output such a value for a real robot. If the robot is not fixed in the world, it is difficult to know the accurate position of a link unless a motion capture with very good accuracy is available, and it is physically impossible to direcly move the a real link so that its position is identical to the output from the controller without controlling the joints. However, those are possible in simulation, and the function to input/output link positions is provided for the use limited in the simulation.

To use this function, specify the I/O for a link using the following functions of the io object:

* **void setLinkInput(Link\* link, int stateTypes)**

 It specifies the types of the link states that are handled as the input to the controller.

* **void setLinktOutput(Link\* link, int stateTypes)**

 It specifies the types of the link state that are handled as the output from the controller.

In the above functions, a target link object is specified to the 'link' parameter. For the stateType parameter, only the **LINK_POSITION** symbol that corresponds to the link position can be specified currently.

In a Link object, its position is stored as a Position type value. This is a custom type of the 'Transform' type defined in the Vector/Matrix library 'Eigen', which is used in the Choreonoid implementation. The value basically stores the elements of a three-dimension homogeneous coordinate transformation matrix. You can access to this value using the following functions of the Link class:

* **Position& T(), Position& position()**

* **Position::TranslationPart translation()**

* **void setTranslation(const Eigen::MatrixBase<Derived>& p)**
   
* **Position::LinearPart rotation()**

* **setRotation(const Eigen::MatrixBase<Derived>& R)**

* **setRotation(const Eigen::AngleAxis<T>& a)**

