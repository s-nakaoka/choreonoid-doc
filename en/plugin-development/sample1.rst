
Description of Sample1Plugin
=====================================

In this document, we describe the implementation of "Sample1Plugin", which is one of the sample plug-ins. This document is intended for those who have read :doc:`hello-world-sample`  and provides some additional explanations. Therefore, those who have not read :doc:`hello-world-sample` yet are advised to read it first

.. contents:: 
   :local:


Source Codes
-----------------

.. highlight:: cpp

The sample codes of this sample are as follows: ::

 #include <cnoid/Plugin>
 #include <cnoid/ItemTreeView>
 #include <cnoid/BodyItem>
 #include <cnoid/ToolBar>
 #include <boost/bind.hpp>

 using namespace boost;
 using namespace cnoid;

 class Sample1Plugin : public Plugin
 {
 public:

     Sample1Plugin() : Plugin("Sample1")
     {
	 require("Body");
     }

     virtual bool initialize()
     {
	 ToolBar* bar = new ToolBar("Sample1");
	 bar->addButton("Increment")
	     ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, +0.04));
	 bar->addButton("Decrement")
	     ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, -0.04));
	 addToolBar(bar);

	 return true;
     }

     void onButtonClicked(double dq)
     {
	 ItemList<BodyItem> bodyItems = 
	     ItemTreeView::mainInstance()->selectedItems<BodyItem>();

	 for(size_t i=0; i < bodyItems.size(); ++i){
	     BodyPtr body = bodyItems[i]->body();
	     for(int j=0; j < body->numJoints(); ++j){
		 body->joint(j)->q += dq;
	     }
	     bodyItems[i]->notifyKinematicStateChange(true);
	 }
     }
 };

 CNOID_IMPLEMENT_PLUGIN_ENTRY(Sample1Plugin)


This sample is stored in the source package under "share/sampleplugins/Sample1Plugin" (Note that these codes may be somewhat different from those stored in the package due to the intention of the description or the difference of the version.)

Plug-in Overview
----------------------

The overview of the behaviour of this plug-in is described below:

Tool bar with two button as shown in the picture below when this plug-in is compiled and installed and Choreonoid is executed.

.. figure:: sample1-bar.png


By pressing these buttons, the pose of the robot model changes.

First, let's load any robot model and view it. For example, if you load "GR001Sample.cnoid", which is a sample project introduced in "Start-up Guide", the robot model of GR001 is displayed. You can just select "File" under Main Menu - "Load" - "OpenHRP model file" and load "GR001/yam;" of GR001 model file. (In this case, check on Item View after loading so that it can be displayed on the Scene View.

When you check the display of the robot on the Scene View, keep the robot item selected on the Scene View. Even when more than one robot model is loaded, you can specify the models subject to pose change if they are in selected status. If you want to move multiple models, you can select multiple models selected by clicking items with "Ctrl" key pressed. Note, If no model is selected on the contrary, the pose of the robot does not change.

Then let's press the button "Increment". Then, the pose of the robot changes a little. If you keep on pressing "Increment" button continuously, similar changes occur and the pose of the robot keeps on changing continuously. Next, press "Decrement" button. Then the pose of the robot returns gradually. By pressing it repeatedly, it goes back to the original pose, then the pose will keep on changing.

As for changes of the pose, pressing "Increment" button increments certain degrees of all the joints of the robot and "Decrement" button does the contrary.

Well, this is just a plug-in not so meaningful, but by implementing this plug-in, you can learn the basics in adding a tool bar, retrieving a selected item and a moving a robot model.

Notification of Dependent Plug-in
-----------------------------------------

This plug-in handles a robot model. In this case, it is necessary to describe: ::

 require("Body");

in the constructor of the plug-in class.

This description defines that this plug-in depends on "BodyPlugin", which is a plug-in attached to the main part of Choreonoid. In fact, the function related to the robot model is implemented as one of the plug-ins that operate on Choreonoid. In this way, the following plug-ins are implemented as a plug-in actually but as the standard function included in the package of the main part of Choreonoid:

* BodyPlugin: It is a plug-in that collects basic functions that handle this centring the robot moel (Body item).
* PoseSeqPlugin: It is a plug-in that collect the data structure and the editing function of the key pose.
* BalancerPlugin: It is a plug-in that provides the balance auto-correction function.
* GRobotPlugin: It is a plug-in that operates a small humanoid robot GR001.

As BodyPlugin function is required this time, require function is called as above. As for the name to be provided to 'require', it is the name provided to the constructor of the base class Plugin in the constructor of each plug-in and it is generally a name with the final "Plugin" part omitted from the class name of the plug-in.

By the way, there is dependency as follows for the above-mentioned plug-ins:

* BodyPlugin
 * PoseSeqPlugin:  Dependent on BodyPlugin
  * BalancerPlugin: Dependent on BodyPlugin and PoseSeqPlugin
 * GRobotPlugin: Dependent on BodyPlugin

Here, BlanacePlugin depends upon both BodyPlugin and PoseSeqPlugin, but PoseSeqPlugin is basically dependent on BodyPlugin. So, PoseSeqPlugin should be enough to require in this case.

The above listed is the plug-ins that Choreonoid is equipped with, but it is possible to develop a separate plug-in dependent on a plug-in newly developed by user. That is to say, there is no difference between a plug-in attached to the main part of Choreonoid and a user-developed one.


Creation of Tool Bar
-------------------------

This plug-in creates a unique tool bar having two buttons.

As the class that corresponds to the tool bar is Tool Bar class, include the header first. ::

 #include <cnoid/ToolBar>

Then create the instance of the tool bar. ::

 ToolBar* bar = new ToolBar("Sample1");

What is provided to the constructor of Tool Bar is the name of this tool bar, which can be used to identify when storing the status in the project file.

As Too Bar has "addButton" function that adds a button, ::

 bar->addButton("Increment")

By


Association of Functions Called at Clicking
-----------------------------------------------

"addButton" returns the added button as an object pointer of ToolButton class. In addition, the following description is further provided to configure the function that is called when the button is clicked:  ::

 bar->addButton("Increment")
     ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, +0.04));

"sigClicked" is one of the signals that ToolButton is equipped with and it calls the function connected when the button is clicked. In this sample, this signal is associated with "onButtonClicked" function and the process when the button n is pressed is described in this function. Association with a function with connect was described in HelloWorld sample. Here we are going to do something more complex.

First, the following part: ::

 bind(&Sample1Plugin::onButtonClicked, this, +0.04)

is added after "this", which specifies the instance when the member function is called, and provides the value "+0.04". By this, the function object that is returned by bind is a function: ::

 void Sample1Plugin::onButtonClicked(double dq)

that calls a member function with the instance being "this" and the argument "dq" being "+0.04". That is a function calling of "this->onButtonClicked(+0.04)". Now, all the values of the arguments are all decided for the original member function. So, the function object will be deemed the same type as: ::

 void function(void)

.

On the other hand, if we look at "src/Base/Button.h" where ToolButton class is defined, the function that obtains "sigClicked" is defined as follows: ::

 SignalProxy< boost::signal<void(bool)> > sigClicked()

and we can see that the type of the function that links "sigClicked" is ::

 void function(bool)

. The argument of bool type informs the toggle status ON/OFF if the button is a toggle button. However, as is is enough to know whether the button is pressed for the button used this time, this argument is not required. When no argument is required, it is also possible to ignore this and 'connect' with a function object having no argument. Consequently, we have successfully associated the function object generated by bind with sigClicked and, as a result, realized the configuration, "Upon clicking Increment button, this->onButtonClicked(+0.04) is called".

It may be a bit complicated to understand but the reason why we make this configuration is to have "Increment" and "Decrement" share the functions to be called. However, as we have to have different behaviours for them, the argument "dq" is prepared for this purpose. Also, it is possible, by using 'bind' in this way, to directly associate the functions common to signal with each other, the description becomes simple.

In addition, for "Decrement" button as well, the association of creation of the button with the function is realised while the parameter to pass to dq is changed to "-0.04": ::

 bar->addButton("Decrement")
     ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, -0.04));

With the above, this->onButtonClicked(-0.04) is called when Decrement button is clicked.


Supplementary: In case of using argument of function defined to signal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The signal "sigClicked" of ToolButton is defined to associate with the function: ::

 void function(bool)

. Though it is not used this time, a brief description is provided in case this bool argument should be used. First, if the function to be associated with is a normal function like: ::

 void onClicked(bool on)

and has the same argument, it is OK to provide the function: ::

 sigClicked()->connect(onClicked)

as it is to the sigClicked() of the ToolButton object. With the above, the toggle status is provided to the argument 'on' and the function onClicked is called when the button is clicked.

On the other hand, even if a similar function is used, the assistance of 'bind' is required in case it is defined as a member function. If the member function is defined as follows: ::

 void Sample1Plugin::onButtonClicked(bool on)

, it is necessary to describe as follows: ::

 sigClicked()->connect(bind(&Sample1Plugin::onButtonClicked, this, _1))

. The value "_1" added to the end of 'bind' is an object in Bind library that represents "to bring the first argument of the original function". It is desirable to master these descriptions as they are often used for plug-in development for Choreonoid. Anyway, it is not a difficult thing to master as you have only to put characters like "_1" or "_2" in the argument you want to use.


Registration of Tool Bar
----------------------------

When you have created Tool Bar, the following is defined for the instance "bar" of Tool Bar: ::

 addToolBar(bar);

. As addToolBar is a member function of Plugin class (a member function of the fundamental class ExtensionManager, more precisely) and it is necessary to register Tool Bar with this function after creation of Tool Bar.

In this sample, the instance of the original ToolBar class was created and then Tool Bar was structured externally with addButton. This method can be used for a simple tool bar but if the content of the tool bar becomes complicated, the normal method to take is to define a new class inheriting ToolBar class and implement the content of the tool bar in that class.


Description when Button is Pressed
-------------------------------------

The process when the button is pressed is described in the following member function: ::

 void onButtonClicked(double dq)

. The argument dq is the delta quantity of the joint angle and is configured when it is connected with the button signal sigClicked.

The processes in this function is explained below: 

Retrieval of BodyItem Selected
----------------------------------

First, with the following part: ::

 ItemList<BodyItem> bodyItems =
     ItemTreeView::mainInstance()->selectedItems<BodyItem>();

the Body item that is selected by the user in Item Tree View is retrieved.

To do it, the instance of Item Tree View is obtained first with ItemTreeView::mainInstance(). This operation is the same as the case where MessageView was retrieved in HelloWorld sample.

Then the list (layout) of the items selected can be obtained by calling the member function "selectedItems" of ItemTreeView. This function is a template function having an item-type parameter and is designed to return the items that comply with the specified type from all the selected items. In this case, by specifying BodyItem type with "<BodyItem>", only Body items are to be retrieved.

The list of the items is returned by a template class called ItemList. This is also designed to take the item type as the template parameter and has the layout that can store the items of that type. By specifying BodyItem type for this as well, the layout where the selected BodyItems are stored is retrieved.

Other functions are available in ItemTreeView, including "checkedItems", which is a function that "returns the list of the checked items", "isItemSelected", "to check whether an item is selected or not" and "sigSlectionChanged", which is a signal "issued when the user changed the selection of an item". So, you can retrieve the items to be processed flexibly by utilising them.

Now we have retrieved the target Body items, we proceed with processing each BodyItem individually. Since ItemList class is based on std::vector, similar descriptions to std:vector are supported. Using this: ::

 for(size_t i=0; i < bodyItems.size(); ++i) {

The loop that performs the processing for each BodyItem is described.


Retrieval of Body Object
-----------------------------

In the loop that performs the processing for each BodyItem, the pointer to "Body" object is obtained first with the following description: ::

 BodyPtr body = bodyItems[i]->body();

. BodyPtr is a smart pointer to a Body object. You can take it as "Body" for the time being as a more detailed explanation is provided later.

The pointer of BodyItem can be obtained with BodyItems[i], but BodyItem itself does not define directly the actual data structure and the processing function of the model, which are defined actually in "Body" class in Body library (under src/Body). BodyItem provides an additional description so that this class is wrapped and made available as an item of Choreonoid. A Body object owned by BodyItem can be retrieved by calling a "body" function in this way.

The reason for this scheme is to isolate the model's data structure and processing functions per se from GUI so that they can be used in generic manners in various programmes. Therefore, these parts independent of GUI are first defined as "Body library" under src/Body. On the other hand, "Body plug-ins" under "src/BodyPlugin" cover the GUI part such as wrapping of these classes as an item, tool bars and views, so the roles are defined to both respectively. In Choreonoid, in this way, there are "classes independent of GUI" in the first place and they are usually "wrapped as an item" for use. Naturally, it is no problem to implement each of the items directly unless it is necessary to make them independent of GUI.

Supplementary: Modular Structure of Main Part of Choreonoid
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We have explained isolation of Body library from Body plug-ins. There are some other similar parts in the main part of Choreonoid and the following modules are available in the basic part of the main part of Choreonoid:

* Modules defined independently of GUI:

 * Util library (src/Util): defines classes and functions that are used by different parts
 
 * Collision library (src/Collision): defines the collision detection process among polygon (triangle) models
 
 * Body library (src/Body): defines modelling of substances/joint substances and kinematics and dynamics-related processes thereof

 These modules can be used from an external programme not a plug-in of Choreonoid.

* GUI modules

 * Base module (src/Base): defines the base part of GUI of Choreonoid
 
 * Body plug-in (src/BodyPlugin): defines the model-related processes associated with Body library
 
 * Other plug-ins

The dependency of these modules is as illustrated in the figure below:

.. figure:: module-dependencies.png


Supplementary: Regarding Smart Pointer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

"BodyPtr" in the above code is a "smart pointer" that stores the Body class pointers. In brief, a smart pointer is "a pointer that you need not delete". So, you don't have to worry when it should be deleted or you are prevented from using a deleted pointer by mistake resulting in a crash.

Smart pointers used in Choreonoid are implemented by "Smart Pointers" library of Boost. This library provides several pointer types so that they can be properly used. The base among them is "shared_ptr" type. It is used in the form as follows if there is Hoge class for example: ::

 boost::shared_ptr<Hoge> hoge = new Hoge();

. By describing as above, you can access a member function or a variable as follows: ::

 hoge->function();

as in the original pointer type "Hoge*". If you use this type as in: ::

 boost::shared_ptr<Hoge> hoge2 = hoge;

then, you can copy it to a different variable.

When all the smart pointer variables that store this pointer are destructed, the pointer per se will be deleted automatically.

In case the original pointer is required: ::

 Hoge* p = hoge.get();

You can use get() function as above for conversion.

By the way, the description "boost::shared_ptr<Hoge>" may be too long. So, in Choreonoid, you can define as follows: ::

 typedef boost::shared_ptr<Hoge> HogePtr;

so that the smart pointer can be used according to the naming rule of "class name+Ptr".

This is all about the basics of share-ptr, but the most frequently used smart pointer type in Choreonoid is "intrusive_ptr". For Body class, too, BodyPtr is defined as a smart pointer based on this type. How to use is almost the same as shared_ptr. The difference from shared_ptr is that intrusive_ptr has the "reference counter" to decide whether or not to delete in the object internally. (shared_ptr reserves this area separately from the heap.) Due to this difference, intrusive_ptr is slightly faster

* in processing than shared_ptr, and
* a problem can less easily happens when it is converted mutually with the original pointer type.

Because of this advantage, this type is mainly used in Choreonoid.

The class that provides a smart pointer based on intrusive_ptr is defined as follows: ::

 class Body : public cnoid::Referenced

inheriting "cnoid::Referenced" class defined in Util library. Then, with by defining: ::

 typedef boost::intrusive_ptr<Body> BodyPtr;

you can start using this smart pointer as BodyPtr type.

Because "a problem can less easily happen if this smart pointer type is converted mutually with the original pointer type", as stated above, you can store one as a smart pointer only where it is required.

In fact, a function that returns an object of cnoid::Reference type is basically defined as: ::

 Body* functionToReturnBodyObject();

so that the original pointer type is returned. Either "BodyPtr" or "Body*" can be used as a variable that receives an object returned by this type of function. (In fact, it is no problem to use "Body*" instead in the codes of this sample.)

On the other hand, a function that takes an object of cnoid::Referenced type as a variable is basically defined as: ::

 void doSomething(BodyPtr body);

and described with the smart pointer type. For this description as well, you can use either "BodyPtr" or "Body*" for the variable that is provided when a function is called.

However, you can use the original pointer only when the object is stored in the smart pointer and the use by the original pointer is temporary. On the contrary, you had better store in the smart pointer an object that has to be owned for long time. If you are sure, you can use a smart pointer. (except some cases.)

Above-mentioned mutual conversion between a smart pointer and its original pointer cannot basically be used in case of shared_ptr. shared_ptr can be initialised with the original pointer, but you cannot substitute the original pointer there, and it is assumed that initialisation is executed immediately after the pointer is renewed. In case with intrusive_ptr, on the other hand, you can freely initialise by or substitute with the original pointer as long as the above-mentioned restriction is complied.

Choreonoid is designed based on intrusive_ptr for the purpose to ensure flexibility in pointer descriptions in this way. However, as you have to be careful of "the condition of using the original pointer", the benefit of using smart pointers that "you don't have to care whether an object exists or not by just using smart pointers" may be degraded. Normally, you don't have to mind to that extent, so you had better be familiar with the above descriptions.

Change of Joint Angle
-------------------------

Let's go back to the sample code. The following code changes the joint angle of the robot model stored in Body object: ::

 for(int j=0; j < body->numJoints(); ++j){
     body->joint(j)->q += dq;
 }

As Body class can get the number of the joints with "numJoints" function, it is used to rotate the loop so that the angle of all the joints are changed. What is obtained by "joint(j) function in the loop is the object of Link class that corresponds to the joint id j. This class stores the joint angle in the member variable "q" and its value is changed by the quantity "dq".

Note that Body library is forked from the one developed by `OpenHRP3 <http://fkanehiro.github.io/openhrp3-doc/en/index.html>`_  to start the development of Choreonoid and Body classes and Link classes used are currently almost the same as those in OpenHRP3. So, those have an experience of programming with OpenHRP3 library can leverage the knowledge about it and refer to the `Programming Manual <http://fkanehiro.github.io/openhrp3-doc/en/programming.html>`_  of OpenHRP3 to some extent, provided, however, that many parts have been modified, especially the matrix and vector library has been replaced with Eigen from tvmet. Please be careful of these differences.


Notification of Status Change
---------------------------------

What we have done in the above code is only to update the variables that store a joint angle. This operation falls short of reflecting the result to the entire model including the position and the posture of the links and of updating the display on the GUI. To do this, execute the following: ::

 bodyItems[i]->notifyKinematicStateChange(true);

.

"notifyKinematicStateChange" is a function defined in BodyItem class, which informs Choreonoid system that a kinematic state change has been executed to the model and reflects the change on the display on the GUI. Please note that it is not a function defined in Body class but in BodyItem class instead. In this way, it is the role of BodyItem class to implement the GUI-related part additionally.

notifyKinematicsStateChange function is declared as follows: ::

 void notifyKinematicStateChange(bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);

Here, true is provided in the first argument "requestFK".

As mentioned above, the position and the posture of the links do not change by just modifying the value of the joint angle variable q. A forward kinematic calculation is required for this purpose. You can declare the Body object as: ::

 body->calcForwardKinematics();

and, by giving true in requestFK, the forward kinematic calculation is done at the same time. If you want to update the speed or the acceleration, add true respectively. You can try and see what will happen when you have time if you execute notifyKinematicsState without specifying true to requestFK. You can see that, while the join angles displayed in "Joint Slider View" change, the pose of the robot displayed on "Scene View" does not change.

.. note:: The reason why notifyKinematicsStateChange executes calcForwardKinematics also is not simply to do without calcForwardKinematics. Choreonoid is designed considering that multiple objects function in co-operation and notifyKinematicStateChange is provided based on this principle. For example, an object handling the upper body and another handling the lower body exist and operate independently of the other in some cases. And both may function at the same timing. In such a case, if each of them change the joint angle, makes kinematic calculation and update the GUI individually, the process may be duplicated as a result. Rather than that, it is more effective to update the joint angle respectively first and then, when the processes of the both are completed, perform kinematic calculation and the GUI update. To realise this, notifyKinematicStateChange is described so that it does not perform kinematic calculation and the GUI update whenever it is called, but instead posts such operations as the events necessary and that, after all the updates that should be executed at the same timing are completed, it perform kinematic calculation and the GUI update only once.

Note that, when notifyKinematicStateChange is called, "signKinematicStateChanged" signal prepared by BodyItem class is issued eventually. Therefore, if you want any process to be executed when the kinematic state of the model has changed, you can describe it in the function connected to this signal. In fact, the view function of BodyItem in Joint Slider View and Scene View of Choreonoid is realised by connecting this signal. As a result, by just calling notifyKinematicStateChange, all the relevant views are updated.

The calling party need not be aware of any views at all and, even in case a new view function is added by a plug-in, it can function as the existing view functions by adding it simply. As you can see in this effort, Choreonoid aims to realise flexible function expansion. That is not something special but it is also referred to as Model-View-Controller (MVC) architecture or Document-View architecture, both of which are well-known software design techniques.


Compile
----------

As we have discussed how to compile a plug-in using HelloWorld sample, we deal with only some points that must be considered additionally in this sample.

What we have to consider additionally in this sample is that, as we mentioned in the paragraph regarding "Constructor", this plug-in is dependent on BodyPlugin. So, it is necessary to provide an additional description about the library files to be linked.

Regarding this necessity, it is OK if the link with the common library of a plug-in called "libCnoidXXXPlugin.so" stored under the Choreonoid installation destination "lib/choreonoid-x.x" is established in case of Linux. In case of BoduPlugin, it will be "libCnoidBodyPlugin.so".

In case of Windows, "import library file" called "CnoidXXXPlugin.lib" stored under the same directory is relevant. When you establish a link to this file, the DLL file called "CnoidXXXPlugin.dll" stored in the same directory is linked at execution. Be careful that you have to turn ON the option "INSTALL_SDK" in the CMake settings for the import library file (.lib). Otherwise it will not be installed.

In case you write a Makefile targeting the installed Choreonoid, specify the library name "choreonoid-body-plugin" to pkg-config, then the option fulfilling the above condition is returned.

In case of compile using CMake in the compile environment of the main part of Choreonoid, the system compiles so that this dependency is fulfilled if you write "CnoidBodyPlugin" in "target_link_libraries" command. (In this case, you need not describe CnoidBase explicitly as CnoidBodyPlugin is dependent on CnoidBase.)

For the actual Makefile and CMakeList.txt, please refer to those under "share/sampleplugins/Sample1Plugin" and "extplugin/sample/Sample1Plugin".


