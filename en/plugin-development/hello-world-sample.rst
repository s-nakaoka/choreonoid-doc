
Description of HelloWorld sample
=====================================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>


In this document, we describe the implementation of "HelloWorldPlugin", which is one of the sample plug-ins.

.. contents::
   :local:


Source Codes
---------------

.. highlight:: cpp

The sample codes of this sample are as follows: ::

 #include <cnoid/Plugin>
 #include <cnoid/MenuManager>
 #include <cnoid/MessageView>
 #include <boost/bind.hpp>
 
 using namespace cnoid;
 using namespace boost;
 
 class HelloWorldPlugin : public Plugin
 {
 public:
     
     HelloWorldPlugin() : Plugin("HelloWorld")
     {
 
     }
     
     virtual bool initialize()
     {
         Action* menuItem = menuManager().setPath("/View").addItem("Hello World");
         menuItem->sigTriggered().connect(bind(&HelloWorldPlugin::onHelloWorldTriggered, this));
         return true;
     }
 
 private:
     
     void onHelloWorldTriggered()
     {
         MessageView::mainInstance()->putln("Hello World !");
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
 

These codes are included as "HelloWorldPlugin.cpp" file under "sample\HelloWorldPlugin". Please use them for your testing. (Note that these codes may be somewhat different from those stored in the package due to the intention of the description or the difference of the version.)

When you compile and install this sample, the item "Hello World" will be added to "View" under Main Menu. Select this menu, then "Hello World" will be output to the message view. The plug-in performs only this simple action, but the subsequent paragraphs provide the basics how to develop a plug-in for Choreonoid.


Including of Header
----------------------------

First, the general description of the headers included by this sample is provided. ::

 #include <cnoid/Plugin>
 #include <cnoid/MenuManager>
 #include <cnoid/MessageView>

These three headers are the headers that Choreonoid framework provides. They are basically stored in "cnoid" sub-directory, which is an include path and you can include it by adding this directory as a prefix as shown above. As there is no extension to the header file for including, write the name of the target header in a format without an extension as shown above. A format without an extension is also employed for the header file of the standard C++ library as well as in C++ libraries such as Eigen, Qt and OpenSceneGraph, which Choreonoid uses as its bases. So, this is format is one of the standard formats for C++. Choreonoid also supports this format to improve the description uniformity with these libraries. (Note that this format seems to be called as a "header" instead of a "header file". However, many formats with an extension are still in use. It is difficult to use properly the terms.)

The three headers that are included here have the following functions:

* cnoid/Plugin

 It is a header where Plugin class is defined. When you create a plug-in, define a plug-in by including this header and create a class that inherits Plugin class.

* cnoid/MenuManager

 It is a header where MenuManger class, which manages the menu, is defined. When add an item to the menu, include this header to enable MenuManager

* cnoid/MessageView

 It is a header where "MessageView", which outputs a text message, is defined. When you want to have a text output to the message view, include this header.

The actual state of these headers are found under src/Base on the source tree. Please directly refer to these header files for details of the class definition. (Note, as for the actual state of the header file, an extension .h is suffixed.) Note that, by using a tool called Doxygen, you can generate a reference manual that lists the details of the class definitions, but, for the time being, addition of comments for generating a description text does not function completely. We will continue improving this issue. ::

 #include <boost/bind.hpp>

The operation includes the library header "Bind" from Boost C++ library collection. Bind is a library that generates flexibly a function object, which is frequently used in Choreonoid to call a function to process an event in the mechanism called "Signal". This will be explained in detail later.

It is desirable to learn the overview of some other libraries also provided by Boost, as they are used in Choreonoid. In concrete terms, the libraries such as Smart Ptr, Signals, Function, Format, Dynamic Bitset and Multi-Array are involved in developing plug-ins in addition to Bind. See  `Boost official documents <http://www.boost.org/doc/libs/>`_  for details of these libraries. In addition, Mr Inaba's `Let’s Boost page <http://www.kmonos.net/alang/boost/>`_ and  `"Boost C+; Libraries Programmin" <http://www.kmonos.net/pub/BoostBook/>`_ are usable for reference, too.

Note that the format of the boost header file has .hpp extension. (It is difficult to unify such descriptions in C++.)


using Instruction in Namespace
----------------------------------

In the following code, it is instructed to omit the description of the namespace "cnoid" and "boost" respectively. ::

 using namespace cnoid;
 using namespace boost;

cnoid is a namespace of Choreonoid where basically all the classes and functions that are provided by Chorenoid are defined. For example, the Plugin class should be described as cnoid::Plugin if the namespace is included, but by prescribing as above it is possible to describe simply Plugin by omitting the namespace part.

However, as the purpose of use of a namespace is to avoid a collision of names, it is not desirable to execute using instructions excessively. In principle, use of using instructions should be avoided in a header file but all the entire description should be provided including the namespaces. For an implementation file (.cpp) on the other hand, if a name collision does not matter, you can simplify the code by making the description as above.

In this example, a using instruction is made also for the namespace "boost" of Boost library. As Boost library is frequently used for plug-in development for Choreonoid, it may be somewhat easier by omission of the boost namespaces. However, in turn, coost includes a lot of functions and classes, which may result in name collisions and confusions. Therefore, you should properly use the description method of namespaces depending on the situation. A syntax called "using declaration" is also available to realise omissions for a certain specific class.


Definition of Plugin Class
------------------------------

Next, the class corresponding to HelloWorld plug-in is defined. ::

 class HelloWorldPlugin : public Plugin
 { 
     ...
 };


In this way, a plug-in of Choreonoid shall be defined as a class that inherits cnoid::Plugin (cnoid:: is omitted in this example). You can name the inherited class freely, but the name should end with "Plugin" for convenience. Please be careful lest the name of an existing plug-in should collide with the new name.

As a minimum set of the functions to be defined for a class of plug-in, we have:

* Constructor
* initialize Function

Below, how to describe these functions is described:

Constructor
-------------- 

The description of a constructor is as follows: ::

 HelloWorldPlugin() : Plugin("HelloWorld")
 {
 
 }

For a plug-in class constructor, it is necessary to call it by adding the name of the plug-in to the constructor of Plugin class that forms the base. Normally, a name with the ending "Plugin" part truncated from the class name.

In this same, the content of the constructor is not specifically described, but when a plug-in requires another plug-in, the dependency must be informed to the system by using "require" function. See the description of Sample1Plugin for this.


initialize Function
----------------------

Initialisation of a plug-in is described with initialize function as follows: ::

 virtual bool initialize()
 {
     ...
 }

The initialize function is a virtual function that is defined in the Plugin class forming the base and its actual behaviour is implemented by overwriting it. As virtual functions, we have functions like finalize and description.

Initialize functions are called in the order considering the dependency of the plug-ins after those plug-ins are loaded in the memory. Then, create the necessary objects and make them return true if the initialisation is successful. Make them return false if the initialisation is not successful. By doing so, the system can determine if the plug-ins are successfully initialised.

Addition of Menu
--------------------

Next, let's examine the description in an initialize function. ::

 Action* menuItem = menuManager().setPath("/View").addItem("Hello World");

Here, a menu is added. menuManager() is a member function of Plugin class (more precisey it is a function defined in Extensionmanager class, which forms the base of Plugin class), which returns a MenuManger object that manages the main menu.

By executing setPath("/View") to this object, the location of the current management target is set to the sub-menu called "View" of the route menu. In this way, the menu hierarchy can be expressed by delimiting with a slash just like the case with a file path, which is used as the menu path.

As setPath() is configured to return its own MenuManager object after the path is configured, an item called "Hello World" is added to the sub-menu "View" by calling addItem ("Hello World) to this.

addItem returns the added menu item as (the pointer to) Action object. This example describes that this object is stored once in a variable called menuItem and that the operation to this object is executed in the next line.

Note that the sub-menu "View" is actually translated into Japanese as "表示" menu in case the application is operated in the Japanese environment. It is because of the multi-lingual function, but the menu path in the source code must be described in the original English string. As for the original description, you can refer to the sources like Base/MainWindow.cpp or configure C, for example, to LANG environment variable and start Choreonoid in the English environment. How to use the multi-lingual function in detail will be described in a separate document.


Association with Functions of Menu
-------------------------------------

In the following code, the functions that are called when the user selects the added menu are configured: ::

 menuItem->sigTriggered().connect(bind(&HelloWorldPlugin::onHelloWorldTriggered, this));

This description calls a member function called "onHelloWorldTriggered" of HelloWorldPlugin class when the menu is selected. The meaning of this code is described in detail below.

First, the signal called "sigTriggered" owned by Action class is retrieved by menuItem -> sigTriggered(). A signal is an object that informs any event that occurs, which is realised by using Signals library of Boost in Choreonoid. Each signal is defined to each specific event. "signTriggered" is a signal that informs the event that "the user has selected the menu".

It is possible for a signal to set a function that is called when an event occurs using a member function called "connect". The argument for a connect function can be anything if it "can be regarded as a function object" which is "the same type as or a convertible type to" "the function type defined for the signal". This definition may not be clear enough, so, first, let's examine "src/Base/Action.h", in which Actions class is defined, to learn "the function type defined for sigTriggered". You can see that the function that retrieves sigTriggered is defined as follows. ::

 SignalProxy< boost::signal<void(void)> > sigTriggered()

. The internal description "void(void)" in the type of the return value means the signal "sigTriggered" is defined so that it should be associated with the function of the type: ::

 void function(void)

Therefore,

if the function you want to associate with is defined as an ordinary function as below: ::

  void onHelloWorldTriggered(void)
 {
     ...
 }

by adding this function, it can be described as: ::

 menuItem->sigTriggered().connect(onHelloWorldTriggered);

. It is almost the same as the use of a call-back function in C language.

You can describe as above in this sample, but you may well associate a member function of the class instead of a ordinary function in a practical plug-in development. In this example, therefore, a member function of the class is specifically associated.

However, a (non-static) member function has a hidden argument (?) of "this" in fact, which argument identifies this instance. Therefore, even if you try to pass a member function to a connect function like an ordinary function, it will not be successful as the instance when calling the member function is called. (Naturally, it results in a compile error.)

In such a case, Bind library of Boost is useful. "bind" function provided by Bind library creates a function object added with an argument properly arranged from an existing function. This explanation may be difficult to understand, but you can take it that "a member function is used for the purpose to convert a member function to an ordinary function". The statement for this is the part: ::

 bind(&HelloWorldPlugin::onHelloWorldTriggered, this)

.

First, the original function is given to the first argument of bind. In this example, the member function "onHelloWorldTriggerd" of a member function of HelloWorldPlugin class as "&HelloWorldPlugin::onHelloWorldTriggered". Note that it is necessary to state it expressly as a pointer-type by adding "&". In short, it is OK to describe the statement in the form of "&ClassName::MemberFunctionName".

"this" is provided as the the second argument of bind. This argument specifies the instance when this member function is called. As shown in this example, "this" is provided as an argument to associate the member functions of the same class with the same instance. This type of operation is often used in the real world. However, it is also possible to associate with a member function of another class, in which case, instead of "this", the proper instance of that class should be given as an argument.

Now, we have completed that configuration that "onHelloWorldTriggered is called when the user seelcts the menu".

While what we have done is very simple, the mechanisms that support it such as signal and bind may look a bit complicated. Nevertheless, what we have explained so far is nothing but the beginning part and you need to learn more in detail to manage the framework of Choreonoid. Additional explanations about these mechanisms will be provided in the description of other samples, but they are outside the scope of this guide. So, users are advised to read the documents regarding Boost, Signals and Bind libraries. It is strongly recommended to find and read web pages and books related to Boost, introduced in the above paragraph. It is enough to understand the overview. The actual ways to use are almost patterned and not so difficult.

Note that the variable "menuItem" is defined in this sample to describe "addition of menu" and "association of functions" separately. If, however, this is not required, you may well state together as follows: ::

 menuManager().setPath("/View").addItem("Hello World")->
     sigTriggered().connect(bind(&HelloWorldPlugin::onHelloWorldTriggered, this));


Supplementary: Qt signals and Chorenoid signals
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Action class (cnoid::Action class), which is used in this sample, is expanded by inheriting "QAction" class of Qt library and newly defined in Base module of Chorenoid (under src/Base). The purpose of the expansion is to improve the usability of QAction in Choreonoid and the main content of the expansion is additions of signal retrieving functions such as sigTriggered(). Also, some similar classes that are expanded from the frequently-used Qt classes are defined and "Q", which is the prefix of Qt, is removed from their class names (as they are defined in the namespace of cnoid, the precise name is "the name with cnoid::Q part omitted.)

Needless to say to those familiar with Qt, Qt is equipped with a unique signal system called "signal/slot" and QAction has "triggered" signal based on this system. With the use of this signal system, you can perform the same operation as explained above. Originally, Boost.Signals were developed arising from signal/slot of Qt. The expanded Action class captures the original Qt signal and re-processes as Boost.Signal-based signal, which is not so smart an operation.

Then, why do we have to make it possible a Boost.Signals-based description by specifically expanding the class instead of directly using the signal system of Qt? The answer is to improve the integrity with the part that is not dependent on Qt and anyway the descriptions will be more flexible and simple.

First, Choreonoid includes non-GUI modules that are not dependent on Qt. They are, for example, Util modul under src/Util and Body module under src/Body. As it is assumed that they are used, for example, by the control software that operates of the internal PC of a robot independently of GUI of Choreonoid, it is desirable not to make it dependent on a large GUI library as much as possible. Also in the history of Choreonoid development, the GUI libraries used have been changing from wxWidgets, Gtk+(Gtkmm) to Qt and we don't know how Qt will change in the future, so it is better to keep the parts dependent on a specific GUI library as few as possible. This is the same in GUI-related modules. So, for any class that does not inherits Qt class, all the necessary signals are described in Boost.Signals base.

On the other hand, Qt signal system can be used for Qt class, but the way of description is different from that of Boost.Signals and it is necessary to make an additional description to the header file and make a pre-processing called MOC. Then, the total integrity of the description will be lost and the coding work will become complicated. That is not acceptable for Choreonoid developers so it was decided to prepare tactless class expansions on our own to persist in the integrity and the simplicity of descriptions.


Description of Behaviour when Menu is Clicked
----------------------------------------------------

The function "onHelloWorldTriggered()", which is called when the menu is selected is implemented as follows: ::

     void onHelloWorldTriggered()
     {
         MessageView::instance()->putln("Hello World !");
     }

The instance of MessageView is retrieved by the class (static) function "instance()" of MessageView class. It is similar to "instance()" function in a so-called singleton class.

MessageView has some functions for text output to the message view. One of them, "putIn" function function is used in this sample to output the message provided with a line feed.

MessageView also provides an osteam-type object using a function called cout(). Using this, you can output the text in iostream description in the same way as output to std:cout.

In this sample, we use MessageView, but in Choreonoid, many other views, tool bars and class-generated instances are available. When you use them, first, the basic operation is to include the class header you want to use and retrieve the instance with a function like mainInstance() and instance(). See the reference manual and the header files generated by Doxigen for the time being for what class provides what functions.


Definition of Plug-in Entry
-------------------------------

Finally, it is necessary to provide the following description for each plug-in class: ::

 CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)

This is a macro defined in cnoid/Plugin header to define the function for Choreonoid system to retrieve the plug-in instance from the plug-in DLL by giving the class name of the plug-in. Without this description, the DLL created would not be recognised as a plug-in. So, don't fail to provide this description.

For each plug-in, t is necessary to create one DLL with one plug-in implemented. Please be reminded that you cannot implement more than one plug-in in one DLL (you cannot describe more than one macro above-mentioned.)

That is all about the sources. Next, we explain how to compile these sources.


How to compile
------------------

The items required to compile and use a plug-in are as follows:

* The header files and the library files of the dependent libraries of Choreonoid (such as Boost, Eigen, Qt, OpenSenegraph, etc.) shall be available from the build tool.
* The header files and the library files that are provided by the main part of Choreonoid shall be available from the build tool.
* Build shall be performed in the build environment and options compatible with the environment where the dependent libraries and the binaries of the main part of Choreonoid were built (Basically, it should be OK if it is the same OS, architecture and compiler.)
* The binary of the plug-in shall be built as the common library or a dynamic library.
* The name of the binary shall be “libCnoidXXXPlugin.so” (XXX represents the plug-in name) in case of Linux and "CnoidXXXPlugin.dll" in case of Windows.
* The binary shall be stored in the plug-in folder of Choreonoid. The plug-in folder will be under lib/chorenoid-x.x/, which is the installation destination of Choreonoid (x.x represent the version number)

If compliant with the above items, it is up to the plug-in developer in what environment and how to compile a plug-in. This document provides two samples of compile as follows:

* Use of Choreonoid already installed
* Use of compile environment of the main part of Choreonoid

In case of Using Chreonoid Already Installed
------------------------------------------------

First, we explain the method of compiling using the main part of Choreonoid which was "made install" as an external library.

When you use this method, check ON "INSTALL_SDK" in CMake option when building Choreonoid. Then, not only the execution file but also the header file and the files necessary for the link of the library can be installed when you execute "make install". In this configuration, "make install", first.

Then it is up to you how to compile, but let's use the following Makefile for example: (This makefile is stored as "ManualMakefile" in HelloWorldPlugin folder.

.. code-block:: makefile

 CXXFLAGS += `pkg-config --cflags choreonoid`
 PLUGIN = libCnoidHelloWorldPlugin.so
 
 $(PLUGIN): HelloWorldPlugin.o
 	g++ -shared `pkg-config --libs choreonoid` -o $(PLUGIN) HelloWorldPlugin.o 
 
 install: $(PLUGIN)
 	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
 clean:
 	rm -f *.o *.so


When you execute 'make' using this Makefile, the binary of the plug-in is created. When you execute "make install", the binary is copied in the plugin directory of Choreonoid. Then, when you execute Choreonoid, the plug-in will be loaded.

Though we did nothing special, but the tool called  `"pkg-config" <http://www.freedesktop.org/wiki/Software/pkg-config>`_  is used for the configuration of include path and link path and the libraries to be linked. "pkg-config" is a standard tool that is used as standard in Unix OS. If the library is supported, text strings like an include path, a link path or the library to be linked can be obtained by calling with the proper option and the target library as we did with the above-mentioned Makefile. By passing the string as the option of the compiler, you can compile without minding the details of the configuration. See the manual of pkg-config for detail.

Note that, in case CMAKE_INSTALL_PREFIX" is modified from the default /usr/local by CMake, pkg-config cannot find as it is the Choreonoid configuration file. In this case, it is necessary to describe the subdirectory of the installation destination of Choreonoid"lib/pkgconfig" in full path in the environment variable "PKG_CONFIG_PATH".

In Windows as well, by installing pkg-config, it should be possible to compile using a Makefile in this way. In case of Windows, however, it is an ordinary way to create a project on IDE in Visual C++ and then compile. In that case, you should configure the include path, the library path and the libraries yourself in the project configuration dialogue of IDE and compile them.

In case of Using Compile Environment of Main part of Choreonoid
------------------------------------------------------------------------

In case Choreonoid is compiled from the source, the same environment can be used for plug-in development. There is nothing difficult. You can simply add a new plug-in source to the Choreonoid source and then compile the together.

If the main part of Choreonoid is successfully compiled, the dependent libraries, not to speak of the headers, etc. of the main part of Choreonoid, should be successfully configured. So, you do not have to take care of these issues when adding a plug-in. If you have good knowledge about CMake, which is a build system employed by Choreonoid, is used, you can describe the build setting easier than writing Makefile. In case of Window as well, A project file in Visual C++ is created as in the case of building the main part of Choreonoid. So, it is possible to build from IDE of Visual C++ without complicated settings.

In that way, this method is recommended for the user who compiles the main part of Choreonoid from the source.

Now, we proceed with the concrete content of work. First, the directory where the additional source is placed for plug-in is usually "ext" directory of the normal Choreonoid source. Therefore, please create a sub-directory for the additional plug-ins under this directory and store CMakeLists.txt where the source of the plug-in and the build setting are described.

.. note::  As for HelloWorldPlugin stored in Choreonoid source, it is stored in the directory "HelloWorldPlugin" under the directory for samples "sample".

.. highlight:: cmake

CMakeLists.txt for HelloWorldPlugin is as follows: ::


 option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)
 if(NOT BUILD_HELLO_WORLD_SAMPLE)
   return()
 endif()
  
 set(target CnoidHelloWorldPlugin)
 set(srcdir ${PROJECT_SOURCE_DIR}/share/sampleplugins/HelloWorldPlugin)
 add_library(${target} SHARED ${srcdir}/HelloWorldPlugin.cpp)
 target_link_libraries(${target} CnoidBase)
 apply_common_setting_for_plugin(${target})

First, with the description as follows: ::

 option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)
 if(NOT BUILD_HELLO_WORLD_SAMPLE)
   return()
 endif()

You can configure so that this sample cannot be compiled if you don't want to. It is set to OFF by default. This configuration can be switched by ccmake command, etc. It is easier to operate the additional plug-in by describing in this way.

Then let's see the subsequent part that will be executed in case BUILD_HELLO_WORLD_SAMPLE is set to ON. First, with the following description: ::

 set(target CnoidHelloWorldPlugin)

The long plug-in name is replaced with a variable 'target'. ::

 set(srcdir ${PROJECT_SOURCE_DIR}/share/sampleplugins/HelloWorldPlugin)
 add_library(${target} SHARED ${srcdir}/HelloWorldPlugin.cpp)

This is the setting to build the common library that supports the plug-in. The description is a bit complicated as the storage of the source is different, but basically it is OK to list the source files in add_library. ::

 target_link_libraries(${target} CnoidBase)

The dependent libraries to be linked are described. If they are libraries and plug-ins in Choreonoid, you have only to write the names in this way. "CnoidBase" is the library that collects GUI frameworks of Choreonoid. You have only to specify it if you only use the basic functions of the frameworks. Also, the libraries on which the dependent library depends will be automatically linked. Unless you use another new library, you have only to describe a library in Choreonoid. ::

 apply_common_setting_for_plugin(${target})

This is a function that provides the common build setting in a plug-in and defined in CMakeLists.txt in the top directory of Choreonoid source. If you write it, it will execute installation at the time of "make install".

See `CMake Manual  <http://www.cmake.org/cmake/help/help.html>`_  for detailed description of CMakeLists.txt Also, you can learn how to write approximately by reading this sample and CMakeLists.txt of other libraries, plug-ins and samples in Choreonoid.


