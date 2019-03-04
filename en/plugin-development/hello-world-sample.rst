
Description of the HelloWorld sample
====================================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>


This document describes the implementation of **HelloWorldPlugin**, which is one of the sample plugins.

.. contents::
   :local:


Source Codes
------------

.. highlight:: cpp

The source code of this sample is as follows: ::

 #include <cnoid/Plugin>
 #include <cnoid/MenuManager>
 #include <cnoid/MessageView>
 
 using namespace cnoid;
 
 class HelloWorldPlugin : public Plugin
 {
 public:
 
     HelloWorldPlugin() : Plugin("HelloWorld")
     {
 
     }
 
     virtual bool initialize() override
     {
         Action* menuItem = menuManager().setPath("/View").addItem("Hello World");
         menuItem->sigTriggered().connect([&](){ onHelloWorldTriggered(); });
         return true;
     }
 
 private:
 
     void onHelloWorldTriggered()
     {
         MessageView::instance()->putln("Hello World !");
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
 

All the files for this sample, including this source code, are stored in the Choreonoid source archive in a directory named sample/tutorial/HelloWorldPlugin. The name of the source code file is HelloWorldPlugin.cpp.

When this sample is built, HelloWorldPlugin is created. When this is imported during the launch of Choreonoid, the item “Hello World” is added under “Display” in the main menu. And when you select this menu item, “Hello World!” is displayed in the message view. 

Although the plugin has only this function, it is useful for learning the basics of Choreonoid plugin development, as you’ll see in the description of this sample below.


Including headers
-----------------

First we will describe the headers that this sample is including. ::

 #include <cnoid/Plugin>
 #include <cnoid/MenuManager>
 #include <cnoid/MessageView>

These three headers are provided by the Choreonoid framework. They are basically stored in the subdirectory include/cnoid of the installation location. Usually, the path as far as include is set in the include path, and the headers are included with the prefix cnoid subdirectory. Also, since the header files for inclusion do not have an extension, write the names of the target headers in a format without an extension as described above.

.. note:: The format of header files without an extension is standard in C++ and adopted in standard C++ libraries. The C++ libraries which Choreonoid uses as its base, such as Eigen, Qt and OpenSceneGraph, also adopt this format. Choreonoid also employs this form with the aim of using a coding style consistent with those libraries. (By the way, this format appears to be called “header” rather than “header file”.)

The three headers included here have the following functions.

* cnoid/Plugin

 This header defines the Plugin class. When you make a plugin, this header must be included, and a plugin is defined as a class which inherits the Plugin class.

* cnoid/MenuManager

 This header defines the MenuManager class, which manages menus. When you need to add items to a menu, include this header so as to be able to use the MenuManager.

* cnoid/MessageView

 This header defines MessageView, which is the view where text messages are output. If you want to display text in the message view, include this header.

The actual implementation of these headers is in src/Base of the source tree. Refer directly to those files if you want to know the actual class definitions. (Note that filenames of the actual header files have the extension .h.) Although it is also possible to use the tool called Doxygen to create a reference manual that lists the details of the class definitions, currently there are insufficient comments to produce a good reference manual. We will continue to work on developing it.

Namespace using directive
-------------------------

The following directive allows the description of the namespace cnoid to be omitted. ::

 using namespace cnoid;

**cnoid** is the namespace of the Choreonoid framework, and basically all classes and functions provided by Choreonoid are defined in this namespace. For example, the **Plugin** class used in this sample needs to be described as cnoid::Plugin so as to include the namespace. But if you describe the using directive in advance, you can omit the namespace prefix cnoid:: and simply write Plugin.

However, it is not a good idea for the using directive to be used indiscriminately, because the purpose of the namespace is to avoid naming conflicts. In principle, you should avoid the using directive for header files, and all the namespaces should be explicitly specified in header files. On the other hand, you can use them as described above in implementation files (.cpp) to simplify their code, as long as it doesn’t lead to naming conflicts.

Definition of the plugin class
------------------------------

Next, we will define the class corresponding to the HelloWorld plugin: ::

 class HelloWorldPlugin : public Plugin
 { 
     ...
 };


A Choreonoid plugin is defined as a class which inherits the Plugin class in this way. You can name an inherited plugin class any way you like, but having Plugin at the end makes the name easily recognizable. It is also important to avoid causing naming conflicts with existing plugins.

At least the following functions must be defined in a plugin class:

* constructor
* initialize function

. Below, we will describe how to describe these functions.

Constructor
------------ 

The constructor is described as follows: ::

 HelloWorldPlugin() : Plugin("HelloWorld")
 {
 
 }

In the constructor of the plugin class, the constructor of the base Plugin class must be called as shown, and the name of the plugin given. Usually, it is given a name with the final Plugin string stripped from the class name.

Although the constructor in this sample has nothing in particular described within it, when the plugin is dependent on other plugins, you need to pass the dependency relationship to the system using the require function here.  Refer to the  :doc:`sample1` section for more about this.

The initialize function
-----------------------

The initialization of a plugin is described in the initialize function as follows: ::

 virtual bool initialize() override
 {
     ...
 }

The initialize function is a virtual function defined in the base Plugin class. Overriding it allows the behavior of each plugin to be implemented. Other virtual functions that assume this kind of override include finalize and description. In order to clarify what is being overridden here, we have added the override keyword introduced since C++11.

After the plugins have been imported, the initialize functions are called in order, taking into consideration the dependency relationship between plugins. When the necessary objects have been created and the initialization succeeds, this function should return true. If the initialization failed, return false. This allows the system to judge whether or not the plugin has been successfully initialized.

Adding menu items
-----------------

Next, let’s look at the code in the initialize function. ::

 Action* menuItem = menuManager().setPath("/View").addItem("Hello World");

Here, a menu item is being added. menuManager() is a member function defined in the Plugin class (to be exact, it is a function defined in the ExtensionManager class, which is the base class of the Plugin class), and it returns a MenuManager object, which manages the main menu.

Calling setPath(“/View”) for this object sets the current management position to View, which is a sub-menu of the root menu. In this way, MenuManager represents the menu hierarchy, separated by slashes like a filepath, and this is called the menu path.

The setPath() function returns its own MenuManager object after setting a path, so you can continue and call addItem(“Hello World”) to add the Hello World item to the View sub menu.

The addItem function returns the added menu item as (a pointer to) an Action object. Here, this returned object is stored as the variable menuItem to begin with, and an operation on the object is described in the next line.

When operating in a Japanese-language environment, the View sub-menu is actually translated and becomes the Display menu. This is done by an internationalization feature, and menu paths in the source code must be written using the original English character strings. You can find out the original descriptions by looking at source files such as Base/MainWindow.cpp. Alternatively, if you set the target language of the OS to English and execute Choreonoid, the menu items will all be the same as in the source code.

.. note:: In the case of Linux, setting the environment variable LANG to C, you will be able to change the target language to English.  If you are setting it from the command line, input: export LANG=C.


Connecting function to menu items
---------------------------------

The following code sets the function which is called when a user selects the added menu item. ::

 menuItem->sigTriggered().connect([&](){ onHelloWorldTriggered(); });

First, menuItem->sigTriggered() gets the “signal” called sigTriggered, which is defined in the Action class. A signal is an object which notifies of an event when it happens. In the Choreonoid framework, many classes have their own unique signals.

Each signal is defined for a specific event, and here sigTriggered is the signal corresponding to the event when a user selects from the menu.

A signal can set the functions which are called when the event happens by using the connect member function. The argument given to connect can be anything, as long as it “can be regarded as a function object” which is “the same type or can be converted to the same type” as the “type of function defined for the signal.” 

First, in order to understand the “function type defined for sigTriggered”, let's look at src/Base/Action.h where the Action class is defined. The function which returns sigTriggered is defined as follows: ::

 SignalProxy<void()> sigTriggered();

. The returned value is defined in the SignalProxy template, where void(void) is described as the template argument. This shows that the signal sigTriggered is defined to connect with the function type ::

 void function()

, or in other words, with the no-argument, no-return-value function

So, if for example, the argument to be combined is defined as the function ::

 void onHelloWorldTriggered()
 {
     ...
 }

this function will be given directly and can be described as follows:、 ::

 menuItem->sigTriggered().connect(onHelloWorldTriggered);

. This is almost the same as using a callback function in C.

We can describe it in this way for this sample, but when developing the actual plugin, there will be many cases where you want to connect not to the original functions but to class member functions. And so, the code in this sample is written so that the signal is connected to a class member function.

However, member functions usually have the implicitly added parameter this, corresponding to the pointer of the object itself, and they identify the object instance based on this parameter. Hence, even if you try to give a member function to the connect function in the same way as a normal function, the this parameter is not set and this results in a compile error.

In order for it to be to be compilable and executable, you must pass it to connect after converting the member function with the this parameter to a function without parameters. There are various ways to do this, but usually using the **lambda expression**, which was introduced from C++11. Here, this would result in the following description: ::

 [&](){ onHelloWorldTriggered(); }

This format, written using square brackets, round brackets, and curly brackets, is the lambda expression. Refer to a specification or explanation of C++11 for more details about the lambda expression. In this case, the this variable is captured using [&], and with no argument, it is described by calling the onHelloWorldTriggered member function. Written in a simpler format, this is as follows: ::

 [this](void){ this->onHelloWorldTriggered(); }

. In this description, we define a function that “executes this->onHelloWorldTriggered (), without an argument, using the this variable of the scope defining this lambda expression, and with no return value”. It may be a little confusing written like this, but essentially we are creating a function that calls the onHelloWorldTriggered member function defined in the HelloWorldPlugin class.

With the above coding, we have configured the setting so that “onHelloWorldTriggered is called when the user selects from the menu.”

In order to use the Choreonoid framework, you need to become familiar with the mechanism of connecting signals and functions in this way. Here, the signal had no parameters, but many signals are defined with parameters. That is introduced in the explanation of other plugins.

In this sample, we have defined the variable menuItem in order to explain “Adding menu items” and “Connecting functions” separately. But if that is unnecessary, it’s okay to write them together as follows: ::

 menuManager().setPath("/View").addItem("Hello World")->
     sigTriggered().connect([&](){ onHelloWorldTriggered(); });

.. note:: Choreonoid signals are based on the Boost.Signals library, but it is a proprietary implementation. Boost.Signals is now getting a bit old, but you may still find that document useful. However, while Boost.Signals used Boost's Bind library to construct the callback function, now that we have the lambda expression in C++11, it's better to use that.

Supplemental: Qt signals and Choreonoid signals
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Action class mentioned above is a class which extends the **QAction** class of the **Qt** GUI library used by Choreonoid by inheriting it, so it is defined in the “Base” module (in src/Base) of Choreonoid. (Actually, it is defined in the cnoid namespace.)

The purpose of the extension is to allow the use of QAction in the form of a Choreonoid signal, and this improves the usability (simplicity of coding) of the Qt object in Choreonoid. Many classes are defined which similarly extend commonly-used QT classes, and they are named by simply removing the “Q” prefix from the original name. (They are defined in the cnoid namespace, so the exact names are the “cnoid::Qt class names with the Q removed.”)

It goes without saying for anyone experienced in programming using Qt, but Qt has an original signal system called **Signal/Slot**, and QAction also has a signal named triggered based on this system. If you use it, you can do the same as described above. As a matter of fact, the details of the extension to the Action class also catches the original Qt signal and re-processes it as a Choreonoid signal type. In this slightly foolish way, which also generates overheads, a Qt signal is converted into a Choreonoid signal.

This conversion improves the consistency of signal processing in the code used in the Choreonoid framework. A Qt signal takes a quite original form, such as requiring a description beyond the scope of the C++ language specifications and additional processing by the moc tool during compiling in order to handle it. Choreonoid signals, on the other hand, can be handled within the usual C++ language specifications, and we believe that unifying them will improve the unity of the coding.

Coding the behavior when the menu is clicked
--------------------------------------------

Called functions are implemented as follows when a menu item is selected: ::

 void onHelloWorldTriggered()
 {
     MessageView::instance()->putln("Hello World !");
 }

MessageView is the class corresponding to Choreonoid’s message view. Message view is an object that exists only on Choreonoid, so it is defined as a so-called singleton class. We are getting a single instance of MessageView with the instance function, which is a singleton class idiom.

MessageView has several functions for text output, and here we use one of them, the putln function, to output the given message with line breaks.

MessageView also provides a function called cout() that returns an object of ostream class. By using this, you can output text in the iostream descriptive style, in the same way as outputting to std::cout.

In addition to MessageView used in this sample, other useful views and toolbars are available in Choreonoid, and they can be used from plugins in the same way as the message view. These are used by including the header of the view or toolbar you want to use and getting the instance with the instance function of the corresponding class. Look up the reference manual generated by Doxygen or the header files if you want to know what kinds of functions are provided by each class.


Defining the plugin entry
-------------------------

Finally, we will write the following code: ::

 CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)

This uses the CNOID_IMPLEMENT_PLUGIN_ENTRY macro defined in the cnoid/Plugin header. Giving the name of a plugin to this macro defines the function with which the Choreonoid system obtains the plugin instance from the DLL of the plugin. Please don’t forget to write this macro, because the plugin DLL cannot be recognized as a plugin without it.

Note that each plugin needs to be created in a DLL which implements only one plugin. Note that a single DLL cannot implement multiple plugins (the above macro cannot be written more than twice).

That completes the description of the source code. Next we describe how to build this plugin.

.. _hello-world-build:

How to build
------------

The following conditions must be met in order to build and use a plugin:

* The header files and library files of the Choreonoid dependency libraries, such as Qt, must be available to the build tools.
* The header files and library files provided by Choreonoid must be also available to the build tools.
* The build environment and options must be the same as those used for building the dependency libraries and the main Choreonoid binaries. (Basically using the same compiler on the same OS and PC architecture will meet this condition.)
* The binary for the plugin must be built as a shared (dynamic-link) library.
* The name of the binary must be libCnoidXXXPlugin.so (XXX corresponds to the plugin name) for Linux, or CnoidXXXPlugin.dll for Windows.
* The binary must be installed in the plugin directory of Choreonoid. The plugin folder is lib/choreonoid-x.x/ (x.x corresponds to the version number) in the Choreonoid installation location.

As long as the above conditions are met, any method can be used to compile a plugin. Here, we will describe the following three methods.

1. Build together with the main part of Choreonoid
2. Build the plugin separately using CMake
3. Build the plugin by writing the Makefile directly

.. note:: For methods #2 and #3, it is assumed that the build is done on Linux. When the build uses Visual Studio (Visual C++) on Windows, it’s simplest to use method #1. Otherwise, you need to create a Visual C++ project and set the include path, library path, library, etc. yourself in the settings dialog box.

.. _hello-world-build-together:

Build together with the main part of Choreonoid
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you are using Choreonoid built from source, you can build your own plugin together with Choreonoid itself. This means adding the source of your own plugin to the source of Choreonoid and building them together, with the plugin as an add-on.

Information for building the main part of Choreonoid is managed by CMake. In addition to the headers and libraries that make up Choreonoid, CMake also has information for building Choreonoid’s external dependency libraries. One point about this method is that this kind of information is also utilized for building your own plugin. In this case, since the description required for building your plugin is the minimum necessary, you can create a plugin easily. Therefore, this method is recommended for anyone using Choreonoid built from source.

The basic process is as follows. First, in the Choreonoid source directory there is a directory called ext, which is where additional plugins are placed. So, first create the sub-directory for the plugins in this ext directory, and then store the plugin source files and the CMakeLists.txt file for the build inside.

In the case of the HelloWorld plugin, the files will be structured as follows: ::

 + the Choreonoid source directory
   + ext
     - HelloWorldPlugin.cpp
     - CMakeLists.txt

.. highlight:: cmake

And the following will be written in CMakeLists.txt: ::

 set(target CnoidHelloWorldPlugin)
 add_cnoid_plugin(${target} SHARED HelloWorldPlugin.cpp)
 target_link_libraries(${target} CnoidBase)
 apply_common_setting_for_plugin(${target})

Here, the line ::

 set(target CnoidHelloWorldPlugin)

sets the plugin name. Name the plugin in this way, starting with Cnoid and ending with Plugin. Here we set this name to a variable called target so that it can be used with the commands below. It doesn’t necessarily have to be set as a variable, but doing so keeps the plugin name settings consistent. ::

 add_cnoid_plugin(${target} SHARED HelloWorldPlugin.cpp)

This description is needed for the actual build of the plugin. Add_cnoid_plugin is a command defined in the CMake file of the main part of Choreonoid, and you can build a plugin by specifying a plugin name and source file name here. This command is described in CMakeLists.txt in the top directory of the Choreonoid source, so refer to it for further details. Basically, it is the add_library command used when creating a library with CMake, customized for plugins and used in the same way as the add_library command. ::

 target_link_libraries(${target} CnoidBase)

This is a description for explicitly specifying the library on which the plugin depends, which in this case is the CnoidBase library included in the main part of Choreonoid. CnoidBase is a library which implements the base GUI of Choreonoid, and it also includes the message view implementation used in this sample. This library must be linked whenever we are dealing with a Choreonoid plugin. This description will link the CnoidBase library to the HelloWorld plugin.

In CMake, if you specify libraries defined in the same project with target_link_libraries, it will also link to all the other libraries that those libraries are dependent on. For example, since CnoidBase is also dependent on the Qt library, the Qt library will be linked to the HelloWorld plugin by the above description. In this way, you can use this method to complete the description without needing to worry too much about the details of all the libraries to be linked. ::

 apply_common_setting_for_plugin(${target})

This command configures settings that should be commonly applied to plugins. This function is defined in the CMakeLists.txt in the top directory. This description is usually also done for plugins. This will allow plugins to also be installed by make install, for example.

Refer to the  `CMake manual <http://www.cmake.org/cmake/help/help.html>`_ for details on how to write CMakeLists.txt. In addition, reading the CMakeLists.txt files in the libraries included in Choreonoid, or in other plugins and samples, should give you a rough idea of how to write them.

When you have written CMakeLists.txt in this way and placed it in the subdirectory in ext along with the plugin's source file, build the main part of Choreonoid. The CMakeLists.txt of this plugin will be detected automatically, and the HelloWorld plugin will be built along with Choreonoid.

.. note:: Note that this method performs the build on the main part of Choreonoid. Since the CMakeLists.txt mentioned above cannot build a plugin by itself, running the cmake command in the plugin's directory will not work. In that case, it may affect the CMake of Choreonoid, so please avoid such operations.

In the plugin’s CMakeLists.txt, it is a good idea to write the following at the beginning. ::

 option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)
 if(NOT BUILD_HELLO_WORLD_SAMPLE)
   return()
 endif()

With this code, the option BUILD_HELLO_WORLD_SAMPLE will be created in CMake's settings. Here, the default is **OFF**, in which case the build of this plugin will be skipped. If you want to build the plugin, set the option to **ON** in the CMake settings. By making it possible to switch whether to build a plugin or not in this way, it should be easier to develop and operate plugins.


.. _hello-world-stand-alone-build:

Build the plugin separately using CMake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

How to build a plugin together with Choreonoid was described above, but you may want to build a plugin separately from Choreonoid. In addition to that method being easier to develop, there may be cases where Choreonoid is installed from a binary package, etc. and you cannot use the Choreonoid build environment anyway.

In such cases, it is possible to build the plugin separately. Here, we will look at how to do this using CMake.

In this case, the Choreonoid source files are not necessary, but the SDK consisting of Choreonoid headers and libraries needs to be installed. This will be installed if you set the option INSTALL_SDK to ON in CMake when building Choreonoid. If you are using a binary package, use one that includes this SDK.

Then prepare the following CMakeLists.txt along with the plugin source file. ::

 cmake_minimum_required(VERSION 3.1.0)
 project(HelloWorldPlugin)
 find_package(Choreonoid REQUIRED)
 add_definitions(${CHOREONOID_DEFINITIONS})
 include_directories(${CHOREONOID_INCLUDE_DIRS})
 link_directories(${CHOREONOID_LIBRARY_DIRS})
 set(target CnoidHelloWorldPlugin)
 add_library(${target} SHARED HelloWorldPlugin.cpp)
 target_link_libraries(${target} ${CHOREONOID_BASE_LIBRARIES})
 install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})

Place this CMakeLists.txt in the same directory as the plugin source files. Then, perform the build in the usual way using cmake. When you perform the installation operation, the built plugin files will be installed in the Choreonoid plugin directory. On Linux, go to the plugin source directory on the terminal, execute ::

 cmake .

and then run ::

 make

to perform the build. If the build is successful, execute ::

 make install

to install it. (Root privileges may be required, depending on the installation destination.)

.. note:: Note that this method does not place the plugin source files or CMakeLists.txt in the Choreonoid ext directory. Executing cmake is not performed on Choreonoid itself, but it is executed directly for this plugin.

The details of CMakeLists.txt are as follows: ::

 cmake_minimum_required(VERSION 3.1.0)

The required version of CMake is specified. Set the appropriate version, taking into account the installed CMake version and the contents of CMakeLists.txt. ::

 project(HelloWorldPlugin)

Sets the CMake project. As we are building the plugin separately here, this needs to be set. ::

 find_package(Choreonoid REQUIRED)

Gets the information about the installed Choreonoid. Choreonoid is definitely required, since we are creating a Choreonoid plugin, so REQUIRED is specified here. If Choreonoid is found, its information is set to the following variables.

.. list-table::
 :widths: 40,60
 :header-rows: 1

 * - Variable
   - Details
 * - CHOREONOID_DEFINITIONS
   - compile options
 * - CHOREONOID_INCLUDE_DIRS
   - header file directory
 * - CHOREONOID_LIBRARY_DIRS
   - library file directory
 * - CHOREONOID_UTIL_LIBRARIES
   - libraries that have to be linked when using the Util module
 * - CHOREONOID_BASE_LIBRARIES
   - libraries that have to be linked when using the Base module
 * - CHOREONOID_PLUGIN_DIR
   - directory where the plugin files are installed

.. note:: In order to enable find_package to function, you need to have CMake's package detection path include the Choreonoid installation destination. But if Choreonoid is installed in a directory other than the default /usr/local, it may not be included in the detection path. In this case, Choreonoid will not be detected by find_package. In order to make it detectable, set the Choreonoid installation directory as the environment variable CHOREONOID_DIR or CMAKE_PREFIX_PATH. For details, refer to the information about find_package in the CMake manual.

Next, the information acquired using find_package is used as follows: ::

 add_definitions(${CHOREONOID_DEFINITIONS})
 include_directories(${CHOREONOID_INCLUDE_DIRS})
 link_directories(${CHOREONOID_LIBRARY_DIRS})

This description set the compile options, include paths, and link paths appropriately. ::

 set(target CnoidHelloWorldPlugin)

Set the plugin name to the variable target. ::

 add_library(${target} SHARED HelloWorldPlugin.cpp)

The plugin will become a shared library, so you can perform the build with the add_library command that is standard in CMake.

When :ref:`building together with Choreonoid <hello-world-build-together>`  , the plugin was built using the command add_cnoid_plugin, which extended add_library. But if you are building the plugin separately, use add_libary directly. ::

 target_link_libraries(${target} ${CHOREONOID_BASE_LIBRARIES})

Set the libraries that have to be linked to the plugin By using the CHOREONOID_BASE_LIBRARIES variable acquired with find_package, you can link to the set of libraries that form the base of the plugin. ::

 install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})

When you perform the installation operation, the built plugin files will be installed in the Choreonoid plugin directory. The installation destination can be specified in this way with the CHOREONOID_PLUGIN_DIR variable.

.. _hello-world-makefile-build:

Build the plugin by writing the Makefile directly
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When building a plugin separately, it is also possible to directly write the Makefile, which is the configuration file of the make command. Regarding this method, Choreonoid's SDK must be installed, in the same way as when you  :ref:`hello-world-stand-alone-build` .

Note that this method is not really recommended. We now have the excellent CMake tool, so it makes sense to use that. We will also introduce this method for special circumstances where you need to write a Makefile.

When writing a Makefile directly, use the tool called  `pkg-config <http://www.freedesktop.org/wiki/Software/pkg-config>`_ rather than CMake's find_package command to get the information required for the build. Below is an example of a Makefile using this.

.. code-block:: makefile

 CXXFLAGS += -fPIC `pkg-config --cflags choreonoid`
 PLUGIN = libCnoidHelloWorldPlugin.so
 
 $(PLUGIN): HelloWorldPlugin.o
   g++ -shared -o $(PLUGIN) HelloWorldPlugin.o `pkg-config --libs choreonoid`
 
 install: $(PLUGIN)
   install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
 clean:
   rm -f *.o *.so

If you run make using this Makefile, the plugin binaries will be generated, and if you run make install, the generated binaries should be installed in the Choreonoid plugin directory. If you then run Choreonoid, the plugins will be imported.

pkg-config is a tool commonly used on a Unix-type OS, and by executing the pkg-config command with appropriate options, like the above Makefile, you can get character strings such as the include path of the corresponding libraries, link paths, and library files. By passing this as a compiler option, it becomes possible to compile without describing those settings directly. Refer to the pkg-config manual for details.

.. note:: For pkg-config, the Choreonoid installation destination must be included in the detection path. You can use the environment variable PKG_CONFIG_PATH to add the detection path. If Choreonoid is installed somewhere other than the default /usr/local directory, set the installation directory named lib/pkgconfig to PKG_CONFIG_PATH.

Build file samples
^^^^^^^^^^^^^^^^^^

Build files corresponding to the three methods introduced here are stored in the directory of the HelloWorld sample. They can each be used in the following ways:

1. :ref:`hello-world-build-together`

 Corresponds to the sample directory CMakeLists.txt file. Copy the HelloWorld directory to the ext directory and build Choreonoid. 
 
2. :ref:`hello-world-stand-alone-build`

 This also corresponds to the sample directory CMakeLists.txt file. Execute cmake in the HelloWorld directory to perform the build.

3. :ref:`hello-world-makefile-build`

 Corresponds to the file named ManualMakefile. Either change the file name to Makefile or confer the option -f ManualMakefile when executing make.

Both #1 and #2 will result in the same CMakeLists.txt file, but the content is divided between the processing of #1 and #2. It determines whether or not it is a build of Choreonoid itself, and if it is, the content for #1 is processed. Otherwise the content for #2 is processed.
