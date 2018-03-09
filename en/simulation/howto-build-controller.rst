
Building the controller
==================================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents:: 
   :local:

Building the controller
---------------------------

The section on :doc:`howto-implement-controller` focused on the structure of the source code in order to give a summary of how the SimpleController is implemented. The controller we used in that example was one of the samples included in Choreonoid and was available as part of having built or installed Choreonoid. However, when the user wants to develop a new controller, it is not enough to simply write the source code as-is. The SimpleController uses C++, so you need to build (compile and link) the controller from source code into a binary file. Here we discuss how to build a controller, using the SimpleController as an example.

There are, generally speaking, two core ways of building the controller.

1. Building alongside Choreonoid
2. Building separately from Choreonoid

Below, we discuss each method in detail.

.. _simulation-build-controller-method1:

Building alongside Choreonoid
--------------------------------------

.. highlight:: cmake

As with the sample controller, this method involves also building the controller when you build Choreonoid. What this also implies is that when building the controller first, if you issue the command to build it alongside Choreonoid, it will be built as one part of the Choreonoid package. If you are building Choreonoid from source, this is the most convenient method.

Choreonoid makes use of CMake to specify build instructions. This method therefore includes the build code for the controller ahead of time. Ordinarily, the build instructions in CMake operate by way of the CMakeLists.txt file. In this workflow, we will be using that file to configure our build instructions for the controller. By using the add_cnoid_simple_controller, a CMake function defined in Choreonoid, you can easily specify the controller build settings. The arguments given to this function are as follows: ::

 add_cnoid_simple_controller(controller name  source file ...)

You can specify a single source file or multiple ones.

Next, you must ensure that this CMakeLists.txt file is recognized by the Choreonoid build routine. To do so, use the “ext” directory found in the Choreonoid source directory. If there is a CMakeLists.txt file within any directory below “ext,” Choreonoid will recognize this as a build target and import it. Create a directory below ext with a name of your choice and store the corresponding source file as CMakeLists.txt there.

As a specific example, consider using the prior MyController C++ source code to describe a file named MyController.cpp. First create a subdirectory within the Choreonoid ext directory and use it to store your file. The name can be anything; for the purposes of clarity, we will give the directory the controller name and name it MyController.

Next, create a CMakeLists.txt with the content below and save it in the same directory. ::

 add_cnoid_simple_controller(MyController MyController.cpp)

The directory and file hierarchy should now be as follows. ::

 Choreonoid source directory
  + ext
    + MyController
      - CMakeLists.txt
      - MyController.cpp

Next, once you build Choreonoid itself, MyController will also be built. You should invoke cmake and make in the Choreonoid build directory. If the build succeeds, you will now find the file MyController.so (for Linux; for Windows, the file will be MyController.dll) in the default SimpleController directory, as discussed in the section on  :ref:`simulation-set-controller-to-controller-item` .

.. note:: Avoid issuing cmake directly against the controller’s CMakeLists.txt created under ext. In this method, the CMakeLists.txt for the controller is used to import one aspect of the build instructions for Choreonoid and is not intended to apply to standalone instances of cmake.

.. note:: You can also import from source directories in a path other than ext. In this case, you should set the path of the source directory you want to import by using the **ADDITIONAL_EXT_DIRECTORIES** parameter within the Choreonoid CMake file. Separating with a semicolon allows you to set multiple items.

.. note:: There may be cases, such as where a controller is linked to external libraries, where the controller structure is more complicated and requires additional CMakeLists.txt settings beyond add_cnoid_simple_controller. In these cases, you can refer to the CMake manual and the definitions for the add_cnoid_simple_controller function to adjust accordingly. (The  add_cnoid_simple_controller function is defined within src/Body/CMakeListst.txt in the Choreonoid source directory.)

Building separately from Choreonoid
--------------------------------------

This method presumes that Choreonoid is installed on the system, with the controller used to operate it being build separately.

“Installation” here refers to, after building Choreonoid from source, copying the files needed to run the program to the requisite system directories and passing the path to the executable files and library files. See below for details on installation.

* Install method 1: :doc:`../install/build-ubuntu`
* Install method 2: :doc:`../install/build-windows`

Setting compile options
~~~~~~~~~~~~~~~~~~~~~~~~~~

If Choreonoid is installed, you should configure the include path and library path pointing to it and then build. Note that you must also specify several other compile options.

As an example, if you have installed Choreonoid to the /usr/local/ path and use gcc (g++) to compile, you would set the below options when compiling. (you should replace /usr/local/ with the actual directory in which you installed to.)

* **-std=c++11** (enable C++ 11）
* **-fPIC** (compile against shared libraries）
* **-I/usr/local/include** (add the install path）

The link options are additionally set as follows:

* **--shared** (link as shared library）
* **-L/usr/local/lib** (add link path）
* **-lCnoidUtil -lCnoidBody** (link Choreonoid Util library and Body library）

The Util and Body libraries are some of the constituent libraries that make up the design of Choreonoid. The Util library is a utility library that incorporates a range of functionality, while the Body library is a library incorporating the various functionality associated with  :doc:`../handling-models/bodymodel` . The simple controller makes use of the functionality of these libraries, so as a minimum you must build against them.

.. note:: By default, /usr/local/include/ and /usr/local/lib/ may be included in the compiler’s install and library paths. In this case, you do not need to explicitly add the above “-l/usr/local/include/ or “-L/usr/local/lib/”. However, if you have installed Choreonoid to a directory other than /usr/local/, you will generally need to add the corresponding path.

Installing controllers
~~~~~~~~~~~~~~~~~~~~~~~~~~

The binary file generated for the controller is ordinarily copied (installed) into the default controller directory. The default directory is:

* /usr/local/lib/choreonoid-x.x/simplecontroller (x.x corresponds to version numbers）

.

.. note:: A default controller directory is used to make it easy to centrally save files or allow for easy access to controller items in one place. If you have reason to store the controller in a different directory, you are free to do so.

Using pkg-config
~~~~~~~~~~~~~~~~~~~~~~

.. highlight:: sh

Installing Choreonoid will enable you to configure compile options by way of `pkg-config <https://www.freedesktop.org/wiki/Software/pkg-config/>`_ .

Specifically, invoking ::

 pkg-config --cflags choreonoid-body

will output the options necessary for compiling the program against the Body library. Invoking ::

 pkg-config --libs choreonoid-body

will output the options necessary to link to programs making use of the Body library.

Using this command lets you build programs making use of Choreonoid without having to worry about where Choreonoid is installed or which libraries to link to.

The choreonoid-body part of this command is an identifier that corresponds within pkg-config to the Choreonoid Body library.  Installing Choreonoid lets you use the below identifiers to poll information on Choreonoid’s various libraries.

* **choreonoid-util** : Util library
* **choreonoid-body** : Body library
* **choreonoid-base** : Base library
* **choreonoid-body-plugin** : Body plugin library

To build the SimpleController, you generally only need to use choreonoid-body.

.. note:: The Base library is the core library used to develop Choreonoid plugins. The Body plugin library is designed to allow external use of the Body plugin functionality as a library; it is used when developing other plugins that depend on the Body plugin.

In order to use the above identifier in pkg-config, the Choreonoid install path must be recognized by the pkg-config system. To install in the default installation directory (/usr/local), the path should be automatically detected by pkg-config. To install in a different directory, you must set the path using the environment variable PKG_CONFIG_PATH when installing Choreonoid.

For example, if you have installed Choreonoid in the /usr path of the user’s home directory, you should execute the following: ::

 export PKG_CONFIG_PATH=$HOME/usr/lib/pkgconfig

.

.. _simulation-build-controller-commands:

Example build commands
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here we describe an actual build command using Ubuntu Linux.

We assume the controller source file is MyController.cpp. After storing this in a directory of your choice, navigate to that directory from a command prompt.

Compile using the below command. ::

 g++ -std=c++11 -fPIC `pkg-config --cflags choreonoid-body` -c MyController.cpp

Executing this will create MyController.o, the object used to compile MyController.cpp.

Next, issue the following command to create a link. ::

 g++ --shared -std=c++11 -o MyController.so MyController.o `pkg-config --libs choreonoid-body`

This will generate the MyController.so file. This is the controller’s binary file and can be used by calling the controller module for the SimpleController item.

If necessary, you can also install it to the default directory. ::

 cp MyController.so `pkg-config --variable=simplecontrollerdir choreonoid-body`

Using pkg-config in this way allows you to obtain the default directory of the SimpleController. If you installed somewhere other than /usr/local, add sudo to the above command and execute it: ::

 sudo cp MyController.so `pkg-config --variable=simplecontrollerdir choreonoid-body`

.

.. note:: Similar to the method discussed in the section on :ref:`simulation-build-controller-method1` , if the controller consists of multiple source files or if you are linking libraries other than CnoidBody, the increased complexity of the controller layout may mean the above command is not enough to successfully build with. That case is outside the scope of this tutorial and has to do with issues of general program development, so we will not venture into that topic here.

Makefile example
~~~~~~~~~~~~~~~~~~~~~~

.. highlight:: makefile
   :linenothreshold: 5

Issuing the above commands each time we want to build something would be tedious. To avoid this and simplify the build process, we use the Make command. The Make command refers to a Makefile that describes the build process. Below is an example showing a Makefile used to build MyController. ::

 CONTROLLER=MyController.so
 SRC=MyController.cpp
 OBJ=$(SRC:%.cpp=%.o)
 
 $(CONTROLLER): $(OBJ)
	g++ --shared -std=c++11 -o $(CONTROLLER) $(OBJ) `pkg-config --libs choreonoid-body`
 
 %.o: %.cpp
	g++ -std=c++11 -fPIC `pkg-config --cflags choreonoid-body` -c $<
 
 install: $(CONTROLLER)
 	install -s $(CONTROLLER) `pkg-config --variable=simplecontrollerdir choreonoid-body`
 clean:
	rm -f *.o *.so

Note that, by design, Makefiles require that you use indentation at the start of lines 6, 9, 12, and 14. (These must be tabs and not spaces; otherwise, it will produce an error.)

.. highlight:: sh

Create a file named “Makefile” with the above content and save it to the directory where MyController.cpp is located. Navigate to that directory via the command line and invoke: ::

 make

This will begin building the controller. Next, execute: ::

 make install

to install to the default controller directory. (As needed, you should prepend make install with sudo.)

This will produce the same outcome as issuing the build commands discussed in the section on :ref:`simulation-build-controller-commands` .

For details on Makefile syntax, refer to the `Make documentation <https://www.gnu.org/software/make/manual/>`_ .

.. note:: Though we have abbreviated the discussion here, compiling and linking generally takes the -O2 and -O3 options. These are used to enable optimization and increase the runtime speed of the program generated. For debugging purposes, you can pass the -g flag to also generate debug data. These details can be found in your compiler’s manual or in a variety of resources on C/C++ development.

It is seldom the case that a Makefile is directly written from scratch. This is because it is today standard to use advanced build tools like CMake. CMake is also used to build Choreonoid proper, and is used in the section describing how to :ref:`simulation-build-controller-method1` . However, CMake can also be used to build a controller separately from Choreonoid. Note that the CMake invocation method and notation of CMakeLists.txt differs slightly in that case from the method described in the section on :ref:`simulation-build-controller-method1` . For further details on CMake, see the `CMake documentation <https://cmake.org/documentation/>`_ .
