
Plugin System
=============

.. contents::
   :local:
   :depth: 1


What is a Plugin
----------------

Choreonoid allows you to add a new function by installing an additional program module. Such a module is called a "plugin". Actually, many of the standard functions of Choreonoid are implemented as plugins.

Although there may be various types of functions added by plugins, you can generally consider a plugin as something that adds a new type item, view, or toolbar to enable you to handle new data or to use a new operation/edit interface.


Example of Plugins
------------------

The following table lists some of the plugins provided by Choreonoid as standard. As you can see, a wide range of functions are provided as plugins, ranging from basic functions to special functions.

.. tabularcolumns:: |p{4.0cm}|p{11.0cm}|

.. list-table::
 :widths: 30,70
 :header-rows: 1

 * - Plugin
   - Overview
 * - BodyPlugin
   - Plugin that provides basic functions for handling the models of robots and objects. It defines the item classes and views related to displaying robot models, editing positions and postures, kinetics simulation, etc.
 * - PoseSeqPlugin
   - Plugin that is a collection of functions for robot motion choreography by keyframing.
 * - BalancerPlguin
   - Plugin that adds an automatic balance correction function for biped robots to motion choreography by PoseSeqPlugin.
 * - SimpleControllerPlugin
   - Plugin that defines a unique controller form for robot simulation. The plugin provides a simple controller form in which the DLL of the controller is directly loaded and direct connection is made. It is also used in the sample simulation.
 * - CorbaPlugin
   - Plugin that provides basic functions related to the communication middleware CORBA.
 * - OpenRTMPlugin
   - Plugin for using OpenRTM in Choreonoid. It enables simulation with an RT-component as a controller and other operations. 
 * - PythonPlugin
   - Plugin to execute Python scripts in Choreonoid. Python scripts can automate Choreonoid operations.
 * - MediaPlugin
   - Plugin to play audio and video files in Choreonoid.


Dependencies between Plugins
----------------------------

Some plugins require functions of other plugins. In this case, there are dependencies between plugins. For example, the following figure shows dependencies between the above plugins.

.. figure:: images/plugin-dependencies.png

The directions of the arrows in the figure indicate the directions of dependence. For example, PoseSeqPlugin is dependent on BodyPlugin, and thus BodyPlugin must also be loaded to use PoseSeqPlugin. BalancerPlugin is dependent on PoseSeqPlugin, and thus PoseSeqPlugin and BodyPlugin are required to use BalancerPlugin. On the other hand, BodyPlugin, CorbaPlugin, and PythonPlugin themselves can be loaded without requiring other plugins.

Users do not need to pay particular attention to plugin dependencies because, normally, dependent plugins are also built when a plugin is built. However, being aware of these dependencies may be useful because it may be possible that a dependent plugin has not been installed correctly, when a plugin cannot be loaded normally.

.. note:: For this, it would be desirable if Choreonoid could provide a function to display a list of available plugins and the dependencies between them. However, since Choreonoid has not provided such a function yet, users must obtain such information through manuals for now. In the future, we would like to realize a function to display plugin information.

As you can see from plugin dependencies, Choreonoid can provide additional functions while a plugin uses functions of other plugins. Thus, new functions can be developed efficiently and it can be expected that users can operate the developed functions in the same way as the existing functions.

.. _basics_plugin_files:

Plugin File
-----------

A plugin is a binary file in the form of a "shared library" or "dynamic link library", and is normally stored in the plugin directory of Choreonoid. The plugin directory is "lib/choreonoid-x.x" in the build and installation directories of Choreonoid. A version number of Choreonoid is displayed in x.x.

For Linux, the file name of a plugin file of Choreonoid is "libCnoid" followed by the plugin name and then ".so", which is the shared library extension. For example, the file name of the above BodyPlugin is "libCnoidBodyPlugin.so".

For Windows, the file name is "Cnoid" followed by the plugin name and then the extension ".dll". Therefore, in the case of BodyPlugin, the file name is "CnoidBodyPlugin.dll".

Plugin files stored in the plugin directory are automatically loaded and become available when Choreonoid starts. If prescribed plugins are not loaded normally, check whether the plugins (including dependent plugins, if any) are located in the plugin directory correctly.

.. note:: If you use Choreonoid built from source, be careful when rebuild and install it by updating the source. There is no problem if all the plugins built previously are updated and installed. In some cases, however, plugins may be discontinued or renamed. Also, the configuration of plugins to be built with the build settings of CMake may be changed. In such a case, the files of plugins that are no longer built remain in the build and installation directories, resulting in being loaded when Choreonoid starts. However, their contents are obsoleted, and thus may cause problems, such as a Choreonoid crash. Therefore, when the configuration of plugins to be built is changed, especially when the behavior of Choreonoid becomes abnormal, it is recommended to delete all the plugin files from the plugin directory and then perform re-installation. 


Building Plugins
----------------

Although several plugins are built and installed in Choreonoid as standard, there are other plugins available as options. In addition, in some cases, you want to use plugins distributed separately from the Choreonoid main unit. In such a case, you must build and install beforehand the plugin you want to use.

For plugins that come with the Choreonoid main unit, see the description in :doc:`../install/install` - :doc:`../install/options` to build them. Basically, you should simply turn on the "ENABLE_XXX_PLUGIN" option (XXX is the plugin name) to build the plugin when configuring settings for CMake at build time.

For plugins distributed separately from the main unit, build and install them by following the description of them.

Developing Plugins
------------------

You can also add a new function to Choreonoid by developing a plugin.

.. For information on how to develop a plugin, see the :doc:`../plugin-development/index` in this manual.

The following functions have actually been achieved or applied by users' actual development of new plugins:

* Operation interface for the biped humanoid robots "HRP-2" and "HRP-4C"
* Function to capture human postures acquired by Kinect into a model in Choreonoid
* Function to perform simulation by connecting with a ROS node
* Research and development of a new simulation engine
* Function to make a motion plan focused on gripping in Choreonoid ( `graspPlugin <http://choreonoid.org/GraspPlugin/>`_ )
* Research and development of techniques for applying motion data of humans whose motions are captured
