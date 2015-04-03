
Optional Functions Overview
===========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

This section describes the main optional functions that can be selected for the build configuration of the development version of Choreonoid.

* **ENABLE_PYTHON**

 Build any Python wrapper available for each module and plug-in. To use it, it is necessary to install the library for development of Python.

 For a module, the library function of Choreonoid can be used from Python by using Python wrapper in combination. For example, by importing "cnoid Body" module, which is Python wrapper of Body wrapper, it is also possible to load the robot model file on Python and make different calculations. As for plug-in Python wrapper, it will become possible to use the plug-in function on Choreonoid from Python script by using it in combination with Python plug-in described below.

* **BUILD_PYTHON_PLUGIN**

 It builds Python plug-in. It is necessary to turn ON ENABLER_PYTHON, too. This plug-in makes possible reading and running Python script and the functions that operates on Choreonoid including Python console.

* **BUILD_PYTHON_SIM_SCRIPT_PLUGIN**

 It builds PythonSimScript plug-in to use Python script for the simulation setup. It is also necessary to build Python plug-in.

* **BUILD_POSE_SEQ_PLUGIN**

 It builds PoseSeq plug-in, which provides provides the choreographic function by key pause. It is configured to ON by default.

* **BUILD_BALANCER_PLUGIN**

 It builds Balancer plug-in, which provides the auto-balance correction for the choreographic function. When choreographing a biped robot, you can make it not fall down (logically) by using this function.

* **BUILD_SIMPLE_CONTROLLER_PLUGIN**

 It builds SimpleController plug-in. This plug-in makes available the format that is directly connected in the Choreonoid process regarding the controller programme of the robot in the simulation. Currently, it is positioned as a sample of this format and only the minimal set of functions is implemented. It is turned ON as default.

* **BUILD_SIMPLE_CONTROLLER_SAMPLES**

 It builds the controller that supports SimpleController plug-in and a simulation sample.

* **BUILD_ODE_PLUGIN**

 It builds ODE plug-in. With this plug-in, "Open Dynamics Engine (ODE)", which is a library of open source dynamics calculations, can be used as a calculation engine for the simulation function of Choreonoid. To use this, it is necessary to install `Open Dynamics Engine (ODE) <http://www.ode.org/>`_ .

* **BUILD_BULLET_PLUGIN**

 It builds Bullet plug-in. With this plug-in, "Bullet Physics", which is a library of open source dynamics calculations, can be used as a calculation engine for the simulation function of Choreonoid. To use this, it is necessary to install  `Bullet Physics Library <http://bulletphysics.org>`_ .

* **ENABLE_CORBA**

 It builds the modules that are the bases of CORBA-related functions. To introduce these modules, it is necessary to install `omniORB <http://omniorb.sourceforge.net/>`_ library.

* **BUILD_CORBA_PLUGIN**

 This plug-in provides CORBA-related functions. EMABLE_CORBA must be enabled, too.

* **BUILD_OPENRTM_PLUGIN**

 It builds OpenRTM plug-in. With this plug-in, RT component (RTC), which is an RT middleware component can be used for simulation. To use this, it is necessary to install `OpenRTM-aist <http://openrtm.org/>`_ 1.1. It is also necessary to build the above-mentioned CORBA plug-in.

* **BUILD_OPENRTM_SAMPLES**

 It builds simulation samples using RT component.

* **BUILD_OPENHRP_PLUGIN**

 It builds OpenHRP plug-in. With OpenHRP plug-in, it will become possible to use the robot simulation using the control programme developed for OpenHRP3 and the on-line viewer. It is also necessary to build CORBA plug-in.

* **BUILD_OPENHRP_PLUGIN_FOR_3_0**

 OpenHRP plug-in and the sample controller simulation of OpenHRP support API (IDL) of OpenHRP version 3.1 as default. By turning on this option, it supports OpenHRP version 3.0, too.

* **BUILD_OPENHRP_SAMPLES**

 It builds the controller simulation samples compliant with IDL of OpenHRP3. This sample is wrapped with IDL of OpenHRP3 of SimpleController sample.

* **BUILD_MEDIA_PLUGIN**

 It builds Media plug-in that replays a media file. Some platforms require different libraries to handle media files.

* **BUILD_GROBOT_PLUGIN**

 It builds GRobot plug-in. With this plug-in, it will become possible to synchronise the motions of the production version of the biped robot "G-Robots GR001" to the motions produced using Choreonoid.
 

