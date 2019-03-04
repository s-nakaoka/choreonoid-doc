
Preparing to use the OpenRTM plugin
===================================

This page explains the preparations required before being able to use the OpenRTM plugin.

.. contents::
   :local:

.. highlight:: sh

.. _openrtmplugin_install_openrtm:

Installing OpenRTM-aist
-----------------------

Use of the OpenRTM plugin requires that OpenRTM-aist is installed in the OS.

Although the official OpenRTM-aist website and related servers are currently closed, a temporary `OpenRTM-aist web on the github <http://openrtm.org/>`_  has been installed, and you can download OpenRTM-aist from there. The latest downloadable version is 1.1.2. Download and install this version. Earlier versions are no longer supported.

Installing on Ubuntu 16.04
~~~~~~~~~~~~~~~~~~~~~~~~~~

On Ubuntu 16.04, you can install the C++ version by entering as follows from the command line as described on the page above.  ::

 git clone https://github.com/n-ando/xenial_package.git
 cd xenial_package/xenial/main/binary-amd64/
 sudo dpkg -i openrtm-aist_1.1.2-0_amd64.deb
 sudo dpkg -i openrtm-aist-example_1.1.2-0_amd64.deb
 sudo dpkg -i openrtm-aist-dev_1.1.2-0_amd64.deb

Other packages related to OpenRTM-aist include the Python version and tools such as RTSystemEditor and RTCBuilder. Install them as required using the installation process described on a previous page.

.. _openrtmplugin_patch_for_version112:

Dealing with bugs in version 1.1.2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There are bugs in OpenRTM-aist 1.1.2, and as a result:

* it fails when using OpenRTM-related functions on Choreonoid
* a MARSHAL_InvalidEnumValue exception is raised when operating RTC created in C++ with RTShell

These are known bugs.

These bugs are caused by the OpenRTM-aist header file OutPort.h. While there does seem to be a version of OutPort.h in which the bug has been fixed, the OpenRTM-aist 1.1 series has not released this version, so you will need to patch it yourself. This has been verified to resolve the above-mentioned bugs.

OutPort.h is installed in the /usr/include/openrtm-1.1/rtm/ directory when installing on Ubuntu 16.04 as described above. Copy the modified version of OutPort.h downloaded from the link below and overwrite the version in this directory. (You need to do this before building the OpenRTM plugin and samples.)

* :download:`Modified version of OutPort.h for OpenRTM-aist 1.1.2 <files/OutPort.h>`

.. note:: The modified version that was previously distributed here only fixed an earlier bug and was updated on September 6, 2018 with a modification dealing with the second bug. Anyone using the previous modified version of OutPort.h should update it by downloading the new version from the above link.


Building the OpenRTM plugin
---------------------------

The OpenRTM plugin is included in the Choreonoid source code. You can build by setting the following CMake options to **ON** with the :ref:`build-ubuntu-cmake` done before building Choreonoid.

* **ENABLE_CORBA**            : CORBA ON/OFF - Option for enabling CORBA.
* **BUILD_CORBA_PLUGIN**      : CORBA Plugin - Plugin for using CORBA for communication. It is used to communicate with the RT component.
* **BUILD_OPENRTM_PLUGIN**    : OpenRTM Plugin - Plugin for linking with OpenRTM-aist.

Also, set the following options to **ON** as necessary.

* **BUILD_OPENRTM_SAMPLES**   : Sample for OpenRTM plugin - Sample RT component running on OpenRTM-aist

Once the settings are complete, carry out the build of Choreonoid.

* :doc:`../install/build-ubuntu`
* :doc:`../install/build-windows`

.. _openrtmplugin_setup_corba:

CORBA settings
--------------

Setting the max. message size of omniORB
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When using OpenRTM, it is a good idea to increase the maximum message size of omniORB. omniORB is a CORBA library used in the implementation of OpenRTM-aist, and it is installed when installing OpenRTM. There is a settings file named /etc/omniORB.cfg, so edit this file with root privileges. Inside the settings file, there is a description: ::

 giopMaxMsgSize = 2097152   # 2 MBytes.

and this displays the maximum message size.

By default, it is set to 2MB. But in this case, if you try to send data of 2MB or more at a time, for example image data or a point-cloud data transmission, it will not send successfully. A value of 2MB is too small, so let’s increase it. If you want to set it to 20MB, for example, rewrite the line as: ::

 giopMaxMsgSize = 20971520

.

.. _openrtm_install_clear_omninames_cache:

Clearing the omniNames cache
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The CORBA communication standard, which OpenRTM uses as its base, uses something called a “name server”. This is used to register the addresses of “CORBA objects” handled by CORBA on the network. When you install omniORB, a name server named omniNames is also installed and is used by default.

This omniNames has a “cache” function that restores the information about the registered objects when the OS is restarted. With this cache, information accumulates about objects that do not exist, and this may affect the behavior of the system. Since the address of a CORBA object also includes its IP address, this problem easily occurs when there are changes to the PC configuration on the network or when the network itself changes.

In order to avoid this problem, it is a good idea to clear the cache every time there is a change to the network configuration. Clearing the cache

when using Linux can be done using a shell script named **reset-omninames.sh**. This is in the Choreonoid build directory or the **bin** directory of the installation location. This script is executed by running ::

 reset-omninames.sh

from the command line. (If bin is not already included, add it to your PATH.)

You need administrator privileges to execute this script. If you are prompted for a password at the time of execution, enter the password and execute it.

If OpenRTM related operations are not running properly, the cache may be compromised. If that is the case, you should stop this system completely and then execute this script.
