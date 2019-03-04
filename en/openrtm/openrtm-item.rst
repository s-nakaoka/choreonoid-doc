
Items used with the OpenRTM plugin
==================================

The OpenRTM plugin uses the following items.

RTSystem Item
-------------

In OpenRTM, the RT function element is called an RT Component (RT-Component: RTC), and a robot system (RT system) is constructed using multiple RTCs.

An “RTSystem item" is a project item for managing the robot system (RT system) and is used to manage the elements (RTC) that make up the system on Choreonoid.
    
The following properties are additionally available for the RTSystem item.

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 15,12,4,75
  :header-rows: 1

  * - Parameter
    - Default value
    - Type
    - Detail
  * - Auto connection
    - True
    - bool
    - Specify whether to automatically execute the connection between the RTCs that make up the RT system when reading items.
  * - Vendor name
    - \-
    - string
    - Set the name of the vendor that created the target RT system. The value set here is used as “VendorName” of RTSProfile when saving the system. The default value can be specified in Preferences.
  * - Version
    - \-
    - string
    - Set the target RT system version number. The value set here is used as “Version” of RTSProfile when saving the system. The default value can be specified in Preferences.
  * - File (profile save destination)
    - \-
    - string
    - With the OpenRTM plugin, information on the RT system is saved using RTSProfile format (format standardized by RT middleware). In this case, specify the save destination of RTSProfile.
  * - State Check
    - Polling
    -
    - | Set the method for checking the status of the RTCs that make up the RT system.
      | Polling: The status of each RTC is automatically updated at the cycle specified by “Polling Cycle”.
      | Manual: The status of each RTC, etc. is updated manually.
  * - Polling Cycle
    - 1000
    - int
    - Set the cycle for checking the status of the RTC that constitutes the RT system. The unit is ms.
  * - CheckAtLoading
    - true
    - bool
    - When importing a project, specify whether to check the status of the RTCs that make up the target RT system. If it is time-consuming to check the status, for reasons such as an RT system made up of a large number of RTCs, or when the RTCs are running on multiple PCs, turning this value OFF means the project read time can be shortened.

ControllerRTC item
------------------

This is an item for defining the RTCs for controlling the system defined by the RTSystem item. It controls the target system according to instructions from Choreonoid.

The following properties are additionally available for the ControllerRTC item.

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 15,12,75
  :header-rows: 1

  * - Parameter
    - Type
    - Detail
  * - RTC module
    - string
    - Set the name of the ControllerRTC to be actually used. A file selection dialog box is displayed when editing properties, so it is also possible to select and specify the ControllerRTC to be used.
  * - Base directory
    - string
    - Set the directory where the ControllerRTC being used is kept.
  * - RTC instance name
    - string
    - Set the instance name to identify the ControllerRTC. In OpenRTM it is possible to launch multiple RTCs of the same type. In this case, specify it in order to identify the instance of the controller to be actually used.
  * - Execution context
    - string
    - | Set the execution context for use by ControllerRTC. The following execution contexts can be selected:
      | SimulationExecutionContext: execution context for execution of simulation
      | SimulationPeriodicExecutionContext: execution context for carrying out some sort of periodic execution during simulation execution.
  * - Periodic rate
    - int
    - Set the frequency for executing ControllerRTC.


BodyIoRTC item
--------------

This item defines the RTC for controlling robots. Set the RTC for input and output to various devices mounted on the robot. During simulation execution, handles processing so that the input/output for various devices is synchronized with the simulation.

The following properties are additionally available for the BodyIoRTC item.

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 15,12,75
  :header-rows: 1

  * - Parameter
    - Type
    - Detail
  * - RTC module
    - string
    - Set the name of the target RTC. A file selection dialog box is displayed when editing properties, so it is also possible to select and specify the RTC to be used.
  * - Base directory
    - string
    - Set the directory where the target RTC being used is kept.
  * - RTC instance name
    - string
    - Set the instance name to identify the ControllerRTC. In OpenRTM it is possible to launch multiple RTCs of the same type. In this case, specify it in order to identify the instance of the RTC to be actually used.
  * - Periodic rate
    - int
    - Set the frequency for executing ControllerRTC.


RTC item
--------

This is an item for using regular RTCs. It is used when using RTCs other than “ControllerRTC” and “RTC for coordinating with robots (BodyIoRTC)”.

The following properties are additionally available for the RTC item.

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 15,12,75
  :header-rows: 1

  * - Parameter
    - Type
    - Detail
  * - RTC module
    - string
    - Set the name of the target RTC. A file selection dialog box is displayed when editing properties, so it is also possible to select and specify the RTC to be used.
  * - Base directory
    - string
    - Set the directory where the target RTC being used is kept.
  * - Execution context
    - string
    - Set the execution context for use by the target RTC. The types of selectable execution context depend on the version of OpenRTM-aist you are using, so refer to the OpenRTM-aist website for details.
  * - Periodic rate
    - int
    - Set the frequency for executing ControllerRTC.


BodyRTC item (not recommended)
------------------------------

This is an old version of the item that defines the RTC for controlling robots. The settings done with “BodyRTC item” can be defined using “BodyIoRTC item”, so you should use the latter.
