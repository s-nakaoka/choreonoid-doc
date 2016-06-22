
Viewing of Sensor State
=======================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents:: 
   :local:

.. highlight:: cpp


How to Check Sensor Value
-------------------------

You may want to check the state value of a sensor mounted in the robot to verify the status of the robot. Choreonoid has the following functions to cope with this:

* Body State View
* Sensor Visualizer Item

The target of these functions is a body object that is loaded as a body item in Choreonoid, and the sensor information stored in the body object is referred to.

To apply these functions to a simulation result, it is necessary to enable  :ref:`simulation-device-state-recording` so that the sensor state in the simulation can be output to the body object. By doing so, the sensor state can be presented easily without any additional modification to the controller, etc. of the robot.

Body State View
---------------

"Body State View" is available as a view to present the sensor state values as they are in numeric value.

To use this function, select "View" under Main Menu and "Show View" then "Body State" to display the view. This view displays no data by default. By selecting any body item in Item Tree View, it presents a list of the values of the sensors mounted in the body model.

For example, if SR1 sample model is selected, the view will look like as follows:

.. image:: images/BodyStateViewSR1.png

In the initial state, all data show 0 as in the figure, but the values start to change once the simulation is started. Also, the state of the relevant time is presented when replaying the simulation result that has been recorded.

Note that the sensor types that can be displayed in this view are force sensor, acceleration sensor and rate gyro sensor for the time being.


Sensor Visualizer Item
----------------------

"Sensor Visualizer Item" is available as the function to visualize the sensor state using 3DCG in Scene View. Using this, you can capture the robot status intuitively.

To use this function, create this item first. Select "File" under Main Menu - "New" - "SensorVisualizer" and allocate the created item as a child item of the target body item.

By checking the item in Item Tree View in this status, the current sensor state values of the body object are displayed as arrow markers in Scene View.

The supported sensor is currently force sensor only. Of the six-axes force-torque elements that a force sensor has, only three-axes force elements can be visualized and they are presented with arrow markers as vectors from the position of the force sensor.

The following is an example where the force sensors mounted in the both ankles are visualized in the SR1 walk sample.

.. image:: images/sr1-force-visualizer.png

For the convenience of easy understanding of sensor visualization markers, Scene View is in  :ref:`basics_sceneview_wireframe` .

The length of the markers can be adjusted in the "visual ratio" property of the sensor visualizer item. Adjust the value of this property so that the markers are best viewable.
