
Viewing of Sensor State
==========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

.. contents:: 
   :local:

.. highlight:: cpp


How to Check Sensor Value
------------------------------

You may want to check the state value of a sensor mounted in the robot to verify the status of the robot. Choreonoid has the following functions to cope with this:

* Body Status View
* Sensor Visualisation Items

The target of these functions are the body models that are imported as the body models in Choreonoid and the sensor information stored in them is referred to.

To apply these functions to a simulation result, it is necessary to enable  :ref:`simulation-device-state-recording` so that the sensor state in the simulation can be output to the body model. By doing so, the sensor state can be presented easily without any additional manipulation to the controller, etc. of the robot.

Body Status View
---------------------

"Body Status View" is available as a view to present the sensor state values as they are in numeric value.

Select "View" under Main Menu and "Present Views" then "Body Status" to display the view to use this function. This view displays no data by default. By selecting any body item in Item Tree View, it presents a list of the values of the sensors mounted in the body model.

For example, if AR1 sample model is selected, the view will look like as follows:

.. image:: images/BodyStateViewSR1.png

In the initial state, all data show 0 as in the figure, but the values start to change once the simulation is started. Also, the state of the relevant time is presented when replaying the simulation result that has been recorded.

Note that the sensor types that can be displayed in this view are force sensor, acceleration sensor and rate gyro sensor for the time being.


Sensor Visualisation Item
-------------------------------

"Sensor Visualisation Item" is available as the function to visualise the sensor state using 3DCG in Scene View. Using this, you can capture the robot status intuitively.

To use this function, create this item first. Select "File" under Main Menu - "New" - "Visualise sensors" and allocate the created item as a sub-item of the target body item.

By checking the item in this status, the current sensor value of the model is displayed as an arrow marker in Scene View.

The supported sensor is currently force sensor only. Of the six force torque direction elements that a force sensor has, only three force direction elements can be visualised and they are presented with arrow markers as vectors from the position of the force sensor.

The following is an example where the force sensors mounted in the both ankles are visualised in the sample of walking of SR1.

.. image:: images/sr1-force-visualizer.png

For the convenience of easy understanding of sensor visualisation markers, Scene View is in  :ref:`basics_sceneview_wireframe` .

The length of the marker can be adjusted in the "Ratio of display" property of the sensor visualisation item. Adjust the value of this property so that the markers are best viewable.

