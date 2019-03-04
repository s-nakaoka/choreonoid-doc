Robot models
============

The :doc:`overview`  section gave an overview of the :ref:`wrs2018_overview_robots` . Here we will explain the model data used for handling each robot on Choreonoid.

.. contents::
   :local:

Model files
-----------

Choreonoid's models are described as “Body format” files with the extension .body. Refer to the  :doc:`../handling-models/modelfile/index`  section of this manual for summarized details of the specifications, etc. of this format.

When parameter adjustment or modification are performed on the standard robot model, or when a new robot model is being created, you need to edit this format model file. The model file is text data in YAML format, so it is usually edited with a text editor. Refer to  :doc:`../handling-models/modelfile/modelfile-newformat`  for information on how to edit the file.

The model file can be imported into Choreonoid as a “body item”. Refer to :doc:`../handling-models/index`  for information on how to handle body items.

.. _wrs_standard_model_directory:

Standard model directory
------------------------

Choreonoid contains many model files as samples. They are stored under the “standard model directory” of Choreonoid. For the source code, the standard model directory is share/model. If make install was used for Choreonoid, the installation location will be share/choreonoid-x.x/model. (x.x is the version number. Refer to :doc:`../install/directories` .)

The WRS2018 standard robot model is included as a sample model in Choreonoid, and it is stored under the standard model directory.

WAREC-1
-------

The WAREC-1 model is stored in a subdirectory named WAREC1 in the standard model directory. The main file is WAREC1.body. By loading this file from Choreonoid, it is possible to simulate WAREC-1, etc.

Double-arm construction robot
-----------------------------

The Double-arm construction robot model is stored in a subdirectory named DoubleArmV7 in the standard model directory.

This model uses tracks, and two versions are available: a version that performs a simple simulation of the tracks; and a version that uses AGX Dynamics to perform a simulation closer to the real machine. “S” for “simplified” or “A” for “AGX” are appended to the basic model name “DoubleArmV7”:

* simple track version:  DoubleArmV7S.body
* AGX track version: DoubleArmV7A.body

and the files are stored using these names.

The simple track version can be used with Choreonoid’s standard features, and a WRS2018 sample is also available. If you use these, you can try out the WRS2018 simulation even if you do not have an AGX Dynamics license. However, please note that the track movement will be somewhat different from that of the actual machine.

With the AGX track version, the behavior of the tracks is close to that of the actual machine. If you have an AGX Dynamics license, you should use this. This is also used in the competition.

Aizu Spider
-----------

The Aizu Spider robot model is stored in a subdirectory named AizuSpider in the standard model directory.

A total of 6 variations of this model are available. First, with regard to the arm mounted on the robot, three variations are available: with no arm (N), with one arm (S), and with two arms (D). This model is also equipped with tracks, and with regard to the tracks, there is a simple version (S) and an AGX version (A). Combining these, we get the following 6 variations.

* No arm, simple track version: AizuSpiderNS.body
* No arm, AGX track version: AizuSpiderNA.body
* One arm, simple track version: AizuSpiderSS.body
* One arm, AGX track version: AizuSpiderSA.body
* Two arms, simple track version: AizuSpiderDS.body
* Two arms, AGX track version: AizuSpiderDA.body

The arm mounted on this robot is the JACO2 arm made by Kinova, which is an existing product.

Quadcopter
----------

The quadcopter model is stored in a subdirectory named multicopter in the standard model directory under the file name quadcopter.body. The  :doc:`../multicopter/index`  is required in order to perform this model’s flight simulation.

