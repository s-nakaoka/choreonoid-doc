
Standard body motion file format
==========================================

.. contents::
   :local:
   :depth: 2

.. highlight:: YAML

Overview
--------------

In Choreonoid, “body motions” are the data that represent the motion trajectories of a :doc:`../handling-models/bodymodel` .

* :ref:`simulation-result-item-output`
* The “body motion item” that stores these data is used as the output destination of motions created using the choreography function, among others.

This section describes the standard file format (standard body motion file format) that Choreonoid uses to read and write body motions.

.. _bodymotion-basic-specification:

Basic specifications
-------------------------

The standard body motion file is a text file written in YAML.

The file name will usually have the extension “.seq.” A file in YAML format can also end in “.yaml” or “.yml,” but we use “.seq” to make the file easier to distinguish from other YAML files. “Seq” comes from “Sequence” and is usually pronounced as “seek.”

Because body motions express the movements of an entire model, a standard body motion file contains several kinds of trajectory data:

* Link position and orientation trajectories
* Joint angle trajectories
* Other trajectory data (ZMP orbit, etc.)

The standard body motion format stores these various kinds of trajectory data in one file by using a YAML hierarchical data structure. Each type of motion trajectory data is called a “component.” The file format itself can list any type of motion trajectory component.

For body motions, discretized trajectories are specified in the form of multiple time series of “frames.” Each frame is linked to a time point on the time axis. These time points occur at regular intervals. The two kinds of values that express these intervals are

* Time step
* Frame rate

“Time step” is the time interval between frames. “Frame rate” is the number of frames per unit of time. These two values have a reciprocal relationship. Time step is usually expressed in seconds. The corresponding frame rate is usually expressed in number of frames per second.

In the standard body motion file, the frame interval is specified by using the frame rate.

You can also use the standard body motion file to describe data about frames that do not necessarily occur at fixed intervals. In such cases, no frame rate is specified. Instead, the time point for each individual frame is specified in the file. We will explain this in more detail later.


Basic structure
---------------------

The structure of a standard body motion file is as follows. ::

 # Top node
 type: CompositeSeq
 content: BodyMotion
 formatVersion: 2
 frameRate: 1000
 numFrames: 7261
 components: 
   - 
     # Component 1
     type: MultiSE3Seq
     content: LinkPosition
     numParts: 1
     frameRate: 1000
     numFrames: 7261
     SE3Format: XYZQWQXQYQZ
     frames: 
     # Component 1 frame data
       - frame 1 data
       - frame 2 data
                .
                .
                .
   - 
     # Component 2
     type: MultiValueSeq
     content: JointDisplacement
     numParts: 2
     frameRate: 1000
     numFrames: 7261
     frames: 
     # Component 2 frame data
       - frame 1 data
       - frame 2 data
                .
                .
                .

Any text after # is an explanatory comment that can usually be left out. The above is a schematic representation of frame data. The content you see in this representation is explained below.

Since this file is in YAML format, data that are on the same level have the same indentation. In the above example, the values for each component are written on the indentation level that comes after the top level, and the indentation level needs to remain consistent.


Top node
------------

At the top level of the text, you can use the following keys in mapping nodes.

.. list-table:: Top level node
 :widths: 30, 70
 :header-rows: 1

 * - Key
   - Explanation
 * - type
   - Specify CompositeSeq
 * - content
   - Specify BodyMotion
 * - formatVersion
   - The version of the format. Set this to 2
 * - frameRate
   - Specify the basic frame rate (number of frames/sec) for the whole operation
 * - numFrames
   - Specify the total number of frames for the whole operation
 * - components
   - Specify components in a sequence format

“Type” and “content” express fixed values.

The purpose of formatVersion is to ensure that even when the description format changes in the future, data written in the old format can still be read. The current version described in this document is 2, so we specify 2 here. If formatVersion is set to 1 or not specified, the file will be interpreted as being in the old format. We will not explain the old format here.

“FrameRate” and “numFrames” describe values that apply to the whole operation. You can also specify values individually for each component, but it is better to set uniform values that apply to the whole operation. Also, if you specify values here, there is no need to specify them for each individual component. “NumFrames” is provided for informational purposes only. The frame number values that are specified for each component represent the actual number of frames.

Under components, you describe the components that express actual motion trajectory data. You can describe components that express various kinds of motion trajectories in a sequence format.

Component nodes
--------------------

One type of motion trajectory data is described per component node. The keys shown below can be used in all components.

.. list-table:: Shared across all component nodes
 :widths: 30, 70
 :header-rows: 1

 * - Key
   - Explanation
 * - type
   - Specifies data for a motion trajectory as a character string
 * - content
   - Specifies the purpose of the data as a character string
 * - numParts
   - Number of elements in one frame. Available for Multi-type data formats
 * - frameRate
   - The frame rate. If nothing is set here, the value specified in the top node is used
 * - numFrames
   - Specifies the number of frames
 * - frames
   - Lists frame data as a YAML sequence

“Type” specifies the data type and “content” the purpose of the data. Both keys are expressed as a character string. Currently, you can use the following types.

.. list-table:: Component types
 :widths: 15, 50, 35
 :header-rows: 1

 * - type
   - Data type
   - Example
 * - MultiValueSeq
   - Data for a time series of frames consisting of multiple floating point number values
   - JointDisplacement (joint displacement trajectory)
 * - MultiSE3Seq
   - Data for a time series of frames consisting of multiple SE(3) values (position and orientation, each in three-dimensional space)
   - LinkPosition (link position and orientation trajectory)
 * - Vector3Seq
   - Data for a time series of frames consisting of the value of a single three-dimensional vector
   - ZMP (zero moment point trajectory)

If frameRate is specified in the top node, you should match the value to that value. Alternatively, you can skip setting values in individual components if these values are already specified in the top node. If you do not set values in individual components, the values from the top node will be used.

NumFrames relates to the top node in the same way as frameRate does. However, the actual number of frames is determined by the number set in the frame number data under “frames.” NumFrames is used only for informational purposes.

The details of each type and content are explained below.

.. _bodymotion-multivalueseq-type:

MultiValueSeq type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Data for a time series of frames consisting of multiple floating point number values. You can think of the data for each frame as consisting of multiple scalar values or the value of a single multidimensional vector. Of course, these both represent the same thing.

Joint angle trajectories are a specific application for this data type. When looking at joint angle trajectories, specify “JointDisplacement” as content. We use displacement instead of angle because some joints are linear joints, not rotary joints. JointDisplacement can apply to both, and “joint displacement” as we use it below includes joint angles.

For now, the only MultiValueSeq-type content that is supported by body motions is JointDisplacement. However, the file format can store any kind of content. When loading something as a body motion in Choreonoid, components other than JointDisplacement will be ignored, but you can still use that other content with other software.

The MultiValuseSeq type starts with “Multi,” meaning that “numParts” will be enabled in the component node. In this case, you need to specify the number of elements (number of dimensions) per frame. When used as a joint displacement trajectory, numParts specifies the number of joints.

Under “frames,” each frame is described as one YAML sequence that includes individual numeric values for numParts. These values are arranged in order of joint ID. Angles of rotary joints are expressed in radians. Angles of linear joints are expressed in meters.

Here is an example of how this component is described.

.. code-block:: yaml
 :dedent: 0

   - 
     type: MultiValueSeq
     content: JointDisplacement
     numParts: 2
     frameRate: 100
     numFrames: 100
     frames: 
       - [ 0.0,  0.0  ]
       - [ 0.01, 0.01 ]
       - [ 0.01, 0.02 ]
       - [ 0.02, 0.03 ]
       - [ 0.02, 0.04 ]
               .
               .
               .

This is an example that describes two joints. Only data for the first five frames are shown. In a real file, values for 100 frames would be specified under numFrames.

.. _bodymotion-multise3seq-type:

MultiSE3Seq type
~~~~~~~~~~~~~~~~~~~~~~~~~~

Data for a time series of frames consisting of multiple SE(3) values. The SE(3) values express position and orientation (rotation), each in three-dimensional space.

Link position and orientation trajectories are a specific application for this data type. When looking at link position and orientation trajectories, specify “LinkPosition” as content.

For a single-link model, you need this type of trajectory data to represent the model’s movement. For a multi-link model, you can use JointDisplacement data to represent movement. However, if you want to represent the movement of the entire model, you still need the position and orientation trajectory of the root link. That is why body motions normally include data about the root link’s position and orientation trajectory.

The number of links that are actually included in one frame is specified by numParts, like with MultiValueSeq-type components. Links are arranged in link index order (the order in which depth-first search traverses the link tree). Normally, the first element corresponds to the root link.

SE(3) includes values for six dimensions that express position and orientation. There are various ways to express the three dimensions of orientation, including rotation matrix, quaternions, and roll-pitch-yaw. You also need to decide how to arrange those elements. For MultiSE3Seq-type components, you can set this order using the “SE3Format” key. Here are the symbols you can use to do this.

.. list-table:: SE3Format types
 :widths: 20, 80
 :header-rows: 1

 * - Symbol
   - Description
 * - XYZQWQXQYQZ
   - Describes orientation in quaternions. After X, Y, Z, which indicate position, quaternion values are given as W, X, Y, Z
 * - XYZQXQYQZQW
   - Describes orientation in quaternions, as with XYZQWQXQYQZ, but the order of the quaternion values is X, Y, Z, W
 * - XYZRPY
   - Describes orientation in roll-pitch-yaw format. After X, Y, and Z, which indicate position, orientation values are given as R, P, Y

In both cases, one SE(3) value is described as one YAML sequence. The standard format is “XYZQWQXQYQZ.” To give an example of this format, if the position (X, Y, Z) is (1, 2, 3) and the orientation quaternions (W, X, Y, Z) are (1, 0, 0, 0), the values are specified as follows: ::

 [ 1, 2, 3, 1, 0, 0, 0 ]

These kinds of SE(3) values are also arranged as a YAML sequence for numParts under “frames.”

Here is an example of how this component is described.

.. code-block:: yaml
 :dedent: 0

  - 
    type: MultiSE3Seq
    content: LinkPosition
    numParts: 1
    frameRate: 100
    numFrames: 100
    SE3Format: XYZQWQXQYQZ
    frames: 
      - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
      - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
      - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
      - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
      - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
                 .
                 .
                 .

This example shows that even when the value of numParts is 1, the YAML sequence for each frame is double-nested. When the value of numParts is 2 or more, multiple SE(3) values ​​are specified as follows for each frame. ::

- [ [ X1, Y1, Z1, QW1, QX1, QY1, QZ1 ], [ X2, Y2, Z2, QW2, QX2, QY2, QZ2 ], ... , [ Xn, Yn, Zn, QWn, QXn, QYn, QZn ] ]

*Here, labels such as Xn represent numerical values ​​that correspond to each element of the nth SE(3) value. “...” is where the third to n-1th SE(3) values ​go.

Vector3Seq type
~~~~~~~~~~~~~~~~~~~~~~

Data for a time series data of frames consisting of the value of a single three-dimensional vector.

Specific applications for this data type include a center of gravity trajectory, a zero moment point (ZMP) trajectory, and so on.

ZMP is currently officially supported by body motions. Specify “content” as “ZMP.” Also, if the “isRootRelative” key is set to “true,” the coordinate system becomes relative to the root link. If this key is not specified or set to “false,” the values specified in the global coordinate system will be used.

Because only one value can be given per frame for this type, numParts cannot be used.

Here is an example of how this component is described.

.. code-block:: yaml
 :dedent: 0

  - 
    type: Vector3Seq
    content: ZMP
    frameRate: 100
    numFrames: 100
    frames: 
      - [ 0.0, 0.0,   0.0 ]
      - [ 0.0, 0.001, 0.0 ]
      - [ 0.0, 0.002, 0.0 ]
      - [ 0.0, 0.003, 0.0 ]
      - [ 0.0, 0.004, 0.0 ]
               .
               .
               .

Combining a link position and orientation trajectory and a joint displacement trajectory
------------------------------------------------------------------------------------------------

Link position/orientation trajectories (MultiSE3Seq-type LinkPosition data) and joint displacement trajectories (MultiValueSeq-type JointDisplacement data) form the basis of a model’s movements.

In the case of a single-link rigid-body model, there are no joints, so all you need is a link position/orientation trajectory with numParts set to 1. For a multi-link model with joints, however, you need to properly combine link position/orientation trajectories and joint displacement trajectories to express the model’s movements. You can combine these trajectories in the following formats.

1. Link position/orientation trajectories for all links
2. Link position/orientation trajectory for the root link + joint displacement trajectories for all joints
3. Link position/orientation trajectories for all links + joint displacement trajectories for all joints

In format 1, all movements are expressed with link position/orientation data. This allows you to express the model’s full range of movements when all links are rigid-body links.

In format 2, link position/orientation values are provided only for the root link, and joint displacement trajectories for all joints are provided on top of that. This format allows you to get link orientation/position data for other links besides the root link through forward kinematics calculations that make use of the joint displacement values. This is the standard format for expressing a robot’s range of movements. One advantage of this format is that it requires a much smaller volume of data than format 1. This is because you need six-dimensional SE(3) values to represent link position/orientation values for a single link, while you can represent one joint with only a one-dimensional floating point number. Also, in robotics, people often want to access joint displacement values. This format provides such data directly. The downside of this format is that sometimes, the link position/orientation values obtained through forward kinematics calculations do not correspond to the position and orientation seen in an actual robot or simulation result. Sometimes the rigidity of links and joints is insufficient in an actual robot. Even in simulations, people sometimes try to recreate that effect. The calculation methods of the joint constraints may also result in some or many points of divergence.

In format 3, there can be no such link position/orientation-related divergences. This format also has the advantage that joint displacement values are directly accessible. However, this format involves the largest amount of data of all three formats.

A standard body motion file supports all of the above combinations as formats. You should make sure to describe the data in the format that most suits your needs while taking the advantages and disadvantages of each format into consideration.

How to describe variable inter-frame interval data
--------------------------------------------------------------

As described in the :ref:`bodymotion-basic-specification` , body motions in Choreonoid assume that all frames come at time points that occur at regular intervals.

However, depending on the data about a robot’s actions, arranging frames at regular time intervals is sometimes not appropriate. For example, when the state of a robot is output to a log and recorded, these logs may not always be output at regular intervals. Depending on the control or communications processing conditions of a robot's computer, that computer may not be able to dedicate time to processing logs. It is not unusual for logs to be output at variable time intervals as a result of such conditions. Alternatively, if the robot remains stationary for a long period, it would be wasteful to keep retaining data at frequent time intervals.

It is common to add a time stamp to each individual frame instead of keeping the frame intervals constant. Here, we will call this data “variable inter-frame interval data” or “time-specified frame data.”

It is also possible to describe data of this format in the standard body motion file.

To do this, add the following in a node: ::

 hasFrameTime: true

As with other parameters, you can specify this information in the node of each individual component or the top node. If you do the latter, you no longer need to specify the information in the nodes of individual components.

If you add this parameter, a numerical value corresponding to the time of the frame will be added before the data for each frame.

For example, adding this parameter to a :ref:`bodymotion-multivalueseq-type` component will have this result:

.. code-block:: yaml
 :dedent: 0

   - 
     type: MultiValueSeq
     content: JointDisplacement
     numParts: 2
     numFrames: 100
     hasFrameTime: true
     frames: 
       - [ 0.0, 0.0,  0.0  ]
       - [ 0.1, 0.01, 0.01 ]
       - [ 0.3, 0.01, 0.02 ]
       - [ 0.4, 0.02, 0.03 ]
       - [ 0.7, 2,    0.04 ]
               .
               .
               .

In this example, the first numerical value that appears for each frame under “frames” expresses the time. In this case, the times are set at 0.0, 0.1, 0.3, 0.4, 0.7. In this way, you can specify time points that do not have regular intervals. However, for each frame, you must make sure to specify a time value that is higher than that of the previous frame. This format does not support a description method that lets time go backward.

If you add this parameter, every frame will have three numerical values associated with it. However, note that the number of frame elements will always be the 2 set in numParts.

Adding this parameter to the example we provided for a :ref:`bodymotion-multise3seq-type` component will have this result:

.. code-block:: yaml
 :dedent: 0

  - 
    type: MultiSE3Seq
    content: LinkPosition
    numParts: 1
    numFrames: 100
    hasFrameTime: true
    SE3Format: XYZQWQXQYQZ
    frames: 
      - [ 0.0, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
      - [ 0.1, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
      - [ 0.3, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
      - [ 0.4, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
      - [ 0.7, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
                 .
                 .
                 .

In this case, the SE(3) values for each frame form one sequence. Note that the time value for each frame is written one level above this sequence of SE(3) values.

Importing variable inter-frame interval data
--------------------------------------------------

As we already mentioned and as stated in the :ref:`bodymotion-basic-specification` , Choreonoid 's body motions are designed to work with fixed inter-frame interval data and cannot store variable inter-frame interval data. At present, Choreonoid also does not support any other data structures that support variable inter-frame interval data. This means that variable inter-frame interval data cannot be read directly into Choreonoid.

However, it is possible to import files with variable inter-frame interval data into Choreonoid as files with fixed inter-frame interval data. During import, each frame in the source file is associated with the frame in the destination file that is closest to the original frame’s time point, and that value is maintained up to the next frame in the destination file.

.. note:: Interpolation between frames would probably result in an import with a smoother motion trajectory, but such a function is not available for now. You can only associate the values from the data in the source file with the corresponding frames in the destination file.

An interpolation function would require first determining the frame rate of the destination file. Choreonoid’s interface currently does not offer a function that would allow you to set that frame rate. The frameRate value specified in the body motion file is used instead. Please specify your desired frameRate value in the file. You only need to do this once, in the top node.

The import itself can be done in the same way as loading a normal body motion file. In the main menu, choose “File” - “Load” and select “Body Motion.” You can then select the file you want to import at the top of the file dialog. If this file has variable inter-frame interval data, it will automatically be processed as an imported file.


About description styles
---------------------------------------

There are two types of writing formats in YAML: the block style and the flow style. When describing the hierarchical structure, the block style uses indentation, and the flow style uses braces “{}” (for mapping) and square brackets “[]” (for sequences).

Choreonoid’s file format can use both formats or a combination of them. In general, however, files are written as in the examples above. In short, flow style is used to describe data for individual frames while block style is used for other parts. We believe that writing the data like this makes files look orderly and easy to read. Output files from Choreonoid will also look like this.

Note that YAML is a superset of the JSON format, which means that if you describe everything in flow style, the resulting file will be in JSON format. Please do this if you want to treat motion data as a JSON file. (However, note that JSON does not seem to allow comments.)

Sample files
----------------

In “motion/SR1” of the share directory (see  :doc:`../install/directories`), you will find several standard body motion files for the SR1 model. These are read from the controller and used as motion pattern data in samples such as “SR1Walk.cnoid” and “SR1 WalkinHouse.cnoid.”

You can also see what a standard body motion file is like by saving a body motion item generated as the :ref:`simulation-result-item-output` . When the generated body motion item is set to the selected state, go to the main menu and choose “File” - “Save Selected Item as.” Use the file saving dialog to save the file.