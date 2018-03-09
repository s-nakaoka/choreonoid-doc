
Body file reference manual
===================================

.. contents::
   :local:
   :depth: 2

Summary
-----------

This is a reference manual on the Body format model files (Body files) used by Choreonoid by default.

Model file rules
~~~~~~~~~~~~~~~~~~~~~~~
Each model file defines rules for a single robot or environment model. In order to distinguish these files from normal YAML files (.yaml), the extension is set to .body.

YAML syntax
------------------
For details on YAML syntax, please refer to the `Programmer’s Guide to YAML (Beginner) <http://magazine.rubyist.net/?0009-YAML>`_.

node list
------------

The below types of nodes are defined in the specifications. By assembling these into instances, you can create models. By combining node instances into a hierarchical structure, you can define the format of the model.

Nodes that define the link structure and dynamics/mechanisms parameters are as follows.

* :ref:`body-file-reference-link-node`
* :ref:`body-file-reference-rigid-body-node`
* :ref:`body-file-reference-transform-node`

The below nodes are defined as nodes used to define the shape and display of links.

* :ref:`body-file-reference-shape-node`
* :ref:`body-file-reference-geometry-node`
 * :ref:`body-file-reference-box-node`
 * :ref:`body-file-reference-sphere-node`
 * :ref:`body-file-reference-cylinder-node`
 * :ref:`body-file-reference-capsule-node`
 * :ref:`body-file-reference-cone-node`
 * :ref:`body-file-reference-extrusion-node`
 * :ref:`body-file-reference-elevation-grid-node`
* :ref:`body-file-reference-appearance-node`
* :ref:`body-file-reference-material-node`
* :ref:`body-file-reference-resource-node`

The below nodes are defined as nodes used to define sensors and devices.

* :ref:`body-file-reference-acceleration-sensor-node`
* :ref:`body-file-reference-rate-gyro-sensor-node`
* :ref:`body-file-reference-force-sensor-node`
* :ref:`body-file-reference-camera-node`
* :ref:`body-file-reference-range-sensor-node`
* :ref:`body-file-reference-spot-light-node`

The below node is defined as a node used to define closed link mechanisms.

* :ref:`body-file-reference-extra-joint-node`

The below node is defined as a node used to group other nodes.

* :ref:`body-file-reference-group-node`

The following provides details on each node.

Headers
------------

Placed at the beginning of a file and specifying the format of the model file.

.. list-table:: Header fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - format
   - Set as “ChoreonoidBody.”
 * - formatVersion
   - Specifies the version of the model file format. The current version is 1.0.
 * - angleUnit
   - This is used to specify the units to use for joint angles in the model file. Set as “degree” or “radian.”
 * - name
   - Set the model name.
 * - rootLink
   - Set the root link name.


Link structure, nodes defining dynamics and mechanism parameters
-----------------------------------------------------------------------------

.. _body-file-reference-link-node:

Link nodes
~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: Link node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Link
 * - name
   - The link name. You can specify any value so long as it does not overlap with another value in the model.
 * - parent
   - The parent link. Specified by calling the name of the parent link (the string declared in the name field). This is not used for root links.
 * - translation
   - Location relative to the parent link for the link-local frame. For root links, used as the default position at import.
 * - rotation
   - The orientation relative to the parent link of the link-local frame. The orientation is expressed as four values that correspond to angle of rotation (Axis-Angle format). For root links, used as the default position at import.
 * - jointId
   - The joint ID. Specify an integer value greater than zero. You can specify any value so long as it does not overlap with another value in the model. This need not be specified if the link is not a joint (a root link or where the jointType is fixed), or where you do not intend to access it by ID.
 * - jointType
   - The joint type. Select from fixed, **free** (applies only to root links),  **revolute** (rotating joint), **prismatic** (direct joint),  or **pseudoContinousTrack** (a caterpillar track). 
 * - jointAxis
   - The joint axis. Specify the axis joint as a list containing the three elements of the 3D vector. Use unit vectors for the value here. Where the joint axis corresponds to any of the X, Y, or Z coordinates for the link-local coordinates, or the inverse, you can also declare this using the corresponding letter for that axis (X, Y, or Z, or -X, -Y, or -Z).
 * - jointAngle
   - The initial angle of the joint. Specified in degrees.
 * - jointDisplacement
   - The initial angle of the joint. Specified in radians.
 * - jointRange
   - The range of motion of the joints. Give a list containing the two values of maximum and minimum. Declaring the value as “unlimited” allows for removing range of motion restrictions. Where the minimum and maximum absolute values are the same and they are respectively negative and positive, you can specify just one of these (as a scalar value).
 * - maxJointVelocity
   - Specify the joint rotation/velocity range as a scalar value (>=0). This value defines a negative and positive range. If the jointType is revolute, sets the maximum angular velocity (degrees/sec); otherwise, sets the maximum velocity (m/sec).
 * - jointVelocityRange
   - The range of the joint rotation/velocity. Give a list containing the two values of maximum and minimum. This is given precedence over maxJointVelocity.
 * - rotorInertia
   - The rotor moment of inertia. By default, set to 0.0.
 * - gearRatio
   - The gear ratio. By default, it is set to 1.0. The equivalent rotor moment of inertia is set as gearRatio*gearRatio*rotorInertia.
 * - centerOfMass
   - The position of the center of gravity. Set using link-local coordinates
 * - mass
   - The mass in kg.
 * - inertia
   - The moment of inertia. Given as a list of nine inertia tensors. Due to the symmetry of inertia tensors, you need only list the six elements of the upper triangle.
 * - import
   - Import an aliased node here. import: *defined_alias
 * - elements
   - Give the child nodes that are constituent elements of the link.


.. note::
	The first Link node is treated as the root node for the model.

.. note::
	Rigid body parameters (centerOfMass, mass, inertia) can also be set using the RigidBody node (explained later). In that case, use elements to set the RigidBody node as a child node of the Link node.

.. _body-file-reference-rigid-body-node:

RigidBody nodes
~~~~~~~~~~~~~~~~~~~~~~~

RigidBody nodes define the rigid body parameters of a link.

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: RigidBody node items
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - RigidBody
 * - centerOfMass
   - The position of the center of gravity. Set using link-local coordinates
 * - mass
   - The mass in kg.
 * - inertia
   - The moment of inertia. Given as a list of nine inertia tensors. Due to the symmetry of inertia tensors, you need only list the six elements of the upper triangle.
 * - elements
   - Specify the link shape and sensors using a child node.

.. _body-file-reference-transform-node:

Transform nodes
~~~~~~~~~~~~~~~~~~~~~~~

Control the translation, rotation, and scaling of nodes beneath it.

.. list-table:: Transform node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Transform
 * - translation
   - Location offset
 * - rotation
   - Orientation offset
 * - scale
   - Increase/decrease size
 * - elements
   - Describes a child node subject to conversion.


Nodes used to define link shape and appearance
------------------------------------------------------

.. _body-file-reference-shape-node:

Shape nodes
~~~~~~~~~~~~~~

.. list-table:: Shape node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Shape
 * - geometry
   - Describes the link shape using any :ref:`body-file-reference-geometry-node` .
 * - appearance
   - Describes the link color and texture as an :ref:`body-file-reference-appearance-node` .

.. _body-file-reference-geometry-node:

Geometric shape nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Geometric shapes available are the following nodes: Box, Sphere, Cylinder, Capsule, Cone, Extrusion, ElevationGrid, and IndexedFaceSet.

.. _body-file-reference-box-node:

Box nodes
'''''''''''''''

Box nodes are geometric nodes which set the dimensions of a rectangular figure.

.. list-table:: Box node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Set as “Box.”
 * - size
   - The length, depth, and height of the box.

.. _body-file-reference-sphere-node:

Sphere nodes
'''''''''''''''''

The Sphere node is a geometric node used to describe spherical forms.

.. list-table:: Sphere node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Sphere
 * - radius
   - The radius of the sphere.

.. _body-file-reference-cylinder-node:

Cylinder nodes
''''''''''''''''''''

Cylinder nodes are geometric nodes which set the dimensions of a cylindrical figure.

.. list-table:: Cylinder node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Cylinder
 * - radius
   - radius
 * - height
   - height
 * - bottom
   - true: has base (default)  false: no base
 * - top
   - true: has top (default)  false: no top

.. _body-file-reference-capsule-node:

Capsule nodes
''''''''''''''''''''

Capsule nodes are geometric nodes which set the dimensions of capsules (cylinder + 2 spheres).

.. list-table:: Capsule node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Capsule
 * - radius
   - radius
 * - height
   - height

.. _body-file-reference-cone-node:

Cone nodes
''''''''''''''''

Cone nodes are geometric nodes which specify the dimensions of conical figures.

.. list-table:: Cone node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Cone
 * - radius
   - The radius of the bottom
 * - height
   - height
 * - bottom
   - true: has base (default)  false: no base

.. _body-file-reference-extrusion-node:

Extrusion nodes
'''''''''''''''''''''

Extrusion nodes are geometric nodes which are used to express the dimensions of an extruded form.

.. list-table:: Extrusion node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Extrusion
 * - crossSection
   - | Specified as coordinate points of the cross-section to extrude. Format as:
     | crossSection: [ x0, z0, x1, z1, x2, z2, ・・・, xn, zn ]
     | with X and Z coordinates given. You can include line breaks and spaces.
     | crossSection: [ x0, z0,
     |                 x1, z1,
     |                  ：
 * - spine
   - | Specify, in terms of endpoint coordinates, the straight line section on which to move across the cross-section set with crossSection.
     | spine: [ x0, y0, z0, x1, y1, z1, ・・・, xn, yn, zn ]
 * - orientation
   - The crossSection rotation for each spine point is set using a list of axis-angle parameters (x, y, z, θ). If you specify only one set, the same rotation is applied to the entire spine. If the set is smaller than the number of spines, then no rotation is applied to the remainder; if the set is larger than the number of spines, it is ignored.
 * - scale
   - The scaling factor for each point on the spine of the cross-section specified with crossSection. Set the X axis scaling factor and Z axis scaling factor to correspond to the number of spines. If you specify only one set, the same scaling factor is applied to the entire spine. If you specify fewer than the number of spines, the remainder will receive a scaling factor of 0 and be treated as 1. If you specify more than the number of spines, it will be ignored.
 * - creaseAngle
   - The threshold value used to change the shading using the light source and angle of the normal vector. If the creaseAngle is smaller than zero, smooth shading is used. By default, this value is 0.
 * - beginCap
   - true: cross-section exists for start edge (default) false: no cross-section for start edge
 * - endCap
   - true: cross-section exists for end edge (default) false: no cross-section for end edge

*See: http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Extrusion


.. _body-file-reference-elevation-grid-node:

ElevationGrid nodes
'''''''''''''''''''''''''''''

The ElevationGrid node is a geometric node used to describe terrain forms with height applied for each lattice point on a grid.

.. list-table:: ElevationGrid node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - ElevationGrid
 * - xDimension
   - The number of grids on the X axis
 * - zDimension
   - The number of grids on the Z axis
 * - xSpacing
   - The grid interval on the X axis
 * - zSpacing
   - The grid interval on the Z axis
 * - ccw
   - true: vertex order is counterclockwise false: vertex order is clockwise
 * - creaseAngle
   - The threshold value used to change the shading using the light source and angle of the normal vector. If the creaseAngle is smaller than zero, smooth shading is used. By default, this value is 0.
 * - height
   - Specifies an array indicating the height of each lattice point. Requires correspondence with the number of lattice points (xDimension*zDimension).

*See: http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#ElevationGrid


.. _body-file-reference-IndexedFaceSet-node:

IndexedFaceSet nodes
''''''''''''''''''''''''

The IndexedFaceSet node is a geometric shape node that describes a shape based on polygons created from a list of vertices.

.. list-table:: IndexedFaceSet node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - IndexedFaceSet
 * - coordinate
   - | Specifies vertex coordinates. coordinate: [ x0, y0, z0, x1, y1, z1, ・・・, xn, yn, zn ]
     | Format as above, with a list of X, Y, and Z coordinates.
 * - coordIndex
   - | Specified as a polygon face, with an index applied from 0 to N of the coordinates set with coord. An index of [-1] implies that the current face has ended.
     | The index is given as a list, as: coordIndex: [ 0, 1, 2, 3, -1, 3, 2, 4, 5, -1, ...]. The order of vertices is counterclockwise.
 * - texCoord
   - | Used when applying textures. Given as two-dimensional coordinates used to map a texture to a vertex. Used as:
     | texCoord: [ s0, t0, s1, t1, ・・・, sm, tm ]
     | The bottom left of the texture is (0.0, 0.0), and the upper right is (1.0, 1.0).
 * - texCoordIndex
   - | As with coordIndex, used to select coordinates for textures for each vertex. Must include the same number of indexes as the coordIndex field and include a [-1] value for the surface ending in the same position.
     | If nothing is given for this parameter, it inherits that used for coordIndex.
 * - creaseAngle
   - The threshold value used to change the shading using the light source and angle of the normal vector. If the creaseAngle is smaller than zero, smooth shading is used. By default, this value is 0.
 
*See: http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#IndexedFaceSet


.. _body-file-reference-appearance-node:

Appearance nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: Appearance node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - material
   - Describes the material on the surface of an object as a :ref:`body-file-reference-material-node` .
 * - texture
   - Describes the texture of the surface of an object as a :ref:`body-file-reference-texture-node` .
 * - textureTransform
   - Describes the translation, rotation, and scaling of a texture as a :ref:`body-file-reference-textureTransform-node` .

.. _body-file-reference-material-node:

Material nodes
~~~~~~~~~~~~~~~~~~~~~

.. list-table:: Material node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - ambientIntensity
   - The rate of refraction of ambient light (0.0-1.0)
 * - diffuseColor
   - The diffusion rate (color of objects) per each RBG value (a list ranging from 0.0-1.0 for each RGB value).
 * - emissiveColor
   - The emissive color of the object itself (a list ranging from 0.0-1.0 for each RGB value).
 * - shininess
   - The gleam/shininess (0.0-1.0).
 * - specularColor
   - The rate of specular reflection (highlight color of light) (given as a list ranging from 0.0-1.0 for each RGB value)
 * - transparency
   - Opacity (0: transparent - 1: opaque)

.. _body-file-reference-texture-node:

Texture nodes
~~~~~~~~~~~~~~~~~~~

.. list-table:: Texture node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - url
   - The texture file path.
 * - repeatS
   - Given as a repeat horizontal texture.
 * - repeatT
   - Given as a repeating perpendicular texture.
   
.. _body-file-reference-textureTransform-node:

TextureTransform nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: TextureTransform node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - translation
   - Location offset
 * - rotation
   - Orientation offset
 * - scale
   - Increase/decrease size
 * - center
   - The center point of rotation and scale.

*See: http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#TextureTransform

.. _body-file-reference-resource-node:

Resource nodes
~~~~~~~~~~~~~~~~~~~~~

Load a mesh created in a CAD or another modeling tool.

.. list-table:: Resource node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Resource
 * - uri
   - Link shape and mesh file path
 * - node
   - Give the node name when importing only a specific node from within a mesh file.

.. _body-file-reference-devices:

Nodes defining sensors and devices
----------------------------------------

Device nodes
~~~~~~~~~~~~~~~~~~~~

Display common settings shared among devices.

.. list-table:: Device node common fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - id
   - The device ID.
 * - translation
   - Give the local coordinate system position as an offset value from the parent node coordinate system.
 * - rotation
   - The orientation in the local coordinate system, given as an offset value from the parent node coordinate system ([x, y, z, θ]  vectors: θ rotation around [x, y, z]).

.. note::
  Each sensor node is added below the Link node to which that sensor is applied. For example, if you have attached an accelerometer to the wait of the sample model, you would use the following.

.. code-block:: yaml

    links:
      - 
        name: WAIST
        elements:
          -
            type: AccelerationSensor
            id: 0

.. _body-file-reference-acceleration-sensor-node:

AccelerationSensor nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

AccelerationSensor nodes are defined as 3-axis accelerometers.

.. list-table:: AccelerationSensor node fields
 :widths: 15,85
 :header-rows: 1

 * - Field
   - Details
 * - type
   - AccelerationSensor
 * - maxAcceleration
   - The maximum measurable acceleration. Specify as a list containing the three elements of the 3D vector.

.. _body-file-reference-rate-gyro-sensor-node:

RateGyroSensor nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

RateGyroSensor nodes are defined as 3-axis angular sensors.

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: RateGyroSensor node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - RateGyroSensor
 * - maxAngularVelocity
   - The maximum measurable angular velocity. Specify as a list containing the three elements of the 3D vector.

.. _body-file-reference-force-sensor-node:

ForceSensor nodes
~~~~~~~~~~~~~~~~~~~~~~~~

ForceSensor nodes are defined as force/torque sensors.

.. list-table:: ForceSensor node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - ForceSensor
 * - maxForce
   - The maximum measurable force. Specify as a list containing the three elements of the 3D vector.
 * - maxTorque
   - The maximum measurable torque. Specify as a list containing the three elements of the 3D vector.

.. _body-file-reference-camera-node:

Camera nodes
~~~~~~~~~~~~~~~~~~~~

Camera nodes are defined as visual sensors.

.. list-table:: Camera node fields
 :widths: 30,70
 :header-rows: 1

 * - Key
   - Details
 * - type
   - Camera
 * - format
   - | Specify the type of data to be polled from the sensor.
     | ・"COLOR"  polls color data
     | ・"DEPTH"  polls depth data
     | ・"COLOR_DEPTH"  polls color depth data
     | ・"POINT_CLOUD"  polls the 3D point cloud
     | ・"COLOR_POINT_CLOUD"  polls the color data’s 3D point cloud
 * - on
   - Use true/false to turn the camera on/off
 * - width
   - The image width
 * - height
   - The image height
 * - fieldOfView
   - The camera view angle
 * - nearClipDistance
   - The distance to the nearest clip plane in view
 * - farClipDistance
   - The distance to the furthest clip plane in view
 * - frameRate
   - How many images the camera should output per second

.. note::
    The view orientation is defined as follows. Forward line of sight: negative Z axis position in the local coordinate system.Upward line of sight: positive Y axis position in the local coordinate system.

.. note::
    Internally, when the format is set as COLOR, Camera is used. When set as anything other than COLOR, RangeCamera is used.

.. _body-file-reference-range-sensor-node:

RangeSensor nodes
~~~~~~~~~~~~~~~~~~~~~~~~~

The RangeSensor node is used to define range sensors.

.. list-table:: RangeSensor node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - RangeSensor
 * - on
   - 
 * - yawRange
   - The horizontal angle for the scan distance. Measured as the angle of yawRange, with 0 as a starting point and angles measured in multiples of the yawStep for either side. If there is no horizontal scan functionality in the sensor, this is treated as 0. Given as a multiple of yawStep in the range between 0 and 360.
 * - yawStep
   - The steps (increments) of horizontal angular distance measured during a scan.
 * - pitchRange
   - The perpendicular surface angle when scanning a distance. Measured as the angle of pitchRange, with 0 as a starting point and angles measured in multiples of the pitchStep for either side. If there is no perpendicular scan functionality in the sensor, this is treated as 0. Given as multiples of pitchStep in the range between 0 and 170. (If giving a large value, processing time lengthens and measurement fidelity worsens.)
 * - pitchStep
   - The steps (increments) of perpendicular surface angle measured during a scan.
 * - scanRate
   - The frequency of scans per second (Hz).
 * - minDistance
   - The minimum measurable distance (meters).
 * - maxDistance
   - The maximum measurable distance (meters).

.. note::
   The orientation of the sensor with respect to the link on which it is installed. In the coordinate system, the negative direction on the Z axis is the front orientation of measurement. When scanning, the horizontal measurement plane is XZ, and the perpendicular measurement plane is YZ. This is the same as VisionSensor; if changing a model where this was substituted for VisionSensor, the position and orientation remain the same. The order of rotation when scanning the horizontal and perpendicular is yaw, then pitch.
   
.. _body-file-reference-spot-light-node:

SpotLight nodes
~~~~~~~~~~~~~~~~~~~~~~~

SpotLight nodes define lights.

.. list-table:: SpotLight node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - type
   - SpotLight
 * - on
   - Give true or false to turn the light on or off.
 * - color
   - The light color (give as 0.0-1.0 for each RGB value).
 * - intensity
   - The intensity of brightness (five as 0.0-1.0).
 * - direction
   - The orientation of the light. Give orientation as a list of three elements (3D vectors).
 * - beamWidth
   - The angle of beam width at maximum shininess. By default, this is 90 degrees.
 * - cutOffAngle
   - The angle at which the light is fully shut out. By default, this is 45 degrees.
 * - cutOffExponent
   - Given as a non-negative value. By default, set at 1.0.
 * - attenuation
   - The rate of attenuation. Specified as a list of three non-negative elements.


Nodes defining closed link mechanisms
--------------------------------------------

.. _body-file-reference-extra-joint-node:

ExtraJoint nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ExtraJoint nodes define the mechanism of closed links. They presume that one joint in a closed link is connected with a ball joint, and generate binding force to prevent the two links from separating.

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: ExtraJoint node fields
 :widths: 15,85
 :header-rows: 1

 * - Field
   - Details
 * - link1Name
   - The name of the joint that receives the ball joint.
 * - link2Name
   - The name of the joint with the ball joint.
 * - link1LocalPos
   - link1Name: gives binding position of a joint in terms of the local coordinates for that joint.
 * - link2LocalPos
   - link2Name: gives binding position of a joint in terms of the local coordinates for that joint.
 * - jointType
   - The type of joint binding. For ball, 1 fixed point of attachment. For piston, only acts on the axis given with jointAxis.
 * - jointAxis
   - When the jointType is piston, the direction of motion is given in terms of local coordinates for the link1name joint.


A sample closed link mechanism can be found in share/model/misc/ClosedLinkSample.body.


Nodes use to group other nodes
----------------------------------------

.. _body-file-reference-group-node:

Group nodes
~~~~~~~~~~~~~~~~~~

This is used to group select nodes together.

.. list-table:: Group node fields
 :widths: 15,85
 :header-rows: 1

 * - Key
   - Details
 * - name
   - Group name

.. code-block:: yaml

  (Example)
  elements:
    - &SUBSYSTEM
      type: Group
      name: SUBSYSTEM
      elements:
        -
          (Group 1 element)
        -
          (Group 1 element)
         :

Adding an alias to the group node allows you to, when there is a layout the same as SUBSYSTEM in a separate location, invoke it with the below:

.. code-block:: yaml

  elements: *SUBSYSTEM

.
