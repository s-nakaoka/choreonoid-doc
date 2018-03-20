
Additional parameters for physical material
=====================================================

When user uses AGXDynamics Plugin the following physical material (properties) can be used.

.. contents::
   :local:
   :depth: 2

Sample
--------

User can find the sample of AGXDynamicsPlugin material from below.
The motion will be changed as per value of parameters.

* choreonoid/samples/AGXDynamics/agxMaterialSample.cnoid

How to set the material
--------------------------
Friction coefficient and restitution coefficient of links in AGXSimulator can be set by below procedure.

1. Describe Material and ContactMaterial in material file.
2. Set Material defined in material file in body file.

How to describe in material file
--------------------------------------

| Material file is the list file that describe the properties like friction and restitution coefficient.
| In this file user can describe the same or different ContactMaterial for each material.
| When user descibes the material name defined in this file in the Body file, the material on the model can be set.
| Material file is loaded by setting in the property of world item.
| The default setting is  ``choreonoid/share/default/materials.yaml`` , and automatically loaded.

.. code-block:: yaml

  materials:
    -
      name: Ground
      roughness: 0.5
      viscosity: 0.0
    -
      name: agxMat5
      density: 1.0

  contactMaterials:
    -
      materials: [ Ground, agxMat5 ]
      youngsModulus: 1.0e5
      restitution: 0.1
      spookDamping: 0.08
      friction: 0.416667
      surfaceViscosity: 1.0e-8
      adhesionForce: 100
      adhesivOverlap: 0.2
      frictionModel: [ cone, direct ]
      contactReductionMode: reduceGeometry
      contactReductionBinResolution: 3


Explanation of material parameters
------------------------------------

Bulk material
~~~~~~~~~~~~~~~

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - parameter
    - default value
    - unit
    - data type
    - explanation
  * - density
    - 1000
    - kg/m3
    - double
    - density that is used for calculating of mass of link, inertia tensor, and center of mass.
  * - youngsModulus
    - 4.0e8
    - Pa
    - double
    - Young's modulus that represents the hardness of link(rigid body). Smaller value may cause penetration between links.
  * - poissonRatio
    - 0.3
    - \-
    - double
    - Poisson's ratio

Surface material
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - parameter
    - default value
    - unit
    - data type
    - explanation
  * - viscosity
    - 0.0
    - \-
    - double
    - viscous restitution.The pair of viscous restitution becomes restitution coefficient.
  * - spookDamping
    - 0.075
    - s
    - double
    - spook damping. Relax the penetration of links(rigid bodies).
  * - roughness
    - 0.5
    - \-
    - double
    - Roughness, Corresponds to friction coefficient.
  * - surfaceViscosity
    - 5e-09
    - \-
    - double
    - Surface viscosity. The pair of each surface viscosity becomes the surfaceViscosity of ContactMaterial. It defines how "wet" a surface is.
  * - adhesionForce
    - 0.0
    - N
    - double
    - adhesion force that determines attactive force acting between two colliding rigid bodies in the normal direction. Acting like adhesive agent.
  * - adhesivOverlap
    - 0.0
    - m
    - double
    - an absolute distance that defines the contact overlap where the adhesive force is active. It becomes active when penetration of link is larger than adhesiveOverlap

.. note::
  If ContactMaterial is set, it is prioritized. Surface material of Material is not used.

.. _agx_wire_material:

Wire Material
~~~~~~~~~~~~~~~~~

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - Parameter
    - Default value
    - unit
    - data type
    - explanation
  * - wireYoungsModulusStretch
    - 6e10
    - Pa
    - double
    - young's modulus for strech
  * - wireSpookDampingStretch
    - 0.075
    - s
    - double
    - spook damping for stretch
  * - wireYoungsModulusBend
    - 6e10
    - Pa
    - double
    - young's modulus for bending.
  * - wireSpookDampingBend
    - 0.075
    - s
    - double
    - spook damping for bending

Explanation of ContactMaterial parameters
------------------------------------------------

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - parameter
    - default value
    - unit
    - data type
    - explanation
  * - youngsModulus
    - 2.0e8
    - Pa
    - double
    - young's modulus
  * - restitution
    - 0.0
    - \-
    - double
    - restitution coefficient. 0:completely inelastic collision、1:completely elastic collision
  * - spookDamping
    - 0.075
    - s
    - double
    - spook damping
  * - friction
    - 0.5
    - \-
    - double
    - friction coefficient
  * - secondaryFriction
    - -1.0
    - \-
    - double
    - secondary friction coeefient. It is activated when friction model is orientedBox and secondaryFriction>=0.
  * - surfaceViscosity
    - 1.0e-8
    - \-
    - double
    - surface viscosity coeeficient. Complaiance for friction constraint.
  * - secondarySurfaceViscosity
    - -1.0
    - \-
    - double
    - secondary surface viscosity coefficient. It is activated when friction model is orientedBox and secondaryFriction>=0.
  * - adhesionForce
    - 0.0
    - N
    - double
    - adhesion force
  * - adhesivOverlap
    - 0.0
    - m
    - double
    - adhesive overlap
  * - frictionModel
    - [ default, default ]
    - \-
    - | string
      | string
    - | friction model : default(cone), cone, box, scaledBox, orientedBox
      | solver    : default(split), split, direct, iterative, iterativeAndDirect

  * - contactReductionMode
    - default
    - \-
    - string
    - the way of contact reduction: default(reduceGeometry), reduceGeometry, reduceALL, reduceNone
  * - contactReductionBinResolution
    - 0
    - \-
    - uint8_t
    - bin resolution(number of bins per dimension) of contact reduction. In case of zero, the parameters of AGXSimulator item are used.
  * - primaryDirection
    - [ 0, 0, 0 ]
    - Unit vector
    - Vec3
    - primary direction of the vector when orientedBox friction model is used.

  * - referenceBodyName
    - \-
    - \-
    - string
    - reference body name when orientedBox friction model is used.
  * - referenceLinkName
    - \-
    - \-
    - string
    - reference link name when orientedBox friction model is used.

.. note::
  AGX Dynamics does not distinguish between dynamic friction coefficient and static friction coefficient. Actually, the difference in value is around 10-20%, and most situations do not have to worry about it.


.. _not_defined_contact_material:

If the ContactMaterial is not defined
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| It is desirable that the all Material pairs are described in ContactMaterial, but it is difficult.
| If the ContactMaterial is not defined, AGX Dynamic compute parameters of ContactMaterial from the parameters of Material as follows.
| When paramters of Material are not set, default paramters are used.

* youngsModulus = (m1.youngsModulus * m2.youngsModulus)/(m1.youngsModulus + m2.youngsModulus)
* restitution = sqrt((1-m1.viscosity) * (1-m2.viscosity))
* spookDamping = max(m1.spookDamping, m2.spookDamping)
* friction = sqrt(m1.roughness * m2.roughness)
* surfaceViscosity = m1.surfaceViscosity + m2.surfaceViscosity
* adhesionForce = m1.adhesionForce + m2.adhesionForce


How to describe the material in the body file
----------------------------------------------

| This section describes how to set material in the body file.
| You can select the types of setting center of gravity, mass and inertia with massType.
| If massType is mass, values of center of mass, mass and inertia which described in the body file are directly used.
| If massType is density, values of center of mass, mass and inertia are automatically calculated by AGX Dynamics.
| The default type is mass.

.. code-block:: yaml

  massType: mass             # Use values of center of mass, mass, inertia which described in the body file
  massType: density          # Calculate values of center of mass, mass, inertia automatically

| You can set the material with material:.
| Default is Default which is defined in the material file choreonoid_dev/share/default/materials.yaml.

.. code-block:: yaml

  material: Default          # Default material
  material: Ground           # Ground material defined in choreonoid_dev/share/default/materials.yaml or user defined material file
  material: useLinkInfo      # Use parameters of material described in the body file

Below are examples of how to describe.

.. note::

  Currently, you could not get or check the result values of center of mass, mass, inertia from the Choreonoid Links and GUI when using massType: density

Conventional description
~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Conventional description of Choreonoid
* Use centerOfMass, mass inertia which are described in the body file
* Parameters of material are set default values except density
* ContactMaterial will be default vs xxxxxx

.. code-block:: yaml

  links:
    -
      name: box1
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]

Using material file (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Use material file to set material including density

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density     # Use density to calculate center of mass, mass, inertia automatically
      material: steel       # Use material steel defined in the material file
      density: 1.0          # If density is written here, use this value. It override density of steel material.

Using conventional description and material file (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* massType: mass <- use center of mass, mass, inertia described in body file
* Other material parameters use the value of the material file

.. code-block:: yaml

  links:
    -
      name: box1
      massType: mass      # Use center of mass, mass, inertia described in body file
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]
      material: steel     # Use steel material described in the material file


Describe all material paramters directly (Not recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* When set material: useLinkInfo, you can describe material parameters in body file
* The values of ContactMaterial are calculated according to :ref:`not_defined_contact_material`

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density
      material: useLinkInfo
      density: 1.0
      youngsModulus:
      poissonRatio:
      viscosity:
      spookDamping:
      roughness:
      surfaceViscosity:
      adhesionForce:
      adhesivOverlap:


Describe everything  (Not recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Every paramters are described in the body file
* You will be confused which parameters are used in the simulation
* So this is absolutely not recommended

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density               # Use center of mass, mass, inertia described in body file
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]
      material: steel                 # Use material defined in the material file
      density: 1.0                    # Use this density for automatic calculate
      youngsModulus:                  # Below are not used
      poissonRatio:
      viscosity:
      spookDamping:
      roughness:
      surfaceViscosity:
      adhesionForce:
      adhesivOverlap:
