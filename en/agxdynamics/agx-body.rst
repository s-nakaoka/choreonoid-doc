Additional parameter for body model
===========================

When user uses AGXDynamics plugin, user can use following additional parameters for his body model.

.. contents::
   :local:
   :depth: 2


How to write
----------------

.. code-block:: txt

  links:
    -
      name: Arm
      jointCompliance: 1e-8
      jointSpookDamping: 0.0333
      jointMotor: true
      jointMotorCompliance: 1e-8
      jointMotorSpookDamping: 0.0333
      jointMotorForceRange: [ -1000, 1000 ]
      jointRangeCompliance: 1e-8
      jointRangeSpookDamping: 0.0333
      jointRangeForceRange: [ -1000, 1000 ]
      jointLock: true
      jointLockCompliance: 1e-8
      jointLockSpookDamping: 0.0333
      jointLockForceRange: [ -1000, 1000 ]
      convexDecomposition: true
      autoSleep: true

  collisionDetection:
    excludeTreeDepth: 3
    excludeLinks: [ ]
    excludeLinksDynamic: [ ]
    excludeLinkGroups:
      -
        name: groupA
        links: [ linkA, linkB, linkC, ... ]
      -
        name: groupB
        links: [ linkZ, linkY, linkX, ... ]
    excludeSelfCollisionLinks: [ linkP ]
    enableAGXWireContact: true
    excludeLinksWireContact: [ linkQ, linkR, ... ]

.. _agx_autosleep:

About parameters
-----------

link
~~~~~~~~~

.. list-table::
  :widths: 10,9,4,4,75
  :header-rows: 1

  * - Parameter
    - Default value
    - unit
    - data type
    - explanation
  * - jointCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - compliance for joint
  * - jointSpookDamping
    - 0.0333
    - s
    - double
    - spook damping for joint
  * - jointMotor
    - false
    - -\
    - bool
    - activation for joint motor
  * - jointMotorCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - compliance for joint motor
  * - jointMotorSpookDamping
    - 0.0333
    - s
    - double
    - scoop damping for joint motor
  * - jointMotorForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - Maximum/Minimum torque for joint motor, limit of torque
  * - jointRangeCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - compliance for limitaion of joint angle
  * - jointRangeSpookDamping
    - 0.0333
    - s
    - double
    - scoop damping for limitation of joint angle
  * - jointRangeForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - Maximum/Minimum torque for limitation of joint anvle, limitation of torque
  * - jointLock
    - false
    - -\
    - bool
    - activation of lock joint
  * - jointLockCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - compliance for lock joint
  * - jointLockSpookDamping
    - 0.0333
    - s
    - double
    - scoop damping for lock joint
  * - jointLockForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - Maximum/Minimum toque for lock joint, limitation of torque
  * - convexDecomposition
    - false
    - -\
    - bool
    - activation/deactivation of convex decomposition by true/false
  * - autoSleep
    - false
    - -\
    - bool
    - activation/disactivation of auto sleep by true/false. It provides the function of removing non-moving solid from the solver, then reduce the calculation amount.property of :doc:`agx-simulator-item` needs to be changed to true.


setting of collision detection
~~~~~~~~~

.. list-table::
  :widths: 15,7,4,6,75
  :header-rows: 1

  * - Parameter
    - Default value
    - unit
    - data type
    - explanation
  * - excludeLinksDynamic
    - \-
    - \-
    - string list
    - disactivate collision of designated link
  * - | excludeLinkGroups:
      | -
      |   name
      |   links
    - \-
    - \-
    - |
      |
      | string
      | string list
    - | disactivate collisions between the links registered in group
      |
      | name of group
      | name of link
  * - excludeSelfCollisionLinks
    - \-
    - \-
    - string list
    - disactivate self-collision of designated link and body.
  * - excludeLinksWireCollision
    - \-
    - \-
    - string list
    - disactivate of collision between designated link and AGXWire.


Convex Decomposition(divide into 凸(convex) object)
---------------------------------

AGX Dynamics has a function to divide tri-mesh into convex object.
Set true in convexDecomposition for link paramaeter, convex decomposition (from tri-mesh) is activated.
It will contribute to improve the perormance of collision detection.

.. note::
  Complex object/shape may be failed.

.. note::
  It may cause different behavior when collides, because the contact point(s) is(are) different between tri-mesh and convex decomposite object.

Samples are available in below directory.

* Project file: chorenoid/sample/AGXDynamics/agxConvexDecomposition.cnoid
* Body file: chorenoid/sample/AGXDynamics/vmark.body

If you run a sample, convex decomposition is activated and the object is devided into some convex objects.

.. image:: images/convexdecomposition.png
   :scale: 70%
