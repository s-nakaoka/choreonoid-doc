Sample controller
=================

In the task simulation samples, controllers to operate the robots with the gamepad are available. Here we will explain how to operate the different controllers.

.. contents::
   :local:

Supported gamepads
^^^^^^^^^^^^^^^^^^
The gamepads currently supported are listed below.

* Sony PlayStation 4 gamepad (Dualshock4)
* Sony PlayStation 3 gamepad (Dualshock3)
* Logicool F310
* Microsoft Xbox controller
* Microsoft Xbox 360 controller

It’s okay to use one of the above if you already have one, but if you need to purchase a new one, we recommend the PlayStation 4 gamepad (Dualshock 4). The Dualshock 4 is easily obtainable, has excellent operability and durability, and it is also used in the development of the competition tasks.

.. note:: A separate Micro USB cable is required in order to use the Dualshock 3 or 4 with a PC. Alternatively, the Dualshock 4 can be used with a `USB wireless adapter <https://support.playstation.com/s/article/DUALSHOCK-4-USB-Wireless-Adapter?language=en_US>`_ . We have not looked into connecting it directly using Bluetooth.

Connect the gamepad to the PC to be used for the simulation.

.. _wrs_sample_controller_aizu_spider:

Operating the Aizu Spider
^^^^^^^^^^^^^^^^^^^^^^^^^

For the Aizu Spider, the configuration uses the main Aizu Spider body as the chassis, with between 0 and 2 JACO2 arms attached.

To operate the gamepad, first switch between the operation targets using the logo Button (PS button). At first the robot chassis is the target, and each time you push the button, it switches between

chassis - arm 1 - arm 2 - chassis - ...

.

The operation of the chassis is as follows.

.. list-table::
 :widths: 10, 10

 * - Direction keys
   - tracks
 * - Left stick
   - tracks
 * - Right stick
   - raise or lower all flippers
 * - L2 button + right stick
   - raise or lower front left flipper
 * - R2 button + right stick
   - raise or lower front right flipper
 * - L1 button + right stick
   - raise or lower rear left flipper
 * - R2 button + right stick
   - raise or lower rear right flipper
 * - Right stick button (press right stick)
   - Align the positions of all the flippers

| ※ The direction keys operate the tracks regardless of what the operation target is.
| ※ L1, L2, R1, R2 can be used in any combination.  For example, if you hold down both L1 and L2 and operate the right stick, you can operate the two left flippers at the same time.

The operation of the arm(s) is as follows.

.. list-table::
 :widths: 10, 10

 * - Left stick horizontal axis
   - Arm first joint
 * - Left stick vertical axis
   - Arm second joint
 * - Right stick vertical axis
   - Arm third joint
 * - Right stick horizontal axis
   - Arm fourth joint
 * - X, B button (square、circle buttons)
   - Arm fifth joint
 * - L1, R2 buttons
   - Arm sixth joint
 * - L2, R2 triggers
   - Open and close the hand

.. _wrs_sample_controller_doublearmv7:

Operating the double-arm construction robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The method for operating the double-arm construction robot (DoubleArmV7) using a gamepad is described below.

.. list-table::
 :widths: 10, 10

 * - Direction keys
   - tracks
 * - L1 button + left stick
   - tracks
 * - L1 button + right stick
   - tracks
 * - Left stick horizontal axis
   - Arm base yaw axis
 * - Right stick horizontal axis
   - Arm first joint yaw axis
 * - Left stick vertical axis
   - Arm first joint pitch axis
 * - Right stick vertical axis
   - Arm second joint pitch axis
 * - Y, A buttons (triangle, cross buttons)
   - End effector pitch axis
 * - X, B button (square、circle buttons)
   - End effector yaw axis
 * - L1 button + X button, B button (square, circle buttons)
   - End effector roll axis
 * - R1 button, R2 trigger
   - End effector open and close
 * - Logo button (PS button)
   - Switch the operation target arm

The L1 button is used to switch some of the stick and button operation targets. Where you see “L1 button + ...”, hold down the L1 button and operate the “...” part.

Additional notes
^^^^^^^^^^^^^^^^

The manual operation using the game controller that we have introduced here provides the minimum necessary functionality for testing. It is very low level and incapable of operating the inverse kinematics of the hand. There is no need at all to use the same operation method in the actual competition. In the actual competition, rather than competing based on manual operation skills, we would like participants to construct an efficient operation interface and improve the autonomy of the robots, so as to compete based on their robots’ ability to carry out tasks.
