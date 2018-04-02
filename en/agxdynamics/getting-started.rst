Getting started
================================

.. contents::
   :local:
   :depth: 1

How to run the AGX Dynamics simulation
---------------------------------------

| The way how simulator with AGXDynamics plugin is the same as :ref:`other plugin simulators<simulation_creation_and_configuration_of_simulator_item>` .
| Create the simulator item "AGXSimulator" and make activation by placing as child of world item.

Creating and setting of AGXSimulator item
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| Create item by "file" - "new" - "AGXSimulator" on main menu.
| After creation, place AGXSimulator item under world item on item tree view.
| AGXSimulator item has properties that other simultor item does not have.
| Please see details in  :doc:`agx-simulator-item` 

How to execute simulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Slect AGXSimulator under item tree view, and press the execution button.

Exclusive model for AGXDynamics plugin.
----------------------------------------

| Some models are exclusive for AGXDynamics Plugin, i.e. crawler model and wire model.
| Please see details from the table of contents at left side.

Samples
-----------------------

The sample project using AGXDynamics is available under choreonoid/samples/AGXDynamics.
Try it.
