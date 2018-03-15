
Installing and building AGX Dynamics plugin
---------------------------------------------

| AGX Dynamics plugin is included in the Choreonoid source code.
| Set the following cmake option **ON** before building Choreonoid.
| You can check the description of building Choreonoid at ref: `build-ubuntu-cmake`.

* **BUILD_AGX_DYNAMICS_PLUGIN**      : AGXDynamicsPlugin - Enable rigid body simulation with AGX Dynamics
* **BUILD_AGX_BODYEXTENSION_PLUGIN** : AGXBodyExtensionPlugin - Including dedicated models such as wire, breakable joint, etc. Only works with AGXDynamicsPlugin.

| Now, let's start building and installing.
| First, move to the Choreonoid source directory and execute cmake and ccmake.

.. code-block:: txt

   cd choreonoid
   cmake .
   ccmake .

Enable following options ON, then execute configure and generate.

* BUILD_AGX_DYNAMICS_PLUGIN             ON
* BUILD_AGX_BODYEXTENSION_PLUGIN        ON

Make sure that CMake Errors are not occurred.
Execute make, make install as follows:

.. code-block:: txt

   make -j4
   make install

.. note::

   Since AGXBodyExtensionPlugin is depeneded on AGXDynamicsPlugin, ccmake will not show the option BUILD_AGX_BODYEXTENSION_PLUGIN when BUILD_AGX_DYNAMICS_PLUGIN is OFF.
   Set BUILD_AGX_DYNAMICS_PLUGIN ON and execute configure once.

.. note::

   When ccmake configure is executed, the AGX Dynamics installation path AGX_DIR is automatically set.
   In case that AGX_DIR is not set, please set it manually. The default path is /opt/Algoryx/AgX-<version>.
