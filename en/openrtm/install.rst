
How to build the OpenRTM plugin
----------------------------------------

| The OpenRTM plug-in is included in the Choreonoid source code.
| You can build by setting the following option to ** ON ** with cmake option of ref: `build-ubuntu-cmake` which is done before building Choreonoid.

* **BUILD_OPENRTM_PLUGIN**    : OpenRTM Plugin - Plug-in for linking with OpenRTM-aist
* **BUILD_CORBA_PLUGIN**      : CORBA Plugin - Plug-in for using CORBA communication.Used to communicate with the RT component.
* **ENABLE_CORBA**            : CORBA ON/OFF - Options for enabling CORBA.

| Also, set the following options to ** ON ** as necessary.

* **BUILD_OPENRTM_SAMPLES**   : Sample for OpenRTM plug-in - Sample RT component running on OpenRTM-aist

| I will explain the details of build and installation method.
| First move to the choreonoid source directory and execute the command until ccmake.

.. code-block:: txt

   cd choreonoid
   cmake .
   ccmake .

Enable the following options in ccmake and execute configure, generate.

* BUILD_CORBA_PLUGIN      ON
* BUILD_OPENRTM_PLUGIN    ON
* ENABLE_CORBA            ON
* BUILD_OPENRTM_SAMPLES   ON (ƒIƒvƒVƒ‡ƒ“)

Make sure Cmake Error is not on.Then, execute make, make install, build and install as follows.

.. code-block:: txt

   make -j4
   make install

.. note::

   When ccmake configure is executed, OpenRTM-aist's path (OPENRTM_DIR) is automatically set.If it is not set, please set it manually.

