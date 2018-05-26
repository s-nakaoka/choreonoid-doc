
How to build the OpenRTM plugin
----------------------------------------

| The OpenRTM plug-in is included in the Choreonoid source code.
| You can build by setting the following option to ** ON ** with cmake option of ref: `build-ubuntu-cmake` which is done before building Choreonoid.

* **BUILD_OPENRTM_PLUGIN**    : OpenRTM Plugin - Plug-in for linking with OpenRTM-aist
* **BUILD_CORBA_PLUGIN**      : CORBA Plugin - Plug-in for using CORBA communication.Used to communicate with the RT component.
* **ENABLE_CORBA**            : CORBA ON/OFF - Options for enabling CORBA.

| Also, set the following options to ** ON ** as necessary.

* **BUILD_OPENRTM_SAMPLES**   : Sample for OpenRTM plug-in - Sample RT component running on OpenRTM-aist

