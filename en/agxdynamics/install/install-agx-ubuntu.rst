
Installation of AGX Dynamics(Ubuntu Linux)
================================================

.. contents::
   :local:
   :depth: 1

| This section describes the installation of AGX Dynamics Ubuntu version.
| Before installation, you should download the deb package from the AGX Dynamics download site and locate it in the /var/tmp directory.
| In this description, agx-setup-2.19.1.2-x64-ubuntu-16.04-double.deb is used for explanation.

Installation
----------------------------

| AGX Dynamics is installed by executing the following command. By default it is installed in the /opt/Algoryx/AgX-<version> directory.
| Next, place the AGX license file(agx.lic) in the installation directory so that you can run AGX Dynamics.
| Add the execution command of setup_env.bash in .profile file.
| The script will set environment variables of AGX automatically when you login in to the OS.

.. code-block:: txt

   # Installation
   cd /var/tmp
   sudo dpkg -i agx-setup-2.19.1.2-x64-ubuntu_16.04-double.deb    // Install the AGX Dynamics package
   ls -al /opt/Algoryx/AgX-2.19.1.2                               // Check AGX Dynamics is installed in the /opt/Alogrxy directory

   # Locate the AGX license file
   sudo cp -i agx.lic  /opt/Algoryx/AgX-2.19.1.2

   # Set the environment variables of AGX
   cd ~                                                           // Move to home dir
   cp -p .profile .profile_20171010                               // Backup .profile file for safety
   echo "source /opt/Algoryx/AgX-2.19.1.2/setup_env.bash" >> .profile
   diff .profile .profile_20171010                                // Check diff
   source .profile                                                // Set environment variables on this shell
   env | grep -i agx                                              // Check that AGX_DIR and AGX_BINARY_DIR are sat correctly


Running test
----------------------------

Execute the AGX Dynamics sample as follows to check AGX Dynamics is installed correctly.

.. code-block:: txt

   cd /opt/Algoryx/AgX-2.19.1.1/bin
   ./tutorial_trackedVehicle


.. note::
   On a virtual machine such as VMWare, the application window may not opened and failed.

   .. code-block:: txt

      $ ./tutorial_trackedVehicle
         AGX Library 64Bit AgX-2.19.1.1-81db33e Algoryx(C)
         Tutorial Tracked Vehicle (agxVehicle::Track)
        --------------------------------
      Caught exception: Failed to find window with number: 0
   ..

   In this case please check with the following command. --agxOnly is without graphics, --stopAt 5 is stop simulation after 5 seconds.

   .. code-block:: txt

      ./tutorial_trackedVehicle --agxOnly --stopAt 5
         AGX Library 64Bit AgX-2.19.1.1-81db33e Algoryx(C)
         Tutorial Tracked Vehicle (agxVehicle::Track)
         --------------------------------
      Loading scene took 0.236783 sec
      Stepping 301 timesteps (5.01667 sec simulated time) took 1.69487 sec
