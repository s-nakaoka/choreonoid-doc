Switching 3D rendering engine
=============================

.. contents::
   :local:
   :depth: 1

.. highlight:: sh

3D rendering engine
-------------------

Choreonoid has a “rendering engine” for drawing robot and environment models as three-dimensional computer graphics (3D-CG).

The following two rendering engines are currently available.

1. OpenGL fixed shader rendering engine
2. GLSL rendering engine

No.1 uses the old rendering API of OpenGL. This API was developed more than 20 years ago, so it can’t be used for sophisticated rendering. But it has the advantage of being usable in many environments, such as an old PC, slow GPU, or when running on a virtual machine.

No.2 uses the new OpenGL API and uses the GLSL shading language that allows for high-quality rendering. Rendering speed is also increased. Currently, it can generally be used with a typical GPU, but it may not work perfectly if the GPU or OS is slightly old or if the GPU driver is an open source version. And it may not work well on a virtual machine.

Up to now, Choreonoid has used No.1 as standard, but we want to move to No.2 in future. As a provisional measure, No.1 is currently used by default, and it can be switched to No.2 using the environment variable.

Basically, it is better to use No.2 when the environment allows it to work normally. However, if you encounter problems such as Choreonoid not launching or issues with rendering, you should use No.1.

How to switch
-------------

Use the CNOID_USE_GLSL environment variable to switch between rendering engines.

If this variable is set to 1, the GLSL rendering engine will be used. If you want to switch back to the fixed shader rendering engine, delete the environment variable setting.

On Ubuntu
~~~~~~~~~

When Choreonoid is launched from the command line, you can use commands such as ::

 CNOID_USE_GLSL=1 choreonoid ...

to switch the rendering engine to GLSL. In this case, you can select the rendering engine every time you launch.

Alternatively, if you set ::

 export CNOID_USE_GLSL=1

in advance, you can use the GLSL rendering engine when you later launch Choreonoid.

If this is stated in .profile in the home directory, it doesn’t need to be set each time.

On Windows
~~~~~~~~~~

On Windows, you should be able to use the command prompt to switch in the same way as described above.

But since the command prompt is not widely used on Windows, it is normally done using system settings.

For details on how to set environment variables on Windows, refer to the explanation about the :ref:`build_windows_openrtm_plugin` in  :doc:`build-windows`. 

