
Directory Structure
================

Choreonoid is installed approximately in the following directory structure. ::

 + installation destination
   + bin
   + lib
     + choreonoid-x.x
   + share
     + choreonoid-x.x
       + project
       + model

In case of Windows: ::

 + installation destination
   + bin
   + lib
     + choreonoid-x.x
   + share
     + project
     + model


"x.x" part of "choreonoid-x.x" represents the version of Choreonoid. Actually, it will be a number like "1.5".

The content of each of the directories is as follows:

**bin**
 It contains the execution file.

**lib**
 It contains the libraries.

**lib/choreonoid-x.x**
 It contains plug-in files. Any other file related to the system execution will be placed in a sub-folder for each type.

**share/choreonoid-x.x or share**
 Different data files like documents and samples will be placed here. This directory is called **"share directory"** . The actual files will be placed in sub-folders like "project" and "model" according to their type.


Note that the share directory has the same folder in the source archive, too, and it contains sample data and other. You may use it for using the sample. In case of building from source codes in Linux, bin and lib directories will be generated in the build directory and the binary files that are generated there are also executable.
