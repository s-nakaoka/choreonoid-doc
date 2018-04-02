
Performance Tuning
=========================

If the computational speed of the simulation is slow or the behavior seems to be unstable, it is a good idea to make adjustments referring to the following.

Improve computation time performance
-------------------------------------

* Reduce the number of objects (rigid bodies, joints)
* Use primitive shapes (Box, Sphere, Cylinder, etc)
* Avoid to use the triangle mesh shape (collision detection tends to slow)
* Increase time step
* Increase the number of threads
* When direct solver is selected in friction model, increase surface Viscosity
* Use Contact Warmstarting
* Use AMOR


Improve stability of collision response
----------------------------------

* Set mass properties appropriately
* Avoid to use extremely small size shape
* Use primitive shapes (Box, Sphere, Cylinder, etc)
* Avoid to use the triangle mesh shape


Improve stability of controlling (feedback controll)
-----------------------------------------------------

* Set mass properties appropriately
* Position control and velocity controll are more stable than torque control in the physics engine
* The physics engine absorbs the integration error and suppresses divergence