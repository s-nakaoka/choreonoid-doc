
AGXSimulator item
=======================

The following properties can be used in AGXSimulator item.

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 10,9,4,4,75
  :header-rows: 1

  * - Parameter
    - Default Value
    - Unit
    - Data type
    - Explanation
  * - NumThreads
    - 1
    - thread
    - unsigned int
    - The number of thread AGX Dynamics uses, which activate the function of parallerization for solver and collision detection. It can be confimed by checking CPU usage via top command etc.
  * - ContactReduction
    - true
    - \-
    - bool
    - activate/disable for contact reduction function by true/false. The benefit of ContactReduction is reduction of calcuration by reduction of unnecessary contacts.
  * - ContactReductionBinResolution
    - 3
    - piece
    - unsigned int
    - number of bin for contact redcution. (1-10). It is used for 6-dimensional binning algoruthm.
  * - ContactReductionThreshold
    - 12
    - piece
    - unsigned int
    - threshold for starting contact reduction. Once the number of contacts are more than designated threshold, contact reduction is activated.
  * - ContactWarmstarting
    - false
    - \-
    - bool
    - If the status of contact is the same as the last step, using the soution of the last solver, the calculation can be converged quickly.
  * - AutoSleep
    - false
    - \-
    - bool
    - Automatic sleeping function for not-moving rigid body from the solver, which contributes to reducing calculation by true/faulse. Each link needs to be set autoSleep.Details in  :doc:`agx-body` .
  * - SaveToAGXFileOnStart
    - false
    - \-
    - bool
    - Save the simulation in .agx file format when simulation starts. The stored directry is the one that executable binary is saved or current directry when executed. It can be used for debag of AGXDynamics and performance confirmation.
