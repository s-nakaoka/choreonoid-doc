
AGXSimulator Item
=======================

The following properties can be used in AGXSimulator Item.

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
    - The number of thread AGX Dynamics uses, which activate the function of parallelization for solver and collision detection. It can be confimed by checking CPU usage via top command etc.
  * - ContactReduction
    - true
    - \-
    - bool
    - activate/disable for contact reduction function by true/false. The benefit of ContactReduction is reduction of calcuration by reduction of unnecessary contacts.
  * - ContactReductionBinResolution
    - 3
    - piece
    - unsigned int
    - number of bin for contact reduction. (1-10). It is used for 6-dimensional binning algorithm.
  * - ContactReductionThreshold
    - 12
    - piece
    - unsigned int
    - threshold for starting contact reduction. Once the number of contacts are more than designated threshold, contact reduction is activated.
  * - ContactWarmstarting
    - false
    - \-
    - bool
    - If the status of contact is the same as the last step, using the solution of the last solver, the calculation can be converged quickly.
  * - AMOR
    - false
    - \-
    - bool
    - Merge the relatively resting rigid bodies together and reduce the amount of solver calculation. Specify true or false. Each link needs to be set AMOR. Details in to :doc:`agx-body`.
  * - AutoSleep(deprecated)
    - false
    - \-
    - bool
    - Automatic sleeping function for resting rigid body from the solver, which contributes to reducing calculation by true/faulse. Each link needs to be set autoSleep. Details in :doc:`agx-body` .
  * - SaveToAGXFileOnStart
    - false
    - \-
    - bool
    - Save the simulation in .agx file format when simulation starts. The stored directry is the one that executable binary is saved or current directry when executed. It can be used for debugging of AGXDynamics and performance checking.
