:orphan:

Compliance and spook damping of constraints
=============================================

| AGX Dynamics performs numerical calculation of simultaneous equations which have equation of motion and constraints.
| When the constraint condition is not satisfied, it is necessary to satisfy the condition in some way,
| AGX Dynamics uses a viscoelasticity (spring damper) model.
| Notice that, for formulation and numerical calculation, AGX Dynamics transforms the viscoelastic model as follows and uses parameters such as compliance and spook damping.

.. code-block:: text

  F = -Kx - Dv   # Spring Damper model
  F = -1/Cx - Dv # Replace K with compliance 1/C
  CF = -x - CDv  # Multiply C^2 at both sides
  CF = -x - hv   # Merge CD to h

  K = 1/C        # Spring Coefficient [N/m] or [Nm/rad]
  C = 1/K        # Compliance [m/N] or [rad/Nm]

  h = CD = D/K   # Spook Damping [s]
  D = hK = h/C   # Damping Coefficient [Ns/m] or [Nm s/rad]
