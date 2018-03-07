:orphan:

拘束のコンプライアンスとスプークダンパ
=====================================

| AGXDynamicsは運動方程式と拘束条件を連立させて数値計算を行います。
| 拘束条件を満たしていない場合には何かしらの方法で条件を満たすようにする必要があり、
| AGXDynamicsは粘弾性(バネダンパ)モデルを利用しています。
| ただし、定式化と数値計算の関係で粘弾性モデルを少し変形させており、以下の関係があります。

.. code-block:: text

  F = -Kx - Dv   # Spring Damper model
  F = -1/Cx - Dv # Replace K with compliance 1/C
  CF = -x - CDv  # Multiply C^2 at both sides
  CF = -x - hv   # Merge CD to h

  K = 1/C        # Spring Coefficient [N/m] or [Nm/rad]
  C = 1/K        # Compliance [m/N] or [rad/Nm]

  h = CD = D/K   # Spook Damping [s]
  D = hK = h/C   # Damping Coefficient [Ns/m] or [Nm s/rad]
