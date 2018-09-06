ボディモデルの追加パラメータ
============================

AGXDynamicsプラグインを利用の際には、ボディモデルについて下記のパラメータを追加で利用することができます。

.. contents::
   :local:
   :depth: 2

記述方法
--------

.. code-block:: text

  links:
    -
      name: Arm
      jointCompliance: 1e-8
      jointSpookDamping: 0.0333
      jointMotor: true
      jointMotorCompliance: 1e-8
      jointMotorSpookDamping: 0.0333
      jointMotorForceRange: [ -1000, 1000 ]
      jointRangeCompliance: 1e-8
      jointRangeSpookDamping: 0.0333
      jointRangeForceRange: [ -1000, 1000 ]
      jointLock: true
      jointLockCompliance: 1e-8
      jointLockSpookDamping: 0.0333
      jointLockForceRange: [ -1000, 1000 ]
      convexDecomposition: true
      AMOR: true
      autoSleep: true

  collisionDetection:
    excludeTreeDepth: 3
    excludeLinks: [ ]
    excludeLinksDynamic: [ ]
    excludeLinkGroups:
      -
        name: groupA
        links: [ linkA, linkB, linkC, ... ]
      -
        name: groupB
        links: [ linkZ, linkY, linkX, ... ]
    excludeSelfCollisionLinks: [ linkP ]
    enableAGXWireContact: true
    excludeLinksWireCollision: [ linkQ, linkR, ... ]

.. _agx_autosleep:

パラメータの説明
----------------

リンク
~~~~~~

.. list-table::
  :widths: 10,9,4,4,75
  :header-rows: 1

  * - パラメータ
    - デフォルト値
    - 単位
    - 型
    - 意味
  * - jointCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - 関節のコンプライアンス。小さくすると外れにくく、大きくすると外れやすくなります。
  * - jointSpookDamping
    - 0.0333
    - s
    - double
    - 関節のスプークダンパ
  * - jointMotor
    - false
    - -\
    - bool
    - 関節のモーターの有効化。ActuationModeがJOINT_TORQUEまたはJOINT_VELOCITYの時には自動的に有効になります。
  * - jointMotorCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - 関節のモーターのコンプライアンス。速度指定制御に利用します。小さくすると目標速度に到達するように大出力をします。大きくすると外力(重力や接触力)に抵抗できなくなり、目標速度に到達しなくなります。
  * - jointMotorSpookDamping
    - 0.0333
    - s
    - double
    - 関節のモーターのスプークダンパ
  * - jointMotorForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - 関節のモーターの最大最小力、トルク制限
  * - jointRangeCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - 関節角制限のコンプライアンス。小さくすると制限角度に納まるように大出力をします。大きくすると、外力(重力や接触力)によって制限角度外に出る可能性があります。
  * - jointRangeSpookDamping
    - 0.0333
    - s
    - double
    - 関節角制限のスプークダンパ
  * - jointRangeForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - 関節角制限の最大最小力、トルク制限
  * - jointLock
    - false
    - -\
    - bool
    - 関節ロックの有効化。角度指定制御に利用します。ActuationModeがJOINT_ANGLEの時には自動的に有効になります。
  * - jointLockCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - 関節ロックのコンプライアンス。小さくすると目標角度に到達するように大出力をします。大きくすると外力(重力や接触力)に抵抗できなくなり、目標角度に到達しなくなります。
  * - jointLockSpookDamping
    - 0.0333
    - s
    - double
    - 関節ロックのスプークダンパ
  * - jointLockForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - 関節ロックの最大最小力、トルク制限
  * - convexDecomposition
    - false
    - -\
    - bool
    - 凸分割の有効化、無効化。true、falseを指定します。
  * - AMOR
    - false
    - -\
    - bool
    - 相対的に静止している剛体同士を一体化させ、ソルバの計算量を減らします。true、falseを指定します。合わせて　:doc:`agx-simulator-item`　のプロパティも設定する必要があります。
  * - autoSleep
    - false
    - -\
    - bool
    - オートスリープの有効可、無効化。true、falseを指定します。静止している剛体をソルバから除き、計算量を減らします。:doc:`agx-simulator-item` のプロパティAutoSleepも合わせてtrueにしておく必要があります。


干渉設定
~~~~~~~~

.. list-table::
  :widths: 15,7,4,6,75
  :header-rows: 1

  * - パラメータ
    - デフォルト値
    - 単位
    - 型
    - 意味
  * - excludeLinksDynamic
    - \-
    - \-
    - string list
    - 指定のリンクの干渉を無効化します
  * - | excludeLinkGroups:
      | -
      |   name
      |   links
    - \-
    - \-
    - |
      |
      | string
      | string list
    - | グループに登録されているリンク間の干渉を無効化します。
      |
      | グループ名
      | リンク名
  * - excludeSelfCollisionLinks
    - \-
    - \-
    - string list
    - 指定のリンクとボディ間の自己干渉を無効化します
  * - excludeLinksWireCollision
    - \-
    - \-
    - string list
    - 指定のリンクとAGXWireとの干渉を無効化します

Convex Decomposition(凹形状の凸分割)
------------------------------------

AGXDynamicsは、三角形メッシュの形状を凸形状に分割する機能を持っています。
リンクパラメータのconvexDecompositionをtrueとすると、三角形メッシュ形状の凸分割を実行します。
凸分割を行うことで干渉チェックの性能が上がる可能性があります。

.. note::
  複雑な形状の凸分割は失敗する可能性があります。

.. note::
  三角形メッシュと凸分割形状とでは接触点が変わる可能性があるので、干渉時の振る舞いが異なる可能性があります。

サンプルは以下にあります。

* プロジェクトファイル: chorenoid/sample/AGXDynamics/agxConvexDecomposition.cnoid
* ボデイファイル: chorenoid/sample/AGXDynamics/vmark.body

サンプルを実行すると、凸分割が実行され、複数の凸形状で構成された形状となります。

.. image:: images/convexdecomposition.png
   :scale: 70%
