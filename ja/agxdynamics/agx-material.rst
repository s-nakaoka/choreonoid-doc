
AGX物理マテリアル
==============

AGXDynamicsプラグインを利用時には以下の物理マテリアル(物性)を利用することができます。

.. contents::
   :local:
   :depth: 2

マテリアル設定の手順
--------------
AGXSimulatorで剛体間の摩擦係数、反発係数などは、

1. マテリアルファイルにMaterial、ContactMaterialを記述
2. ボディファイルにマテリアルファイルで定義したMaterialを設定

で調整することができます。

マテリアルファイル
--------------

| マテリアルファイルは摩擦係数や反発係数などの物性を記述したリストファイルです。
| このファイルは材質(Material)と同じまたは異なる材質の接触物性(ContactMaterial)を記述することができます。
| ここで定義した材質名をBodyファイルに記述することで、モデルに材質を設定することができます。
| マテリアルファイルはワールドアイテムのプロパティに設定することで、読み込まれます。
| デフォルトでは ``choreonoid/share/default/materials.yaml`` が設定されており、自動的に読み込まれます。

.. code-block:: yaml

  materials:
    -
      name: Ground
      roughness: 0.5
      viscosity: 0.0
    -
      name: agxMat5
      density: 1.0

  contactMaterials:
    -
      materials: [ Ground, agxMat5 ]
      youngsModulus: 1.0E5
      restitution: 0.1
      damping: 0.08
      friction: 0.416667
      surfaceViscosity: 1.0E-8
      adhesionForce: 100
      adhesivOverlap: 0.2
      frictionModel: [ cone, direct ]
      contactReductionMode: reduceGeometry
      contactReductionBinResolution: 3


Material
~~~~~~~~~~

バルクマテリアル

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - パラメータ
    - デフォルト値
    - 単位
    - 型
    - 意味
  * - density
    - 1000
    - kg/m3
    - double
    - 密度。リンクの質量、慣性テンソル、重心の自動計算に利用されます。
  * - youngsModulus
    - 4.0E8
    - GPa
    - double
    - ヤング率。剛体の硬さを表します。値が小さいと剛体同士が侵入しやすくなります。
  * - poissonRatio
    - 0.3
    - \-
    - double
    - ポアソン比

サーフェスマテリアル(ContactMaterialが定義されている場合は利用されません)

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - パラメータ
    - デフォルト値
    - 単位
    - 型
    - 意味
  * - viscosity
    - 0.5
    - \-
    - double
    - 反発粘性。反発を表現します。反発粘性のペアが反発係数となります。
  * - damping
    - 0.075
    - s
    - double
    - ダンパ。接触拘束条件を満たすまでの時間。剛体の侵入の緩和に利用します。
  * - roughness
    - 0.416667
    - \-
    - double
    - 表面粗さ。摩擦を表現します。表面粗さのペアが摩擦係数となります。
  * - surfaceViscosity
    - 5E-09
    - \-
    - double
    - 表面粘性。接面方向に働く粘性です。オイルなど濡れを表現する時に利用します。
  * - adhesionForce
    - 0.0
    - N
    - double
    - 粘着力。接着剤のような表現をする時に利用します。形状が接触している時、法線方向に働きます。
  * - adhesivOverlap
    - 0.0
    - m
    - double
    - 粘着力有効距離。剛体の侵入量>有効距離となると粘着力が有効になります。

ワイヤーマテリアル

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - パラメータ
    - デフォルト値
    - 単位
    - 型
    - 意味
  * - wireYoungsModulusStretch
    - 6E10
    - GPa
    - double
    - 引張ヤング率
  * - wireDampingStretch
    - 0.075
    - s
    - double
    - 引張拘束のダンパ
  * - wireYoungsModulusBend
    - 6E10
    - GPa
    - double
    - 曲げヤング率。0にすると鎖のような振る舞いになります。
  * - wireDampingBend
    - 0.075
    - s
    - double
    - 曲げ拘束のダンパ

ContactMaterial
~~~~~~~~~~~~~~~~~

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - パラメータ
    - デフォルト値
    - 単位
    - 型
    - 意味
  * - youngsModulus
    - 2.0E8
    - GPa
    - double
    - ヤング率
  * - restitution
    - 0.0
    - \-
    - doulbe
    - 反発係数。0:完全非弾性衝突、1:完全弾性衝突
  * - damping
    - 0.075
    - s
    - double
    - ダンパ
  * - friction
    - 0.5
    - \-
    - double
    - 摩擦係数
  * - secondaryFriction
    - -1.0
    - \-
    - double
    - 副方向摩擦係数。摩擦モデルorientedBox指定時にsecondaryFriction>=0で有効となります。
  * - surfaceViscosity
    - 1.0E-8
    - \-
    - double
    - 表面粘性係数
  * - secondarySurfaceViscosity
    - -1.0
    - \-
    - double
    - 副方向表面粘性係数。摩擦モデルorientedBox指定時にsecondaryFriction>=0で有効となります。
  * - adhesionForce
    - 0.0
    - N
    - double
    - 粘着力
  * - adhesivOverlap
    - 0.0
    - m
    - double
    - 粘着力有効距離
  * - frictionModel
    - [ default, default ]
    - \-
    - | string
      | string
    - | 摩擦モデル: default(cone), cone, box, scaledBox, orientedBox
      | ソルバ    : default(split), split, direct, iterative, iterativeAndDirect

  * - contactReductionMode
    - default
    - \-
    - string
    - 接触点削減方式: default(reduceGeometry), reduceGeometry, reduceALL, reduceNone
  * - contactReductionBinResolution
    - 0
    - \-
    - uint8_t
    - 接触点削減ビン解像度。0の場合はAGXSimulatorアイテムのパラメータを利用します。
  * - primaryDirection
    - [ 0, 0, 0 ]
    - Unit vector
    - Vec3
    - 摩擦モデルorientedBox指定時の主要方向ベクトル

  * - referenceBodyName
    - \-
    - \-
    - string
    - 摩擦モデルorientedBox指定時の参照Body名
  * - referenceLinkName
    - \-
    - \-
    - string
    - 摩擦モデルorientedBox指定時の参照Link名

.. note::
  AGXDynamicsは動摩擦係数、静止摩擦係数の区別がありません。実際、値の差は10-20%程度であり、ほとんどの状況では気にしなくて良いとの考えです。

.. _not_defined_contact_material:

ContactMaterialが定義されていない場合
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| 全てのMaterialのペアの物性がContactMaterialに記述されているのが望ましいのですが、難しいと思います。
| ContactMaterialが設定されていない場合にはMaterialに記述されているパラメータついて以下の式に従って値を算出します。
| Materialにもパラメータが設定されていいない場合にはデフォルト値が適用されます。

* youngsModulus = (m1.youngsModulus * m2.youngsModulus)/(m1.youngsModulus + m2.youngsModulus)
* restitution = sqrt((1-m1.viscosity) * (1-m2.viscosity))
* damping = max(m1.damping, m2.damping)
* friction = sqrt(m1.roughness * m2.roughness)
* surfaceViscosity = m1.surfaceViscosity + m2.surfaceViscosity
* adhesionForce = m1.adhesionForce + m2.adhesionForce


ボディファイルのマテリアルの記述方法
-----------------------------------

| ボディファイルのマテリアルの記述方法について説明します。
| 重心、質量、慣性はmassTypeで直接指定か密度を使った自動計算を選択することができます。
| デフォルトはmassです。

.. code-block:: yaml

  massType: mass             # 直接指定
  massType: density          # 密度を使った自動計算

| また、材質はmaterialでマテリアルファイルに定義されているマテリアルか直接指定を選択することができます。
| デフォルトはマテリアルファイルに定義されているDefault/defualtです。

.. code-block:: yaml

  material: Default          # デフォルトマテリアル
  material: Ground           # マテリアル
  material: useLinkInfo      # 直接指定

以下は記述例です。

.. note::
  現在のところ、densityを使った重心、質量、慣性テンソルの計算結果はAGXDynamics内部で保持しており、ChorenoidのリンクやGUIから取得、確認することはできません。

従来記法
~~~~~~~~~

* 従来のChoreonoidの記法です。
* 記載されいているcenterOfMass, mass, inertiaを利用します
* Materialはdensityを除いて、defaultとなります
* ContactMaterialはdefault vs xxxxx となります

.. code-block:: yaml

  links:
    -
      name: box1
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]

マテリアルファイルの利用(推奨)
~~~~~~~~~~~~~~~~~~~~~~~~~~

* densityを含むマテリアルファイルに記述されたパラメータを使います

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density     # 密度を利用して重心、質量、慣性テンソルを自動計算する
      material: steel       # マテリアルファイルのsteelを利用
      density: 1.0          # densityが記述されている場合はsteelのdensityをオーバライドして、直接記述されているものを利用します

従来記法+マテリアルリストの利用(推奨)
~~~~~~~~~~~~~~~~~~~~~~~~~~

* massType: massで直接記述されている重心、質量、慣性テンソルを利用します
* その他のマテリアルパラメータはマテリアルファイルのsteelを利用します

.. code-block:: yaml

  links:
    -
      name: box1
      massType: mass      # 直接記述された重心、質量、慣性テンソルを利用する
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]
      material: steel     # マテリアルファイルのsteelを利用


直接記述(非推奨)
~~~~~~~~~~~~~~~~~~~~~~~~~~

* material: useLinkInfoとするとボディファイルに記述されたMaterialのパラメータを利用することができます
* :ref:`not_defined_contact_material` に従ってContactMaterialの値が計算されます

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density
      material: useLinkInfo
      density: 1.0
      youngsModulus:
      poissonRatio:
      viscosity:
      damping:
      roughness:
      surfaceViscosity:
      adhesionForce:
      adhesivOverlap:


全記述(非推奨)
~~~~~~~~~~~~~~~~~~~~~~~~~~

* すべてが記述されている場合です
* どのパラメータが利用されているのか判別がしずらいのでおすすめしません

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density               # 密度を利用して重心、質量、慣性テンソルを自動計算する
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]
      material: steel                 # materialリストを利用
      density: 1.0                    # 記述されたdensityを利用
      youngsModulus:                  # 以下は使用されない
      poissonRatio:
      viscosity:
      damping:
      roughness:
      surfaceViscosity:
      adhesionForce:
      adhesivOverlap:

サンプル
--------

AGXDynamicsPluginのマテリアルのサンプルが以下にあります。
パラメータ値によって動作結果が異なることを確認してみてください。

* choreonoid/samples/AGXDynamics/agxMaterialSample.cnoid
