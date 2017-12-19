
AGXVehicleContinuousTrack (AGXクローラ)
===========================

AGXVehicleContinuousTrackはAGX Dynamicsを使ったクローラモデルです。
実装にはAGX DynamicsのモジュールagxVehicleを利用しています。

.. contents::
   :local:
   :depth: 1


AGXVehicleContinuousTrackの特徴
--------------------------------

* **クローラベルトの自動装着**

  シミュレーションを実行すると、ホイールにクローラベルトが自動的に装着されます。
  ユーザはホイールと各種パラメータを設定するのみで、幾何的な配置を考える必要がありません。

* **クローラベルトがホイールから外れにくい**

  クローラベルトがホイールから外れないように、内部で拘束を付加しています。

* **性能向上のための最適化**

  クローラベルトは複数のプレート状のノードをヒンジでつなぎ合わせて表現をしています。
  ノード一つ一つが剛体であるため、クローラベルトが床に接触すると、複数の接触点が生成されます。
  AGX Dynamicsではこの接触点を利用して、クローラベルトが床に侵入しないように抗力の計算をします。
  接触点が増えると、抗力の計算量が増えることになります。
  計算量が増えると、実時間内で計算が終わらなくなり、パフォーマンスが劣化
  本モデルには複数のノードを一時的にマージし、接触点の数を減らすことで性能劣化を抑える機能を持っています。


サンプル
------------

サンプルを使った利用方法の説明をします。サンプルプロジェクトは以下にあります。

* chorenoid/Share/project/TankJoystickAGX.cnoid               // タンクとLaboのシーン
* chorenoid/Share/project/TankJoystickAGX_Floor.cnoid         // タンクと床のシンプルなシーン

サンプルプロジェクトで利用されるサンプルモデルは

* chorenoid/Share/project/tank_agx.body

| Choreonoidでサンプルプロジェクトをロードし、AGXSimulatorでシミュレーションを実行すると、タンクのホイールにクローラベルトが装着されます。
| PS4コントローラが接続されている場合は左スティックで、接続されていない場合はキーボードのE、D、S、Fでタンクを移動させることができます。


記述方法
------------

サンプルモデルは以下のリンク構成となっています。

.. code-block:: yaml

  name: CHASSIS
  name: TURRET_Y
  name: TURRET_P
  name: TRACK_L      // 左クローラ(内外フレームつき)
  name: WHEEL_L0     // 左前ホイール(モータ駆動)
  name: WHEEL_L1     // 左中ホイール(自由回転)
  name: WHEEL_L2     // 左後ホイール(自由回転)
  name: TRACK_R      // 右クローラ(内外フレームつき)
  name: WHEEL_R0     // 右前ホイール(モータ駆動)
  name: WHEEL_R1     // 右中ホイール(自由回転)
  name: WHEEL_R2     // 右後ホイール(自由回転)

TRACK_L、TRACK_R以外は通常のChoreonoidの記述方法に従って内容を記述します。
TRACK_Lの詳細は以下のようになっており、
内外のフレーム、AGXVehicleContinuousTrackの2種類が記述されています。

.. code-block:: yaml

  -
    name: TRACK_L
    parent: CHASSIS
    translation: [ 0, 0.16, -0.026 ]
    jointType: fixed
    centerOfMass: [ 0, 0, 0 ]
    mass: 1.0
    inertia: [
      0.02, 0,    0,
      0,    0.02, 0,
      0,    0,    0.02 ]
    elements:
      -
        type: AGXVehicleContinuousTrackDevice
        name: TRACK_L
        sprocketNames: [ WHEEL_L0 ]
        rollerNames: [ WHEEL_L1 ]
        idlerNames: [ WHEEL_L2 ]
        upAxis: [ 0, 0, 1 ]
        numberOfNodes: 42
        nodeThickness: 0.01
        nodeWidth:  0.09
        nodeDistanceTension: 2e-4
        nodeThickerThickness: 0.02
        useThickerNodeEvery: 3
        hingeCompliance: 1e-7
        hingeDamping: 0.0333
        minStabilizingHingeNormalForce: 300.0
        stabilizingHingeFrictionParameter: 1e-6
        enableMerge: false
        numNodesPerMergeSegment: 0
        contactReduction: 3
        enableLockToReachMergeCondition: false
        lockToReachMergeConditionCompliance: 1.0E-11
        lockToReachMergeConditionDamping: 0.001
        maxAngleMergeCondition: 1.0E-5
      -
        // 省略


内外のフレームは通常のChoreonoidの記述方法で記述されています。
AGXVehicleContinuousTrackはリンクのelements部分にAGXVehicleContinuousTrackDeviceとして追加します。
AGXVehicleContinuousTrackDeviceは空のリンクやホイールリンク、シャーシリンクなど任意のリンクに追加することができます。
以下にパラメータの説明をします。

.. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 10,7,7,75
  :header-rows: 1

  * - プロパティ
    - デフォルト値
    - 単位/型
    - 意味
  * - type: AGXVehicleContinuousTrackDevice
    - \-
    - string
    - AGXVehicleContinuousTrackを使うことの宣言
  * - name: TRACK_R
    - \-
    - string
    - このクローラベルトの名前

* type: AGXVehicleContinuousTrackDevice
** AGXVehicleContinuousTrackを使うことの宣言

* name: TRACK_R
** このクローラベルトの名前

* sprocketNames: [ WHEEL_L0 ]
** 駆動ホイールの指定
** ベルトとホイール間は拘束され、外れなくなる
** サンプルでは前ホイール

* rollerNames: [ WHEEL_L1 ]
** 拘束なしガイドホイールの指定
** ベルトとホイール間は拘束されない
** サンプルでは中ホイール
** 複数の場合は [ WHEEL_L1, WHEEL_L3, WHEEL_L4 ]と','区切りで記述

* idlerNames: [ WHEEL_L2]
** 拘束ありガイドホイールの指定
** ベルトとホイール間は拘束され、外れなくなる
** サンプルでは後ホイール
** 複数の場合は [ WHEEL_L2, WHEEL_L7, WHEEL_L8 ]と','区切りで記述

* upAxis: [ 0, 0, 1 ]
** モデルの上方向(クローラ進行方向に対して垂直)を指定

* numberOfNodes: 42
** ノード数

* nodeThickness: 0.01
** ノードの厚み

* nodeWidth:  0.09
** ノードの幅(基本はホイールの高さ)

* nodeDistanceTension: 2e-4
** ノード間をつなぐ張力

* nodeThickerThickness: 0.02
** 厚みのあるノードの厚み

* useThickerNodeEvery: 3
** 厚みのあるノードをxノードおきに配置
** 厚みのあるノードを利用しない場合は0

* hingeCompliance: 1e-7
** ノード間をつなぐヒンジのコンプライアンス

* hingeDamping: 0.0333
** ノード間をつなぐヒンジのダンパ

* minStabilizingHingeNormalForce: 300.0
** ノード間をつなぐヒンジの内部摩擦計算のための最小抗力
** ヒンジに摩擦を入れることで挙動の安定化をしている
** ヒンジ間の張力が高くなると、内部摩擦力が強くはたらきクローラベルトの高振動、共振を防ぐ
** 抗力が小さくなったり、負の値になることがあるため、その場合に最小値を利用する

* stabilizingHingeFrictionParameter: 1e-6
** ヒンジの内部摩擦係数
** 値を高くすると錆びた関節を回すような感じになる

* enableMerge: false
** ノードのマージ(統一化)機能のON/OFF
** trueまたはfalseを指定

* numNodesPerMergeSegment: 0
** マージするノードの数

* contactReduction: 3
** 接触点数削減レベルの指定 0(削減なし) - 3(最大)

* enableLockToReachMergeCondition: false
** ノードをマージできるようにするために、ヒンジをロックするかどうか

* lockToReachMergeConditionCompliance: 1.0E-11
** ヒンジロック時のコンプライアンス

* lockToReachMergeConditionDamping: 0.001
** ヒンジロック時のダンパ

* maxAngleMergeCondition: 1.0E-5
** ノードをマージするかどうか判定するための閾値角度
** ヒンジの角度 < 閾値角度になると、ノードがマージされる

パラメータ設定の勘所
------------------------

クローラベルトの安定化
~~~~~~~~~~~~~~~~~~~~~~~~~~

# シミュレーションのタイムステップを固定します
コンプライアンスやダンパなど、タイムステップによって大きく結果が変わるパラメータがあるので、タイムステップを固定します。
ここでは以下の通りとします。
<pre>
dt = 0.005 (200Hz)
</pre>
# ノードのマージ機能をOFFにし、チューニングするパラメータ数を減らします
<pre>
enableMerge: false
enableLockToReachMergeCondition: false
</pre>
# チューニング対象のパラメータは以下に絞られます。まずは下記を全てコメントアウトをしてデフォルトの状態でクローラの動きを確認します。(下記はデフォルト値が入ってます)
<pre>
#nodeDistanceTension: 5.0E-3
#hingeCompliance: 1.0E-10
#hingeDamping: 0.0333
#minStabilizingHingeNormalForce: 100
#stabilizingHingeFrictionParameter: 1.5
</pre>
# おそらくクローラベルトは硬く、針金のような見た目になると思います。
ヒンジ摩擦が強すぎるので、摩擦係数を小さくします。
<pre>
nodeDistanceTension: 0.0     　　　　　   // ノード間の引張力をなくし、調整をわかりやすくします
stabilizingHingeFrictionParameter: 1e-6   // 摩擦係数を小さく。1e-1以下は指数単位で調整していき、針金みたいな曲がり方にならない程度にします
</pre>
# このように設定すると、クローラベルトは若干たわみをもった状態になります。
たわみを取るために引張力を設定します。ひとまずデフォルト値で様子をみると、引張力が強すぎるためか、ベルトが振動します。
そこで振動しない程度に引張力を小さくします。5.0E-4はベルトがホイールに食い込み、5.0E-5は引張があまり効いてないようみえます。
この間で調整をかけて以下のようにします。
<pre>
nodeDistanceTension: 2.0E-4
</pre>
# これでクローラを前後方向はスムーズに動くと思います。
しかし、信地旋回、超信地旋回をさせるとベルトが発振します。
ここで、ヒンジのコンプライアンスとダンパを調整して発振を抑えます。
ダンパは2*dtを目安に設定をします。
コンプライアンスはまずは指数単位で大きくしていき、発振しない程度に調整します。
この場合ですと、1.0E-10は発振し、1.0E-9は発振しなくなりましたので、その間で調整をします。
<pre>
hingeCompliance: 9.0E-10
hingeDamping: 0.01          // 2.0 * dt を目安に設定します。
</pre>
# 最後の仕上げです。
minStabilizingHingeNormalForceはクローラベルトが交差したり、クローラが回転している時にホイールに侵入するようであれば値を小さくしていきます。
たまに振動したりあばれるようでしたら、値を大きくします。
<pre>
minStabilizingHingeNormalForce: 100
</pre>

*作成中*　ノードのマージ
----------------------------

# ノードのマージに関するパラメータ(値はデフォルト)を下記に示します。
<pre>
enableMerge: false
numNodesPerMergeSegment: 3
contactReduction: 1
enableLockToReachMergeCondition: false
lockToReachMergeConditionCompliance: 1.0E-11
lockToReachMergeConditionDamping: 0.05
maxAngleMergeCondition: 1.0E-5
</pre>
# まずは機能を有効化し、パラメータはデフォルト(コメントアウト)のままで様子をみます。
<pre>
enableMerge: true
#numNodesPerMergeSegment: 3
#contactReduction: 1
enableLockToReachMergeCondition: true
#lockToReachMergeConditionCompliance: 1.0E-11
#lockToReachMergeConditionDamping: 0.05
#maxAngleMergeCondition: 1.0E-5
</pre>

仕様
---------

* クローラはシミュレーション実行時に自動で生成されます。bodyファイルロード時はクローラは描画されません。
* AGXVehicleContinuousTrackは自動的に自己接触が設定されます(下記表を参照)。
** クローラベルトとホイールは必ず接触がONになっていないと、すり抜けが発生してしまうためです。
** また、クローラベルトとその他の部分の接触はOFFにすることで性能劣化を抑えています。
|      |WHEEL     |TRACK    |
|WHEEL |///////// |接触ON   |
|その他|設定による|接触OFF  |
