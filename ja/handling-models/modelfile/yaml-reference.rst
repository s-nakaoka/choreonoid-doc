
新YAML形式モデルファイル リファレンスマニュアル
===============================================

.. contents::
   :local:
   :depth: 2

概要
----

YAML形式のモデルファイルのリファレンスマニュアルです。

モデルファイルの規約
~~~~~~~~~~~~~~~~~~~~
モデルファイルひとつにつき、ロボットや環境のモデル１体を記述するようにします。
また、ファイルの拡張子は通常のYAML形式ファイル(".yaml")と区別するため".body" をつけるようにします。

YAML文法
--------
YAMLの文法については `プログラマーのための YAML 入門 (初級編)  <http://magazine.rubyist.net/?0009-YAML>`_
などを参照してください。

ノード一覧
----------

ノードとしては以下のようなものが定義されており、これらのインスタンスを組み上げていくことにより、モデルを作成します。
実モデル定義部においてこれらのノードのインスタンスを組み合わせて階層構造を作ることで、モデルを作成していきます。

リンク構造、動力学/機構パラメータを定義するノードとして、以下のノードが定義されています。

* linksノード
* RigidBodyノード
* Transform

リンクの形状、表示を定義するノードとして、以下のノードが定義されています。

* Shapeノード
* geometryノード
* appearanceノード
* materialノード
* Resource

各種センサ・デバイスを定義するノードとして以下のノードが定義されています。

* AccelerationSensorノード
* RateGyroSensorノード
* ForceSensorノード
* RangeSensorノード 
* Cameraノード
* SpotLightノード

閉リンク機構を定義するノードとして以下のノードが定義されています。

* ExtraJointノード

ノードをグループ化するためのノードとして以下のノードが定義されています。

* Group
以下では各ノードの詳細を説明します。

ヘッダ
-------

ファイルの先頭に置き、モデルファイルのフォーマットを指定します。

.. list-table:: ヘッダのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - format
   - "ChoreonoidBody"を指定。
 * - formatVersion
   - モデルファイルのフォーマットのバージョンを指定。現在のバージョンは1.0。
 * - angleUnit
   - モデルファイルにおける関節角度の単位を指定する項目。"degree"または"radian"を指定。
 * - name
   - モデルの名前を指定。
 * - rootLink
   - ルートリンク名を指定。


リンク構造、動力学/機構パラメータを定義するノード
-------------------------------------------------


linkノード
~~~~~~~~~~

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: Linkノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - name
   - リンクの名称。モデル内で重複しない任意の文字列を指定可能
 * - parent
   - 親リンク。親リンクの名前（nameに記述した文字列）で指定する。ルートリンクの場合は使用しない
 * - type
   - Link(default) または ContinuousTrack
 * - translation
   - 本リンクローカルフレームの親リンクからの相対位置。ルートリンクの場合はモデル読み込み時のデフォルト位置として使われる
 * - rotation
   - 本リンクローカルフレームの親リンクからの相対姿勢。姿勢は回転軸と回転角度に対応する4つの数値で表現 (Axis-Angle形式）。ルートリンクの場合はモデル読み込み時のデフォルト位置として使われる
 * - jointId
   - 関節ID値。0以上の整数値を指定する。モデル内で重複しない任意の値を指定可能。リンクが関節でない場合 （ルートリンクやjointTypeがfixedの場合）や、ID値によるアクセスを必要としない場合は、指定しなくてもよい
 * - jointType
   - 関節タイプ。 **fixed** (固定）、 **free** (非固定。ルートリンクにのみ指定可）、 **revolute** (回転関節）、 **prismatic** (直動関節）、 **pseudoContinousTrack** (簡易無限軌道）、 のどれかを指定
 * - jointAxis
   - 関節軸。3次元ベクトルの3要素のリストとして関節軸の向きを指定する。値は単位ベクトルとする。関節軸がリンクのローカル座標におけるX, Y, Z、及びそれらの逆方向のいずれかに一致する場合は、対応する軸の文字(X, Y, Z,-X,-Y,-Z）によって指定することも可能。
 * - jointAngle
   - 関節の初期角度。degreeで指定。
 * - jointDisplacement
   - 関節の初期角度。radianで指定。
 * - jointRange
   - 関節可動範囲。最小値、最大値の2つの値をリストとして列挙する。値をunlimitedと記述することで、可動範囲の制限を無くすことも可能。最小値と最大値の絶対値が同じでそれぞれ符号がマイナス、プラスとなる場合は 、その絶対値をひとつだけ（スカラ値として）記述してもよい
 * - maxJointVelocity
   - 関節の回転・移動速度の範囲をスカラ値(>=0)で指定。この値のマイナス、プラスの範囲に設定される。jointTypeがrevoluteのときは最大角速度(degree/sec)、それ以外のときは最大速度(m/sec)
 * - jointVelocityRange
   - 関節の回転・移動速度の範囲。最小値、最大値の2つの値をリストとして列挙する。maxJointVelocityより優先される。
 * - rotorInertia
   - ロータ慣性モーメント。default値=0.0。
 * - gearRatio
   - ギア比。default値=1.0。
     等価ロータ慣性モーメントはgearRatio*gearRatio*rotorInertiaで設定される。
 * - centerOfMass
   - 重心位置。リンクローカル座標で指定
 * - mass
   - 質量[kg]
 * - inertia
   - 慣性モーメント。慣性テンソルの9要素をリストとして列挙。慣性テンソルの対称性より、上三角部分の6要素のみを列挙してもよい。
 * - import
   - エイリアスをつけたノードをこの場所に読み込む。 import: \*defined_alias
 * - elements
   - リンクの構成要素となる子ノードを記述


.. note::
	最初に記述するLinkノードはモデルのルートノードとみなされます。

.. note::
	剛体パラメータ(centerOfMass, mass, inertia)は次に述べるRigidBodyノードで記述することも可能です。その場合elementsを用いてRigidBodyノードをLinkノードの子ノードとして配置します。

RigidBodyノード
~~~~~~~~~~~~~~~

RigidBodyノードはリンクの剛体パラメータを定義します。

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: RigidBodyノードの項目
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - centerOfMass
   - 重心位置。リンクローカル座標で指定
 * - mass
   - 質量[kg]
 * - inertia
   - 慣性モーメント。慣性テンソルの9要素をリストとして列挙。慣性テンソルの対称性より、上三角部分の6要素のみを列挙してもよい。
 * - elements
   - 子ノードでリンクの形状やセンサーなどを記述。


Transformノード
~~~~~~~~~~~~~~~

配下のノードを平行移動・回転・拡大縮小します。

.. list-table:: Transformノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - translation
   - 位置のオフセット
 * - rotation
   - 姿勢のオフセット
 * - scale
   - サイズの拡大・縮小
 * - elements
   - 変換を受ける子ノードを記述。


リンク形状・見た目を定義するノード
----------------------------------

Shapeノード
~~~~~~~~~~~

.. list-table:: Shapeノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - geometry
   - リンクの形状を定義するノード
 * - appearance
   - リンクの見た目を定義するノード

geometryノード
~~~~~~~~~~~~~~

.. list-table:: geometryノードのフィールド
 :widths: 15,15,70
 :header-rows: 1

 * - キー
   - 形状種別
   - 内容
 * - type
   - \-
   - | "Box" 直方体
     | "Sphere" 球体
     | "Cylinder"  円柱
     | "Cone"  円錐
     | "Extrusion"  押し出し
     | "ElevationGrid" グリッドの格子点ごとの高さを与えてメッシュを作成
 * - size
   - Box
   - 直方体の縦横奥行きの長さ
 * - radius
   - Sphere
   - 球の半径
 * - radius
   - Cylinder
   - 底面の半径
 * - height
   - Cylinder
   - 高さ
 * - bottom
   - Cylinder
   - true:底面あり(default)  false:底面なし
 * - side
   - Cylinder
   - true:側面あり(default)  false:側面なし
 * - radius
   - Cone
   - 底面の半径
 * - height
   - Cone
   - 高さ
 * - bottom
   - Cone
   - true:底面あり(default)  false:底面なし
 * - side
   - Cone
   - true:側面あり(default)  false:側面なし
 * - crossSection
   - Extrusion
   - | 押し出す断面の形状を頂点の座標で指定(x-z平面)。
     | crossSection: [ x0, z0, x1, z1, x2, z2, ・・・, xn, zn ]
     | のようにx座標,z座標を並べる。改行・スペースを入れて良い。
     | crossSection: [ x0, z0,
     |                 x1, z1,
     |                  ：
 * - spine
   - Extrusion
   - | crossSectionで指定した断面を沿わせて動かす区分的直線を端点の座標で指定。
     | spine: [ x0, y0, z0, x1, y1, z1, ・・・, xn, yn, zn ]
 * - orientation
   - Extrusion
   - spineの各点におけるcrossSectionの回転をaxis-angle形式のパラメータ(x, y, z, θ)を並べて指定。
     1組のみ指定した場合は全spineで同じ回転が使われる。spineの個数より少ない場合は不足分が回転無しになり、spineの個数より多い場合は無視される。
 * - scale
   - Extrusion
   - crossSectionで指定した断面のspineの各点における拡大率。x軸方向の拡大率、z軸方向の拡大率をspineの個数分並べて指定。1組のみ指定した場合は全spineで同じ拡大率になる。spineの個数より指定が少ない場合、未指定分は0倍に拡大され1点になる。spineの個数より多く指定された分は無視される。
 * - creaseAngle
   - Extrusion
   - 光源と法線ベクトルの角度によってシェーディングを変えるための閾値。creaseAngle未満のときはスムーズシェーディングされる。デフォルトは0。
 * - beginCap
   - Extrusion
   - true:開始端側の断面あり(default) false:開始端側の断面なし
 * - endCap
   - Extrusion
   - true:終端側の断面あり(default) false:終端側の断面なし
 * - xDimension
   - ElevationGrid
   - x軸方向のグリッドの数
 * - zDimension
   - ElevationGrid
   - z軸方向のグリッドの数
 * - xSpacing
   - ElevationGrid
   - x軸方向のグリッド間隔
 * - zSpacing
   - ElevationGrid
   - z軸方向のグリッド間隔
 * - ccw
   - ElevationGrid
   - true: 頂点の順序が反時計回り false: 頂点の順序が時計回り
 * - creaseAngle
   - ElevationGrid
   - 光源と法線ベクトルの角度によってシェーディングを変えるための閾値。creaseAngle未満のときはスムーズシェーディングされる。デフォルトは0。
 * - height
   - ElevationGrid
   - 各格子点上の高さを配列で指定。格子点の個数(xDimension*zDimension)分の要素が必要。


.. note::
    Extrusionについては
    http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Extrusion

    ElevationGridについては
    http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#ElevationGrid


appearanceノード
~~~~~~~~~~~~~~~~

.. list-table:: appearanceノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - material
   - 物体表面の材質パラメータを定義するノード

materialノード
~~~~~~~~~~~~~~

.. list-table:: materialノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - ambientIntensity
   - 環境光の反射率(0.0〜1.0)
 * - diffuseColor
   - RGBごとの拡散反射率(物体の色) (RGBそれぞれ0.0〜1.0のリスト)
 * - emissiveColor
   - 物体自体から発光する色 (RGBそれぞれ0.0〜1.0のリスト)
 * - shininess
   - 輝度 (0.0〜1.0)
 * - specularColor
   - 鏡面反射率(光のハイライトの色) (RGBそれぞれ0.0〜1.0のリスト)
 * - transparency
   - 透過度(0:透明 〜 1:不透明)

Resourceノード
~~~~~~~~~~~~~~

リンクの形状にCADやモデリングツールで作成したメッシュを読み込みます。

.. list-table:: Resourceノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - uri
   - リンク形状のメッシュファイルのパス
 * - node
   - メッシュファイル内の特定のノードのみを読み込む場合にノード名を指定

.. _body-file-reference-devices:

各種センサ・デバイスを定義するノード
------------------------------------

Deviceノード
~~~~~~~~~~~~

各種デバイスで共通の設定項目を示します。

.. list-table:: Deviceノードの共通フィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - id
   - デバイスのID
 * - translation
   - ローカル座標系の位置を、親ノード座標系からのオフセット値で指定。
 * - rotation
   - ローカル座標系の姿勢を、親ノード座標系からのオフセット値で指定([x, y, z, θ]  ベクトル[x, y, z]の周りにθ回転)。

使用可能なデバイスの一覧は以下です。

    - AccelerationSensor
    - RateGyroSensor
    - ForceSensor
    - RangeSensor
    - Camera
    - SpotLight

デバイスの継承関係は以下のようになっており、親クラスの設定項目は子クラスでも有効です。
デバイスごとの説明ではそのクラスで追加された設定項目についてのみ説明します。

::

  + Device
    + AccelerationSensor
    + RateGyroSensor
    + ForceSensor
    + RangeSensor
    + Camera
      + RangeCamera
    + Light
      + PointLight
        + SpotLight


.. note::
  各種センサノードはそのセンサが取り付けられているLinkノードの下に取り付けます。 例えば、サンプルモデルの腰部(WAIST)に加速度センサを取り付けている場合は、次のように記述します。

.. code-block:: yaml

    links:
      - 
        name: WAIST
        elements:
          -
            type: AccelerationSensor
            id: 0


AccelerationSensorノード
~~~~~~~~~~~~~~~~~~~~~~~~

AccelerationSensorノードは、3軸加速度センサを定義します。

.. list-table:: AccelerationSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - maxAcceleration
   - 計測可能な最大加速度。3次元ベクトルの3要素のリストとして指定する。




RateGyroSensorノード
~~~~~~~~~~~~~~~~~~~~

RateGyroSensorノードは、3軸角速度センサを定義します。


.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: RateGyroSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - maxAngularVelocity
   - 計測可能な最大角速度。3次元ベクトルの3要素のリストとして指定する。


ForceSensorノード
~~~~~~~~~~~~~~~~~

ForceSensorノードは、力／トルクセンサを定義します。

.. list-table:: ForceSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - maxForce
   - 計測可能な力の最大値。3次元ベクトルの3要素のリストとして指定する。
 * - maxTorque
   - 計測可能なトルクの最大値。3次元ベクトルの3要素のリストとして指定する。


RangeSensorノード
~~~~~~~~~~~~~~~~~

RangeSensorノードは、距離センサを定義します。


.. list-table:: RangeSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - on
   - 
 * - scanAngle
   - 距離をスキャンする角度。0度を中心として、その両側にscanStepの倍数の角度でscanAngleの範囲内の角度が計測される。センサにスキャン機能がない場合は0とする。
 * - scanStep
   - スキャン中に距離が計測される角度の刻み幅
 * - scanRate
   - １秒間あたり行うスキャン回数[Hz]
 * - minDistance
   - 計測可能な最小距離[m]
 * - maxDistance
   - 計測可能な最大距離[m]

.. note::
   このセンサが取り付けられているリンクに対するこのセンサの姿勢。センサ座標系において、Z軸マイナス方向が計測正面、スキャンする場合の計測面はXZ平面となります。 これはVisionSensorと同じですので、従来VisionSensorで代用していたモデルを変更する場合は 位置、姿勢はそのまま使えます。

Cameraノード
~~~~~~~~~~~~~~~~~~

Cameraノードは、視覚センサを定義します。

.. list-table:: Cameraノードのフィールド
 :widths: 30,70
 :header-rows: 1

 * - キー
   - 内容
 * - format
   - | センサから取得する情報の種類を指定する。
     |   ・"COLOR"  色情報を取得
     |   ・"DEPTH"  深さ情報を取得
     |   ・"COLOR_DEPTH"  色情報と深さ情報を取得
     |   ・"POINT_CLOUD"  3次元点群を取得
     |   ・"COLOR_POINT_CLOUD"  色情報と3次元点群を取得
 * - on
   - true/falseでカメラのON/OFFを指定
 * - width
   - 画像の幅
 * - height
   - 画像の高さ
 * - fieldOfView
   - カメラの視野角度
 * - nearClipDistance
   - 視点から前クリップ面までの距離
 * - farClipDistance
   - 視点から後クリップ面までの距離
 * - frameRate
   - カメラが毎秒何枚の画像を出力するか

.. note::
    視点の姿勢は以下のように定義されます。視線前方向 ・・・ ローカル座標系でZ軸の負の向き   視線上方向 ・・・ ローカル座標系でY軸の正の向き。

.. note::
    内部的にはformatが"COLOR"のときCamera、"COLOR"以外のときRangeCameraとして扱われます。

SpotLightノード
~~~~~~~~~~~~~~~

SpotLightノードは、ライトを定義します。

.. list-table:: SpotLightノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - on
   - true/falseでライトのON/OFFを指定します。
 * - color
   - ライトの色(R,G,Bそれぞれの値を0.0〜1.0で指定)
 * - intensity
   - 明るさを0.0〜1.0で指定。
 * - direction
   - 光の向き。3次元ベクトルの3要素のリストとして方向を指定。
 * - beamWidth
   - 最大輝度で光の広がる角度。デフォルトは90度。
 * - cutOffAngle
   - 完全に光が遮断される角度。デフォルトは45度。
 * - cutOffExponent
   - 非負の値を指定。デフォルトは1.0。
 * - attenuation
   - 減衰率。非負の3要素のリストを指定。


閉リンク機構を定義するノード
------------------------------


ExtraJointノード
~~~~~~~~~~~~~~~~

ExtraJointノードは閉リンク機構を定義します。閉リンクの1つの関節がボールジョイントで接続されていると考え、2つのリンクが離れないように拘束力を発生させます。

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: ExtraJointノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - link1Name
   - ボールジョイントを受けているジョイント名
 * - link2Name
   - ボールジョイントが付いているジョイント名
 * - link1LocalPos
   - link1Nameジョイントの拘束位置をそのジョイントのローカル座標で指定
 * - link2LocalPos
   - link2Nameジョイントの拘束位置をそのジョイントのローカル座標で指定
 * - jointType
   - 拘束の種類  ball：1点で固定  piston：jointAxisで指定した軸の向きにのみ動く
 * - jointAxis
   - jointTypeがpistonのとき、可動方向をlink1Nameジョイントのローカル座標で指定。


閉リンク機構のサンプルとして "share/model/misc/ClosedLinkSample.body" があります。


ノードをグループ化するノード
---------------------------

Groupノード
~~~~~~~~~~~~~~

一部のノードをグループ化するために使用します。

.. list-table:: Groupノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - キー
   - 内容
 * - name
   - グループの名前

.. code-block:: yaml

  (使用例)
  elements:
    - &SUBSYSTEM
      type: Group
      name: SUBSYSTEM
      elements:
        -
          (グループの１要素)
        -
          (グループの１要素)
         :

としてグループノードにエイリアスをつけておくと、別の場所にSUBSYSTEMと同じ構成があるとき、

.. code-block:: yaml

  elements: *SUBSYSTEM

で記述できます。

