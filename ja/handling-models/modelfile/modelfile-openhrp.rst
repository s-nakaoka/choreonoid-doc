
OpenHRPモデルファイル
=====================

.. contents::
   :local:
   :depth: 1

概要
----

OpenHRP形式のモデルファイルは、3次元モデルを記述するための言語である "VRML97" をベースとしています。
モデルファイルひとつにつき、ロボットや環境のモデル１体を記述するようにします。
また、ファイルの拡張子にはVRML97の拡張子である ".wrl" をつけるようにします。

モデルファイルの基本的な構成は、 

* PROTO宣言部（構造体宣言部）
* 実モデル定義部（PROTOを用いたインスタンス表記部）

からなります。

PROTO宣言部ではVRML97では定義されていない新たなノードを定義するため，C言語における構造体にあたる "PROTO" と呼ばれるノードを使います。PROTOノードとしては以下のようなものが定義されており、これらのインスタンスを組み上げていくことにより、モデルを作成します。

リンク構造、動力学/機構パラメータを定義するノードとして、以下のノードが定義されています。（これらのノードは、ヒューマンフィギュアを記述するためのフォーマット "h-anim1.1" で制定されているものをベースとして拡張/変更したものとなっています。）

* Humanoidノード
* Jointノード
* Segmentノード
* ExtraJointノード 

実モデル定義部においてこれらのノードのインスタンスを組み合わせて階層構造を作ることで、モデルを作成していきます。例えば
人間型ロボットの場合は以下のような構造となります。::

 Humanoid sample（一塊のモデルのルート）
   + Joint 腰部 (ヒューマノイドの中心。空中に浮遊する非固定点)
   |　....
   |
   |  + Joint 胸部
   |    + Joint 頭部
   |    + Joint 左腕部
   |    + Joint 右腕部
   |
   + Joint 左脚部
   |
   + Joint 右脚部

つまり、空中に浮いた「腰部」に「左脚」、「右脚」に対応する鎖と「胸部」へと繋がる鎖がつながっており、さらに「胸部」から、「頭部」、「左腕」、「右腕」の鎖がつながっている、という構造です。

また、各種センサを定義するための以下のPROTOノードも用意されています。

* AccelerationSensorノード
* GyroSensorノード
* VisionSensorノード
* ForceSensorノード
* RangeSensorノード 

これらのノードを用いることにより、モデルにセンサ情報を含めることが可能です。

以下では各ノードの詳細を説明します。

リンク構造、動力学/機構パラメータを定義するノード
-------------------------------------------------

Humanoidノード
~~~~~~~~~~~~~~

Humanoidノードは、モデルのルートノードです。 ::

	PROTO Humanoid [
	  field         SFVec3f     bboxCenter        0 0 0
	  field         SFVec3f     bboxSize          -1 -1 -1
	  exposedField  SFVec3f     center            0 0 0
	  exposedField  MFNode      humanoidBody      [ ]
	  exposedField  MFString    info              [ ]
	  exposedField  MFNode      joints            [ ]
	  exposedField  SFString    name              ""
	  exposedField  SFRotation  rotation          0 0 1 0
	  exposedField  SFVec3f     scale             1 1 1
	  exposedField  SFRotation  scaleOrientation  0 0 1 0
	  exposedField  MFNode      segments          [ ]
	  exposedField  MFNode      sites             [ ]
	  exposedField  SFVec3f     translation       0 0 0
	  exposedField  SFString    version           "1.1"
	  exposedField  MFNode      viewpoints        [ ]
	]
	{
	  Transform {
	    bboxCenter       IS bboxCenter
	    bboxSize         IS bboxSize
	    center           IS center
	    rotation         IS rotation
	    scale            IS scale
	    scaleOrientation IS scaleOrientation
	    translation      IS translation
	    children [
	      Group {
		children IS viewpoints
	      }
	      Group {
		children IS humanoidBody 
	      }
	    ]
	  }
	}

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: Humanoidノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - bboxCenter
   - OpenHRPでは使用しません。
 * - bboxSize
   - OpenHRPでは使用しません。
 * - center
   - Jointノードの "center" を参照してください。
 * - humanoidBody
   - 子ノードをぶら下げるフィールドです。0個以上のJointノード、0または1個のSegmentノードをぶらさげます。
 * - info
   - モデルに関するコメントを記述するフィールドです。
 * - joints
   - 定義したJointの一覧を格納するフィールドです。
 * - name
   - モデルの名前を指定するフィールドです。
 * - rotation
   - Jointノードの "rotation" を参照してください。
 * - scale
   - Jointノードの "scale" を参照してください。
 * - scaleOrientation
   - Jointノードの "scaleOrientation" を参照してください。
 * - segments
   - 定義したSegmentの一覧を格納するフィールドです。
 * - sites
   - OpenHRPでは使用しません。
 * - translation
   - Jointノードの "translation" を参照してください。
 * - version
   - モデルのバージョン番号を指定するフィールドです。
 * - viewpoints
   - 仮想環境における視点位置を指定するフィールドです。


.. note::
	モデルのルートノードとなるHumanoidノードがただ一つだけ存在するようにします。また、Humanoidノードのjointsフィールド、segmentsフィールドには、それぞれモデル中で使用されているJoint名、Segment名をすべて列挙します。


Jointノード
~~~~~~~~~~~

Jointノードはリンクフレームを定義します。 ::

	PROTO Joint [
	  exposedField     SFVec3f      center              0 0 0
	  exposedField     MFNode       children            []
	  exposedField     MFFloat      llimit              []
	  exposedField     MFFloat      lvlimit             []
	  exposedField     SFRotation   limitOrientation    0 0 1 0
	  exposedField     SFString     name                ""
	  exposedField     SFRotation   rotation            0 0 1 0
	  exposedField     SFVec3f      scale               1 1 1
	  exposedField     SFRotation   scaleOrientation    0 0 1 0
	  exposedField     MFFloat      stiffness           [ 0 0 0 ]
	  exposedField     SFVec3f      translation         0 0 0
	  exposedField     MFFloat      ulimit              []
	  exposedField     MFFloat      uvlimit             []
	  exposedField     SFString     jointType           ""
	  exposedField     SFInt32      jointId             -1
	  exposedField     SFVec3f      jointAxis           0 0 1

	  exposedField     SFFloat      gearRatio           1
	  exposedField     SFFloat      rotorInertia        0
	  exposedField     SFFloat      rotorResistor       0
	  exposedField     SFFloat      torqueConst         1
	  exposedField     SFFloat      encoderPulse        1
	]
	{
	  Transform {
	    center           IS center
	    children         IS children
	    rotation         IS rotation
	    scale            IS scale
	    scaleOrientation IS scaleOrientation
	    translation      IS translation
	  }
	}

.. tabularcolumns:: |p{2.5cm}|p{12.5cm}|

.. list-table:: Jointノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - name
   - Joint名を指定するフィールドです。
 * - translation
   - ローカル座標系の位置を設定するフィールドです。親ノードからのオフセット値を指定します。
 * - rotation
   - ローカル座標系の姿勢を設定するフィールドです。親ノードからのオフセットを指定します。
 * - center
   - 関節回転中心の位置を指定するフィールドです。ローカル座標系原点からのオフセットで指定します。
 * - children
   - 子ノードをぶら下げるフィールドです。0個以上のJointノード、0または1個のSegmentノードをぶらさげます。
 * - jointType
   - 関節タイプを設定するためのフィールドです。free, slide, rotate, fixed, crawler のうちのいずれかを指定します。"free" は任意軸方向への並進・任意軸回りの回転が可能で、rootリンクが固定されないモデルのrootリンクに設定します（6自由度）。"rotate" はjointAxisで指定する軸回りの回転のみ可能です(1自由度)。"slide" はjointAxisで指定する軸方向への並進直動関節となります(1自由度)。"fixed" は関節を固定します(自由度なし)。"crawler"を指定すると、付随するリンクが簡易的なクローラとして機能するようになります。この詳細は :doc:`../../simulation/crawler-simulation` を参照してください。
 * - jointId
   - 関節番号を指定するためのフィールドです。 jointIdは関節角度等の属性値を配列形式に並べて格納する際に何番目の要素に入れるかを指定するために利用されます。多くの場合、ロボットのコントローラ開発において関節角度を読み取ったり、指定したりできるのは制御可能な関節のみですから、そのような関節にjointIdを付ける、と考えていただければよろしいかと思います（必ずそうでなければならないということではありません）。以下、Idのつけ方に関するルールを示します。jointIdは0から始まる。jointIdには連続した整数値を用いる（間が空いたり、重複したりしていないこと）。
 * - jointAxis
   - 関節の軸を指定するためのフィールドです。OpenHRPのバージョン2までは文字列の"X"、"Y"、"Z"のいずれかで軸を指定していましたが、 OpenHRP3以降ではベクトルを用いて任意方向への軸を指定可能となっています。 旧バージョンの指定法もサポートはされますが、今後は新しい指定法をお使いください。
 * - ulimit
   - 関節回転角度の上限値[rad]を指定するフィールドです。（デフォールト値："+∞"）
 * - llimit
   - 関節回転角度の下限値[rad]を指定するフィールドです。（デフォールト値："-∞"）
 * - uvlimit
   - 関節回転角速度の上限値[rad/s]を指定するフィールドです。（デフォールト値："+∞"）
 * - lvlimit
   - 関節回転角速度の下限値[rad/s]を指定するフィールドです。（デフォールト値："-∞"）
 * - gearRatio
   - ギヤ比: モータから関節までの減速比が1/100で あれば、100と記述します
 * - gearEfficiency
   - 減速器の効率。効率が 60%であれば0.6と記述します。 このフィールドがなければ、効率100%の減速器を想定します。
 * - rotorInertia
   - モータ回転子の慣性モーメント [kgm^2]


.. note:: ulimit, llimit, uvlimit, lvlimit については、シミュレーションでは通常使用されません。コントローラがこれらの値を読み込んで限界値を超えないように制御するために定義されているパラメータとなっています。

関節は、Jointノードを用いて定義します。Jointノードは、リンクフレームの情報を含みます。関節の親子関係は、そのままJointノードの親子関係に対応します。例えば人間の腕を考えたとき、「肩→肘→手首」の順に関節が存在するわけですから、この場合のリンク構造はJointノードを用いて、下図の様に定義します。

.. figure:: images/joint1.png 
	:align: center

	腕のリンク構造

1関節にn自由度(n≧2)を持たせたい場合、その関節は、原点が一致したn個の関節から構成されていると考えることが出来ます。この場合はリンクフレームの原点を重ねるようにしてJointをn個定義します。例えば人間の肘は下図のように2自由度存在すると考えられますから、この場合は、下図の様に定義します。

.. figure:: images/joint2.png
	:align: center

	肘のリンク構造

この場合は、以下のように定義します。

.. code-block:: yaml

	DEF 肘0 Joint {      #← 肘の曲げ
	  children [
	    DEF 肘1 Joint {  #← 肘のひねり

		:
		:
		:
	    }
	  ]
	  translation 0 0 0  #← 座標原点を合わせる
	}


Segmentノード
~~~~~~~~~~~~~

Segmentノードはリンク形状を定義します。

.. code-block:: yaml

	PROTO Segment [
	  field         SFVec3f   bboxCenter        0 0 0
	  field         SFVec3f   bboxSize          -1 -1 -1
	  exposedField  SFVec3f   centerOfMass      0 0 0
	  exposedField  MFNode    children          [ ]
	  exposedField  SFNode    coord             NULL
	  exposedField  MFNode    displacers        [ ]
	  exposedField  SFFloat   mass              0 
	  exposedField  MFFloat   momentsOfInertia  [ 0 0 0 0 0 0 0 0 0 ]
	  exposedField  SFString  name              ""
	  eventIn       MFNode    addChildren
	  eventIn       MFNode    removeChildren
	]
	{
	  Group {
	    addChildren    IS addChildren
	    bboxCenter     IS bboxCenter
	    bboxSize       IS bboxSize
	    children       IS children
	    removeChildren IS removeChildren
	  }
	}


.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: Segmentノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - bboxCenter
   - OpenHRPでは使用しません。
 * - bboxSize
   - OpenHRPでは使用しません。
 * - centerOfMass
   - 重心位置を指定するフィールドです。
 * - children
   - 子ノードをぶら下げるフィールドです。ここに、形状を定義するノードを追加します。
 * - coord
   - OpenHRPでは使用しません。
 * - displacers
   - OpenHRPでは使用しません。
 * - mass
   - 質量を指定するフィールドです。
 * - momentsOfInertia
   - 重心位置回りの慣性テンソルを指定するフィールドです。
 * - name
   - Segment名を指定するフィールドです。
 * - addChildren
   - OpenHRPでは使用しません。
 * - removeChildren
   - OpenHRPでは使用しません。


リンク形状は、Segmentノードに定義します。Segmentノードは、Jointノードの子ノードとして複数個設定でき、Transformノードの子ノードとして記述することも可能です。

.. code-block:: yaml

	DEF JOINT1 Joint {
	  children [
	    DEF SEGMENT1 Segment {
	      children [
		  :
	      ]
	    }
	    Transform {
	      translation 0 0 0.5
	      rotation 1 0 0 1.57
	      children DEF SEGMENT2 Segment {
		children [
		  :
		]
	      }
	    }
	  ]
	}


例えば、人間の肩から肘にかけての形状を定義したい場合、この形状が肩のリンクフレームに属するとすると、下図のようになります。

.. figure:: images/joint3.png
	:align: center

	肘のリンクフレーム

.. code-block:: yaml

	DEF 肩 Joint {
	  children [
	    DEF 肩から肘 Segment {
	      children [
		:
		:    #←ここに実際の形状を記述する
		:
	      ]
	    }
	    DEF 肘 Joint {
		:
		:
		:
	    }
	  ]
	}


Segmentノードのchildrenフィールド下に実際の形状を定義します。形状の定義にはモデリングツールを使用されることをお勧めします。簡単な形状であればテキストエディタを使用して手作業で編集することも可能です。

.. node::
	”Inline”と言う定義にて各Segmentごとの形状を異なるファイルに記述することもできます。
	
ExtraJointノード
~~~~~~~~~~~~~~~~

ExtraJointノードは閉リンク機構を定義します。閉リンクの1つの関節がボールジョイントで接続されていると考え、2つのリンクが離れないように拘束力を発生させます。

.. code-block:: yaml

	PROTO ExtraJoint [
	  exposedField SFString link1Name 	""
	  exposedField SFString link2Name 	""
	  exposedField SFVec3f  link1LocalPos 	0 0 0
	  exposedField SFVec3f  link2LocalPos 	0 0 0
	  exposedField SFString jointType 	"xyz"
	  exposedField SFVec3f  jointAxis 	1 0 0
	]
	{
	}


.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: ExtraJointノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - link1Name
   - ボールジョイントを受けているジョイント名を指定します。
 * - link2Name
   - ボールジョイントが付いているジョイント名を指定します。
 * - link1LocalPos
   - link1Nameジョイントの拘束位置をそのジョイントのローカル座標で指定します。
 * - link2LocalPos
   - link2Nameジョイントの拘束位置をそのジョイントのローカル座標で指定します。
 * - jointType
   - 拘束する軸数を指定します。xyz：互いに直交する3軸　xy：jointAxisで指定した軸に直交する２軸　z：jointAxisで指定した１軸
 * - jointAxis
   - link1Nameジョイントのローカル座標で単位ベクトルを指定します。ベクトルの意味は、jointTypeの指定で変わります。

	
閉リンク機構のサンプルとして "model/misc/ClosedLinkSample.wrl" が share ディレクトリにありますので、参考にして下さい。

.. _oepnrhp_modelfile_sensors:

各種センサを定義するノード
--------------------------

AccelerationSensorノード
~~~~~~~~~~~~~~~~~~~~~~~~

AccelerationSensorノードは、3軸加速度センサを定義します。

.. code-block:: yaml

	PROTO AccelerationSensor [
	  exposedField SFVec3f    maxAcceleration -1 -1 -1
	  exposedField SFVec3f    translation     0 0 0
	  exposedField SFRotation rotation        0 0 1 0
	  exposedField SFInt32    sensorId        -1
	]
	{
	  Transform {
	    translation IS translation
	    rotation    IS rotation
	  }
	}


.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: AccelerationSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - maxAcceleration
   - 計測可能な最大加速度を指定します。
 * - translation
   - ローカル座標系の位置を、親ノード座標系からのオフセット値で指定します。
 * - rotation
   - ローカル座標系の姿勢を、親ノード座標系からのオフセット値で指定します。
 * - sensorId
   - センサのIDを指定します。センサIDは一つのモデル内の同一種類のセンサに対して0番から順に番号の飛びや重複がないように設定して下さい。このIDは同一種類のセンサからのデータを並べる際に順番を決定するために使用されます。

	
各種センサノードはそのセンサが取り付けられているJointノードの下に取り付けます。 例えば、サンプルモデルの腰部(WAIST)に加速度センサを取り付けられている場合は、次のように記述します。

.. code-block:: yaml

	DEF WAIST Joint
	{
	     :
	  children [
	    DEF gsensor AccelerationSensor
	    {
		:
	    }
	     :
	  ]
	}


GyroSensorノード
~~~~~~~~~~~~~~~~

Gyroノードは、3軸角速度センサを定義します。

.. code-block:: yaml

	PROTO Gyro [
	  exposedField SFVec3f    maxAngularVelocity -1 -1 -1
	  exposedField SFVec3f    translation        0 0 0
	  exposedField SFRotation rotation           0 0 1 0
	  exposedField SFInt32    sensorId           -1
	]
	{
	  Transform {
	    translation IS translation
	    rotation    IS rotation
	  }
	}

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|
	
.. list-table::　GyroSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - maxAngularVelocity
   - 計測可能な最大角速度を指定します。
 * - translation
   - ローカル座標系の位置を、親ノード座標系からのオフセット値で指定します。
 * - rotation
   - ローカル座標系の姿勢を、親ノード座標系からのオフセット値で指定します。
 * - sensorId
   - センサのIDを指定します。

	
VisionSensorノード
~~~~~~~~~~~~~~~~~~

VisionSensorノードは、視覚センサを定義します。

.. code-block:: yaml

	PROTO VisionSensor
	[
	  exposedField  SFVec3f     translation       0 0 0
	  exposedField  SFRotation  rotation          0 0 1 0
	  exposedField  SFFloat     fieldOfView       0.785398
	  field         SFString    name              ""
	  exposedField  SFFloat     frontClipDistance 0.01
	  exposedField  SFFloat     backClipDistance  10.0
	  exposedField  SFString    type              "NONE"
	  exposedField  SFInt32     sensorId          -1
	  exposedField  SFInt32     width             320
	  exposedField  SFInt32     height            240
	  exposedField  SFFloat     frameRate         30
	]
	{
	  Transform
	  {
	    translation IS translation
	    rotation    IS rotation
	  }
	}

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: VisionSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - translation
   - 視点の位置を、親ノード座標系からの相対位置で指定します。
 * - rotation
   - 視点の姿勢を、親ノード座標系からの相対姿勢で指定します。視点の姿勢は以下のように定義されます。視線前方向 ・・・ ローカル座標系でZ軸の負の向き視線上方向 ・・・ ローカル座標系でY軸の正の向き。視線ベクトル
 * - fieldOfView
   - カメラの視野角度を指定します。単位はradで、(0、pi)の値が設定可能です。
 * - name
   - センサの名称を指定します。
 * - frontClipDistance
   - 視点から前クリップ面までの距離を指定します。
 * - backClipDistance
   - 視点から後クリップ面までの距離を指定します。
 * - type
   - センサから取得する情報の種類を指定します。"COLOR"色情報を取得します。"DEPTH"深さ情報を取得します。"COLOR_DEPTH"色情報と深さ情報を取得します。"NONE"いずれの情報も取得しません。
 * - sensorId
   - センサのIDを指定します。
 * - width
   - 画像の幅を指定します。
 * - height
   - 画像の高さを指定します。
 * - frameRate
   - カメラが毎秒何枚の画像を出力するかを指定します。

	
ForceSensorノード
~~~~~~~~~~~~~~~~~

ForceSensorノードは、力／トルクセンサを定義します。

.. code-block:: yaml

	PROTO ForceSensor [  
	  exposedField SFVec3f maxForce -1 -1 -1
	  exposedField SFVec3f maxTorque -1 -1 -1
	  exposedField SFVec3f translation 0 0 0
	  exposedField SFRotation rotation 0 0 1 0
	  exposedField SFInt32 sensorId -1
	]
	{
	  Transform {
	translation IS translation
	    rotation IS rotation
	  }
	}

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|
	
.. list-table:: ForceSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - maxForce
   - 計測可能な力の最大値を設定します。
 * - maxTorque
   - 計測可能なトルクの最大値を設定します。
 * - translation
   - ローカル座標系の位置を、親ノード座標系からのオフセット値で指定します。
 * - rotation
   - ローカル座標系の姿勢を、親ノード座標系からのオフセット値で指定します。
 * - sensorId
   - センサのIDを指定します。
	

RangeSensorノード
~~~~~~~~~~~~~~~~~

RangeSensorノードは、距離センサを定義します。

.. code-block:: yaml

	PROTO RangeSensor [
	   exposedField SFVec3f    translation       0 0 0
	   exposedField SFRotation rotation          0 0 1 0
	   exposedField MFNode     children          [ ]
	   exposedField SFInt32    sensorId          -1
	   exposedField SFFloat    scanAngle         3.14159 #[rad]
	   exposedField SFFloat    scanStep          0.1     #[rad]
	   exposedField SFFloat    scanRate          10      #[Hz]
	   exposedField SFFloat    maxDistance	    10
	]
	{
	   Transform {
	     rotation         IS rotation
	     translation      IS translation
	     children         IS children
	   }
	}

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: RangeSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1

 * - フィールド
   - 内容
 * - translation
   - このセンサが取り付けられているリンクに対するこのセンサの位置
 * - rotation
   - このセンサが取り付けられているリンクに対するこのセンサの姿勢。センサ座標系において、Z軸マイナス方向が計測正面、スキャンする場合の計測面はXZ平面となります。 これはVisionSensorと同じですので、従来VisionSensorで代用していたモデルを変更する場合は 位置、姿勢はそのまま使えます。
 * - sensorId
   - このロボットに取り付けられているRangeSensorの中での通し番号
 * - scanAngle
   - 距離をスキャンする角度[rad]。0度を中心として、その両側にscanStepの倍数の角度でscanAngleの範囲内の角度が計測されます。センサにスキャン機能がない場合は0とします。
 * - scanStep
   - スキャン中に距離が計測される角度の刻み[rad]
 * - scanRate
   - １秒間あたり行うスキャン回数[Hz]
 * - maxDistance
   - 計測可能な最大距離[m]

