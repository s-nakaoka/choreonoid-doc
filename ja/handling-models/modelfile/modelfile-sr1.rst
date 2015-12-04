
SR1サンプルモデル
=================

ここでは、シンプルなヒューマノイドロボットのサンプルモデルである "SR1" モデルの表している内容について説明します。

このモデルはshareディレクトリの "model/SR1" ディレクトリ以下にある以下のファイルで記述されています。

* SR1.wrl: モデルファイル本体
* SR1.yaml: 追加情報ファイル

ここではモデルファイルの本体である "SR1.wrl" の記述内容について紹介します。追加情報ファイルの内容については、次の :doc:`modelfile-yaml` を参照してください。（Choreonoidにおいては、モデルの追加情報ファイルが用意されている場合は、通常そちらを読み込むようにします。）


関節構造
--------

SR1モデルは、 腰1関節(WAIST)、胴体4関節(WAIST_JOINT0〜CHEST)、 頭2関節(HEAD_JOINT0, HEAD_JOINT1)、 腕16関節(LARM_SHOULDER_P〜LARM_WRIST_R、RARM_SHOULDER_P〜RARM_WRIST_R)、 足14関節(LLEG_HIP_R〜LLEG_ANKLE_R、RLEG_HIP_R〜RLEG_ANKLE_R)で構成され、 次に示すJoint-Segment階層構造を持ちます。 Joint位置と各Jointに設定された座標系、Segment名と実際のリンクの対応については、下図を参照してください．

.. code-block:: yaml

	(Joint Jointノード名 : Segment Segmentノード名)
	Humanoid SAMPLE
	|
	| # Root
	+-humanoidBody
	  |
	  | # Upper half body
	  +-Joint WAIST : Segment WAIST_LINK0
	  |   Joint WAIST_P : Segment WAIST_LINK1
	  |     Joint WAIST_R : Segment WAIST_LINK2
	  |       Joint CHEST : Segment WAIST_LINK3
	  |       |
	  |       | # Cameras
	  |       +-VisionSensor LeftCamera
	  |       +-VisionSensor RightCamera
	  |       |
	  |       | # Left arm
	  |       +-Joint LARM_SHOULDER_P : Segment LARM_LINK1
	  |       |   Joint LARM_SHOULDER_R : Segment LARM_LINK2
	  |       |     Joint LARM_SHOULDER_Y : Segment LARM_LINK3
	  |       |       Joint LARM_ELBOW : Segment LARM_LINK4
	  |       |         Joint LARM_WRIST_Y : Segment LARM_LINK5
	  |       |           Joint LARM_WRIST_P : Segment LARM_LINK6
	  |       |             Joint LARM_WRIST_R : Segment LARM_LINK7
	  |       |
	  |       | # Right arm
	  |       +-Joint RARM_SHOULDER_P : Segment RARM_LINK1
	  |           Joint RARM_SHOULDER_R : Segment RARM_LINK2
	  |             Joint RARM_SHOULDER_Y : Segment RARM_LINK3
	  |               Joint RARM_ELBOW : Segment RARM_LINK4
	  |                 Joint RARM_WRIST_Y : Segment RARM_LINK5
	  |                   Joint RARM_WRIST_P : Segment RARM_LINK6
	  |                     Joint RARM_WRIST_R : Segment RARM_LINK7
	  |
	  | # Left Leg
	  +-Joint LLEG_HIP_R : Segment LLEG_LINK1
	  |   Joint LLEG_HIP_P : Segment LLEG_LINK2
	  |     Joint LLEG_HIP_Y : Segment LLEG_LINK3
	  |       Joint LLEG_KNEE : Segment LLEG_LINK4
	  |         Joint LLEG_ANKLE_P : Segment LLEG_LINK5
	  |           Joint LLEG_ANKLE_R : Segment LLEG_LINK6
	  |
	  | # Right Leg
	  +-Joint RLEG_HIP_R : Segment RLEG_LINK1
		Joint RLEG_HIP_P : Segment RLEG_LINK2
		  Joint RLEG_HIP_Y : Segment RLEG_LINK3
		    Joint RLEG_KNEE : Segment RLEG_LINK4
		      Joint RLEG_ANKLE_P : Segment RLEG_LINK5
			Joint RLEG_ANKLE_R : Segment RLEG_LINK6


.. figure:: images/SampleRobotJoint.png
	:align: center

	ロボットの各Jointの位置と座標系


.. figure:: images/SampleRobotSegment.png
	:align: center

	ロボットの各リンクと対応するSegment名


視覚センサ
----------

上記のとおり視覚センサは、CHESTの下に2個取り付けられています。

.. code-block:: yaml

	CHEST
	  +-VisionSensor LeftCamera
	  +-VisionSensor RightCamera

また、視覚センサの座標系は図6.の通りです。赤い軸がX軸、緑の軸がY軸、青の軸がZ軸を表します。視線方向は、Z軸のマイナス方向です。位置姿勢の具体的な数値については、sample.wrlを参照してください。

.. figure:: images/cameracs.png
	:align: center

	視覚センサの座標系
