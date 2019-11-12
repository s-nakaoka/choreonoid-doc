ROSによる遠隔操作サンプル
=========================

ここではROSを用いて遠隔操作を行うサンプルについて紹介します。 :doc:`teleoperation-rtm` と同様、 :doc:`simulation-samples` を遠隔操作化するものとなっています。

.. contents::
   :local:

.. highlight:: sh

ROS環境のセットアップ
---------------------

ROS環境のセットアップ方法について :doc:`../ros/index` にまとめていますので、そちらの説明に従って必要なROSパッケージのインストール（ビルド）を行ってください。

なお、 :ref:`wrs2018_install_choreonoid` で示したように、WRS2018のシミュレーションを実行するにあたってはChoreonoidのオプション機能がいくつか必要となります。具体的にはCMakeで設定する以下のオプションになります。

* BUILD_WRS2018
* BUILD_AGX_DYNAMICS_PLUGIN
* BUILD_AGX_BODYEXTENSION_PLUGIN
* BUILD_SCENE_EFFECTS_PLUGIN
* BUILD_MULTICOPTER_PLUGIN
* BUILD_MULTICOPTER_SAMPLES
* BUILD_COMPETITION_PLUGIN

これと :ref:`note_on_ros_python_version` で説明している Pythonバージョン2への切り替え（USE_PYTHON3オプションの無効化）を合わせて、catkin config で以下のオプション設定をしておきます。 ::

 catkin config --cmake-args -DBUILD_WRS2018=ON -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DBUILD_COMPETITION_PLUGIN=ON -DUSE_PYTHON3=OFF

.. _teleoperation_ros_build_packages:

遠隔操作サンプルの実行
----------------------

ROSを用いた遠隔操作のサンプルは、 :doc:`simulation-samples` で紹介したサンプルに "-ROS" のサフィックスをつけた名前で提供しています。

例えばタスクT1についてAizuSpiderやDoubleArmV7のモデルに対して、以下のようなスクリプトがあります。

* T1M-AizuSpiderSS-ROS.py
* T1M-AizuSpiderSA-ROS.py
* T1L-DoubleArmV7S-ROS.py
* T1L-DoubleArmV7A-ROS.py

:doc:`simulation-samples` で説明したのと同じ要領で、ROS版のサンプルを読み込んでください。サンプルはCatkinワークスペース内のdevel以下に格納されているものを使用します。例えば ::

 cd (ワークスペースディレクトリ)/devel/share/choreonoid-1.8
 rosrun choreonoid_ros choreonoid WRS2018/script/T1M-AizuSpiderSS-ROS.py

などとします。

遠隔操作用のノードやツールも起動しておく必要があります。まず操作をゲームパッドで行うため、ゲームパッドを接続した上で、choreonoid_joyパッケージのノードを以下のように起動します。 ::

 rosrun choreonoid_joy node

これでゲームパッドの状態がトピックとして配信されるようになります。

これはROSのjoyパッケージと同様の機能を果たすものなのですが、軸やボタンのマッピングがChoreonoid標準になるという点が異なります。対応しているゲームパッドであれば、機種によらず軸やボタンのマッピングが同じになります。Choreonoidのサンプルはこのマッピングで作られているため、それらを動かす際にはこのchoreonoid_joyを使うのがよいです。

次にカメラ画像の表示をできるようにしましょう。これはいろいろなやり方があるかと思いますが、ここでは rqt_image_view ツールを使うことにします。以下のようにしてこれを起動してください。 ::

 rosrun rqt_image_view rqt_image_view

このツールの左上にどのトピックの画像データを表示するか指定するコンボボックスがありますので、そこで表示したいカメラ画像を指定します。AizuSpiderの場合、 "/AizuSpider/FRONT_CAMERA/image" を選択してください。

以上で準備は完了です。Choreonoid上でシミュレーションを開始してください。うまくいけば、rqt_image_view上にAizuSpiderのカメラ画像が表示されます。また、ゲームパッドでロボットを操作できるようになります。

DoubleArmV7のサンプルも同様に実行することができます。DoubleArmV7の場合、カメラ画像のトピックは "/DoubleArmV7/FRAME_FRONT_CAMERA/image" を選択してください。

.. note:: 本サンプルでは上述のトピックに対応するカメラ画像のみがシミュレートされています。他のカメラの画像もシミュレートしたい場合は、 :doc:`../simulation/vision-simulation` を参照の上、 "GLVisionSimulator" アイテムの設定を行ってください。ただしシミュレート対象のカメラを増やすと、シミュレーションが遅くなる可能性があります。

PC2台を用いた遠隔通信
---------------------

ROSの場合でも当然シミュレーション側と操作側を別々のPCとすることが可能です。

その場合、シミュレーション用のPCでChoreonoidのシミュレーションプロジェクトを起動し、遠隔操作用のPCでchoreonoid_joyノードとrqt_image_viewを起動します。

2つのPC間でROSノードが通信できるようにするため、共通のROSマスターを使用する必要があります。

概要としては、ROSマスターを設置するホスト(PC)を決め、そちらでroscoreを起動します。そしてもう一方のPCでは、環境変数 ROS_IPに自身のIPアドレスを、ROS_MASTER_URI にマスターのアドレスを設定しておきます。

例えば、

* シミュレーション用PCをマスターとする
* シミュレーション用PCのIPアドレス: 192.168.0.10
* 操作用PCのIPアドレス: 192.168.0.20

という構成の場合は、シミュレーション用PCでroscoreを起動し、操作用PCでは、 ::

 export ROS_IP=192.168.0.20
 export ROS_MASTER_URI=http://192.168.0.10:11311

とします。（ホスト名でアドレスが引けるようになっている場合は、IPアドレスではなくホスト名で指定してもOKです。）

設定が完了したら、シミュレーション用PCのChoreonoidでシミュレーションを開始します。すると遠隔操作用PCのrqt_image_viewにカメラ画像が表示され、遠隔操作用PCに接続されているゲームパッドでロボットの操作ができるようになるはずです。





