ROSによる遠隔操作サンプル
=========================

ここではROSを用いて遠隔操作を行うサンプルについて紹介します。 :doc:`teleoperation-rtm` と同様、 :doc:`simulation-samples` を遠隔操作化するものとなっています。

.. contents::
   :local:

.. highlight:: sh

ROSのインストール
-----------------

ROSのインストールがまだの方は、 `ROS.org <http://wiki.ros.org>`_ - `ROS/Installation <http://wiki.ros.org/ROS/Installation>`_ の記述に従ってインストールを行ってください。

ROSのバージョンについては、Ubuntu 16.04上でKinetic、Ubuntu 18.04上でMelodicでの動作を確認をしています。

Ubuntu 16.04の場合、以下のようにしてインストールできます。 ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
 sudo apt-get update
 sudo apt-get install ros-kinetic-desktop-full
 sudo rosdep init
 rosdep update
 echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc

Ubuntu 18.04の場合は、以下のようにしてインストールできます。 ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
 sudo apt update
 sudo apt install ros-melodic-desktop-full
 sudo rosdep init
 rosdep update
 echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
 source ~/.bashrc

.. note:: 最後の source コマンドは、setup.bash の内容を現在のシェルに反映させるためのもので、インストール（上記設定）直後に続けて同じシェルで作業する場合に必要となるものです。インストール後にあらためてシェルを起動する場合は、上記の設定によりsetup.bashの内容が反映されますので、このコマンドは必要ありません。

ChoreonoidをROSで使う場合、ビルドツールCatkinの新しいバージョン ( `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_）を使用します。これは ::

 sudo apt install python-catkin-tools

でインストールすることができます。

Catkinワークスペースの作成
--------------------------

Choreonoid用のCatkinワークスペースを作成します。

ワークスペースは通常これはホームディレクトリ上に作成します。ワークスペースの名前は通常 "catkin_ws" とします。この名前は変更しても結構ですが、その場合は以下の説明の "catkin_ws" をその名前に置き換えるようにしてください。

まず空のワークスペースを作成します。 ::

 mkdir catkin_ws
 cd catkin_ws
 mkdir src
 catkin init

パッケージソースの追加
----------------------

作成したワークスペースの "src" ディレクトリ内に、Choreonoid関連ソースのリポジトリをクローンします。 ::

 cd src
 git clone https://github.com/s-nakaoka/choreonoid.git
 git clone https://github.com/s-nakaoka/choreonoid_rosplugin.git
 git clone https://github.com/s-nakaoka/choreonoid_ros_samples.git
 git clone https://github.com/s-nakaoka/choreonoid_joy.git

ここで、各リポジトリは以下のソフトウェアに対応します。

* `choreonoid <https://github.com/s-nakaoka/choreonoid>`_ : Choreonoid本体
* `choreonoid_rosplugin <https://github.com/s-nakaoka/choreonoid_rosplugin>`_ : ChoreonoidでROSを使用するためのROSプラグイン
* `choreonoid_ros_samples <https://github.com/s-nakaoka/choreonoid_ros_samples>`_ : ChoreonoidでROSを使用するサンプル
* `choreonoid_joy <https://github.com/s-nakaoka/choreonoid_joy>`_ : ジョイスティック（ゲームパッド）をChoreonoidのマッピングで使うためのROSノード

これらがワークスペースで扱うパッケージのソースになります。各リポジトリの内容はなるべく最新に保つようにしてください。

複数のリポジトリの更新等を一括して行うためのツールとして、 `wstool <http://wiki.ros.org/wstool>`_ や `vcstool <https://github.com/dirk-thomas/vcstool>`_  があります。個人的には、vcstoolの方が使い勝手が良いように思います。vcstoolを使う場合は、 ::

 sudo apt install python3-vcstool

でインストールできます。

使い方は ::

 vcs help

で確認してください。

例えば全てのリポジトリに対してgit pullを実行したい場合は、 ::

 vcs pull

とします。

.. _teleoperation_ros_build_packages:

パッケージのビルド
------------------

ワークスペース上のパッケージのビルドを行います。

まだ通常の :ref:`wrs2018_install_choreonoid` を行っていない場合は、念の為Choreonoidのパッケージインストールスクリプトを実行しておきましょう。choreonoidのソースディレクトリに移動し、 ::

 misc/script/install-requisites-ubuntu-16.04.sh

もしくは ::

 misc/script/install-requisites-ubuntu-18.04.sh

を実行します。(使用しているUbuntuのバージョンに合うものを実行してください。）

本来はCatkin用の依存パッケージ情報で解決すべきなのですが、そこがまだ完全でない可能性があるため、念の為これを実行しておいていただけるとよいかと思います。（既にChoreonoidをインストール済みの場合は必要ありません。）

.. note:: 通常の :ref:`wrs2018_install_choreonoid` は、ROSを使用しない場合のインストール方法です。ROSを使う場合は本ページのやり方でインストールしていただければOKです。ただし、ディレクトリ構成や実行ファイルにパスが通っているかどうかといった点は異なってきますので、これまで紹介したサンプルのについてはその点を加味して試すようにしてください。

次に、CMakeのオプションを設定します。 :ref:`wrs2018_install_choreonoid` で示したように、WRS2018のシミュレーションを実行するにあたってはChoreonoidのオプション機能がいくつか必要となり、これをCMakeのオプションで有効にしました。具体的には、

* BUILD_WRS2018
* BUILD_AGX_DYNAMICS_PLUGIN
* BUILD_AGX_BODYEXTENSION_PLUGIN
* BUILD_SCENE_EFFECTS_PLUGIN
* BUILD_MULTICOPTER_PLUGIN
* BUILD_MULTICOPTER_SAMPLES
* BUILD_COMPETITION_PLUGIN

といったオプションです。

他にOpenRTMも併用したい場合は以下のオプションが必要となりますが、ミドルウェアとしてはROSのみを使用する場合は必要ありません。

* ENABLE_CORBA
* BUILD_CORBA_PLUGIN
* BUILD_OPENRTM_PLUGIN
* BUILD_OPENRTM_SAMPLE

注意点として、ROSのKineticはPythonのバージョン2.7を使いますが、ChoreonoidはデフォルトでPython3を使うようになっています。この場合、Pythonバージョン2と3の共有ライブラリが競合するせいか、落ちてしまうことがあるようです。そこで、CMakeの以下のオプションについても設定します。

* USE_PYTHON3: ONだとPython3、OFFだとPython2を使用する

ROS Kineticでは、これをOFFとしなければなりません。ROS MelodicはPython3を使用するようなので、このオプションはデフォルトのONのままにしておいてください。

catkin上でのビルドの場合、このようなオプションの設定はワークスペースの設定として行います。具体的にはcatkin configに --cmake-argsオプションを与えて、 ::

 catkin config --cmake-args -DBUILD_WRS2018=ON -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DBUILD_COMPETITION_PLUGIN=ON -DUSE_PYTHON3=OFF

のように設定します。

Melodicでは最後の -DUSE_PYTHON3=OFF を除去して、 ::

 catkin config --cmake-args -DBUILD_WRS2018=ON -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DBUILD_COMPETITION_PLUGIN=ON

などとしてください。

設定後 ::

 catkin config

を実行すると、ワークスペースの設定が表示されます。そこに ::

 Additional CMake Args:  -DBUILD_WRS2018=ON -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON
 -DBUILD_COMPETITION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DUSE_PYTHON3=OFF 

といった表示があればOKです。

.. note:: このように設定すると、ワークスペースの全てのパッケージに対してこれらのオプションが有効になってしまい、他のパッケージで意図しないオプションが有効になってしまうこともあり得ます。しかしCatkinではパッケージごとに個別にCMakeのオプションを設定する機能が無い（ `要望はあるものの見送られている <https://github.com/catkin/catkin_tools/issues/205>`_ ）ようですので、やむを得ずこのようにしています。

設定が完了したら、ビルドを行いましょう。ワークスペース内のディレクトリであれば、以下のコマンドでビルドできます。 ::

 catkin build

ビルド方法の詳細については `Catkin Command Line Tools のマニュアル <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ を参照してください。

.. note:: Catkin環境であっても、Emacsから "M-x compile" コマンドを使ってビルドすることも可能です。その場合は、"M-x compile" 実行時にビルド用のコマンドとして "catkin build --no-color" を入力します。"--no-color" を入れることで、Cakin出力の色付け用の制御コードを無効化し、表示が汚くなることを回避できます。また、"-v" オプションを追加して "catkin build -v --no-color" とすることで、ビルド時に実際のコマンド（コンパイルオプションなど）を確認することができます。

ビルドに成功すると、 ::

 [build] Summary: All 4 packages succeeded!

といった表示がされます。

ビルドをすると、 ワークスペースのdevelディレクトリに "setup.bash" というファイルが生成されます。このスクリプトに記述されている設定は、ワークスペース内のパッケージを実行したりする際に必要となりますので、デフォルトで実行されるようにしておきます。通常はホームディレクトリの .bashrc ファイルに ::

 source $HOME/catkin_ws/devel/setup.bash

という記述を追加しておきます。

すると端末起動時に自動でこのファイルが実行され、設定が読み込まれるようになります。

初回ビルド時はまだこの設定が取り込まれていませんので、端末を起動し直すか、上記のコマンドをコマンドラインから直接入力して、設定を反映させるようにしてください。

.. note:: Catkinの設定スクリプトを実行すると、Catkin環境外で別途インストールしているChoreonoidの実行に影響することがあるので注意が必要です。これはCatkinの設定スクリプトにより、共有ライブラリのパスにCatkinワークスペースのdevel/libディレクトリが加わる(環境変数 LD_LIBRARY_PATH にこのパスが追加される）のが原因です。 この設定により、Catkin環境外のChoreonoidを実行する際に、Catkin内で生成されているChoreonoidの共有ライブラリを読み込んでしまうことがあります。その場合、ソースコードのバージョンやビルド設定などに違いがあると、Choreonoidがうまく動かなかったり、落ちてしまったりします。つまり、異なる環境でビルドしたものを混ぜてはいけないということになります。この問題を避けるためには、Catkin外のChoreonoidを実行する際にはCatkinの設定スクリプトは無効化しておきます。（ChoreonoidではRPATHという仕組みがデフォルトで使用されており、これによってこのような問題も避けられるはずなのですが、環境によってはうまく機能しないことがあるようです。）


Choreonoidの実行
----------------

まずROSのマスターを起動していない場合は、起動しておきます。 ::

 roscore

Catkinワークスペース上でビルドした場合、上記のsetup.bashスクリプトにより、実行ファイルへのパスは通っている状態です。従って、ディレクトリのどこでも、単にchoreonoidと入力すればChoreonoidが起動します。 ::

 choreonoid

Cakin上でビルドした場合、サンプルのファイルは "catkin_ws/devel/share/choreonoid-1.8" 以下にインストールされます。
このディレクトリに ::

 cd ~/catkin_ws/devel/share/choreonoid-1.8

などとして移動して、 ::

 choreonoid WRS2018/script/T1M-AizuSpiderSS.py

と入力することで、 :doc:`simulation-samples` を実行できます。以下の説明でもこの方法でプロジェクトを実行するものとします。

遠隔操作サンプルの実行
----------------------

ROSを用いた遠隔操作のサンプルは、 :doc:`simulation-samples` で紹介したサンプルに "-ROS" のサフィックスをつけた名前で提供しています。

例えばタスクT1についてAizuSpiderやDoubleArmV7のモデルに対して、以下のようなスクリプトがあります。

* T1M-AizuSpiderSA-ROS.py
* T1M-AizuSpiderSS-ROS.py
* T1L-DoubleArmV7A-ROS.py
* T1L-DoubleArmV7S-ROS.py

:doc:`simulation-samples` で説明したのと同じ要領で、ROS版のサンプルを読み込んでください。例えば ::

 choreonoid WRS2018/script/T1M-AizuSpiderSA-ROS.py

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





