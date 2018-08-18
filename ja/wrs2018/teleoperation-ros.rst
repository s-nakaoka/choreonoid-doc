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

Ubuntu 16.04 の場合、以下のようにしてインストールできます。 ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
 sudo apt-get update
 sudo apt-get install ros-kinetic-desktop-full
 sudo rosdep init
 rosdep update
 echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc

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

* choreonoid: Choreonoid本体
* choreonoid_rosplugin: ChoreonoidでROSを使用するためのROSプラグイン
* choreonoid_ros_samples: ChoreonoidでROSを使用するサンプル
* choreonoid_joy: ジョイスティック（ゲームパッド）をChoreonoidのマッピングで使うためのROSノード

これらがワークスペースで扱うパッケージのソースになります。各リポジトリの内容はなるべく最新に保つようにしてください。

複数のリポジトリの更新等を一括して行うためのツールとして、 `wstool <http://wiki.ros.org/wstool>`_ や `Vcstool <https://github.com/dirk-thomas/vcstool>`_  があります。個人的には、Vcstoolの方が使い勝手が良いように思います。Vcstoolを使う場合は、 ::

 sudo apt install python3-vcstool

でインストールできます。

使い方は ::

 vcs help

で確認してください。

例えば全てのリポジトリに対してgit pullを実行したい場合は、 ::

 vcs pull

とします。

パッケージのビルド
------------------

ワークスペース上のパッケージのビルドを行います。

まだ通常の :ref:`wrs2018_install_choreonoid` を行っていない場合は、念の為Choreonoidのパッケージインストールスクリプトを実行しておきましょう。choreonoidのソースディレクトリに移動し、 ::

 misc/script/install-requisites-ubuntu-16.04.sh

を実行します。(Ubuntu 18.04の場合は、install-requisites-ubuntu-18.04.sh を実行します。）

本来はCatkin用の依存パッケージ情報で解決すべきなのですが、そこがまだ完全でない可能性があるため、念の為これを実行しておいていただけるとよいかと思います。（既にChoreonoidをインストール済みの場合は必要ありません。）

.. note:: 通常の :ref:`wrs2018_install_choreonoid` は、ROSを使用しない場合のインストール方法です。ROSを使う場合は本ページのやり方でインストールしていただければOKです。ただし、ディレクトリ構成や実行ファイルにパスが通っているかどうかといった点は異なってきますので、これまで紹介したサンプルのについてはその点を加味して試すようにしてください。

次に、CMakeのオプションを設定します。 :ref:`wrs2018_install_choreonoid` で示したように、WRS2018のシミュレーションを実行するにあたってはChoreonoidのオプション機能がいくつか必要となり、これをCMakeのオプションで有効にしました。具体的には、

* BUILD_AGX_DYNAMICS_PLUGIN
* BUILD_AGX_BODYEXTENSION_PLUGIN
* BUILD_SCENE_EFFECTS_PLUGIN
* BUILD_MULTICOPTER_PLUGIN
* BUILD_MULTICOPTER_SAMPLES
* ENABLE_CORBA
* BUILD_CORBA_PLUGIN
* BUILD_OPENRTM_PLUGIN
* BUILD_OPENRTM_SAMPLE

といったオプションです。

catkin上でのビルドの場合、このようなオプションの設定はワークスペースの設定として行います。具体的にはcatkin configに --cmake-argsオプションを与えて、 ::

 catkin config -cmake-args BUILD_AGX_DYNAMICS_PLUGIN -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON -DBUILD_OPENRTM_SAMPLES=ON

のように設定します。

設定後 ::

 catkin config

を実行すると、ワークスペースの設定が表示されます。そこに ::

 Additional CMake Args:  -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DENABLE_CORBA=ON
 -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON -DBUILD_OPENRTM_SAMPLES=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON

といった表示があればOKです。

.. note:: このように設定すると、ワークスペースの全てのパッケージに対してこれらのオプションが有効になってしまい、他のパッケージで意図しないオプションが有効になってしまうこともあり得ます。しかしCatkinではパッケージごとに個別にCMakeのオプションを設定する機能が無い（ `要望はあるものの見送られている <https://github.com/catkin/catkin_tools/issues/205>`_ ）ようですので、やむを得ずこのようにしています。

設定が完了したら、ビルドを行いましょう。ワークスペース内のディレクトリであれば、以下のコマンドでビルドできます。 ::

 catkin build

ビルド方法の詳細については `Catkin Command Line Tools のマニュアル <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ を参照してください。

ビルドに成功すると、 ::

 [build] Summary: All 4 packages succeeded!

といった表示がされます。

ビルドをすると、 ワークスペースのdevelディレクトリに "setup.bash" というファイルが生成されます。このスクリプトに記述されている設定は、ワークスペース内のパッケージを実行したりする際に必要となりますので、デフォルトで実行されるようにしておきます。通常はホームディレクトリの .bashrc ファイルに ::

 source $HOME/catkin_ws/devel/setup.bash

という記述を追加しておきます。

すると端末起動時に自動でこのファイルが実行され、設定が読み込まれるようになります。

初回ビルド時はまだこの設定が取り込まれていませんので、端末を起動し直すか、上記のコマンドをコマンドラインから直接入力して、設定を反映させるようにしてください。


Choreonoidの実行
----------------

まずROSのマスターを起動していない場合は、起動しておきます。 ::

 roscore

Catkinワークスペース上でビルドした場合、上記のsetup.bashスクリプトにより、実行ファイルへのパスは通っている状態です。従って、ディレクトリのどこでも、単にchoreonoidと入力すればChoreonoidが起動します。 ::

 choreonoid


