ROS環境におけるChoreonoidの実行
===============================

.. highlight:: sh

.. _choreonoid_ros_run_ros_master:

ROSマスターの起動
-----------------

ROSのマスターを起動していない場合は、起動しておきます。（通常このための端末を新たに起動して、そこで実行します）。 ::

 roscore

.. _choreonoid_ros_run_choreonoid_node:

Choreonoidの起動
----------------

ROS環境においてChoreonoidは通常ROSノードとして扱われることになります。これをChoreonoidノードと呼びます。

Choreonoidノードを起動するコマンドはchoreonoid_rosパッケージに含まれており、コマンド名は choreonoid になります。従って、例えばrosrunコマンドを使用して、 ::

 rosrun choreonoid_ros choreonoid

とすることでChoreonoidをROSノードとして起動できます。

ノード起動時には、他のROSノードと同様に、ROSのリマップに関するオプションを付与することができます。

また、Choreonoid本体のオプションも付与することが可能です。


サンプルプロジェクトの実行
--------------------------

サンプルプロジェクトとして、choreonoid_ros_samples に含まれる "ROS-Tank" プロジェクトを実行してみましょう。

choreonoid_ros_samplesパッケージがインストールされていれば、以下のコマンドでサンプルを起動できます。 ::

 roslaunch choreonoid_ros_samples tank.launch

このサンプルは、ゲームパッドによって戦車風の "Tank" モデルを手動操作するというシミュレーションを、ROSの仕組みを用いて実現するというものです。このサンプルはこれを実現する最低限の実装となっており、ROSのjoyトピックをSubscribeするシンプルコントローラによってTankモデルの制御を行います。

まずサンプルの起動方法について紹介しましたが、実際にはこのサンプルを実行するにはPCにゲームパッド（ジョイスティック）が接続されていなければなりません。ゲームパッドはUSB接続できるものなら大抵使用可能ですので、まずはゲームパッドをPCに接続してから、上記のコマンドを実行するようにしてください。プロジェクトが起動するとシミュレーションも開始され、ゲームパッドの軸を操作することでTankを動かすことができます。このサンプルが動作すればChoreonoidのROS環境へのインストールは成功しています。


補足: roslaunch の実行に失敗する場合の対処方法
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

上記のroslaunchコマンドを実行しようとすると ::

 RLException: [tank.launch] is neither a launch file in package [choreonoid_ros_samples] nor is [choreonoid_ros_samples] a launch file name
 The traceback for the exception was written to the log file

といったエラーメッセージが表示されて、うまく実行できない場合があるようです。

この場合、 :ref:`loading_catkin_workspace_setup_script` を再度実行すると、実行できるようになる可能性があります。つまり、 ::

 source ~/catkin_ws/devel/setup.bash

を端末上で実行するか、あるいはこのスクリプトが ~/.bashrc に記述済みの場合は、端末を新たに起動してそこで操作します。するとワークスペースの情報が更新されて、launchファイルも認識され、実行できるようになることがあるようです。

どうもROS（Catkin）のワークスペースでは、新たなパッケージや、パッケージの新たな要素が追加になる場合に、 setup.bash で読み込まれる内容が更新されることがあるようです。そしてその更新が端末内に取り込まれていないと、そのような追加要素を対象とした操作がうまくいかないことがあるのでしょう。パッケージの新たに導入する際や、自前パッケージの開発中などには、この点注意する必要がありそうです。


補足: ゲームパッドのマッピングについて
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ゲームパッドの軸やボタンのマッピングは、メーカーや機種ごとに異なっており、マッピングによってはサンプルの操作体系と合わない場合もあります。上記のサンプルではなるべくマッピングを合わせるようになっており、

* `ロジクールF310 <http://gaming.logicool.co.jp/ja-jp/product/f310-gamepad>`_
* `DUALSHOCK4 <http://www.jp.playstation.com/ps4/peripheral/cuhzct1j.html>`_
* DUALSHOCK3
* `Xbox用コントローラ <https://www.xbox.com/ja-JP/xbox-one/accessories/controllers/xbox-black-wireless-controller>`_
* Xbox360用コントローラ

といったゲームパッドの場合はこれが機能します。これら以外のゲームパッドの場合は思うように操作できないかもしれませんが、ご了承ください。

roslaunchの実行内容
~~~~~~~~~~~~~~~~~~~

.. highlight:: xml

このサンプルはroslaunchを用いて複数のROSノードを起動することで実現しています。launchファイルは以下のようになっています。 ::

 <launch>
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
         args="$(find choreonoid_ros_samples)/project/ROS-Tank.cnoid --start-simulation" />
 </launch>

この記述によって、以下の2つのノードを起動しています。

* choreonoid_joy: ジョイスティック（ゲームパッド）の状態をjoyトピックとしてpublishするノード
* choreonoid: Choroenoid本体のノード

choreonoid_joyと同様の処理を行うノードとしてROS標準のjoyノードがあるのですが、そちらはゲームパッドのマッピングを合わせる機能はありません。choreonoid_joyはChoreonoidのライブラリを用いてゲームパッドのマッピングをChoreonoid標準のマッピングに合わせるようになっており、puslishされる情報はそれを反映したものとなっています。

Choreonoid本体については、choreonoid_ros_samplesのprojectディレクトリに含まれる "ROS-Tank.cnoid" というプロジェクトを読み込ませています。また、 "--start-simulation" オプションを付与することで、Choreonoid起動時に同時にシミュレーションも開始するようにしています。
