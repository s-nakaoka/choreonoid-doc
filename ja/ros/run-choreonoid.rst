ROS環境におけるChoreonoidの実行
===============================

.. highlight:: sh

ROSマスターの起動
-----------------

ROSのマスターを起動していない場合は、起動しておきます。（通常このための端末を新たに起動して、そこで実行します）。 ::

 roscore

Choreonoidの起動
----------------

ROS環境においてChoreonoidは通常ROSノードとして扱われることになります。

ChoreonoidのROSノードはchoreonoid_rosパッケージに含まれており、ノード起動コマンドは choreonoid になります。従って、例えばrosrunコマンドを使用して、 ::

 rosrun choreonoid_ros choreonoid

とすることでChoreonoidをROSノードとして起動できます。

ノード起動時には、他のROSノードと同様に、ROSのりマップに関するオプションを付与することができます。

また、Choreonoid本体のオプションも付与することが可能です。


サンプルプロジェクトの実行
--------------------------

サンプルプロジェクトとして、choreonoid_ros_samples に含まれる "ROS-Tank" プロジェクトを実行してみましょう。

choreonoid_ros_samplesパッケージがインストールされていれば、以下のコマンドでサンプルを起動できます。 ::

 roslaunch choreonoid_ros_samples tank.launch

このサンプルは、ゲームパッドによって戦車風の "Tank" モデルを手動操作するというシミュレーションを、ROSの仕組みを用いて実現するというものです。このサンプルはこれを実現する最低限の実装となっており、ROSのjoyトピックをSubscribeするシンプルコントローラによってTankモデルの制御を行います。

まずサンプルの起動方法について紹介しましたが、実際にはこのサンプルを実行するにはPCにゲームパッド（ジョイスティック）が接続されていなければなりません。ゲームパッドはUSB接続できるものなら大抵使用可能ですので、まずはゲームパッドをPCに接続してから、上記のコマンドを実行するようにしてください。プロジェクトが起動するとシミュレーションも開始され、ゲームパッドの軸を操作することでTankを動かすことができます。このサンプルが動作すればChoreonoidのROS環境へのインストールは成功しています。

ゲームパッドについて
~~~~~~~~~~~~~~~~~~~~

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
