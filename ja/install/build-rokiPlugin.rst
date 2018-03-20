
RokiPluginのビルド
==========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>


.. contents:: 目次
   :local:

.. highlight:: YAML

Roki のインストール
--------------------------------

RokiPluginを利用するためには、RoKi（ Robot Kinetics library） をインストールしておく必要があります。RoKiは、Linuxのみ対応しています。

最初にCUREをインストールします。

`Rokiホームページ <http://www.mi.ams.eng.osaka-u.ac.jp/open-j.html>`_ からCUREのページに移動し、cure-1.0.0-beta5.tgzをダウンロードし展開します。
 
READMEというファイルがありますので、基本的にはそれに従ってインストールします。

.. note:: デフォルトのインストール先は、 **ホーム/usr** です。以下は、ここにインストールする場合の説明です。インストール先を変更する場合はconfigファイルを編集し、以下の内容も適宜変更してください。
 
インストール先のファルダーは予め作成し、その中にbinとlibのファルダーも作成しておいたほうが良いようです。ターミナルを起動し、ホームディレクトリで、次のようにします。 ::

 mkdir usr
 cd usr
 mkdir bin
 mkdir lib

環境変数PATHとLD_LIBRARY_PATHにこのディレクトリを追記しておきます。ホームディレクトリにある .profile ファイルをエディタで開きます。ファイルの最後に以下の内容を貼り付け保存します。 ::

 export PATH=$PATH:$HOME/usr/bin
 export LD_LIBRARY_PATH=$HOME/usr/lib:$LD_LIBRARY_PATH

この設定を反映させるため、 ::

 source .profile
 
を実行します。Cureのソースを展開したディレクトリに移動し、以下を実行します。 ::

 make
 make install
 
成功していれば、ホーム/usr/binの下にcure-config、ホーム/usr/libの下にlibcure.soというファイルが作られているはずです。

次にZMのページに移動し、ファイルをダウンロードし、展開します。展開したディレクトリに移動して make, make install を行います。

Zeo、Rokiの順に同じようにインストールします。

プラグインのビルド
---------------------

choreonoidのビルドの際にCMakeの設定で、 **BUILD_ROKI_PLUGIN** という項目を "ON" にし、**ROKI_DIR** にRokiのインストール先を指定してください。

シミュレーションの実行
-------------------------

RokiPluginを用いたシミュレーションは :ref:`他の物理シミュレータ<simulation_creation_and_configuration_of_simulator_item>` を利用する方法と同様です。シミュレータアイテム「RokiSimulator」を生成し、ワールドアイテムの子アイテムとして配置することで実行可能となります。

RokiSimulatorアイテムのプロパティ
---------------------------------

RokiSimulatorアイテムには以下のプロパティがあります。

.. list-table:: RokiPluginプロパティ
 :widths: 15,60
 :header-rows: 1

 * - プロパティ
   - 内容
 * - staticfriction
   - 静止摩擦
 * - kineticfriction
   - 動摩擦
 * - contactType
   - 物体の接触時のタイプrigid（剛体）かelastic（弾性）を選択
 * - solverType
   - VertかVolumeを選択
 * - compensation
   - 接触パラメータ　contactTypeがrigidでuseContactFileがfalseのとき有効
 * - relaxation
   - 接触パラメータ　ontactTypeがrigidでuseContactFileがfalseのとき有効
 * - elasticity
   - 接触パラメータ　contactTypeがelasticでuseContactFileがfalseのとき有効
 * - viscosity
   - 接触パラメータ　contactTypeがelasticでuseContactFileがfalseのとき有効
 * - useContactFile
   - 接触パラメータの設定をファイルから読み込むか否か
 * - contactFileName
   - 接触パラメータの設定ファイル名　useContactFileがtrueのとき有効

パラメータの詳細については、 `Rokiホームページ <http://www.mi.ams.eng.osaka-u.ac.jp/open-j.html>`_ をご参照ください。

関節ダイナミクスのシミュレーション
-------------------------------------

Rokiでは、関節ダイナミクスのシミュレーションが可能です。このサンプルプロジェクトは、RokiArm2Dof.cnoidになります。

関節ダイナミクスのパラメータはモデルファイルarm_2dof.bodyに記述されています。このモデルでは、２つの関節に同じパラメータを適用するため、 :ref:`body-file-reference-link-node` の **import** を使用しています。エイリアス機能については :ref:`modelfile_yaml_alias` をご覧ください。関節毎に異なるパラメータを設定する場合は、Linkノードに直接記入します。 ::

 actuator1: &actuator1
   rotorInertia: 1.65e-6
   gearRatio: 120.0
   gearInertia: 5.38e-6
   motorAdmittance: 0.42373
   motorConstant: 2.58e-2
   motorMinVoltage: -24.0
   motorMaxVoltage: 24.0
   jointStiffness: 0.0
   jointViscosity: 2.2
   jointFriction: 4.32
   jointStaticFriction: 4.92
  
 links:
    .......
   -
     name: Joint1
      .......
     import: *actuator1
      .......
   -
     name: Joint2
      .......
     import: *actuator1
      .......
      
関節パラメータは以下の通りです。

.. list-table:: 
 :widths: 15,40
 :header-rows: 1

 * - パラメータ
   - 内容
 * - motorconstant
   - モータ定数（トルク定数）
 * - admitance
   - 端子間アドミッタンス（端子間抵抗の逆数）
 * - minvoltage
   - 最小入力電圧
 * - maxvoltage
   - 最大入力電圧
 * - inertia
   - モータ回転子慣性モーメント
 * - gearinertia
   - 減速機慣性モーメント
 * - ratio
   - 減速比
 * - stiff
   - 関節剛性係数
 * - viscos
   - 関節粘性係数
 * - coulomb
   - 関節乾性係数（動摩擦トルク）
 * - staticfriction
   - 最大静止摩擦トルク

破壊のシミュレーション
-----------------------------

Rokiでは、破断の起こる箇所を関節としてモデルファイルに記述しておくことで、破断のシミュレーションが可能です。このサンプルのプロジェクトは、RokiBreakWall.cnoidになります。

破断するモデルはbreakWall.bodyに記述されています。破断の起こる箇所を関節として定義し、関節のタイプはfreeとします。そして、 **break** パラメータに破断が起こる力・トルクのノルム閾値を、力、トルクの順に記述します。 ::

 links :
   -
    name: BASE
    jointType: fixed
     ................
    elements:
      Shape:
        geometry: { type: Box, size: [ 0.099, 0.049, 0.099 ] }
   -
    name: link1
    parent: BASE
    translation : [ 0, 0, 0.05 ]
    jointType: free
      .............
    break: [ 200.0, 200.0 ]
      .............
    elements:
      Shape:
        geometry: { type: Box, size: [ 0.099, 0.049, 0.099 ] }
  -
    name: link2
    parent: link1
    translation : [ 0, 0, 0.1 ]
    jointType: free
      .............
    break: [ 10.0, 10.0 ]
      .............
      
破断した後の物体が、お互いにすり抜けてしまわないように、breakWallモデルアイテムの自己干渉検出のプロパティはtrueにする必要があります。しかし、それでは、破断前にも自己干渉がおきてしまいます。これを避けるため、breakWallモデルでは、リンクの幾何形状を、リンク間に僅かな隙間ができるように設定しています。

RokiSimulationaアイテムの全リンク位置姿勢出力のプロパティをtrueにします。

