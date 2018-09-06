
オプション機能
==============

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

ここでは、Choreonoidのビルド設定において選択可能なオプション機能について、主要なものの概要を記します。

オプション機能はCMakeの設定で有効化します。この操作については、 :doc:`build-ubuntu` の :ref:`build-ubuntu-cmake` もしくは :doc:`build-windows` の :ref:`build-windows-cmake` を参照してください。

* **BUILD_ASSIMP_PLUGIN**

 Assimpライブラリを利用してCOLLADA等の3Dモデルファイルを読み込めるようにするAssimpプラグインをビルドします。デフォルトでONになっています。

* **ENABLE_PYTHON**

 各モジュール、プラグインにPythonラッパが用意されている場合、それをビルドします。利用にあたってはPythonの開発用ライブラリもインストールしておく必要があります。

 モジュールについてはPythonラッパを併用することでPythonからコレオノイドのライブラリ機能が利用可能になります。例えばBodyモジュールのPythonラッパである "cnoid.Body" モジュールをインポートすることで、Python上でロボットモデルファイルを読み込んで各種計算を行うこともできます。プラグインのPythonラッパについては、以下で説明するPythonプラグインと併用することで、コレオノイド上でプラグインの機能をPythonスクリプトから利用することが可能となります。

 このオプションはデフォルトでONになっています。

* **BUILD_PYTHON_PLUGIN**

 Pythonプラグインをビルドします。ENABLE_PYTHON も ON にしておく必要があります。本プラグインにより、Pythonスクリプトの読み込み・実行や、コレオノイド上で動作するPythonコンソール等の機能が利用可能になります。デフォルトでONになっています。

* **BUILD_PYTHON_SIM_SCRIPT_PLUGIN**

 シミュレーションのセットアップ等にPythonスクリプトを利用するためのPythonSimScriptプラグインをビルドします。Pythonプラグインのビルドも必要です。デフォルトでONになっています。

* **BUILD_POSE_SEQ_PLUGIN**

 キーポーズによる振り付け機能を提供するPoseSeqプラグインをビルドします。デフォルトでONとなっています。

* **BUILD_BALANCER_PLUGIN**

 振り付け機能において自動バランス補正を行う機能を提供するBalancerプラグインをビルドします。二足歩行ロボットの振り付けを行う際にはこの機能を使うことで（理論上は）転倒しない動作を作成することが出来ます。デフォルトでONになっています。

* **BUILD_ODE_PLUGIN**

 ODEプラグインをビルドします。このプラグインにより、オープンソースの動力学計算ライブラリである "Open Dynamics Engine (ODE)" を、コレオノイドのシミュレーション機能の計算エンジンとして利用できます。利用の際には、 `Open Dynamics Engine (ODE) <http://www.ode.org/>`_ をインストールしておく必要があります。

* **BUILD_BULLET_PLUGIN**

 Bulletプラグインをビルドします。このプラグインにより、動力学計算ライブラリである "Bullet Physics" ライブラリを、コレオノイドのシミュレーション機能の計算エンジンとして利用できます。Bulletの詳細については `Bullet Physics Library <http://bulletphysics.org>`_ を、ビルド方法については、 :doc:`build-bulletPlugin` をご覧ください。
 
* **BUILD_ROKI_PLUGIN**

 ROKIプラグインをビルドします。このプラグインにより、動力学計算ライブラリ"RoKi"をコレオノイドのシミュレーション機能の計算エンジンとして利用できます。ROKIの詳細については `Rokiホームページ <http://www.mi.ams.eng.osaka-u.ac.jp/open-j.html>`_ を、ビルド方法については、 :doc:`build-rokiPlugin` をご覧ください。

* **BUILD_SPRINGHEAD_PLUGIN**

 Springheadプラグインをビルドします。このプラグインにより、動力学計算ライブラリ"Springhead"をコレオノイドのシミュレーション機能の計算エンジンとして利用できます。Springheadの詳細については `Springheadホームページ <http://springhead.info/wiki/>`_ を、ビルド方法については、 :doc:`build-springheadPlugin` をご覧ください。
 
* **BUILD_PhysX_PLUGIN**

 PhysXプラグインをビルドします。このプラグインにより、動力学計算ライブラリ"PhysX"をコレオノイドのシミュレーション機能の計算エンジンとして利用できます。PhysXの詳細については `PhysXホームページ <http://www.nvidia.co.jp/object/physx_new_jp>`_ を、ビルド方法については、 :doc:`build-physxPlugin` をご覧ください。
 
* **ENABLE_CORBA**

 コレオノイドが提供するCORBA関連機能のベースとなるモジュールをビルドします。本モジュールの導入には、 `omniORB <http://omniorb.sourceforge.net/>`_ ライブラリのインストールが必要です。

* **BUILD_CORBA_PLUGIN**

 CORBA関連機能を提供するプラグインです。ENABLE_CORBAも有効になっている必要があります。

* **BUILD_OPENRTM_PLUGIN**

 OpenRTMプラグインをビルドします。このプラグインにより、RTミドルウェアのコンポーネントであるRTコンポーネント(RTC)をシミュレーションで用いることが可能となります。利用には `OpenRTM-aist <http://openrtm.org/>`_ 1.1 のインストールが必要です。また、上記のCORBAプラグインもビルドしておく必要があります。

* **BUILD_OPENRTM_SAMPLES**

 RTコンポーネントを利用したシミュレーションのサンプルをビルドします。

* **BUILD_OPENHRP_PLUGIN**

 OpenHRPプラグインをビルドします。OpenHRPプラグインにより、OpenHRP3用に開発された制御プログラムによるロボットシミュレーションや、オンラインビュアー機能を利用することが出来ます。CORBAプラグインもビルドしておく必要があります。

* **BUILD_OPENHRP_PLUGIN_FOR_3_0**

 OpenHRPプラグインとOpenHRPのサンプルコントローラ・シミュレーションは、デフォルトではOpenHRPバージョン3.1のAPI(IDL)に対応します。このオプションをONにすることで、OpenHRPバージョン3.0のIDLにも対応します。

* **BUILD_OPENHRP_SAMPLES**

 OpenHRP3のIDLに準拠したコントローラ・シミュレーションのサンプルをビルドします。本サンプルは SimpleController のサンプルをOpenHRP3のIDLでラップしたものとなっています。

* **BUILD_MEDIA_PLUGIN**

 メディアファイルの再生を行うMediaプラグインをビルドします。プラットフォームによってはメディアファイルを扱うための各種ライブラリが必要となります。

* **BUILD_GROBOT_PLUGIN**

 GRobotプラグインをビルドします。このプラグインにより、エイチ・ピー・アイ・ジャパンによる小型二足歩行ロボット "G-Robots GR001" の実機の動作を、コレオノイドで作成したモーションと同期させることが出来ます。

.. :doc:`../choreograph-tutorial/index` では、このGR001を対象に操作方法を説明していますので、GR001を持っていてこのチュートリアルを試す場合には、このプラグインをビルドしておいてください。
.. なお、現在のところ MacOS X ではこのプラグインは利用不可能となっています。

