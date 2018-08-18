シミュレーション環境の構築
==========================

.. contents::
   :local:

.. highlight:: sh

シミュレーション用PCの用意
--------------------------

まずはシミュレーション用のPCを用意して、Choreonoidをインストールします。

使用するPCのOSとスペックについては、 :ref:`wrs2018_overview_simulator` で提示した情報を参考にして用意してください。

OSはUbuntu 16.04 64bit版の使用を前提としています。Ubuntuの日本語版ISOイメージは `Ubuntu Japanese Team <https://www.ubuntulinux.jp/home>`_ のサイトからダウンロードできますが、現在の最新版は18.04となっており、16.04をダウンロードする場合は少し込み入ったリンクをたどる必要あがあります。 `日本国内のダウンロードサイト <https://www.ubuntulinux.jp/ubuntu/mirrors>`_ の一番下に「Japanese Teamのリリースイメージ」とありますので、そこに挙げられているサーバの中から適当に選んで、 "releases/16.04/ubuntu-ja-16.04-desktop-amd64.iso" をダウンロードしてください。

なお、今回のサンプルはUbuntu 18.04でも動作することを確認しています。ただしOpenRTMがまだ18.04に対応していませんので、OpenRTMを使用するサンプルは実行することができません。ROSを使用したサンプルについては、18.04でも動作します。

.. note:: Ubuntuはネイティブインストールされたものを使用してください。仮想マシンでも動かないことはありませんが、シミュレーションが遅くなったり、一部不具合が生じる可能性があります。どうしても仮想マシンで試したい場合は、 `VMWareを用いたUbuntu16.04仮想マシンの構築 <http://choreonoid.org/ja/workshop/vmware.html>`_ を参考にしてください。ただし、WRS2018のサンプルシミュレーションが正常に動作することは保証できません。

Gitのインストール
-----------------

以下の作業を進めるにあたって、バージョン管理システムのGitが必要となります。まだインストールしていない場合は、以下のコマンドでインストールしておきます。 ::

 sudo apt install git

.. _wrs2018_install_agx:

AGX Dynamicsのインストール
--------------------------

AGX Dynamicsのライセンスをお持ちの場合は、あらかじめ AGX Dynamics をインストールしておきます。販売元より提示されたAGX Dynamicsのダウンロードサイトから、対応するUbuntuバージョン（通常はx64、Ubuntu 16.04）用のパッケージをダウンロードします。また、USBドングルの提供を受けている場合は、それをPCに挿しておくようにしてください。

パッケージがダウンロードできたら、:doc:`../agxdynamics/install/install-agx-ubuntu` の説明に従ってインストールを行います。

AGX Dynamicsのラインセンスをお持ちでない場合、この作業はスキップしてください。


.. _wrs2018_install_openrtm:

OpenRTM-aistのインストール
--------------------------

:doc:`teleoperation-rtm` を実行する場合はOpenRTM-aistをインストールしておきます。

現在OpenRTM-aistの公式サイトや関連サーバが停止しておりますが、代わりに臨時の `OpenRTM-aist web on the github <http://openrtm.org/>`_ が設置されており、ここからOpenRTM-aistをダウンロードすることができます。現在ダウンロードできる最新版はバージョン 1.1.2 となっているので、このバージョンをダウンロードしてインストールしましょう。

Ubuntu16.04の場合は、上記ページの説明に従って、コマンドラインから以下のように入力することにより、C++版をインストールすることができます。 ::

 git clone https://github.com/n-ando/xenial_package.git
 cd xenial_package/xenial/main/binary-amd64/
 sudo dpkg -i openrtm-aist_1.1.2-0_amd64.deb
 sudo dpkg -i openrtm-aist-example_1.1.2-0_amd64.deb
 sudo dpkg -i openrtm-aist-dev_1.1.2-0_amd64.deb

OpenRTM-aist関連のパッケージとしては、他にPython版やRTSystemEditor/RTCBuilderといったツールもあります。それらは本サンプルの実行では必要ありませんが、ご自分のシステムの構築や実行にが必要な場合は、上記ページの説明に従ってインストールしておいてください。

.. _wrs2018_install_choreonoid:

Choreonoidのインストール
------------------------

`Choreonoid最新版（開発版）マニュアル <../manuals/latest/index.html>`_ の `ソースコードからのビルドとインストール (Ubuntu Linux編) <../manuals/latest/install/build-ubuntu.html>`_　に従って、Choreonoidの最新の `開発版 <../manuals/latest/install/build-ubuntu.html#id4>`_ をインストールします。

インストールの詳細は上記ドキュメントを参照いただくとして、Ubuntu 16.04においては、以下のコマンドを実行していきます。

まずGitリポジトリからChoreonoidのソースコードを取得します。 ::

 git clone https://github.com/s-nakaoka/choreonoid.git

取得したソースコードのディレクトリに移動します。 ::

 cd choreonoid

依存パッケージのインストールを行います。 ::

 misc/script/install-requisites-ubuntu-16.04.sh

(Ubuntu 18.04 の場合は、install-requisites-ubuntu-18.04.sh を実行します。）

CMakeによるビルドの設定を行います。Choreonoidのデフォルトの機能だけ利用するのであれば、 ::

 cmake .

を実行します。

ただしWRS2018のサンプルを実行するためには、以下のオプションも有効（ON）にする必要があります。

* AGX Dynamics を利用する場合

 * BUILD_AGX_DYNAMICS_PLUGIN
 * BUILD_AGX_BODYEXTENSION_PLUGIN

* 煙や炎を再現する場合

 * BUILD_SCENE_EFFECTS_PLUGIN

* マルチコプタを使用する場合

 * BUILD_MULTICOPTER_PLUGIN
 * BUILD_MULTICOPTER_SAMPLES

* OpenRTMを利用する場合

 * ENABLE_CORBA
 * BUILD_CORBA_PLUGIN
 * BUILD_OPENRTM_PLUGIN
 * BUILD_OPENRTM_SAMPLES

これらのオプションの設定はccmakeコマンドを使ってインタラクティブに行うこともできますが、cmakeコマンドに-Dオプションを与えることも可能です。例えば、BUILD_SCENE_EFFECTS_PLUGINをONにするには、以下のように入力します。 ::

 cmake -DBUILD_SCENE_EFFECTS_PLUGIN=ON

このオプションは複数つけることができます。上記のオプション全てを有効にする場合は、以下のように入力してください。 ::

 cmake -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON -DBUILD_OPENRTM_SAMPLES=ON 

AGX DynamicsやOpenRTMをインストールしていない場合は、それぞれに対応するオプションを上記のコマンドライン引数から除去して実行してください。

次に、makeコマンドでビルドを行います。 ::

 make

なお、マルチコアCPUをお使いの場合は、makeコマンドに -j オプションをつけてビルドを並列化するとよいです。例えば次のようにします。 ::

 make -j 8

この場合、最大で8つのプロセスを同時に実行してビルドを行います。4コア8スレッドのCPUの場合はこのように入力するとよいでしょう。通常、CPUの論理コア数を指定します。

一度インストールを行った後も、上記の作業を行ったソースディレクトリ上で以下のように実行することで、常に最新版のChoreonoidを利用することができます。 ::

 git pull
 make -j 8

今回は、競技会の開催が近づくまで当面Choreonoidの開発が続くことを予めご了承ください。これを踏まえて、随時最新版に更新しながら準備を進めていただければと思います。何か不具合が生じましたら、 :doc:`support` までご相談ください。


描画に関わる設定
----------------

Choreonoidのインストールにおいては、 :ref:`build_ubuntu_gpu_driver` が可能であれば、必ず導入しておくようにしてください。また、 :doc:`../install/setup-renderer` についても、可能であればGLSL描画エンジンに切り替えるようにしてください。WRS2018のシミュレーションでは高度な描画能力が要求されるため、完全なシミュレーションを行うためにはこれらの設定が欠かせません。

また、 :ref:`build_ubuntu_qt_style` についても適用しておくとよいかと思います。


ゲームパッドの準備
------------------

今回のサンプルでは、ゲームパッドでロボットを操作することができます。これを行うために、ゲームパッドを用意して、PCに接続しておいてください。

使用可能なゲームパッドについては、:doc:`../simulation/tank-tutorial/index` の :ref:`simulation-tank-tutorial-gamepad` を参照してください。おすすめはプレイステーション4用の `DUALSHOCK4 <http://www.jp.playstation.com/ps4/peripheral/cuhzct1j.html>`_ コントローラです。DUALSHOCK4は `USBワイヤレスアダプター <http://www.jp.playstation.com/ps4/peripheral/cuhzwa1j.html>`_ によるワイヤレス接続も可能です。
