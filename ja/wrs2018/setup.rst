競技環境のセットアップ
======================

.. contents::
   :local:


シミュレーション用PCの用意
~~~~~~~~~~~~~~~~~~~~~~~~~~

本競技はUbuntu Linuxを用いて行うことになっており、現在Ubuntu 16.04を対象として競技用タスクの開発を行っておりますので、まずUbuntu 16.04が使えるPC環境を用意して下さい。

PCのスペックについては、以下の条件を満たす環境であればシミュレーションをひととおり試すことができるかと思います。

* CPU: インテルCore iシリーズ、AMD Ryzenシリーズ等
* メモリ： 4GB以上
* GPU（グラフィックスボード）: NVIDIAのGeForceもしくはQuadroのGPUを推奨。インテルのCore i内蔵GPUも可。

AMD製のGPUはLinuxへの対応が十分でない部分があり、動作を保証できません。

実際の競技で使用するPCの仕様は以下を検討中です。

* CPU: Core i7 8700K（6コア、3.7GHz、ターボブースト時最大4.7GHz）
* メモリ: 32GB
* GPU: GeForce GTX 1080

全てのタスクを問題なく実行するためには、この程度のスペックのPCとAGX Dynamicsが必要となります。

なお、仮想マシン上での動作は非推奨です。ネイティブでUbuntu Linuxをインストールした環境でお使いください。

.. _wrs2018_install_choreonoid:

開発版Choreonoidのインストール
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

`Choreonoid最新版（開発版）マニュアル <../manuals/latest/index.html>`_ の `ソースコードからのビルドとインストール (Ubuntu Linux編) <../manuals/latest/install/build-ubuntu.html>`_　に従って、まずChoreonoidの最新の `開発版 <../manuals/latest/install/build-ubuntu.html#id4>`_ をUbuntu Linux 16.04上にインストールしてください。

インストールの詳細は上記ドキュメントを参照いただくとして、概要としては端末を起動して以下のコマンドを入力していけばOKです。 ::

 sudo apt -y install git
 git clone https://github.com/s-nakaoka/choreonoid.git
 cd choreonoid
 misc/script/install-requisites-ubuntu-16.04.sh
 cmake .
 make

なお、最後のmakeは ::

 make -j 8

などとするとコンパイルが並列化されて、処理が早く終わります。-j の後の数値が並列処理数で、CPUの論理コア数＋αの値を入力するとよいです。

.. note:: AGX Dynamicsのライセンスをお持ちの場合は、AGX Dynamicsをインストールし、Choreonoidの "AGX Dynamicsプラグイン" をビルドすることで、AGX Dynamicsを用いたシミュレーションが可能となります。AGX DynamicsプラグインはChoreonoidビルド時にCMakeの設定で **BUILD_AGX_DYNAMICS_PLUGIN** と **BUILD_AGX_BODYEXTENSION_PLUGIN** を ON にしておく必要があります。詳細は `AGX Dynamicsプラグイン <../manuals/latest/agxdynamics/index.html>`_ の `インストール <../manuals/latest/agxdynamics/install/install.html>`_ を参照ください。

一度インストールを行った後も、上記の作業を行ったソースディレクトリ上で以下を実行することで、常に最新版のChoreonoidを利用することができます。 ::

 git pull
 make

今後本競技の開催に向けてしばらくはChoreonoidの開発が続くことになりますので、随時最新版に更新しながらお試しいただけるとよいかと思います。
