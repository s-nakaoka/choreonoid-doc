Choreonoid関連パッケージのビルド
================================

ここではROS環境におけるパッケージとしてChoreonoidをインストール・ビルドします。あわせていくつかのChoreonoid関連パッケージもインストール・ビルドします。

本ドキュメントでは :doc:`../install/build-ubuntu` とは異なる手順でChoreonoidをインストールします。既にそちらの手順でインストール済みのChoreonoidがあったとしても、それとは独立してROS用のChoreonoidを別途インストールすることになりますので、ご注意ください。通常の手順でインストールされたChoreonoidをROS環境で使用することも可能なのですが、そちらについては必要なROSパッケージやドキュメントを現在整備中ですので、当面は本ドキュメントの解説に従って、ROS用のChoreonoidをインストールするようにしてください。ここでインストールするChoreonoidは、通常の手順でインストールされたものとは区別し、それぞれ独立に管理するようにしてください。

.. contents::
   :local:

Catkinワークスペースの作成
--------------------------

Choreonoid用のCatkinワークスペースを作成します。

ワークスペースは通常これはホームディレクトリ上に作成します。ワークスペースの名前は通常 "catkin_ws" とします。この名前は変更しても結構ですが、その場合は以下の説明の "catkin_ws" をその名前に置き換えるようにしてください。

まず空のワークスペースを作成します。 ::

 mkdir catkin_ws
 cd catkin_ws
 mkdir src
 catkin init

.. note:: ここではワークスペースの初期化に "catkin init" というコマンドを使っていますが、同様の操作を行うコマンドとして "catkin_init_workspace" というコマンドもあります。前者はCatkinの新しいコマンド体系である `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ に含まれるコマンドで、後者はCatkinの古い形式のコマンドです。両者はパッケージのビルドに使用するコマンドも異なっており、それぞれ "catkin build" と "catkin_make" になります。ChoreonoidのROS連携は Catkin Command Line Tools の使用を前提としていますので、古い形式のCatkinコマンド（catkin_make等）は使用しないでください。

パッケージソースの追加
----------------------

作成したワークスペースの "src" ディレクトリ内に、Choreonoid本体とROSプラグインのソースコードリポジトリをクローンします。 ::

 cd src
 git clone https://github.com/s-nakaoka/choreonoid.git
 git clone https://github.com/s-nakaoka/choreonoid_ros.git

それぞれ以下のGithubリポジトリに対応しています。

* `choreonoid <https://github.com/s-nakaoka/choreonoid>`_ : Choreonoid本体
* `choreonoid_ros <https://github.com/s-nakaoka/choreonoid_ros>`_ : ChoreonoidでROSの機能を使用するためのROSパッケージ

また、次節以降の解説を参照する場合は、そこで使用するサンプルもクローンしておきましょう。 ::

 git clone https://github.com/s-nakaoka/choreonoid_ros_samples.git
 git clone https://github.com/s-nakaoka/choreonoid_joy.git

それぞれ以下のGithubリポジトリに対応しています。

* `choreonoid_ros_samples <https://github.com/s-nakaoka/choreonoid_ros_samples>`_ : ChoreonoidでROSを使用するサンプル
* `choreonoid_joy <https://github.com/s-nakaoka/choreonoid_joy>`_ : ジョイスティック（ゲームパッド）をChoreonoidのマッピングで使うためのROSノード

各リポジトリの内容はなるべく最新に保つようにしてください。

.. note:: ChoreonoidのROS関連パッケージとしては、上に挙げたもの以外にも、 `choreonoid_ros_pkg <https://github.com/fkanehiro/choreonoid_ros_pkg>`_ や `choreonoid_rosplugin <https://github.com/s-nakaoka/choreonoid_rosplugin>`_ があります。これらは現在公式にサポートされているものではないので、本マニュアルの方法でChoreonoidを使用する際には導入しないようにしてください。これらがバイナリパッケージとしてインストールされていたり、catkinのワークスペースに含まれていたりすると、本ドキュメントで解説する機能がうまく動作しない可能性があります。本マニュアルの記述に従う場合は、ワークスペースを新規に作成して、まずは指定されたパッケージのみを導入し、動作を確認するようにしてください。


リポジトリ管理ツールの使用
--------------------------

複数のリポジトリをまとめて管理するためののツールとして、 `wstool <http://wiki.ros.org/wstool>`_ や `vcstool <https://github.com/dirk-thomas/vcstool>`_  があります。これらを使用することでの複数リポジトリの更新なども一括して行うことができるので、活用されるとよいかと思います。

ここではvcstoolについて簡単に紹介します。vcstoolを使う場合は、 ::

 sudo apt install python3-vcstool

でインストールできます。

使い方は ::

 vcs help

で確認してください。

各リポジトリよりも上位にあるディレクトリで ::

 vcs pull

を実行すると、全てのリポジトリに対して git pull が実行され、全てのリポジトリを最新のものに更新することができます。

.. _teleoperation_ros_build_packages:

依存パッケージのインストール
----------------------------

Choreonoidのビルド・実行に必要となる依存パッケージをインストールしておきます。

Choreonoidのソースディレクトリに移動し、 ::

 misc/script/install-requisites-ubuntu-16.04.sh

もしくは ::

 misc/script/install-requisites-ubuntu-18.04.sh

を実行します。(使用しているUbuntuのバージョンに合うものを実行してください。）

この処理は本来Catkin用の依存パッケージ情報で解決すべきなのですが、Choreonoidについてはそこがまだ完全でない部分があり、インストールを確実にするため、この作業を行っておく必要があります。

なお、OS上でROSとは独立して既に最新のChoreonoidをインストールしている場合この作業は適用済みのはずですので、あらためて実行する必要はありません。

CMakeオプションの設定
---------------------

ChoreonoidのビルドにおいてCMakeのオプションを設定したい場合は、catkin の config コマンドを使用します。

例えばメディアプラグインをビルドしたい場合は、以下のようにします。 ::

 catkin config --cmake-args -DBUILD_MEDIA_PLUGIN=ON

複数のオプションを設定したい場合、オプションを列挙すればOKです。例えば以下のコマンドでメディアプラグインとシーンエフェクトプラグインの両方をONにできます。 ::

 catkin config --cmake-args -DBUILD_MEDIA_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON

設定後 ::

 catkin config

を実行すると、ワークスペースの設定が表示されます。そこに ::

 Additional CMake Args:  -DBUILD_MEDIA_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON

といった表示があればOKです。

.. note:: このように設定すると、ワークスペースの全てのパッケージに対してこれらのオプションが有効になってしまい、他のパッケージで意図しないオプションが有効になってしまうこともあり得ます。しかしCatkinではパッケージごとに個別にCMakeのオプションを設定する機能が無い（ `要望はあるものの見送られている <https://github.com/catkin/catkin_tools/issues/205>`_ ）ようですので、やむを得ずこのようにしています。

設定したオプションを解除したい場合は ::

 catking config --no-cmake-args

を実行します。

以上の方法でCMakeのオプションを設定できますので、ROS環境で使いたいオプションがあればそちらを有効にするようにしてください。

.. _note_on_ros_python_version:

Pythonバージョンの設定
^^^^^^^^^^^^^^^^^^^^^^

ChoreonoidではデフォルトでPythonプラグインとPython用ラッパライブラリがビルドされますが、そこで使用するPythonのバージョンには注意が必要です。本解説が対象としているROSのKineticやMelodicを含めて、ROS1で使用するPythonのバージョンは基本的に2.7となるようです。一方でChoreonoidではデフォルトでPython3を使用するようになっており、そのままではPythonのバージョン2と3が競合してしまい、いろいろと不具合が出る可能性が高いです。

そこで、ChoreonoidのPython機能を使用する場合は、CMake の USE_PYTHON3 というオプションを OFF に設定します。そのようにするとChoreonoidでもPythonバージョン2が使用されるようになります。

catkin においては ::

 catkin config --cmake-args -DUSE_PYTHON3=OFF

とすることでこれを実現できます。

あるいは、ChoreonoidのPython機能が必要ない場合は、以下のようにしてPython機能自体をオフにしてしまってもよいかと思います。 ::

 catkin config --cmake-args -DENABLE_PYTHON=OFF -DBUILD_PYTHON_PLUGIN=OFF -DBUILD_PYTHON_SIM_SCRIPT_PLUGIN=OFF


ビルド
------

設定が完了したら、ビルドを行いましょう。ワークスペース内のディレクトリであれば、以下のコマンドでビルドできます。 ::

 catkin build

ビルド方法の詳細については `Catkin Command Line Tools のマニュアル <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ を参照してください。

ビルドに成功すると、 ::

 [build] Summary: All 4 packages succeeded!

といった表示がされます。

.. note:: Emacsでは "M-x compile" コマンドでビルドを行うことが可能ですが、Catkin環境でもこの機能を利用することができます。ただしCatkinの出力は通常色付けされるのですが、Emacs上ではその制御コードが表示されてしまい、そのままでは表示が見にくくなってしまいます。これを回避するため、 "M-x compile" 実行時にビルド用のコマンドとして "catkin build --no-color" を入力するとよいです。"--no-color" を入れることで、Cakin出力の色付け用の制御コードが無効化され、表示の乱れがなくなります。また、"-v" オプションを追加して "catkin build -v --no-color" とすることで、ビルド時に実際のコマンド（コンパイルオプションなど）を確認することもできます。


ビルドタイプの設定
------------------

一般的に、C/C++のプログラムをビルドする際には、"Release" や "Debug" といったビルドのタイプを指定することができます。Release（リリースモード）の場合は最適化が適用されて実行速度が速くなりますし、Debug（デバッグモード）の場合はデバッグ情報が付与されてデバッガによるデバッグがしやすくなります。

Catkin上でビルドする際にこれらのビルドタイプを指定したい場合は、やはり --cmake-args オプションを使用します。

例えば ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

とすればリリースモードでビルドすることができますし、 ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug

とすればデバッグモードになります。

--cmake-argsオプションは catkin build にも付与できますので、 ::

 catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

などとすることで、ビルドごとにビルドタイプを指定することも可能です。

Choreonoid関連のROSパッケージはデフォルトでReleaseが設定されるようにしてありますが、パッケージによってはデフォルトでビルドタイプをReleaseに設定しないものもありますし、自前のパッケージでそこまで設定していないこともあるかもしれません。その場合最適化が適用されず、ビルドされたプログラムの実行速度が大幅に落ちることになってしまいますので、そのようなパッケージをビルドする可能性がある場合は、上記の方法でReleaseビルドを指定しておくとよいです。

なお、Catkin Command Line Tools の Profile機能を使えば、設定ごとに予めプロファイルとして登録しておき、ビルドの際にプロファイルを指定することで切り替えることもできます。この使い方については、 `Catkin Command Line Tools のマニュアル <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ の `Profile Cookbook <https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html#profile-cookbook>`_ を参考にしてください。


.. _loading_catkin_workspace_setup_script:

ワークスペースセットアップスクリプトの取り込み
----------------------------------------------

ビルドをすると、 ワークスペースのdevelディレクトリに "setup.bash" というファイルが生成されます。このスクリプトに記述されている設定は、ワークスペース内のパッケージを実行したりする際に必要となりますので、デフォルトで実行されるようにしておきます。通常はホームディレクトリの .bashrc ファイルに ::

 source $HOME/catkin_ws/devel/setup.bash

という記述を追加しておきます。

すると端末起動時に自動でこのファイルが実行され、設定が読み込まれるようになります。

初回ビルド時はまだこの設定が取り込まれていませんので、端末を起動し直すか、上記のコマンドをコマンドラインから直接入力して、設定を反映させるようにしてください。

.. note:: このスクリプトは :doc:`install-ros` で導入したROS本体のsetup.bashとは **異なります** ので注意するようにしてください。ワークスペース上のパッケージを正常に動作させるためには、どちらのスクリプトも読み込んでおく必要があります。

.. note:: Catkinの設定スクリプトを実行すると、Catkin環境外で別途インストールしているChoreonoidの実行に影響することがあるので注意が必要です。これはCatkinの設定スクリプトにより、共有ライブラリのパスにCatkinワークスペースのdevel/libディレクトリが加わる(環境変数 LD_LIBRARY_PATH にこのパスが追加される）のが原因です。 この設定により、Catkin環境外のChoreonoidを実行する際に、Catkin内で生成されているChoreonoidの共有ライブラリを読み込んでしまうことがあります。その場合、ソースコードのバージョンやビルド設定などに違いがあると、Choreonoidがうまく動かなかったり、落ちてしまったりします。つまり、異なる環境でビルドしたものを混ぜてはいけないということになります。この問題を避けるためには、Catkin外のChoreonoidを実行する際にはCatkinの設定スクリプトは無効化しておきます。（ChoreonoidではRPATHという仕組みがデフォルトで使用されており、これによってこのような問題も避けられるはずなのですが、環境によってはうまく機能しないことがあるようです。）
