
BulletPluginのビルド
==========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>


.. contents:: 目次
   :local:


Bullet Physics Library のインストール
--------------------------------------------------------

BulletPluginを利用するためには、Bullet Physics Library をインストールしておく必要があります。

まず、Bullet Physics Libraryのソースを取得します。

`Bullet Physics Library <http://bulletphysics.org>`_ のサイトを表示し、左上のDownloadの文字をクリックすると、Bulletのソースが保存されているサイトに移動します。
ここで、 **Source code (zip)** または **Source code (tar.gz)** をクリックすると、各圧縮形式のファイルがダウンロードできますので、適当な場所に展開します。

現在動作を確認をしているバージョンは、bullet 2.87 になります。

展開したディレクトリの中に、README.mdというファイルがあると思います。ここに、インストール方法が記述されていますので、基本的にはそれに従ってください。Bulletのバージョンによって、多少違いがありますが、最近のバージョンではcmakeが使えるようなので、choreonoidのビルドと同様の手順でインストールすることが出来ます。ここでは バージョン2.87 でのインストール方法を紹介します。

Ubuntuの場合
~~~~~~~~~~~~~~~~

cmakeを使います。choreonoidのインストール時に使用していますので既にインストールされていると思います。

展開したディレクトリに移動して、次のように入力して実行します。 ::

 ./build_cmake_pybullet_double.sh

これで、cmakeからビルドまで実行してくれます。

build_cmakeというディレクトリが作成されていますので、そこへ移動します。 ::

 make install
 
を実行するとインストールされます。インストール先を指定する場合は、build_cmakeで ::
 
 ccmake .

とすると、cmakeが起動するので、**CMAKE_INSTALL_PREFIX** の項目にインストール先を指定してから make install を行ってください。他のオプションもここで変更することが出来ます。 **USE_DOUBLE_PRECISION** は、必ずONにして下さい。

Windowsの場合
~~~~~~~~~~~~~~~~~~
READMEには build_visual_studio_vr_pybullet_double.bat を実行するように書かれていますので、これを使用してもいいですが、ここでは使い慣れているcmakeを使用したいと思います。

はじめに、bulletを展開したディレクトリに移動してbuild_cmakeというディレクトリを作成しておきます。

cmakeのGUIを起動し、“where is the source code”の欄にbulletを展開したディレクトリへのパスを、“where is build the binaries”の欄に作成したbuild_cmakeへのパスを入力します。

以下のオプションはONにします。

* **BUILD_EXTRAS**
* **INSTALL_EXTRA_LIBS**
* **INSTALL_LIBS**
* **USE_DOUBLE_PRECISION**
* **USE_MSVC_RUNTIME_LIBRARY_DLL**

また、以下のオプションはOFFにしておいた方が無難です。

* **BUILD_XXX_DEMOS** のすべて
* **BUILD_BULLET3**
* **BUILD_PYBULLET**
* **BUILD_PYBULLET＿XXX**
* **BUILD_UNIT_TESTS**

**CMAKE_BUILD_TYPE** はReleaseにし、インストール先は  **CMAKE_INSTALL_PREFIX** で設定します。

choreonoidの :ref:`build-windows-cmake` と同様に **“Configure”**, **”Generate”** ボタンを押して処理を進めて下さい。

.. note:: BulletのバージョンによってCMakeのオプションに差異があるようです。ここでの解説は、対象バージョンでの例とお考えください。

build_cmakeの中にVisual Studioのソルーションファイルが作成されているはずですので、これを開きます。

choreonoidの :ref:`build-windows-visualstudio` と同様に 画面の表示が **“Release”**、 **"x64"** となっていることを確認し、 **”ソリューションのビルド”** 、 **"INSTALL"** を実行してください。

プラグインのビルド
---------------------

choreonoidのビルドの際にCMakeの設定で、 **BUILD_BULLET_PLUGIN** という項目を "ON" にし、**BULLET_DIR** にBulletライブラリのインストール先を指定してください。

シミュレーションの実行
-------------------------

BulletPluginを用いたシミュレーションは :ref:`他の物理シミュレータ<simulation_creation_and_configuration_of_simulator_item>` を利用する方法と同様です。シミュレータアイテム「Bulletシミュレータ」を生成し、ワールドアイテムの子アイテムとして配置することで実行可能となります。
