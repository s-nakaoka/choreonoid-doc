
コントローラのビルド
====================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

.. contents:: 目次
   :local:

コントローラのビルド方法
------------------------

:doc:`howto-implement-controller` では、シンプルコントローラの場合を対象として、コントローラ実装の概要についてソースコードの記述を中心に解説しました。そこで用いたコントローラはChoreonoidのサンプルのひとつであったため、Choreonoid本体をビルドもしくはインストールすれば使えるようになるものでした。しかし、ユーザが新たにコントローラを開発する場合は、コントローラのソースコードを記述しただけでは使えるようにはなりません。シンプルコントローラはC++を用いて記述するようになっているので、そのソースコードから実際に使えるバイナリファイルを生成するためのビルド（コンパイル、リンク等の作業）が必要となるからです。ここではシンプルコントローラを対象としたビルドの方法について概要を解説します。

コントローラを自前で新たに実装してビルドする方法としては、大きく分けて以下の２つがあります。

1. Choreonoid本体と一緒にビルドを行う
2. Choreonoid本体とは別にビルドを行う

以下ではそれぞれの方法ごとに分けて解説します。

.. _simulation-build-controller-method1:

Choreonoid本体と一緒にビルドを行う方法
--------------------------------------

.. highlight:: cmake

この方法では、サンプルのコントローラと同様に、Choreonoid本体のビルド時に自前のコントローラも一緒にビルドすることになります。逆に言えば、自前のコントローラをビルドする際にも、Choreonoid本体をビルドするコマンドを使用して、Choreonoidの一部としてコントローラをビルドするということになります。Choreonoidをソースコードからビルドしてお使いの場合は、この方法が一番手軽かと思われます。

Choreonoid本体はCMakeを用いてビルドの記述が行われており、本手法はその中に自前のコントローラのビルド記述も含めてしまうというものです。CMakeにおいてビルドの記述には通常 "CMakeLists.txt" という名前のファイルを用いることになっているため、本手法ではコントローラについてもこのファイルにてビルドの記述を行います。この際、Choreonoid本体で定義されている "add_cnoid_simple_controller" というCMakeの関数を用いることで、シンプルコントローラのビルド記述を簡潔に行うことができます。この関数に与える引数は以下のようになっています。 ::

 add_cnoid_simple_controller(コントローラ名 ソースファイル ...)

ソースファイルは１つでも結構ですし、複数記述することも可能です。

あとはこのCMakeLists.txtをChoreonoidのビルドにおいて認識されるようにする必要があります。これを行うためには、Choreonoidのソースディレクトリに含まれる "ext" という名前のディレクトリを用います。このディレクトリ以下に作成したサブディレクトリ内にCMakeLists.txtがあると、それがChoreonoid本体のビルドにおいて認識され、取り込まれるようになります。コントローラについても、ext以下に適当な名前でディレクトリを作成し、そこに対応するソースファイルとCMakeLists.txtを格納してください。

具体的な例として、自前の "MyController" というコントローラのC++ソースコードを、 "MyController.cpp" というファイルに記述するとしましょう。まず、Choreonoidのextディレクトリ内にこのファイルを格納するサブディレクトリを作成して下さい。名前は何でも結構ですが、ここではコントローラ名と対応付けて "MyController" というディレクトリを作成することにします。

そして、以下の内容を記述したCMakeLists.txtを作成し、同じディレクトリに保存してください。 ::

 add_cnoid_simple_controller(MyController MyController.cpp)

この結果、ディレクトリ／ファイル構成は以下のようになります。 ::

 Choreonoidソースディレクトリ
  + ext
    + MyController
      - CMakeLists.txt
      - MyController.cpp

あとはChoreonoid本体をビルドすればMyControllerについてもビルドされます。つまり、Choreonoid本体のビルドディレクトリにてcmakeとmakeを実行します。ビルドに成功すれば、 :ref:`simulation-set-controller-to-controller-item` にて述べたシンプルコントローラの標準ディレクトリ内に、MyController.so (Linuxの場合。Windowsの場合はMyController.dll) というファイルが生成されているはずです。

.. note:: ext以下に作成したコントローラのCMakeLists.txtに対して直接cmakeを実行することは避けてください。本手法ではコントローラのCMakeLists.txtはあくまでChoreonoid本体のビルド記述の一部として取り込まれるものなので、それとは独立してcmakeを適用できるものではありません。

.. note:: ext以外のディレクトリに配置したソースディレクトリを取り込むこともできます。その場合は、Choreonoid本体のCMakeにおいて **ADDITIONAL_EXT_DIRECTORIES** に取り込みたいソースディレクトリへのパスを設定してください。セミコロンで区切ることで、複数設定することも可能です。

.. note:: コントローラが外部のライブラリをリンクして使う場合など、コントローラの構成が複雑になってくると、add_cnoid_simple_controller以外にもCMakeLists.txtの記述が必要になる場合があります。その場合は、CMakeのマニュアルやadd_cnoid_simple_controller関数の定義などを参照して、適切な記述を行うようにしてください。（add_cnoid_simple_controller関数はChoreonoidソースのsrc/Body/CMakeListst.txt内にて定義されています。）

Choreonoid本体とは別にビルドを行う方法
--------------------------------------

この方法では、Choreonoid本体がシステムにインストールされていることを前提とし、それで使うためのコントローラを別途単体でビルドします。

ここで言う「インストール」は、Choreonoidをソースファイルからビルドした後、その実行に必要なファイルをシステムの所定のディレクトリにコピーし、実行ファイルやライブラリファイルへのパスを通すことを意味します。インストールの方法については以下を参照してください。

* :doc:`../install/build-ubuntu` の :ref:`build-ubuntu_install`
* :doc:`../install/build-windows` の :ref:`build-windows-install`

コンパイルオプションの設定
~~~~~~~~~~~~~~~~~~~~~~~~~~

Choreonoidがインストールがされていれば、それに対応するインクルードパスやライブラリパスを設定してビルドを行います。また、それ以外のコンパイルオプションもいくつか指定する必要があります。

例として、Choreonoidを/usr/local以下にインストールしていて、gcc(g++)を用いてコンパイルを行う場合、gccのコンパイルオプションとしては以下のようなものを指定することになります。(/usr/localは実際にインストールしたディレクトリに置き換えて下さい。）

* **-std=c++11** (C++11を有効化）
* **-fPIC** (共有ライブラリ用にコンパイル）
* **-I/usr/local/include** (インクルードパスの追加）

同様に、リンクオプションは以下のようになります。

* **--shared** (共有ライブラリとしてリンク）
* **-L/usr/local/lib** (リンクパスの追加）
* **-lCnoidUtil -lCnoidBody** (ChoreonoidのUtilライブラリとBodyライブラリをリンク）

Util、Bodyのライブラリは、Choreonoid本体を構成するライブラリの一部です。Utilライブラリは様々な機能をまとめたユーティリティライブラリで、Bodyライブラリは :doc:`../handling-models/bodymodel` 関連の機能をまとめたライブラリとなっています。シンプルコントローラはこれらのライブラリの機能を利用していますので、最低限これらのライブラリへのリンクが必要となります。

.. note:: "/usr/local/include" や "/usr/local/lib" は標準でコンパイラのインクルードパスやライブラリパスに含まれている場合があります。その場合、上記の "-I/usr/local/include" や "-L/usr/local/lib" は必要ありません。ただし、Choreonoidを "/usr/local" 以外のディレクトリにインストールしている場合は、対応するパスの追加が通常必要になります。

コントローラのインストール
~~~~~~~~~~~~~~~~~~~~~~~~~~

生成したコントローラ本体のバイナリファイルは、通常コントローラの標準ディレクトリにコピー（インストール）しておきます。標準ディレクトリは、

* /usr/local/lib/choreonoid-x.x/simplecontroller (x.xはバージョン番号に対応）

となります。

.. note:: コントローラの標準ディレクトリについては、ここにファイルをまとめておけば分かりやすい、あるいはシンプルコントローラアイテムからアクセスしやすい、といった理由で用意しています。コントローラを他のディレクトリに格納しておきたい理由があれば、そのようにしても特に問題はありません。

pkg-configの利用
~~~~~~~~~~~~~~~~

.. highlight:: sh

Choreonoidをインストールすると、 `pkg-config <https://www.freedesktop.org/wiki/Software/pkg-config/>`_ を利用したコンパイルオプションの設定ができるようになります。

具体的には、 ::

 pkg-config --cflags choreonoid-body

を実行すると、Bodyライブラリを利用したプログラムのコンパイルに必要なオプションが出力されますし、 ::

 pkg-config --libs choreonoid-body

を実行すると、Bodyライブラリを利用したプログラムのリンクに必要なオプションが出力されます。

このコマンドを用いることにより、Choreonoidがどこにインストールされているか、どのライブラリとリンクする必要があるか、といったことをあまり気にせずに、Choreonoidを利用したプログラムをビルドすることができます。

コマンドで "choreonoid-body" と指定している部分は、pkg-configにおいてChoreonoidのBodyライブラリに対応する識別子です。Choreonoidをインストールすると、以下の識別子でChoreonoidの各ライブラリに関する情報を取得できるようになります。

* **choreonoid-util** : Utilライブラリ
* **choreonoid-body** : Bodyライブラリ
* **choreonoid-base** : Baseライブラリ
* **choreonoid-body-plugin** : Bodyプラグインライブラリ

シンプルコントローラをビルドする際には通常choreonoid-bodyを用いればOKです。

.. note:: Baseライブラリは、Choreonoidのプラグインを開発する際に使用する基盤ライブラリです。また、Bodyプラグインライブラリは、Bodyプラグインの機能をライブラリとして外部から利用できるようにしたもので、Bodyプラグインに依存する他のプラグインを開発する際に使用します。

なお、pkg-configで上記の識別子を利用するためには、Choreonoidのインストール先がpkg-configのシステムから認識されている必要があります。デフォルトのインストール先である "/usr/local" にインストールする場合はそのままでpkg-configから認識されるようになっていますが、それ以外のディレクトリにChoreonoidをインストールする場合は、環境変数 "PKG_CONFIG_PATH" 等の設定が必要になることがあります。

例えば、Choreonoidをホームディレクトリの usr 以下にインストールした場合は、 ::

 export PKG_CONFIG_PATH=$HOME/usr/lib/pkgconfig

を実行しておきます。

.. _simulation-build-controller-commands:

ビルドコマンドの実行例
~~~~~~~~~~~~~~~~~~~~~~

実際にビルドを行うコマンドの例について、Ubuntu Linuxを対象に紹介します。

コントローラのソースファイルは "MyController.cpp" であるとします。これをどこか適当なディレクトリに格納して、コマンドライン上からそのディレクトリに移動してください。

以下のコマンドでコンパイルを行うことができます。 ::

 g++ -std=c++11 -fPIC `pkg-config --cflags choreonoid-body` -c MyController.cpp

これを実行すると MyController.cpp をコンパイルした MyController.o というオブジェクトが生成されます。

次に、以下のコマンドでリンクを行います。 ::

 g++ --shared -std=c++11 -o MyController.so MyController.o `pkg-config --libs choreonoid-body`

これにより、MyController.so というファイルが生成されます。これがコントローラのバイナリファイルで、シンプルコントローラアイテムの「コントローラモジュール」に指定して使うことが可能です。

必要であれば標準ディレクトリへのインストールもしておきます。 ::

 cp MyController.so `pkg-config --variable=simplecontrollerdir choreonoid-body`

このpkg-configの使い方で、シンプルコントローラ用標準ディレクトリのパスを取得することができます。/usr/local 以下にインストールされている場合は、上記コマンドにsudoをつけて ::

 sudo cp MyController.so `pkg-config --variable=simplecontrollerdir choreonoid-body`

として実行してください。

.. note:: :ref:`simulation-build-controller-method1` の場合と同様に、コントローラが複数のソースファイルから構成されたり、CnoidBody以外のライブラリをリンクして使う場合など、コントローラの構成が複雑になってくると、上記のコマンドだけではビルドできなくなるかと思います。その場合の対応は本解説の範囲を超えて一般的なプログラム開発方法の話題となってきますので、ここでは割愛します。

Makefileの例
~~~~~~~~~~~~

.. highlight:: makefile
   :linenothreshold: 5

上で述べたコマンドを毎回実行するのは大変です。これを避けてビルドの操作を簡略化するために、Makeコマンドを使うことができます。Makeコマンドでは、ビルド方法をMakefileという名前のファイルに記述します。以下にMyControllerをビルドするためのMakefileの例を示します。 ::

 CONTROLLER=MyController.so
 SRC=MyController.cpp
 OBJ=$(SRC:%.cpp=%.o)
 
 $(CONTROLLER): $(OBJ)
	g++ --shared -std=c++11 -o $(CONTROLLER) $(OBJ) `pkg-config --libs choreonoid-body`
 
 %.o: %.cpp
	g++ -std=c++11 -fPIC `pkg-config --cflags choreonoid-body` -c $<
 
 install: $(CONTROLLER)
 	install -s $(CONTROLLER) `pkg-config --variable=simplecontrollerdir choreonoid-body`
 clean:
	rm -f *.o *.so

Makefileの仕様上、6、9、12、14行目は行頭からタブを用いてインデントをつける必要がありますので、ご注意下さい。（スペースの場合エラーになります。）

.. highlight:: sh

MyController.cpp を格納しているディレクトリに、上記の内容を記述したファイルを "Makefile" という名前で作成します。コマンドラインからそのディレクトリに移動し、 ::

 make

と入力することでコントローラのビルドを行います。その後 ::

 make install

を実行することでコントローラの標準ディレクトリへのインストールを行います。（make install に関しては、必要に応じてsudoもつけてください。）

これで、 :ref:`simulation-build-controller-commands` で紹介したコマンドを実行したのと同じ結果になります。

Makefileの書き方については、 `Makeのマニュアル <https://www.gnu.org/software/make/manual/>`_ などを参照してください。

.. note:: ここでは省略しましたが、コンパイル・リンクにおいては通常-O2または-O3といったオプションも付与します。これらは最適化を有効にするオプションで、これにより生成されるプログラムの実行速度が速くなります。あるいは、デバッグを行う際には、-g といったデバッグ用のオプションをつけて、デバッグ用の情報も生成されるようにします。これらの詳細については、コンパイラのマニュアルや、C/C++プログラム開発に関する各種情報を参照するようにしてください。

実際にはMakefileを直接書くことはあまりありません。CMake等、より高水準の記述が可能なビルドツールの使用が一般的になっているからです。CMakeはChoreonoid本体のビルドでも使っているため、コントローラを :ref:`simulation-build-controller-method1` でも用いていますが、CMakeはコントローラをChoreonoid本体とは別にビルドする場合でも用いることができます。ただしその場合のCMakeの実行方法やCMakeLists.txtの記述は :ref:`simulation-build-controller-method1` とは少し異なってきますので、ご注意下さい。CMakeについては別途 `CMakeのマニュアル <https://cmake.org/documentation/>`_ を参照してください。
