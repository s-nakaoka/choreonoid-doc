
HelloWorld サンプルの解説
=========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

このドキュメントでは、サンプルプラグインのひとつである"HelloWorldPlugin"の実装について解説します。

.. contents:: 目次
   :local:

ソースコード
------------

.. highlight:: cpp

本サンプルのソースコードは以下のとおりです。 ::

 #include <cnoid/Plugin>
 #include <cnoid/MenuManager>
 #include <cnoid/MessageView>
 
 using namespace cnoid;
 
 class HelloWorldPlugin : public Plugin
 {
 public:
 
     HelloWorldPlugin() : Plugin("HelloWorld")
     {
 
     }
 
     virtual bool initialize() override
     {
         Action* menuItem = menuManager().setPath("/View").addItem("Hello World");
         menuItem->sigTriggered().connect([&](){ onHelloWorldTriggered(); });
         return true;
     }
 
 private:

     void onHelloWorldTriggered()
     {
         MessageView::instance()->putln("Hello World !");
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
 
このソースコードを含む本サンプルのファイル一式がChoreonoidソースアーカイブの "sample/tutorial/HelloWorldPlugin" 以下に格納されています。ソースコードのファイル名は "HelloWorldPlugin.cpp" となります。

本サンプルをビルドすると "HellowWorldプラグイン" が生成されます。これがChoreonoid起動時に読み込まれると、メインメニューの「表示」に「Hello World」という項目が追加されます。そしてこのメニューを選択すると、メッセージビューに "Hello World !" と出力されます。ただそれだけのプラグインですが、以下ではこのサンプルの解説を通して、Choreonoidのプラグイン開発の基本を学んでいくことにします。

ヘッダのインクルード
--------------------

まず、本サンプルがインクルードしているヘッダの概要を説明します。 ::

 #include <cnoid/Plugin>
 #include <cnoid/MenuManager>
 #include <cnoid/MessageView>

これら3つのヘッダは、Choreonoidのフレームワークが提供するヘッダです。それらは基本的にChoreonoidインストール先の "include/cnoid" ディレクトリ内に格納されています。通常はパスの "include" までをインクルードパスに設定しておき、上記のように "cnoid" サブディレクトリをプレフィックスとして付与してインクルードを行います。また、インクルード用のヘッダファイルには拡張子をつけていませんので、上記のように拡張子のない形式で対象ヘッダの名前を書きます。

.. note:: 拡張子の無い形式は、標準C++ライブラリのヘッダファイルでも採用されており、Choreonoidが基盤として利用しているEigen, Qt, OpenSceneGraph といったC++ライブラリでも採用されていて、C++において標準的な形式のひとつです。それらのライブラリとの間で記述の統一性を高めるというねらいもあり、Choreonoidでもこの形式を採用しています。(ちなみに、この形式は「ヘッダファイル」ではなく「ヘッダ」と呼ぶことになっているようです。）

ここでインクルードしている３つのヘッダはそれぞれ以下のような機能を持っています。

* cnoid/Plugin

 Pluginクラスが定義されたヘッダです。プラグインを作る際にはこのヘッダをインクルードし、Pluginクラスを継承したクラスを作ることでひとつのプラグインを定義します。

* cnoid/MenuManager

 メニューの管理を行うMenuManagerクラスが定義されたヘッダです。メニューに項目を追加する際には、このヘッダをインクルードして、MenuManagerを使えるようにします。

* cnoid/MessageView

 テキストメッセージを出力するビューである「メッセージビュー(MessageView)」が定義されたヘッダです。メッセージビューにテキストを出力したい場合は、このヘッダをインクルードします。

これらのヘッダの実態はソースツリーの src/Base 以下にあります。クラス定義の詳細を知りたい場合は、それらのヘッダファイルを直接参照してみてください。（ヘッダファイルの実態については拡張子 .h がついていますので、ご注意ください。）
なお、Doxygenというツールを用いることで、クラス定義の詳細を一覧できるリファレンスマニュアルを生成することも可能なのですが、今のところ解説文生成のためのコメント付けが不十分な状態です。そちらについては今後整備を進めていく予定です。 ::

名前空間のusing指令
-------------------

以下のコードで、"cnoid"という名前空間の記述を省略できるようにしています。 ::

 using namespace cnoid;

cnoidはChoreonoidフレームワークの名前空間で、基本的にChoreonoidが提供するクラスや関数は全てこの名前空間内で定義されています。例えば本サンプルで利用するPluginクラスは名前空間も含めると "cnoid::Plugin" と記述する必要がありますが、このusing指令を記述しておくことで、名前空間部分を省略して単に "Plugin" と記述することが可能になります。

ただし、名前空間は名前の衝突を避けるためのものなので、無闇にusing指令を行うのは良くありません。原則として、ヘッダファイルにおいてはusing指令の利用は避け、名前空間も含めた全ての記述を行うべきでしょう。一方で、実装ファイル(.cpp)においては、名前の衝突が問題にならなければ、上のような記述を行うことでコードを簡潔にすることが出来ます。

プラグインクラスの定義
----------------------

次に、HelloWorldプラグインに対応するクラスを定義しています。 ::

 class HelloWorldPlugin : public Plugin
 { 
     ...
 };


Choreonoidのプラグインは、このようにPluginクラスを継承したクラスとして定義します。継承したクラスの名前は自由につけてもらって結構ですが、最後が "Plugin" で終わる名前にすると、分かりやすくてよいかと思います。また、既存のプラグインと名前が衝突しないように注意してください。

プラグインのクラスにおいて最低限実装すべき関数として、

* コンストラクタ
* initialize 関数

があります。
以下ではこれらの関数の記述について解説します。

コンストラクタ
-------------- 

コンストラクタの記述は以下のようになっています。 ::

 HelloWorldPlugin() : Plugin("HelloWorld")
 {
 
 }

プラグインクラスのコンストラクタでは、このように基底となるPluginクラスのコンストラクタにプラグインの名前を与えて呼び出す必要があります。通常、クラス名から最後の"Plugin"の部分を省いた名前を与えます。

本サンプルではコンストラクタ内には特に記述をしていませんが、プラグインが他のプラグインを必要とする際には、ここで"require"という関数を用いて依存関係をシステムに伝える必要があります。これについては :doc:`sample1` を参照ください。

initialize 関数
---------------

プラグインの初期化は、以下のようにinitialize関数にて記述します。 ::

 virtual bool initialize() override
 {
     ...
 }

initialize関数は、基底となるPluginクラスでもvirtual関数として定義されています。これをオーバーライドすることで各プラグインの挙動を実装するようになっています。このようにオーバーライドすることを前提としたvirtual関数として、他に finalize, description といった関数があります。ここではオーバーライドしていることを明確にするため、C++11から導入されたoverrideキーワードを付けています。

initialize関数はプラグインが読み込まれた後、プラグインの依存関係を考慮した順番で実行されていきます。そして、必要なオブジェクトを生成し、初期化に成功した場合は、trueを返すようにします。初期化が出来なかった場合は、falseを返すようにしてください。これにより、システムはプラグインの初期化が成功したかどうかを判断します。

メニューの追加
--------------

次に、initialize関数内の記述を見ていきましょう。 ::

 Action* menuItem = menuManager().setPath("/View").addItem("Hello World");

ここではメニューを追加しています。menuManager() はPluginクラスのメンバ関数（正確にはPluginクラスの基底であるExtensionManagerクラスで定義されている関数）で、メインメニューを管理するMenuManagerオブジェクトを返します。

このオブジェクトに対して setPath("/View") を行うことで、現在の管理対象位置をルートメニューの"View"というサブメニューに設定しています。このように、MenuManager においてはメニュー階層をファイルパスと同様にスラッシュで区切って表現するようにしており、これをメニューパスとしています。

setPath() はパスの設定後に自身のMenuManagerオブジェクトを返すようになっていますので、これに対してさらに addItem("Hello World") を呼び出すことで、サブメニュー "View" 内に "Hello World" という項目を追加することになります。

addItem は追加されたメニュー項目をActionオブジェクト(へのポインタ)として返します。ここでは、とりえあずこのオブジェクトをmenuItemという変数に格納し、このオブジェクトに対する操作は次の行で行うように記述しています。

なお、日本語環境で動作させている場合、"View"というサブメニューは実際には翻訳されて「表示」というメニューになっています。これは国際化機能によるものなのですが、ソースコードにおけるメニューパスはオリジナルの英語の文字列で記述する必要があります。オリジナルの記述については、Base/MainWindow.cppなどのソースを見てもらえば分かるかと思います。あるいは、OSの対象言語を英語に設定してChoreooidを実行すれば、メニューの表記は全てソースコードと同一のものとなります。

.. note:: Linuxの場合はLANG環境変数に"C"を設定することで対象言語を英語に切り替えることができます。コマンドラインから設定する場合は、export LANG=C と入力します。


メニューの関数への結びつけ
--------------------------

以下のコードでは、追加したメニューをユーザが選択したときに呼び出す関数について設定しています。 ::

 menuItem->sigTriggered().connect([&](){ onHelloWorldTriggered(); });

まず、menuItem->sigTriggered() により、Actionクラスが持つ"sigTriggered"という「シグナル」取得しています。シグナルというのは、何らかのイベントが起こったときにそれを知らせるためのオブジェクトです。Choreonoidフレームワークにおいては多くのクラスがそれぞれ独自のシグナルを有しています。

各シグナルはそれぞれ特定のイベントに対して定義されたものとなっており、"sigTriggered"は、ここでは「ユーザがメニューを選択した」というイベントに対応しています。

シグナルは "connect" というメンバ関数で、イベントが起こったときに呼ばれる関数を設定することが出来ます。connectに与える引数は、「シグナルに対して定義された関数型」に対して「同じ型か変換可能な型」である「関数オブジェクトとみなせるもの」であれば、何でも構いません。

まずは「sigTriggeredに対して定義された関数型」を知るために、Actionクラスが定義されている"src/Base/Action.h"を見てみましょう。するとsigTriggeredを取得する関数は、 ::

 SignalProxy<void()> sigTriggered();

と定義されています。ここで戻り値は "SignalProxy" テンプレートで定義されており、テンプレート引数として "void(void)" という記述があります。これは "sigTriggered" というシグナルが、 ::

 void function()

という形式の関数、つまり引数無し、戻り値無しの関数と結びつけるようになっていることを示しています。

ですので、例えば結び付けたい関数が素の関数として、 ::

 void onHelloWorldTriggered()
 {
     ...
 }

というように定義されていれば、この関数をそのまま与えて、 ::

 menuItem->sigTriggered().connect(onHelloWorldTriggered);

と記述することが可能です。これは、C言語におけるコールバック関数の利用とほぼ同じですね。

本サンプルではこのように記述しても良いのですが、実際のプラグイン開発時には、素の関数ではなく、クラスのメンバ関数を結び付けたい場合が多くあります。そこで本サンプルではあえてクラスのメンバ関数を結びつけるようにしています。

しかし、メンバ関数は通常、オブジェクト自身のポインタに対応する "this" という引数を暗黙的に持っており、この引数によってオブジェクトのインスタンスを識別しています。ですので、メンバ関数を普通の関数と同じようにconnect関数に渡そうとしても、this引数が指定されていないことになり、コンパイルエラーになってしまいます。

コンパイル・実行できるようにするためには、this引数込みのメンバ関数を引数なしの関数に変換したかたちでconnectに渡さなければなりません。これにはいろいろなやり方がありますが、通常はC++11から導入された「ラムダ式」を使用します。ここでは以下の記述がこれに対応します。 ::

 [&](){ onHelloWorldTriggered(); }

このように角括弧、丸括弧、波括弧によって [ ] ( ) { } の形式で書かれるのがC++11のラムダ式です。ラムダ式の詳細についてはC++11の仕様や解説を参照してください。ここでは [&] によってthis変数をキャプチャしており、引数は無しとした上で、メンバ関数の onHellowWorldTriggered 関数を呼ぶように記述しています。より分かりやすく書くと、 ::

 [this](void){ this->onHelloWorldTriggered(); }

となるかと思います。この記述で、「引数無しで、このラムダ式を定義しているスコープのthis変数を使用して、this->onHelloWorldTriggered() を実行し、戻り値は無しとする」関数を定義しています。と書くと少しややこしく感じるかもしれませんが、要はHelloWorldPluginクラスで定義しているメンバ関数onHelloWorldTriggeredを呼ぶ関数を作っているわけです。

以上により、「ユーザがメニューを選択すると onHelloWorldTriggered が呼ばれる」という設定を行うことができました。

Choreonoidのフレームワークを使いこなすためには、このようにシグナルと関数を結びつける仕組みに慣れておく必要があります。ここでは引数無しのシグナルでしたが、引数のあるシグナルも多数定義されています。それについては他のプラグインの解説で紹介します。

なお、本サンプルでは「メニューの追加」と「関数の結びつけ」の２つに分けて説明するため"menuItem"という変数を定義していますが、これが必要なければ以下のようにまとめて書いてもよいかと思います。 ::

 menuManager().setPath("/View").addItem("Hello World")->
     sigTriggered().connect([&](){ onHelloWorldTriggered(); });

.. note:: Choreonoidのシグナルは、Boost.Signalsライブラリを参考にしたものですが、独自の実装となっています。Boost.Signalsは今では古くなってしまったようですが、そのドキュメントも参考になるかと思います。ただしBoost.Signalsではコールバック関数の構築にBoostのBindライブラリを使用するようになっていましたが、今はC++11のラムダ式がありますので、そちらを使用するほうがよいです。

補足：QtのシグナルとChoreonoidのシグナル
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ここで出てきたActionクラスは、Choreonoidが使用しているGUIライブラリ "Qt" の "QAction" クラスを継承して拡張したもので、ChoreonoidのBaseモジュール（src/Base）において定義されています。（実際には名前空間cnoidの中で定義されています。）

拡張の目的はQActionをChoreonoidのシグナルの形態で使用できるようにすることで、これによってChoreonoidにQtオブジェクトの使い勝手（コーディングのしやすさ）を向上させようとしています。よく使われるQtのクラスに対して、Choreonoid用に拡張した同様のクラスが他にも多数定義されており、それらのクラス名はいずれも元のクラス名からQtのプレフィックスである "Q" を取り除いたものとしています（cnoidの名前空間内で定義しているので、正確な名前は "cnoid::Qtのクラス名からQを省いた名前" になります）。

Qtを使用したプログラミングの経験がある方はご存知かと思いますが、Qtは「シグナル／スロット」と呼ばれる独自のシグナルシステムを備えており、QActionについてもこのシステムに基づく"triggered"というシグナルを備えています。これを使えば上で説明したことと同じことが出来ます。実際のところ、Actionクラスにおける拡張内容も、元々のQtのシグナルを捉えて、それをChoreonoidのシグナル型としてあらためて処理しなおすようになっています。このように少し愚直でオーバーヘッドも生じる方法で、QtのシグナルをあえてChoreonoidのシグナルに変換しています。

この変換によって、Choreonoidフレームワークを使用したコードにおいてシグナル処理の統一性を高めています。QtのシグナルはC++言語の仕様の範囲を超えた記述を必要とし、それを処理するためにコンパイル時にmocというツールによる追加の処理が必要になるなど、かなり独自の形態をとっています。一方Choreonoidのシグナルは通常のC++言語の仕様内で処理できるものであり、そちらに統一した方がまとまりがよくなると考えています。

メニューがクリックされた際の挙動の記述
---------------------------------------

メニューが選択されたときに呼ばれる関数の実装は、以下のようになっています。 ::

 void onHelloWorldTriggered()
 {
     MessageView::instance()->putln("Hello World !");
 }

MessageViewはChoreonoidのメッセージビューに対応するクラスです。メッセージビューはChoreonoid上でひとつだけ存在するオブジェクトのため、いわゆるシングルトンクラスとして定義されています。シングルトンクラスのイディオムである "instance" 関数により、MessageViewの単一のインスタンスを取得しています。

MessageViewはテキスト出力のための関数をいくつか備えており、ここではそのうちのひとつである "putln" 関数を用いて、与えたメッセージを改行付きで出力しています。

MessageViewはostream型のオブジェクトを返すcout()という関数も提供しています。これを使えば、std::cout への出力と同様に、iostreamの記述法でテキストを出力することが可能です。

本サンプルではメッセージビューを使いましたが、Choreonoidは他にも様々なビューやツールバーを備えており、メッセージビューと同様に、プラグインから使用することが可能です。その場合、まず使いたいビューやツールバーのヘッダをインクルードし、対応するクラスのinstance関数などでインスタンスを取得して使用します。各クラスがどのような関数を提供しているかについては、今のところはDoxygen生成のリファレンスマニュアルやヘッダファイルなどを参照して調べてください。

プラグインエントリの定義
-------------------------

最後に以下の記述をしています。 ::

 CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)

ここでは cnoid/Plugin ヘッダで定義されている "CNOID_IMPLEMENT_PLUGIN_ENTRY" というマクロを使用しています。このマクロにプラグインのクラス名を記述すると、ChoreonoidのシステムがプラグインのDLLからプラグインインスタンスを取得するための関数が定義されます。この記述をしておかないと、作成したDLLがプラグインとして認識されませんので、忘れないようにしてください。

なお、各プラグインは、ひとつのプラグインを実装したひとつのDLLとして作成する必要があります。ひとつのDLLに複数のプラグインを実装することは出来ません（上記のマクロを２つ以上記述することは出来ません）ので、ご注意ください。

以上でソースの解説は終了です。次に、このプラグインのビルド方法について説明します。

.. _hello-world-build:

ビルド方法
----------

プラグインをビルドして利用するために必要な項目は以下のとおりです。

* Choreonoidの依存ライブラリ(Qt等)のヘッダファイル、ライブラリファイルが、ビルドツールから利用可能になっていること
* Choreonoid本体の提供するヘッダファイル、ライブラリファイルについても、ビルドツールから利用可能になっていること
* 依存ライブラリやChoreonoid本体のバイナリをビルドした環境とコンパチビリティのあるビルド環境・オプションでビルドすること（同じOS,アーキテクチャ、コンパイラであれば基本的には問題ないはず）
* プラグインのバイナリを共有ライブラリもしくはダイナミックライブラリとしてビルドすること
* バイナリの名前は、Linuxであれば "libCnoidXXXPlugin.so" (XXXのところにプラグイン名を入れる）、Windowsであれば "CnoidXXXPlugin.dll" とすること
* バイナリをChoreonoidのプラグインフォルダに格納すること。プラグインフォルダは、Choreonoidインストール先の "lib/choreonoid-x.x/" 以下になる（x.xはバージョン番号に対応)

これらの項目を押さえた上で、どのような方法でプラグインをビルドしても構いません。ここでは以下の３つの方法について紹介したいと思います。

1. Choreonoid本体とまとめてビルドする
2. CMakeを使用してプラグイン単体でビルドする
3. Makefileを直接記述してプラグインをビルドする

.. note:: 方法2, 3については主にLinux上でのビルドを想定しています。Windows上でVisual Studio (Visual C++)を用いてビルドする場合は、方法1を使用するのが簡単です。そうでない場合は、Visual C++のプロジェクトを作成し、その設定ダイアログで、インクルードパスやライブラリパス、ライブラリなどを自前で設定する必要があります。

.. _hello-world-build-together:

Choreonoid本体とまとめてビルドする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Choreonoidをソースからビルドして使用している場合は、自前のプラグインをChoreonoid本体とまとめてビルドすることができます。Choreonoid本体のソース内に自前のプラグインのソースも追加して、本体の付属品としていっしょにビルドしてしまおうという話です。

Choreonoid本体をビルドするための情報はCMakeによって管理されています。ここでCMakeはChoreonoidを構成するヘッダやライブラリに加え、Choreonoidが依存している外部のライブラリについても、ビルドのための情報を持っています。そのような情報を、自前のプラグインのビルドにも活用するというのが、この方法のポイントです。この場合、自前プラグインのビルドに必要な記述は必要最小限で済むため、手軽にプラグインを作成することができます。従って、Choreonoidをソースからビルドして使用しているユーザには、この方法がお勧めです。

具体的な手順は以下のようになります。まず、Choreonoidのソースディレクトリには "ext" というディレクトリがあり、ここに追加のプラグインを配置するようになっています。従って、まずこのextディレクトリにプラグイン用のサブディレクトリを作成し、そこにプラグインのソースファイルやビルドのためのCMakeLists.txtファイルを格納するようにしてください。

HelloWorldプラグインの場合、以下のような構成でファイルを配置します。 ::

 + Choreonoidのソースディレクトリ
   + ext
     - HelloWorldPlugin.cpp
     - CMakeLists.txt

.. highlight:: cmake

そして、CMakeLists.txtには以下のように記述します。 ::

 set(target CnoidHelloWorldPlugin)
 add_cnoid_plugin(${target} SHARED HelloWorldPlugin.cpp)
 target_link_libraries(${target} CnoidBase)
 apply_common_setting_for_plugin(${target})

ここではまず ::

 set(target CnoidHelloWorldPlugin)

で、プラグイン名を設定しています。プラグインの名前は、このように "Cnoid" で始め、"Plugin" で終わるようにします。ここではこの名前をtargetという変数に設定し、以下に続くコマンドで使用できるようにしています。必ずしも変数に設定する必要はありませんが、このようにすることで、プラグイン名の設定を一元化しています。 ::

 add_cnoid_plugin(${target} SHARED HelloWorldPlugin.cpp)

これがプラグインを実際にビルドするための記述です。"add_cnoid_plugin" はChoreonoid本体のCMakeファイルで定義されているコマンドで、ここにプラグイン名やソースファイル名を指定することで、プラグインのビルドすることができます。このコマンドはChoreonoidソースのトップディレクトリにあるCMakeLists.txtにて記述されていますので、詳細を知りたい方はそちらをご確認ください。基本的にはCMakeでライブラリを作成する際に使用する "add_library" コマンドをプラグイン用にカスタマイズしたものとなっており、add_libraryコマンドと同様に使用します。 ::

 target_link_libraries(${target} CnoidBase)

これはプラグインが依存するライブラリを明示するための記述で、ここではChoreonoid本体に含まれる "CnoidBase" ライブラリを指定しています。CnoidBaseはChoreonoidのGUIのベースとなる部分を実装しているライブラリで、本サンプルで使用するメッセージビューの実装もここに含まれています。Choreonoidのプラグインであれば必ずリンクすることが必要なライブラリです。この記述により、HelloWorldプラグインにCnoidBaseライブラリがリンクされるようになります。

なお、CMakeでは、同一のプロジェクトで定義されているライブラリをtarget_link_librariesで指定すると、そのライブラリが依存している全てのライブラリへのリンクも行われるようになります。例えば、CnoidBaseはQtのライブラリにも依存しているため、上記の記述でHelloWorldプラグインにもQtのライブラリがリンクされるようになります。このように、本手法ではリンクすべきライブラリについてあまり細かい部分まで気にせずに完結に記述することができます。 ::

 apply_common_setting_for_plugin(${target})

プラグインに対して共通で適用すべき設定をしてくれるコマンドです。このコマンドもトップディレクトリのCMakeLists.txtにて定義されています。プラグインには通常この記述もしておきます。これにより、例えば "make install" によってプラグインもインストールすれるようになります。

CMakeLists.txt の記述方法の詳細は `CMakeのマニュアル <http://www.cmake.org/cmake/help/help.html>`_ を参照してください。また、Choreonoidに含まれるライブラリや他のプラグイン、サンプルのCMakeLists.txtを読むことで、おおよその書き方が分かってくるかと思います。

このようにCMakeLists.txtを記述し、プラグインのソースファイルとともにext以下のサブディレクトリに配置したら、Choreonoid本体のビルドを実行します。すると本プラグインのCMakeLists.txtが自動的に検出され、Choreonoid本体と共にHelloWorldプラグインもビルドされるようになります。

.. note:: この方法では、ビルドはChoreonoid本体に対して行うことに注意してください。上記のCMakeLists.txtはそれ単体でプラグインをビルドできるものではないため、プラグインのディレクトリに対してcmakeコマンドを実行してもうまくいきません。その場合Choreonoid本体のCMakeにも影響が出ることがあるため、そのような操作はさけるようにしてください。

なお、プラグインのCMakeLists.txtでは、冒頭に以下のような記述をしておくとよいです。 ::

 option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)
 if(NOT BUILD_HELLO_WORLD_SAMPLE)
   return()
 endif()

この記述により、"BUILD_HELLO_WORLD_SAMPLE" というオプションがCMakeの設定に付与されます。ここではデフォルトをOFFとしていて、その場合このプラグインのビルドはスキップされます。プラグインをビルドしたい場合は、CMakeの設定でこのオプションをONにします。このようにプラグインをビルドするかどうかを切り替えられるようにしておくと、プラグインの開発や運用がしやすくなるかと思います。

.. _hello-world-stand-alone-build:

CMakeを使用してプラグイン単体でビルドする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

上ではプラグインをChoreonoid本体とまとめてビルドする方法を紹介しましたが、プラグインをChoreonoid本体とは別にビルドしたいこともあるかと思います。その方が開発しやすいということもあるでしょうし、そもそもChoreonoid本体をバイナリパッケージなどでインストールしていて、本体のビルド環境を利用できない場合もあるかと思います。

そのような場合には、プラグインを単体でビルドすることも可能です。ここではCMakeを利用してこれを行う方法を紹介します。

この場合、Choreonoidのソースファイルは必要ありませんが、Choreonoidのヘッダやライブラリで構成される「SDK」はインストールされている必要があります。これはChoreonoid本体のビルド時にCMakeで "INSTALL_SDK" というオプションをONにしているとインストールされます。バイナリパッケージを使用する場合は、このSDKを含むものを利用するようにします。

その上で、プラグインのソースファイルと共に、以下のようなCMakeLists.txtを用意します。 ::

 cmake_minimum_required(VERSION 3.1.0)
 project(HelloWorldPlugin)
 find_package(Choreonoid REQUIRED)
 add_definitions(${CHOREONOID_DEFINITIONS})
 include_directories(${CHOREONOID_INCLUDE_DIRS})
 link_directories(${CHOREONOID_LIBRARY_DIRS})
 set(target CnoidHelloWorldPlugin)
 add_library(${target} SHARED HelloWorldPlugin.cpp)
 target_link_libraries(${target} ${CHOREONOID_BASE_LIBRARIES})
 install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})

このCMakeLists.txtをプラグインのソースファイルと同じディレクトリに配置します。後は通常のcmakeの使い方で、ビルドを行ってください。インストールの操作を行うと、ビルドされたプラグインファイルがChoreonoidのプラグインディレクトリにインストールされます。Linuxの場合、端末上でプラグインのソースディレクトリに移動し ::

 cmake .

を実行した後に ::

 make

でビルドを行います。ビルドに成功したら ::

 make install

でインストールします。（インストール先によってはroot権限が必要となります。）

.. note:: この方法では、プラグインのソースファイルやCMakeLists.txtを、Choreonoidのextディレクトリ以下に配置するわけでは無いことにご注意ください。cmakeの実行もChoreonoid本体に対して行うのではなく、あくまでこのプラグインに対して直接実行します。

以下ではCMakeLists.txtの記述内容について解説します。 ::

 cmake_minimum_required(VERSION 3.1.0)

必要なCMakeのバージョンを指定しています。インストールされているCMakeのバージョンやCMakeLists.txtの記述内容を考慮して、適切なバージョンを設定するようにしてください。 ::

 project(HelloWorldPlugin)

CMakeのプロジェクトを設定します。今回は単体でビルドするため、これを設定する必要があります。 ::

 find_package(Choreonoid REQUIRED)

インストールされているChoreonoidの情報を取得します。Choreonoidのプラグインを作る以上、Choreonoidは必ず必要なので、ここではREQUIREDを指定しています。Choreonoidが見つかれば、その情報が以下のような変数に設定されます。

.. list-table::
 :widths: 40,60
 :header-rows: 1

 * - 変数
   - 内容
 * - CHOREONOID_DEFINITIONS
   - コンパイルオプション
 * - CHOREONOID_INCLUDE_DIRS
   - ヘッダファイルのディレクトリ
 * - CHOREONOID_LIBRARY_DIRS
   - ライブラリファイルのディレクトリ
 * - CHOREONOID_UTIL_LIBRARIES
   - Utilモジュール使用時にリンクすべきライブラリ
 * - CHOREONOID_BASE_LIBRARIES
   - Baseモジュール使用時にリンクすべきライブラリ
 * - CHOREONOID_PLUGIN_DIR
   - プラグインファイルをインストールするディレクトリ

.. note:: find_packageを機能させるためには、CMakeのパッケージ検出パスにChoreonoidのインストール先が含まれている必要があるのですが、Choreonoidをデフォルトの/usr/local以外のディレクトリにインストールしている場合、それが検出パスに含まれていない可能性があります。この場合はfind_packageでChoreonoidが検出されません。検出されるようにするためには、Choreonoidのインストール先ディレクトリを環境変数CHOREONOID_DIRやCMAKE_PREFIX_PATHに設定するようにしてください。詳しくはCMakeのfind_packageに関するマニュアルを参照してください。

次に、find_packageによって取得された情報を以下のように使用しています。 ::

 add_definitions(${CHOREONOID_DEFINITIONS})
 include_directories(${CHOREONOID_INCLUDE_DIRS})
 link_directories(${CHOREONOID_LIBRARY_DIRS})

この記述により、コンパイルオプション、インクルードパス、リンクパスが適切に設定されます。 ::

 set(target CnoidHelloWorldPlugin)

プラグイン名を変数targetに設定しています。 ::

 add_library(${target} SHARED HelloWorldPlugin.cpp)

プラグインは共有ライブラリになりますので、CMake標準のadd_libraryコマンドでビルドを行うことができます。

:ref:`hello-world-build-together` では、add_libraryを拡張したadd_cnoid_pluginというコマンドでプラグインをビルドしましたが、プラグインを単体でビルドする場合は直接add_libaryを使用するようにします。 ::

 target_link_libraries(${target} ${CHOREONOID_BASE_LIBRARIES})

プラグインにリンクすべきライブラリを指定しています。find_packageで取得されたCHOREONOID_BASE_LIBRARIES変数を使用することで、プラグインの基盤となるライブラリ一式をリンクすることができます。 ::

 install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})

ビルドしたプラグインのファイルをChoreonoidのプラグインディレクトリにインストールするための設定です。インストール先はこのようにCHOREONOID_PLUGIN_DIR変数で指定することができます。

.. _hello-world-makefile-build:

Makefileを直接記述してプラグインをビルドする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

プラグインを単体でビルドする際に、Makeコマンドの設定ファイルであるMakefileを直接記述することも可能です。この方法についても、 :ref:`hello-world-stand-alone-build` 場合と同様に、ChoreonoidのSDKがインストールされている必要があります。

なお、この方法はあまりおすすめはしません。今はCMakeという優れたビルドツールがあるので、そちらを使用するのが賢明です。何か特殊な事情でMakefileを書く必要がある場合のために、この方法も紹介しておきます。

Makefileを直接記述する場合は、CMakeのfind_packageコマンドの代わりに、 `pkg-config <http://www.freedesktop.org/wiki/Software/pkg-config>`_ というツールを用いることで、ビルドに必要な情報を取得します。これを用いたMakefileの例を以下に示します。

.. code-block:: makefile

 CXXFLAGS += -fPIC `pkg-config --cflags choreonoid`
 PLUGIN = libCnoidHelloWorldPlugin.so
 
 $(PLUGIN): HelloWorldPlugin.o
 	g++ -shared -o $(PLUGIN) HelloWorldPlugin.o `pkg-config --libs choreonoid`
 
 install: $(PLUGIN)
 	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
 clean:
 	rm -f *.o *.so

このMakefileを用いてmakeすればプラグインのバイナリが生成され、"make install" を行えばChoreonoidのプラグインディレクトリに生成したバイナリがインストールされるはずです。後はChoreonoidを実行すればプラグインが読み込まれます。

pkg-configはUnix系のOSでよく使用されているツールで、上記のMakefileのように、pkg-configコマンドに適当なオプションをつけて実行することで、対応するライブラリのインクルードパスやリンクパス、ライブラリファイルなどの文字列を得ることができます。これをコンパイラのオプションとして渡すことで、それらの設定を直接記述しなくてもコンパイルすることが可能となります。詳しくはpkg-configのマニュアルをご参照ください。

.. note:: pkg-configについても、Choreonoidのインストール先が検出パスに含まれている必要があります。検出パスの追加は環境変数 "PKG_CONFIG_PATH" を用いて行うことができます。デフォルトの/usr/local以外にChoreonoidをインストールしている場合は、インストール先の "lib/pkgconfig" というをディレクトリを、PKG_CONFIG_PATHに設定するようにしてください。

ビルド用ファイルのサンプル
^^^^^^^^^^^^^^^^^^^^^^^^^^

ここで紹介した3つの方法に対応するビルド用のファイルをHelloWorldサンプルのディレクトリに格納してあります。それぞれ以下のようにして利用可能です。

1. :ref:`hello-world-build-together`

 サンプルディレクトリのCMakeLists.txtが対応。HelloWorldのディレクトリをext以下にコピーして、Choreonoid本体をビルドする。

2. :ref:`hello-world-stand-alone-build`

 こちらもサンプルディレクトリのCMakeLists.txtが対応。HelloWorldのディレクトリ内でcmakeを実行してビルドする。

3. :ref:`hello-world-makefile-build`

 ManualMakefileというファイルが対応。ファイル名をMakefileに変更するか、make実行時に "-f ManualMakefile" というオプションを付与する。

1と2については同じCMakeLists.txtになりますが、その内部で1用と2用で処理を分けて書いています。Choreonoid本体のビルドであるかどうかを判定し、そうであれば1の内容を、そうでなければ2の内容が処理されます。
