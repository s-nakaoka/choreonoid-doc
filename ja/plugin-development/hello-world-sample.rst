
HelloWorld サンプルの解説
=========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>


このドキュメントでは、サンプルプラグインのひとつである"HellowWorldPlugin"の実装について解説します。

.. contents:: 目次
   :local:


ソースコード
------------

.. highlight:: cpp

本サンプルのソースコードは以下のとおりです。 ::

 #include <cnoid/Plugin>
 #include <cnoid/MenuManager>
 #include <cnoid/MessageView>
 #include <boost/bind.hpp>
 
 using namespace cnoid;
 using namespace boost;
 
 class HelloWorldPlugin : public Plugin
 {
 public:
     
     HelloWorldPlugin() : Plugin("HelloWorld")
     {
 
     }
     
     virtual bool initialize()
     {
         Action* menuItem = menuManager().setPath("/View").addItem("Hello World");
         menuItem->sigTriggered().connect(bind(&HelloWorldPlugin::onHelloWorldTriggered, this));
         return true;
     }
 
 private:
     
     void onHelloWorldTriggered()
     {
         MessageView::mainInstance()->putln("Hello World !");
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
 

本コードはソースパッケージの "share/sampleplugins/HelloWorldPlugin" 以下に "HellowWorldPlugin.cpp" というファイルで格納されていますので、テストの際にはそちらをご利用ください。
（なお、説明の都合やバージョンの違いなどにより、このソースはパッケージに収録されているものとは多少異なる場合がありますが、ご了承ください。）

本サンプルをコンパイルしてインストールすると、メインメニューの「表示」に「Hello World」という項目が追加されます。そしてこのメニューを選択すると、メッセージビューに "Hello World !" と出力されます。ただそれだけのプラグインですが、以下ではこのサンプルの解説を通して、Choreonoidのプラグイン開発の基本を学んでいくことにします。



ヘッダのインクルード
----------------------------

まず、本サンプルがインクルードしているヘッダの概要を説明します。 ::

 #include <cnoid/Plugin>
 #include <cnoid/MenuManager>
 #include <cnoid/MessageView>

これら3つのヘッダは、Choreonoidフレームワークが提供するヘッダです。それらは基本的にインクルードパスの"cnoid"というサブディレクトリに格納されており、上記のようにこのディレクトリをプレフィックスとして付加してインクルードを行います。また、インクルード用のヘッダファイルには拡張子をつけていませんので、上記のように拡張子のない形式で対象ヘッダの名前を書きます。拡張子の無い形式は、標準C++ライブラリのヘッダファイルでも採用されており、Choreonoidが基盤として利用しているEigen, Qt, OpenSceneGraph といったC++ライブラリでも採用されていて、C++において標準的な形式のひとつです。それらのライブラリとの間で記述の統一性を高めるというねらいもあり、Choreonoidでもこの形式を採用しています。(ちなみに、この形式は「ヘッダファイル」ではなく「ヘッダ」と呼ぶことになっているようです。しかし拡張子付きのものもまだまだ使われていますので、用語の使い分けが難しいですね…。）

ここでインクルードしている３つのヘッダはそれぞれ以下のような機能を持っています。

* cnoid/Plugin

 Pluginクラスが定義されたヘッダです。プラグインを作る際にはこのヘッダをインクルードし、Pluginクラスを継承したクラスを作ることでひとつのプラグインを定義します。

* cnoid/MenuManager

 メニューの管理を行うMenuManagerクラスが定義されたヘッダです。メニューに項目を追加する際には、このヘッダをインクルードして、MenuManagerを使えるようにします。

* cnoid/MessageView

 テキストメッセージを出力するビューである「メッセージビュー(MessageView)」が定義されたヘッダです。メッセージビューにテキストを出力したい場合は、このヘッダをインクルードします。

これらのヘッダの実態はソースツリーの src/Base 以下にあります。クラス定義の詳細を知りたい場合は、それらのヘッダファイルを直接参照してみてください。（ヘッダファイルの実態については拡張子 .h がついていますので、ご注意ください。）
なお、Doxygenというツールを用いることで、クラス定義の詳細を一覧できるリファレンスマニュアルを生成することも可能なのですが、今のところ解説文生成のためのコメント付けが不十分な状態です。そちらについては今後整備を進めていく予定です。 ::

 #include <boost/bind.hpp>

Boost C++ ライブラリ集より、"Bind"というライブラリのヘッダをインクルードしています。Bindは関数オブジェクトを柔軟に生成するためのライブラリで、Choreonoidにおいては「シグナル」という仕組みでイベント処理の関数を呼び出すためによく利用されます。そちらの詳細については後ほど解説します。

ChoreonoidにおいてはBoostが提供する他のいくつかのライブラリについても利用しますので、それらのライブラリの概要をひととおり把握しておくのが望ましいです。具体的には、Bindに加えて、Smart Ptr, Signals, Function, Format, Dynamic Bitset, Multi-Array といったライブラリがプラグイン開発において関わってきます。
それらのライブラリの詳細については、 `Boost公式ページのドキュメント <http://www.boost.org/doc/libs/>`_ を参照してください。また、稲葉氏による `Let's Boost のページ <http://www.kmonos.net/alang/boost/>`_ や `書籍「Boost C++ Libraries プログラミング」 <http://www.kmonos.net/pub/BoostBook/>`_ も参考資料として有用です。

なお、boostのヘッダファイルについては .hpp という拡張子がつく形式となっていますので、この点注意してください。(C++においてこういった記述を統一することは難しいですね…。）


名前空間のusing指令
-------------------

以下のコードで、それぞれ"cnoid", "boost" の名前空間の記述を省略する指令を行っています。 ::

 using namespace cnoid;
 using namespace boost;

cnoidはChoreonoidの名前空間で、基本的にChoreonoidが提供するクラスや関数は全てこの名前空間内で定義されています。例えば本サンプルで利用するPluginクラスは名前空間も含めると cnoid::Plugin と記述する必要がありますが、あらかじめ上の記述をしておくことで、名前空間部分を省略して単に Plugin と記述することが可能になります。

ただし、名前空間は名前の衝突を避けるためのものなので、無闇にusing指令を行うのは良くありません。原則として、ヘッダファイルにおいてはusing指令の利用は避け、名前空間も含めた全ての記述を行うべきでしょう。一方で、実装ファイル(.cpp)においては、名前の衝突が問題にならなければ、上のような記述を行うことでコードを簡潔にすることが出来ます。

ここでは Boost ライブラリの名前空間"boost"についてもusing指令を行っています。Choreonoidのプラグイン開発では Boost のライブラリも頻繁に使うことになるので、boostの名前空間を省略できると記述が多少楽になるのですが、一方でboostは多くの関数やクラスを含むので、名前の衝突や混乱も生じやすくなってしまいます。従って、名前空間の記述方法については状況に応じて使い分けるようにしてください。また、特定のクラスなどに対してのみ省略を可能とする「using宣言」という構文もありますので、そちらを使うのもよいかと思います。


プラグインクラスの定義
-----------------------

次に、HellowWorldプラグインに対応するクラスを定義しています。 ::

 class HelloWorldPlugin : public Plugin
 { 
     ...
 };


Choreonoidのプラグインは、このように cnoid::Plugin （ここでは cnoid:: を省略）を継承したクラスとして定義します。
継承したクラスの名前は自由につけてもらって結構ですが、最後が "Plugin" で終わる名前とすると分かりやすくてよいかと思います。
また、既存のプラグインと名前が衝突しないように注意してください。

プラグインのクラスにおいて最低限定義すべき関数として、

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

本サンプルではコンストラクタ内には特に記述をしていませんが、プラグインが他のプラグインを必要とする際には、ここで"require"という関数を用いて依存関係をシステムに伝える必要があります。これについてはSample1Pluginの解説をご覧ください。



initialize 関数
---------------

プラグインの初期化は、以下のようにinitialize関数にて記述します。 ::

 virtual bool initialize()
 {
     ...
 }

initialize関数は、基底となるPluginクラスで定義されたvirtual関数になっていて、これをオーバーライドすることで各プラグインの実際の挙動を実装するようになっています。このようなvirtual関数として、他に finalinze, description といった関数があります。

initialize関数はプラグインがメモリに読み込まれた後、プラグインの依存関係を考慮した順番で呼ばれていきます。そして、必要なオブジェクトを生成し、初期化に成功した場合は、true を返すようにします。もし初期化が出来なかった場合は、falseを返してください。これにより、システムはプラグインの初期化が成功したかどうかを判断します。



メニューの追加
--------------

次に、initialize関数内の記述を見ていきましょう。 ::

 Action* menuItem = menuManager().setPath("/View").addItem("Hello World");

ここではメニューを追加しています。menuManager() はPluginクラスのメンバ関数（正確にはPluginクラスの基底であるExtensionManagerクラスで定義されている関数）で、メインメニューを管理するMenuManagerオブジェクトを返します。

このオブジェクトに対して setPath("/View") を行うことで、現在の管理対象位置をルートメニューの"View"というサブメニューに設定しています。このように、MenuManager においてはメニュー階層をファイルパスと同様にスラッシュで区切って表現するようにしており、これをメニューパスとしています。

setPath() はパスの設定後に自身のMenuManagerオブジェクトを返すようになっていますので、これに対してさらに addItem("Hello World") を呼び出すことで、サブメニュー "View" 内に "Hello World" という項目を追加することになります。

addItem は追加されたメニュー項目をActionオブジェクト(へのポインタ)として返します。ここでは、とりえあずこのオブジェクトをmenuItemという変数に格納し、このオブジェクトに対する操作は次の行で行うように記述しています。

なお、日本語環境で動作させている場合、"View"というサブメニューは実際には翻訳されて「表示」というメニューになっています。これは国際化機能によるものなのですが、ソースコードにおけるメニューパスはオリジナルの英語の文字列で記述する必要があります。オリジナルの記述については、Base/MainWindow.cppなどのソースを見てもらえば分かりますし、LANG環境変数にCなどと設定してChoreonoidを英語環境で起動することでも分かります。国際化機能の詳しい利用方法については、別のドキュメントで解説したいと思います。


メニューの関数への結びつけ
--------------------------

以下のコードでは、追加したメニューをユーザが選択したときに呼び出す関数について設定しています。 ::

 menuItem->sigTriggered().connect(bind(&HelloWorldPlugin::onHelloWorldTriggered, this));

この記述により、メニューが選択されたときに、HelloWorldPluginクラスの"onHelloWorldTriggered"というメンバ関数が呼ばれるようになります。以下ではこのコードの意味を少し詳しく解説したいと思います。

まず、menuItem->sigTriggered() により、Actionクラスが持つ"sigTriggered"というシグナルを取得しています。
シグナルというのは、何らかのイベントが起こったときにそれを知らせるためのオブジェクトとなっており、ChoreonoidではBoostのSignalsライブラリを使ってこれを実現しています。
各シグナルはそれぞれ特定のイベントに対して定義されたものとなっており、
"sigTriggered"は、ここでは「ユーザがメニューを選択した」というイベントを知らせるシグナルとなっています。

シグナルは"connect"というメンバ関数で、イベントが起こったときに呼ばれる関数を設定することが出来ます。connectに与える引数は、「シグナルに対して定義された関数型」に対して「同じ型か変換可能な型」である「関数オブジェクトとみなせるもの」であれば、何でも構いません。と言ってもピンと来ないかもしれませんが、まずは「sigTriggeredに対して定義された関数型」を知るために、Actionクラスが定義されている"src/Base/Action.h"を見てみましょう。するとsigTriggeredを取得する関数は、 ::

 SignalProxy< boost::signal<void(void)> > sigTriggered()

と定義されています。ここで戻り値の型の内側に"void(void)"という記述がありますが、これは、"sigTriggered" というシグナルが、 ::

 void function(void)

という形式の関数と結びつけるように定義されていることを示しています。

ですので、例えば結び付けたい関数が普通の関数として、 ::

 void onHellowWorldTriggered(void)
 {
     ...
 }

というように定義されていれば、この関数をそのまま与えて、 ::

 menuItem->sigTriggered().connect(onHellowWorldTriggered);

と記述することが可能です。これは、C言語におけるコールバック関数の利用とほぼ同じですね。

本サンプルではこのように記述しても良いのですが、実際のプラグイン開発時には、普通の関数ではなくてクラスのメンバ関数を結び付けたい場合が多くあります。そこで本サンプルではあえてクラスのメンバ関数を結びつけるようにしています。

しかし、（非staticな）メンバ関数は、実際には"this"という隠し引数(?)を関数の第一引数として持っており、この引数によってインスタンスを識別しています。ですので、メンバ関数を普通の関数と同じようにconnect関数に渡そうとしても、メンバ関数を呼び出す際のインスタンスが分からないため、うまく行きません。（もちろん、コンパイルエラーになります。）

そこで、BoostのBindライブラリが役に立ちます。Bindライブラリの提供する"bind"関数は既存の関数から適当に引数にアレンジを加えた関数オブジェクトを生成してくれます。と言うと少し分かりにくいかもしれませんが、ここでは「メンバ関数を普通の関数にする」という目的で使っていると考えてもらえばよいかと思います。そのための記述が ::

 bind(&HelloWorldPlugin::onHelloWorldTriggered, this)

の部分です。

まず、bindの第一引数には元になる関数を与えます。ここでは "&HelloWorldPlugin::onHelloWorldTriggered" として、HellowWorldPluginクラスのメンバ関数"onHelloWorldTriggered"を与えています。"&"をつけて、明示的にポインタ型として記述しなければならない点に注意してください。要は、"&クラス名::メンバ関数名" のかたちで記述すればOKです。

そして、bindの第二引数には、"this" を与えています。これにより、このメンバ関数を呼び出す際のインスタンスを指定しています。この例のように、メンバ関数内で同じクラスのメンバ関数を同じインスタンスで結びつける際には、this を与えることになり、実際このような状況は多いかと思われます。しかし、他のクラスのメンバ関数を結びつけることも可能で、その際にはthisではなく、そのクラスの適当なインスタンスを引数として与えることになります。

以上により、「ユーザがメニューを選択すると onHellowWorldTriggered が呼ばれる」という設定を行うことができました。

ここでやっていること自体は単純なことなのですが、それを支えているるシグナルやbindといった仕組みについては、少しややこしく感じられたかもしれません。とは言え上で説明したことはまださわりの部分で、Choreonoidのフレームワークを使いこなすためにはもう少し詳しく理解していく必要があります。これらの仕組みに関する追加の説明は他のサンプルの解説時にも行っていきますが、それ自体は本ガイドの対象外にもなってきますので、ユーザの方にはまず Boost の Signals, Bind の各ライブラリに関するドキュメントを読んでいただくのがよいかと思います。上の節で紹介したBoostに関するWebページや書籍などにぜひあたってみてください。概要を理解してもらえばOKで、実際の使い方は概ねパターン化したものになりますので、難しくはありません。

なお、本サンプルでは「メニューの追加」と「関数の結びつけ」の２つに分けて説明するため"menuItem"という変数を定義していますが、これが必要なければ以下のようにまとめて書いてもよいかと思います。 ::

 menuManager().setPath("/View").addItem("Hello World")->
     sigTriggered().connect(bind(&HelloWorldPlugin::onHelloWorldTriggered, this));


補足：QtのシグナルとChoreonoidのシグナル
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ここで出てきたActionクラス(cnoid::Actionクラス)はQtライブラリの"QAction"クラスを継承して拡張したもので、ChoreonoidのBaseモジュール(src/Base以下）において新たに定義されたものです。
拡張の目的はQActionのChoreonoidにおける使い勝手を向上させることにあり、その主な内容はsigTriggered()などのシグナル取得関数の追加となっています。
そして、よく使われるQtのクラスを拡張した同様のクラスが他にもいくつか定義されており、それらのクラス名はいずれも元のクラス名からQtのプレフィックスである"Q"を取り除いたものとしています（cnoidの名前空間内で定義しているので、正確な名前は"cnoid::Qを省いた名前"になります）。

ここで、Qtをご存知の方には言うまでもないことですが、実はQtは「シグナル／スロット」と呼ばれる独自のシグナルシステムを備えており、QActionについてもこのシステムに基づく"triggered"というシグナルを備えています。これを使えば上で説明したことと同じことが出来ますし、そもそも Boost.Signals もこのQtのシグナル／スロットに端を発して開発されてきたものです。Actionクラスにおける拡張内容も、元々のQtのシグナルを捉えて、それをBoost.Signalsベースのシグナルとして処理しなおすという、あまりスマートとは言えないものになっています。

ではなぜQtが備えているシグナルシステムを直接使わず、わざわざクラスを拡張してまで Boost.Signals をベースとする記述を行えるようにしているのか？それは、Qtに依存していない部分との統一性を高めるためでもあり、さらに、いずれにしてもそうした方が記述が簡潔で柔軟なものになると考えたからです。

まず、ChoreonoidはQtに依存していない、非GUIなモジュールも含んでいます。src/Util 以下の Utilモジュール、src/Body 以下の Bodyモジュール等がこれに当てはまります。これらはChoreonoidのGUIからは独立して、例えばロボットの体内PCにて動作する制御ソフトウェアで使われることも想定しているので、なるべく大きなGUIライブラリに依存させない方が望ましいのです。また、Choreonoid開発の歴史においても、使用してきたGUIライブラリは wxWidgets, Gtk+(Gtkmm), Qt と変遷してきており、今後Qtがどうなるかも分からないため、特定のGUIライブラリへ依存する部分は少なくするに越したことはないと考えています。これはGUI関連モジュールにおいても同様です。そこで、Qtのクラスを継承していないクラスについては、必要なシグナルは全てBoost.Signalsベースで記述することとしています。

一方で、QtのクラスについてはQtのシグナルシステムを使うということでも良いのですが、これはBoost.Signalsとは記述の仕方がかなり異なっており、さらにヘッダファイルに追加の記述をした上でMOCというプリプロセス処理を行う必要があります。すると全体的に記述の統一感が失われる上、コーディング作業も少々面倒なものになります。Choreonoidの開発者にとってこれは受け入れがたいところがありましたので、あえて愚直なクラス拡張を自前で用意し、記述の統一性・簡潔性にこだわることとしました。


メニューがクリックされた際の挙動の記述
---------------------------------------

上で述べた、メニューが選択されたときに呼ばれる関数"onHelloWorldTriggered()"の実装は、以下のようになっています。 ::

     void onHelloWorldTriggered()
     {
         MessageView::mainInstance()->putln("Hello World !");
     }

MessageViewクラスのクラス(static)関数 "mainInstance()"により、MessageViewのインスタンスを取得しています。
これは、いわゆるシングルトンクラスにおける"instance()"関数と同様のものです。

.. note:: この関数になぜ"main"を付けているかと言うと、MessageView は場合によっては複数生成して使い分けることもあるかもしれず、そうなるとシングルトンとは言えないと考えたからです。そして、「第一のインスタンス」という意味でこういった関数名にしました。
 同様のことが当てはまる他のクラス(ItemTreeViewやSceneView等)についても、同じ命名としています。
 しかし実際にはそれらのクラスで複数生成して使い分けるというところにはまだ至っておらず、仮にそうしたとしても多くの場合で「メインのインスタンス」が使われると思われるます。従って、それらのクラスに関してこの命名は廃止し、次期バージョンでは素直にinstance()でメインのインスタンスにアクセスできるよう改良する予定です。

MessageView はメッセージビューへのテキスト出力のための関数をいくつか備えており、ここではそのうちのひとつである"putln"関数を用いて、与えたメッセージを改行付きで出力しています。

MessageViewはcout()という関数によってostream型のオブジェクトも提供しています。これを使えば、std::cout への出力と同様に、iostreamの記述法でテキストを出力することが可能です。

本サンプルではMessageViewを使いましたが、Choreonoidは他にも有用な各種ビューやツールバー、および各種クラスの生成済みインスタンスが利用可能となっています。それらを利用する際には、このサンプルと同様に、まず使いたいクラスのヘッダをインクルードし、そのクラスのmainInstance()やinstance()といった関数でインスタンスを取得して使う、というのが基本になります。各クラスがどのような関数を提供しているかについては、今のところはDoxygen生成のリファレンスマニュアルやヘッダファイルなどを参照して調べてください。



プラグインエントリの定義
-------------------------

最後に、各プラグインクラスについて、以下の記述を行う必要があります。 ::

 CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)

これは cnoid/Plugin ヘッダにて定義されたマクロとなっており、プラグインのクラス名を与えることで、ChoreonoidシステムがプラグインのDLLからプラグインインスタンスを取得するための関数を定義します。この記述をしておかないと、作成したDLLがプラグインとして認識されませんので、忘れないようにしてください。

なお、各プラグインは、ひとつのプラグインを実装したひとつのDLLとして作成する必要があります。ひとつのDLLに複数のプラグインを実装することは出来ません（上記のマクロを２つ以上記述することは出来ません）ので、ご注意ください。

以上でソースの解説は終了です。次に、このソースのコンパイルの仕方について説明します。


コンパイルの仕方
----------------

プラグインをコンパイルし利用するために必要な項目は以下のとおりです。

* Choreonoidの依存ライブラリ(Boost, Eigen, Qt, OpenSeneGraph等)のヘッダファイル、ライブラリファイルが、ビルドツールから利用可能になっていること。
* Choreonoid本体の提供するヘッダファイル、ライブラリファイルについても、ビルドツールから利用可能になっていること。
* 依存ライブラリやChoreonoid本体のバイナリをビルドした環境とコンパチビリティのあるビルド環境・オプションでビルドすること（同じOS,アーキテクチャ、コンパイラであれば基本的には問題ないはず）。
* プラグインのバイナリを共有ライブラリもしくはダイナミックライブラリとしてビルドすること。
* バイナリの名前は、Linuxであれば "libCnoidXXXPlugin.so" (XXXのところにプラグイン名を入れる）、Windowsであれば "CnoidXXXPlugin.dll" とすること。
* バイナリをChoreonoidのプラグインフォルダに格納すること。プラグインフォルダは、Choreonoidインストール先の lib/choreonoid-x.x/ 以下になる（x.xはバージョン番号に対応)。

上記の項目を押さえた上で、どのような環境・方法でコンパイルするかは、プラグイン開発者の自由です。
本ドキュメントでは、サンプルのコンパイル用に用意した以下の２つの例について解説したいと思います。

* インストール済みのChoreonoidを使う
* Choreonoid本体のコンパイル環境を使う


インストール済みのChoreonoidを使う場合
--------------------------------------

まず、"make install"されたChoreonoid本体を外部ライブラリとして使ってコンパイルする方法を示します。

これを行う場合、Choreonoidをビルドする際に、CMakeのオプションで "INSTALL_SDK" を ON にしておいてください。
すると、"make install" 実行時に、実行ファイルだけでなく、ヘッダファイルやライブラリのリンクに必要なファイルもインストールされるようになります。この設定でまず"make install"をしておきましょう。

後はどのようにしてコンパイルしてもよいのですが、ここでは例として以下のMakefileを使うことにします。
(このMakefileはHellowWorldPluginのフォルダに格納されています。) 

.. code-block:: makefile

 CXXFLAGS += `pkg-config --cflags choreonoid`
 PLUGIN = libCnoidHelloWorldPlugin.so
 
 $(PLUGIN): HelloWorldPlugin.o
 	g++ -shared `pkg-config --libs choreonoid` -o $(PLUGIN) HelloWorldPlugin.o 
 
 install: $(PLUGIN)
 	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
 clean:
 	rm -f *.o *.so


このMakefileを用いてmakeすればプラグインのバイナリが生成され、"make install"を行えばChoreonoidのプラグインディレクトリにバイナリがコピーされるかと思います。後はChoreonoidを実行すればプラグインが読み込まれます。

特に特殊なことはしていませんが、インクルードパス、リンクパスの設定や、リンクするライブラリの設定は、 `"pkg-config" <http://www.freedesktop.org/wiki/Software/pkg-config>`_ というツールを用いて行っています。"pkg-config"というのは、Unix系OSで標準的に使われているツールで、対応したライブラリであれば、上記のMakefileのように適切なオプションと対象ライブラリ名で呼び出すことで、インクルードパスやリンクパス、リンクすべきライブラリなどの文字列を得ることができます。これをコンパイラのオプションとして渡すことで、それらの設定の詳細を気にしなくてもコンパイルすることが可能となります。詳しくはpkg-configのマニュアルをご参照ください。

なお、CMakeにて"CMAKE_INSTALL_PREFIX"をデフォルトの/usr/localから変更している場合は、そのままでは pkg-configがChoreonoidの設定ファイルを見つけることができません。この場合、環境変数 "PKG_CONFIG_PATH" にChoreonoidインストール先のサブディレクトリ"lib/pkgconfig"をフルパスで記述しておく必要があります。

.. note:: バージョン1.1.0までのChoreonoid配布ソースにて、pkg-configの設定が間違っていました。次期バージョンで修正しますが、それまではソースの"misc/pkgconfig/choreonoid.pc.in" の 14行目の "-lCnoidGuiBase" となっているところを "-lCnoidBase" と置き換えてビルドとインストールをし直してください。

Windowsにおいても、pkg-configをインストールすればこのようなMakefileによるコンパイルも出来ないことはないのかもしれません。しかし、WindowsにおいてはVisual C++のIDE上でプロジェクトを作ってコンパイルするのが一般的ではないかと思います。その場合、IDEのプロジェクト設定ダイアログで、インクルードパスやライブラリパス、ライブラリなどを自前で設定し、コンパイルする必要があります。


Choreonoid本体のコンパイル環境を使う場合
----------------------------------------

Choreonoidをソースからコンパイルしている場合は、その環境をプラグイン開発にも活用することができます。
といっても難しいことは何もなく、単にChoreonoidのソース内に新たなプラグインのソースを追加して、いっしょにコンパイルしてしまおうというだけの話です。

Choreonoid本体がコンパイルできていれば、本体のヘッダ等はもちろんのこと、依存ライブラリについても既に設定ができているはずですので、プラグイン追加の際にはそのあたりのことを気にせずに作業を進めることができます。また、Choreonoidが採用しているビルドシステムであるCMakeを使うことになり、CMakeを理解していればMakefileを書く場合に比べてより楽にビルド設定を記述していくことが出来ます。さらに、WindowsであってもChoreonoid本体のビルドと同様にVisual C++ のプロジェクトファイルが生成されますので、複雑な設定をせずにVisual C++ のIDEからビルドすることが可能です。

そういった次第で、Choreonoid本体をソースからコンパイルしているユーザには、この方法がお勧めできます。

では具体的な作業内容について説明しましょう。
まず、プラグイン用の追加のソースを置くディレクトリは、Choreonoidソースの"extplugin"ディレクトリとしています。
従って、まずこのディレクトリ以下に追加プラグイン用のサブディレクトリを作成し、そこにプラグインのソースやビルド設定を記述したCMakeLists.txtを格納するようにしてください。
なお、サブディレクトリの名前は最後が"Plugin"で終わるようにしてください。今のところ、そのようになっているサブディレクトリのみが、追加のソースとして認識されるようになっています。

HellowWorldPluginについては、この方法によるコンパイルの例として、"extplugin/sample/" 以下に "HelloWorldPlugin" というディレクトリが格納されています。サンプルですので、extplugin直下に置かずに、sampleというサブディレクトリ以下に置くようにしています。また、ソースファイル("HelloWorldPlugin.cpp")については元々「インストール済みのChoreonoidを使う場合」に対応したものを "share/sampleplugins/HelloWorldPlugin" 以下に置いていましたので、そのソースを参照するようにしています。このようにサンプルについては少々特殊なソース配置となっていますが、通常はextplugin以下のサブフォルダ内にソースを格納すればOKです。

.. highlight:: cmake

HellowWorldPlugin用のCMakeLists.txtは以下のようになっています。 ::


 option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)
 if(NOT BUILD_HELLO_WORLD_SAMPLE)
   return()
 endif()
  
 set(target CnoidHelloWorldPlugin)
 set(srcdir ${PROJECT_SOURCE_DIR}/share/sampleplugins/HelloWorldPlugin)
 add_library(${target} SHARED ${srcdir}/HelloWorldPlugin.cpp)
 target_link_libraries(${target} CnoidBase)
 apply_common_setting_for_plugin(${target})

まず、 ::

 option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)
 if(NOT BUILD_HELLO_WORLD_SAMPLE)
   return()
 endif()

の記述により、このサンプルをコンパイルしたくない場合は、コンパイルしないように設定できるようにしています。
デフォルトではOFFとしており、この切り替えは ccmake コマンド等で設定することが出来ます。
追加のプラグインについてもこのように記述しておくと、運用しやすくなってよいかと思います。

では、BUILD_HELLO_WORLD_SAMPLE が ON に設定されているときに実行される、続きの部分を見ていきましょう。まず、 ::

 set(target CnoidHelloWorldPlugin)

で、長いプラグイン名をtargetという変数で置き換えています。 ::

 set(srcdir ${PROJECT_SOURCE_DIR}/share/sampleplugins/HelloWorldPlugin)
 add_library(${target} SHARED ${srcdir}/HelloWorldPlugin.cpp)

プラグインに対応する共有ライブラリをビルドする設定です。ソースの格納先が違う場所なので少しややこしい記述になっていますが、基本的にはadd_libraryでソースファイル名を列挙すればOKです。 ::

 target_link_libraries(${target} CnoidBase)

リンクすべき依存ライブラリを記述しています。Choreonoid内のライブラリやプラグインであれば、このようにその名前を書くだけでOKです。
"CnoidBase"はChoroenoidのGUIフレームワークをまとめたライブラリとなっていて、フレームワークの基本機能を利用するだけであればこれを指定しておけばOKです。
また、依存先がさらに依存しているライブラリについては自動でリンクされますので、新たに他のライブラリを使うのでなければ、このようにChoreonoid内のライブラリを記述するだけでOKです。 ::

 apply_common_setting_for_plugin(${target})

プラグインに共通のビルド設定をしてくれる関数で、ChoreonoidソースのトップディレクトリにあるCMakeLists.txtにて定義されているものです。これを書いておけば、"make install" 時にインストールするといった処理も行ってくれます。

CMakeLists.txt の記述法の詳細は `CMakeのマニュアル <http://www.cmake.org/cmake/help/help.html>`_ を参照してください。また、このサンプルも含めて、Choreonoid内部の他のライブラリやプラグイン、サンプルのCMakeLists.txtを読むことで、おおよその書き方が分かってくるかと思います。


