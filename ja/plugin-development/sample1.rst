
Sample1Pluginの解説
================

このドキュメントでは、サンプルプラグインのひとつである"Sample1Plugin"の実装について解説します。
本ドキュメントは、 :doc:`hello-world-sample` を既に読まれた方を対象とし、追加の解説を行うものとなっています。
従って、 :doc:`hello-world-sample` をまだ読んでない方は、まずそちらを読んでいただきますようお願いします。

.. contents:: 目次
   :local:


ソースコード
------

.. highlight:: cpp

本サンプルのソースコードは以下のとおりです。 ::

 #include <cnoid/Plugin>
 #include <cnoid/ItemTreeView>
 #include <cnoid/BodyItem>
 #include <cnoid/ToolBar>
 #include <boost/bind.hpp>

 using namespace boost;
 using namespace cnoid;

 class Sample1Plugin : public Plugin
 {
 public:

     Sample1Plugin() : Plugin("Sample1")
     {
	 require("Body");
     }

     virtual bool initialize()
     {
	 ToolBar* bar = new ToolBar("Sample1");
	 bar->addButton("Increment")
	     ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, +0.04));
	 bar->addButton("Decrement")
	     ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, -0.04));
	 addToolBar(bar);

	 return true;
     }

     void onButtonClicked(double dq)
     {
	 ItemList<BodyItem> bodyItems = 
	     ItemTreeView::mainInstance()->selectedItems<BodyItem>();

	 for(size_t i=0; i < bodyItems.size(); ++i){
	     BodyPtr body = bodyItems[i]->body();
	     for(int j=0; j < body->numJoints(); ++j){
		 body->joint(j)->q += dq;
	     }
	     bodyItems[i]->notifyKinematicStateChange(true);
	 }
     }
 };

 CNOID_IMPLEMENT_PLUGIN_ENTRY(Sample1Plugin)


本サンプルはソースパッケージの "share/sampleplugins/Sample1Plugin" 以下に格納されています。（なお、説明の都合やバージョンの違いなどにより、このソースはパッケージに収録されているものとは多少異なる場合がありますが、ご了承ください。）


プラグインの概要
--------

まず、本プラグインの動作の概要について説明します。

本プラグインをコンパイル・インストールしてChoreonoidを実行すると、下図に示す２つのボタンを備えたツールバーが追加されます。

.. figure:: sample1-bar.png


これらのボタンを押すことで、ロボットモデルのポーズが変わります。

まずは、適当なロボットモデルをChoreonoid上に読み込んで表示させてください。例えば、「スタートアップガイド」で紹介したサンプルプロジェクト"GR001Sample.cnoid"を読みこめば、GR001のロボットモデルが表示されます。メインメニューの「ファイル」-「読み込み」-「ボディ」を選択して、GR001のモデルファイル"GR001.yaml"のみを読み込んでもらってもOKです。（この場合、読み込んだ後にアイテムビュー上でチェックを入れて、シーンビュー上でロボットが表示されるようにしておいてください。）

シーンビュー上でのロボットの表示を確認したら、アイテムビュー上でロボットのアイテムを選択状態にしておきます。複数のロボットモデルが読み込まれている場合でも、この選択状態によってポーズ変更の対象となるモデルを指定できます。複数のモデルを同時に動かしたい場合は、"Ctrl"キーを押しながらアイテムをクリックするなどして、複数選択状態にしておけばOKです。逆にモデルが何も選択されていないときは、ロボットのポーズは変わりませんので、ご注意ください。

では、"Increment"と書かれたボタンを推してみましょう。するとロボットのポーズが少し変わるかと思います。続けて"Increment"ボタンを押していくと、同様の変化が起きて、ロボットのポーズがだんだんと変わっていくかと思います。次に、"Decrement"ボタンを押してください。するとロボットのポーズが元に戻る方向に変わっていきます。これも何回も押していくと、いったん最初のポーズに戻り、その後もポーズの変化が続いていくかと思います。

ポーズの変化は、"Increment"ボタンの時は、ロボットの全ての関節に対して関節角度を一定角度増やしていく変化を起こしており、"Decrement"ボタンについてはこの逆になります。

このようなあまり意味のなりプラグインではありますが、このプラグインの実装をみることで、ツールバーを追加したり、選択されているアイテムを取得したり、ロボットモデルを動かしたりする際の基本を学ぶことができるかと思います。


依存プラグインの通知
----------

本プラグインではロボットモデルを扱っています。この場合、プラグインクラスのコンストラクタにて、 ::

 require("Body");

という記述をしておく必要があります。

これは、このプラグインが、Choreonoid本体添付のプラグインである"BodyPlugin"に依存していることをシステムに伝えるための記述です。実は、ロボットモデルに関する機能は、Choreonoid上で動作するひとつのプラグインとして実装されています。そのように、Choreonoid本体のパッケージに含まれる標準機能でありながらも実際にはプラグインとして実装されているものとして、以下があります。

* BodyPlugin: ロボットモデル(Bodyアイテム）を中心として、これを扱う基本的な機能をまとめたプラグイン
* PoseSeqPlugin: キーポーズのデータ構造や編集機能をまとめたプラグイン
* BalancerPlugin: バランス自動補正機能を提供するプラグイン
* GRobotPlugin: 小型ヒューマノイドロボットGR001を動かすためのプラグイン

今回はBodyPluginの機能が必要となりますので、上記のようにrequire関数を呼んでいます。requireに与える名前については、各プラグインのコンストラクタで基底クラスPluginのコンストラクタに与えている名前であり、一般的にはプラグインのクラス名から最後の"Plugin"の部分を省いた名前となっています。

ちなみに、上に挙げたプラグインについても、依存関係があり、以下のようになっています。

* BodyPlugin
 * PoseSeqPlugin: BodyPluginに依存
  * BalancerPlugin: BodyPlugin、PoseSeqPuginに依存
 * GRobotPlugin: BodyPluginに依存

ここで、BalancerPluginについては、BodyPluginとPoseSeqPluginの両方に依存していますが、PoseSeqPluginがもともとBodyPluginに依存していますので、このようなときにはrequireするのはPoseSeqPluginだけでOKとなります。

上に挙げたのはChoreonoid本体が備えるプラグインでしたが、ユーザが新たに開発したプラグインについても、もちろんそのプラグインに依存した別のプラグインを開発することが可能です。即ち、本体添付であれユーザ開発であれプラグインの扱いに差はありません。


ツールバーの作成
--------

本プラグインでは２つのボタンを備えた独自のツールバーを作成しています。

ツールバーに対応するクラスはToolBarクラスとなっていますので、まずそのヘッダをインクルードしておきます。 ::

 #include <cnoid/ToolBar>

そして、ツールバーのインスタンスを生成します。 ::

 ToolBar* bar = new ToolBar("Sample1");

ToolBarのコンストラクタに与えているのはこのツールバーの名前で、これはプロジェクトファイルに状態を保存するときなどに識別名として使われます。

ToolBarはボタンを追加する関数 "addButton" を備えていますので、 ::

 bar->addButton("Increment")

とすることで、"Increment"というキャプションのついたボタンを生成しています。


クリック時に呼ばれる関数の結びつけ
-----------------

"addButton"は追加したボタンをToolButtonクラスのオブジェクトポインタとして返します。
これに対して、さらに以下の記述を行って、ボタンがクリックされたときに呼ばれる関数の設定をしています。  ::

 bar->addButton("Increment")
     ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, +0.04));

"sigClicked"はToolButtonが備えているシグナルのひとつで、ボタンがクリックされたときに、接続されている関数を呼ぶというものです。本サンプルでは、このシグナルに"onButtonClicked"という関数を結びつけ、ボタンが押されたときの処理をこの関数内に記述しています。
connectによる関数の結びつけは、HelloWorldサンプルでも解説しましたが、ここではもう少し複雑なことをしていますので、それについて解説します。

まず、 ::

 bind(&Sample1Plugin::onButtonClicked, this, +0.04)

の部分ですが、メンバ関数を呼ぶ際のインスタンスを指定している"this"の後に追加して、"+0.04"という値を与えています。これにより、bindが返す関数オブジェクトは、メンバ関数 ::

 void Sample1Plugin::onButtonClicked(double dq)

について、インスタンスを"this"とし、引数"dq"を"+0.04"として呼び出す関数になります。すなわち"this->onButtonClicked(+0.04)"という関数呼び出しです。これで元のメンバ関数に対して引数の値が全て決まりましたので、この関数オブジェクトは ::

 void function(void)

と同型であるとみなせます。

一方で、ToolButtonクラスが定義されている"src/Base/Button.h"を見ると、"sigClicked" を取得する関数は ::

 SignalProxy< boost::signal<void(bool)> > sigClicked()

と定義されており、"sigClicked"と結びつける関数の型は、 ::

 void function(bool)

という型であることが分かります。
bool型の引数は、ボタンがトグルボタンであるときに、トグル状態のON/OFFを知らせるものとなっています。しかし、今回のボタンはただ押したことが分かればいいというものですので、この引数は不要です。引数が不要な場合は、それを無視して引数の無い関数オブジェクトとconnectすることも可能です。従って、bindによって生成した関数オブジェクトをsigClickedと結びつけることが出来、その結果、「Incrementボタンがクリックされるとthis->onButtonClicked(+0.04)を呼ぶ」という設定が実現することになりました。

少々ややこしいかもしれませんが、なぜこのようなことをしているかというと、"Increment"と"Decrement"で呼び出す関数を共有するためです。ただし、これらで挙動は変えなければなりませんので、そのための引数"dq"は用意します。そして、このようにbindを用いることで、シグナルと共有の関数を直接結びつけることが可能となり、簡潔な記述が実現しています。

後は"Decrement"ボタンについても、dqに渡すパラメータは"-0.04"に変更しつつ、以下のようにボタンの生成と関数の結びつけを同様に行なっています。 ::

 bar->addButton("Decrement")
     ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, -0.04));

これにより、Decrementボタンがクリックされるとthis->onButtonClicked(-0.04)が呼ばれることになります。


補足：シグナルに対して定義された関数の引数を利用する場合
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ToolButtonのシグナル"sigClicked"は、 ::

 void function(bool)

という関数と結びつけるよう定義されていました。今回は利用しませんでしたが、このboolの引数を利用したい場合について簡単に触れておきます。まず、結びつける関数が ::

 void onClicked(bool on)

といったように普通の関数で同じ引数を持つものであれば、 ToolButtonオブジェクトのsigClicked()に対して、 ::

 sigClicked()->connect(onClicked)

とそのまま関数を与えればOKです。これでボタンがクリックされた際には、引数onにトグル状態が与えられて関数 onClicked が呼ばれることになります。

一方で、同様の関数であっても、メンバ関数として定義されている場合は、やはりbindの助けが必要です。メンバ関数が ::

 void Sample1Plugin::onButtonClicked(bool on)

と定義されているとすれば、 ::

 sigClicked()->connect(bind(&Sample1Plugin::onButtonClicked, this, _1))

と記述する必要があります。ここでbindの最後に与えている"_1"は、「元の関数の一番目の引数を持ってくる」ことを表す、Bindライブラリのオブジェクトです。このような記述もChoreonoidのプラグイン開発ではよく使われるものですので、マスターしておくことが望ましいです。といっても、使いたい引数のところに"_1"や"_2"といった記号を入れていくだけですので、慣れれば難しくはありません。


ツールバーの登録
--------

ツールバーの作成が完了したら、ツールバーのインスタンス"bar"に対して、 ::

 addToolBar(bar);

としています。
addToolBarはPluginクラスのメンバ関数(正確には基底クラスExtensionManagerのメンバ関数）で、
ツールバー作成後はこの関数でツールバーを登録しておくことが必要です。

なお、本サンプルでは素のToolBarクラスのインスタンスをまず生成して、それに対してaddButtonで外部からツールバーを構築していきました。
簡単なツールバーの場合はこれでも良いのですが、ツールバーの内容が複雑になって来る場合は、ToolBarクラスを継承したクラスを新たに定義して、そのクラスの内部でツールバーの中身を実装していくというのが、一般的なやり方になるかと思います。


ボタンが押されたときの処理の記述
----------------

ボタンが押されたときの処理は、 メンバ関数 ::

 void onButtonClicked(double dq)

内に記述しています。引数 dq は関節角度の変化量で、ボタンのシグナルsigClickedとの接続時に設定したものです。

以下では、この関数内の処理について説明します。

選択しているBodyItemの取得
-----------------

まず、 ::

 ItemList<BodyItem> bodyItems =
     ItemTreeView::mainInstance()->selectedItems<BodyItem>();

として、アイテムツリービューにおいてユーザが選択状態としているBodyアイテムを取得しています。

これを行うため、まずItemTreeView::mainInstance()でアイテムツリービューのインスタンスを取得しています。
これはHelloWorldサンプルで説明したMessageViewの取得と同様です。

そして、ItemTreeViewのメンバ関数"selectedItems"を呼ぶことで、選択しているアイテムのリスト（配列）を得ることができます。
この関数はアイテムの型がパラメータとなっているテンプレート関数で、選択されている全てのアイテムの中から、指定された型に適合するもの返すようになっています。ここでは "<BodyItem>" としてBodyItem型を指定することで、Bodyアイテムのみを取得対象としています。

アイテムのリストは、ItemListというテンプレートクラスで返されるようになっています。これも同様にアイテムの型をテンプレートパラメータとしてとるようになっており、その型のアイテムを格納する配列となっています。これに対してもBodyItem型を指定することで、選択されたBodyItemを格納した配列を取得しています。

ItemTreeViewクラスには、他にも「チェックされているアイテムのリストを返す」関数である"checkedItems"や、「あるアイテムが選択されているかどうかを調べる」"isItemSelected"、「ユーザがアイテムの選択状態を変えたときに発行される」シグナル"sigSelectionChanged"といったものが定義されているので、これらを用いることで処理対象となるアイテムの取得を柔軟に行うことが可能です。


対象とするBodyアイテムが取得できましたので、次にそれぞれのBodyItemに対して個別に処理を行っていきます。
ItemListクラスは std::vector をベースとしていますので、std::vector と同様の記述が可能となっています。
これを用いて、 ::

 for(size_t i=0; i < bodyItems.size(); ++i) {

として、各BodyItemに対する処理を行うループを記述しています。


Bodyオブジェクトの取得
-------------

各BodyItemに対する処理を行うループ内では、まず ::

 BodyPtr body = bodyItems[i]->body();

として、"Body"オブジェクトへのポインタを取得しています。
BodyPtrはBodyオブジェクトのスマートポインタで、詳しくは後ほど説明しますが、とりあえずは"Body*"のようなものだと思ってください。

BodyItems[i]でBodyItemのポインタが取得できますが、BodyItem自体はモデルの実際のデータ構造や処理関数を直接定義しているものではなく、それらは実際にはBodyライブラリ(src/Body以下)の"Body"クラスにて定義されています。BodyItemはこのクラスをラップして、Choreonoidのアイテムとして使えるように追加の記述をしたものとなっています。BodyItemが保有しているBodyオブジェクトは、このように"body"関数を呼ぶことで取得できます。

なぜこのようになっているかというと、モデルのデータ構造や処理関数自体はGUIとは切り離して、様々なプログラムにおいて汎用的に使えるようにしておくことが望ましいからです。このため、そのようなGUIとは独立した部分はまずsrc/Body以下の"Bodyライブラリ"として定義されています。一方で"src/BodyPlugin"以下の"Bodyプラグイン"においては、それらのクラスのアイテム化や、ツールバー、ビューといった、GUIの部分をカバーしており、両者で役割を分けた設計としています。Choreonoidにおいては、このように「GUIとは独立したクラス」がまずあって、それを「アイテムとしてラップ」して使うというのが、一般的なやり方となっています。もちろん、GUIからは独立させる必要が特になければ、各アイテムで直接全ての実装を行ってしまっても一向にかまいません。


補足： Choreonoid本体のモジュール構造
^^^^^^^^^^^^^^^^^^^^^^^^

BodyライブラリとBodyプラグインの分離について述べましたが、Choreonoid本体には他にもこのような部分があり,
Choreonoid本体の基本部分のモジュールとしては以下のようなものがあります。

* GUIからは独立して定義されているモジュール:

 * Utilライブラリ (src/Util) : 様々な部分から使われるクラスや関数を定義

 * Collisionライブラリ (src/Collision): ポリゴン(三角形）モデル間の干渉検出処理を定義

 * Bodyライブラリ (src/Body) : 物体／関節物体のモデル化と、それらの運動学・動力学関連処理を定義

 これらのモジュールは、Choreonoidのプラグインではない外部プログラムから利用することも可能です。

* GUI のモジュール

 * Baseモジュール (src/Base) : ChoreonoidのGUIのベースとなる部分を定義

 * Bodyプラグイン (src/BodyPlugin): Bodyライブラリと関連するモデル関連処理のGUIを定義

 * その他プラグイン

これらのモジュール間の依存関係は、以下の図のようになっています。

.. figure:: module-dependencies.png


補足： スマートポインタについて
^^^^^^^^^^^^^^^^

上のコードで出てきた"BodyPtr"というのは、Bodyクラスのポインタを格納する「スマートポインタ」です。
スマートポインタは、簡単に言うと「deleteしなくて良いポインタ」で、いつdeleteすればよいかを気にしたり、間違ってdeleteしてしまったポインタを使おうとしてクラッシュしてしまったりということが避けられるというものです。

Choreonoidで使っているスマートポインタの実装は Boostの"Smart Pointers"ライブラリが提供しているものです。
このライブラリは使用形態に応じて使い分けられるよういくつかのスマートポインタ型を提供しています。
その中でも基本となるのは"shared_ptr"という型です。これは例えばHogeというクラスがあったときに、 ::

 boost::shared_ptr<Hoge> hoge = new Hoge();

みたいなかたちで使います。こうしておけば、あとは素のポインタ型"Hoge*"と同様に、 ::

 hoge->function();

みたいなかたちでメンバ関数や変数にアクセスできますし、 ::

 boost::shared_ptr<Hoge> hoge2 = hoge;

などとして、別の変数にコピーすることも可能です。

そして、このポインタを格納している全てのスマートポインタ変数がデストラクトされた時点で、ポインタ自体も自動的にdeleteされることになります。

また、素のポインタが必要な場合は、 ::

 Hoge* p = hoge.get();

のように、get()関数を使うことで変換できます。

ところで、"boost::shared_ptr<Hoge>"という型の記述はこのままでは長くて使いにくいところがあります。
そこでChoreonoidでは、 ::

 typedef boost::shared_ptr<Hoge> HogePtr;

などと定義して、"クラス名+Ptr"という命名規則でスマートポインタを使えるようにしています。

以上がshared_ptrの基本になりますが、実はChoreonoidで中心的に使っているスマートポインタの型は、"intrusive_ptr"というものになります。Bodyクラスについても、BodyPtrはこれをベースとしたスマートポインタとして定義されています。
使い方はshared_ptrと概ね同じなのですが、intrusive_ptrはdeleteするかどうかの判定のための「参照カウンタ」を、オブジェクト内部に持っている点がshared_ptrと異なります。(shared_ptrはこの領域を別途ヒープから確保しています。）これにより、intrusive_ptrはshared_ptrと比べて、

* 僅かに処理が高速

* 素のポインタ型と相互に変換を行っても問題が起きにくい

といった利点があるため、Choreonoidではこちらの型を主に使用しています。

intrusive_ptrをベースとしたスマートポインタを提供するクラスは、 ::

 class Body : public cnoid::Referenced

といったように、Utilライブラリで定義されている"cnoid::Referenced"というクラスを継承して定義します。
そして、 ::

 typedef boost::intrusive_ptr<Body> BodyPtr;

と定義しておけば、BodyPtrという型でこのスマートポインタを使えるようになります。

このスマートポインタは、上にも書いたように「素のポインタ型と相互に変換を行っても問題が起きにくい」ので、必要な箇所だけスマートポイントとして格納するということが可能です。

実際、cnoid::Reference型のオブジェクトを返す関数は基本的に ::

 Body* functionToReturnBodyObject();

といったように、素のポインタ型を返すよう定義されています。
このような関数が返すオブジェクトを受け取る変数としては、"BodyPtr" と "Body*" のどちらも使うことができます。
(実際、本サンプルのコードについても "Body*" を使って問題ありません。）

一方、cnoid::Referenced型のオブジェクトを引数としてとる関数は、基本的に ::

 void doSomething(BodyPtr body);

のように、スマートポインタ型で記述されています。これについても、関数を呼ぶ際に与える変数は "BodyPtr" と "Body*" のどちらでもかまいません。

ただし、素のポインタを使ってよい状況というのは、どこかでそのオブジェクトがスマートポインタに格納されていて、かつ素のポインタでの使用が一時的なものに留まる場合に限ります。逆に、オブジェクトを長期間保有する必要がある場合は、スマートポインタに格納するようにします。よく分からない場合は、スマートポインタを使うようにすればOKです。（とは言え、一部例外もあります。）

以上のようなスマートポインタと素のポインタの相互変換は、shared_ptrの場合は基本的には出来ません。
shared_ptr を素のポインタで初期化することは出来ますが、これに素のポインタを代入することは出来ず、初期化についてもポインタがnewされた直後であることを前提としています。
一方で、intrusive_ptrの場合は、上記の制限を守る限りは、素のポインタによる初期化や代入も自由に行うことが出来ます。

Choreonoidでは、このようにポインタの記述に柔軟性を持たせることができることを好んで、intrusive_ptr をベースとした設計を行っています。ただしこれによって上記のような「素のポインタを使う際の条件」も気にする必要が出てくるので、「スマートポインタさえ使っていればオブジェクトの生存について気にしなくて良い」というスマートポインタの利点をある意味失っていると言えるかもしれません。ただし、通常はそこまで気にする必要はありませんので、まずは上記の記述法に慣れていただければと思います。



関節角の変更
------

サンプルのコードに戻りましょう。以下のコードで、Bodyオブジェクトが格納しているロボットモデルの関節角度を変更しています。 ::

 for(int j=0; j < body->numJoints(); ++j){
     body->joint(j)->q += dq;
 }

Bodyクラスは "numJoints" という関数で関節の数を知ることが出来ますので、これを使って全関節の角度を変えるようループをまわしています。ループ内で "joint(j)" という関数で取得しているのは、jという関節idに対応するLinkクラスのオブジェクトです。このクラスは"q"というメンバ変数に関節角度を格納しており、ここではこの値をdq分だけ変化させています。

なお、Bodyライブラリは `OpenHRP3 <http://fkanehiro.github.io/openhrp3-doc/jp/index.html>`_ で開発したものをフォークしてChoreonoid版の開発を開始しており、ここで使用しているBodyクラスやLinkクラスも今のところ使い方はOpenHRP3とほぼ同様になっています。従って、OpenHRP3のライブラリを用いたプログラミングを行ったことのある方はその知識が活かせますし、OpenHRP3の `プログラミングマニュアル <http://fkanehiro.github.io/openhrp3-doc/jp/programming.html>`_ もある程度は参考にすることが可能です。ただし、変更している箇所も多く、特に行列・ベクトルライブラリが tvmet から Eigen に変更になっているのは大きな変更ですので、それらの点にはご注意ください。


状態変更の通知
-------

上のコードで行ったことは関節角度を格納している変数の更新のみであり、これだけではその結果をリンクの位置姿勢なども含むモデル全体に反映し、さらにGUI上の表示を更新するには不十分です。これを行うため、 ::

 bodyItems[i]->notifyKinematicStateChange(true);

を実行しています。

"notifyKinematicStateChange"はBodyItemクラスで定義された関数で、モデルに対して運動学的な変更がなされたことをChoreonoidのシステムに伝え、GUI上での表示にも反映させるための関数です。Bodyクラスではなく、BodyItemクラスで定義された関数であることにご注意ください。このように、GUIとの関連部分を追加で実装するのが、BodyItemクラスの役割と言うわけです。

notifyKinematicStateChange関数は以下のように宣言されており、 ::

 void notifyKinematicStateChange(bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);

ここでは第一引数である"requestFK"にtrueを与えています。

先ほど述べたように、関節角度の変数qの値を変更しただけでは、各リンクの位置姿勢まで変わるわけではありません。
このためには順運動学計算を行う必要があります。
これはBodyオブジェクトに対して ::

 body->calcForwardKinematics();

を行えばよいのですが、requestFKにtrueを与えると、これを同時に行なってくれます。
また、速度や加速度の値も更新したい場合には、それぞれtrueを追加して与えてください。
requestFKにtrueを与えずにnotifyKinematicStateを実行した場合どうなるかは、余裕があれば試してみてください。
その場合、「関節スライダビュー」等に表示される関節角度は変わるものの、「シーンビュー」上に表示されるロボットのポーズは変わらない、という結果を確認できるかと思います。

.. note:: notifyKinematicStateChange が calcForwardKinematics も行なうことの理由は、単にcalcForwardKinematicsを実行しなくて済むためというわけではありません。Choreonoidは複数のオブジェクトが連携して機能するということを意識して設計しており、notifyKinematicStateChangeもある程度このことを考慮して提供されています。例えば、ロボットのモデルに対して、上半身の姿勢を扱うオブジェクトと、下半身の姿勢を扱うオブジェクトが、それぞれ独立して存在・機能することも考えられます。そして、両者が同じタイミングで機能することもあるでしょう。この場合、もしそれぞれが個別に関節角を変更して運動学計算を行い、GUIの更新をしてしまうと、結果として処理の重複が生じてしまいます。そうなるよりは、それぞれ関節角だけを更新して、両者の処理が終わった時点でまとめて運動学計算とGUIの更新を行うほうが、より効率的です。このため、notifyKinematicStateChangeはそれが呼ばれた時点で運動学計算とGUI更新を行うのではなく、それらが必要であることをイベントとしてポストし、同じタイミングで行われる全ての更新が行われた後で、まとめて１回だけ運動学計算とGUI更新を行うように記述されています。

なお、notifyKinematicStateChange が呼ばれると、最終的にBodyItemクラスが備える"sigKinematicStateChanged" というシグナルが発行されます。従って、モデルの運動学的状態が変化した際に行いたい処理がある場合は、このシグナルと接続した関数内に記述すればOkです。実際Choreonoidが備える関節スライダビューやシーンビューにおけるBodyItemの表示機能は、このシグナルと接続することで実現されています。これにより、notifyKinematicStateChangeを呼ぶだけで、関連する全ての表示が更新されることになるのです。

これは呼ぶ側がどのような表示があるかを把握している必要は全く無く、プラグインによって新たな表示機能が追加される場合でも、それは単に追加するだけで既存の表示機能と同様に機能します。このようなかたちで、Choreonoidでは柔軟性のある機能拡張の実現を目指しています。と言ってもこれは特別なものではなく、Model-View-Controller(MVC)アーキテクチャ、あるいはDocument-Viewアーキテクチャなどと呼ばれているもので、よく知られたソフトウェア設計技法のひとつです。


コンパイル
-----

プラグインのコンパイルの仕方についてはHelloWorldサンプルで解説しましたので、ここでは本サンプルで追加して考慮しなければならない点のみを解説します。

本サンプルで追加して考慮しなければならない点は、上の「コンストラクタ」の節で述べたように、このプラグインがBodyPluginに依存しているという点です。そこで、リンクするライブラリファイルについて、追加の記述が必要になります。

これについては、LinuxであればChoreonoidインストール先の"lib/choreonoid-x.x"以下に格納される"libCnoidXXXPlugin.so"というプラグインの共有ライブラリとリンク出来ていればOKです。BodyPluginの場合は"libCnoidBodyPlugin.so"になります。

また、Windowsでは同じディレクトリに格納される"CnoidXXXPlugin.lib"という「インポートライブラリファイル」になります。このファイルに対してリンクを行うと、実行時に同じディレクトリに格納されている"CnoidXXXPlugin.dll"というDLLファイルとリンクされることになります。ただし、インポートライブラリファイル(.lib)については、CMakeの設定で"INSTALL_SDK"というオプションをONにしておかないとインストールされませんので、ご注意ください。

インストール済みのChoreonoidを対象としたMakefileを書く場合は、pkg-config に対して "choreonoid-body-plugin" というライブラリ名を与えると、上記を満たしたオプションを返してくれます。

また、Choreonoid本体のコンパイル環境でCMakeを使ったコンパイルを行う場合には、"target_link_libraries"コマンドに"CnoidBodyPlugin"を記述することで、この依存関係を満たすようにコンパイルしてくれます。（この場合、CnoidBodyPluginがCnoidBaseに依存していますので、CnoidBaseについては明示的に記述する必要はありません。）

実際のMakefileやCMakeLists.txtについては、それぞれ "share/sampleplugins/Sample1Plugin" 以下のものと "extplugin/sample/Sample1Plugin" 以下のものを参考にしてください。

