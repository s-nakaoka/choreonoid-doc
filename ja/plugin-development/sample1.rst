Sample1Pluginの解説
===================

このドキュメントでは、サンプルプラグインのひとつである"Sample1Plugin"の実装について解説します。本ドキュメントは、 :doc:`hello-world-sample` を既に読まれた方を対象とし、追加の解説を行うものとなっています。

.. contents:: 目次
   :local:


ソースコード
------------

.. highlight:: cpp

本サンプルのソースコードは以下のとおりです。 ::

 #include <cnoid/Plugin>
 #include <cnoid/ToolBar>
 #include <cnoid/ItemTreeView>
 #include <cnoid/BodyItem>
 
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
         auto bar = new ToolBar("Sample1");
         auto button1 = bar->addButton("Increment");
         button1->sigClicked().connect([&](){ onButtonClicked(0.04); });
         auto button2 = bar->addButton("Decrement");
         button2->sigClicked().connect([&](){ onButtonClicked(-0.04); });
         bar->setVisibleByDefault(true);
         addToolBar(bar);
         return true;
     }
 
 private:
 
     void onButtonClicked(double dq)
     {
         auto bodyItems = ItemTreeView::instance()->selectedItems<BodyItem>();
         for(auto& bodyItem : bodyItems){
             auto body = bodyItem->body();
             for(auto& joint : body->joints()){
                 joint->q() += dq;
             }
             bodyItem->notifyKinematicStateChange(true);
         }
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(Sample1Plugin)

このソースコードを含む本サンプルのファイル一式がChoreonoidソースアーカイブの "sample/tutorial/Sample1Plugin" 以下に格納されています。ソースコードのファイル名は "Sample1Plugin.cpp" となります。

プラグインの概要
----------------

まず、本プラグインの動作の概要について説明します。

本プラグインをビルドしてChoreonoidを実行すると、下図に示す2つのボタンを備えたツールバーが追加されます。

.. figure:: sample1-bar.png

これらのボタンを押すことで、ロボットモデルのポーズが変わります。

まずは、適当なロボットモデルをChoreonoid上に読み込んでシーンビュー上に表示させてください。とりあえず適当な :ref:`basics_sample_project` を読み込むのでもよいかと思います。

ロボットモデルの表示を確認したら、アイテムビュー上でロボットのアイテムを選択状態にしておきます。複数のロボットモデルが読み込まれている場合でも、この選択状態によってポーズ変更の対象となるモデルを指定できます。複数のモデルを同時に選択してもOKです。モデルが何も選択されていないときは、ロボットのポーズは変わりませんので、ご注意ください。

では、"Increment"と書かれたボタンを押してみましょう。するとロボットのポーズが少し変わるかと思います。続けて"Increment"ボタンを押していくと、同様の変化が起きて、ロボットのポーズがだんだんと変わっていくかと思います。次に、"Decrement"ボタンを押してください。するとロボットのポーズが元に戻る方向に変わっていきます。これも何回も押していくと、いったん最初のポーズに戻り、その後もポーズの変化が続いていくかと思います。

ポーズの変化は、"Increment"ボタンの時は、ロボットの全ての関節に対して関節角度を一定角度増やしていく変化を起こしており、"Decrement"ボタンについてはこの逆になります。この挙動は :ref:`pose_editing_joint_slider_view` を表示しておくと、より分かりやすくなるかと思います。

このようにポーズを変えること自体にはあまり意味は無いかもしれませんが、このプラグインの実装をみることで、ツールバーを追加したり、選択されているアイテムを取得したり、ロボットモデルを動かしたりする方法が分かります。これらはロボットモデルに対して何か操作をするにあたって、基本となる部分かと思います。

依存プラグインの明示
--------------------

本プラグインではロボットモデルを扱っています。この場合、プラグインクラスのコンストラクタにて、 ::

 require("Body");

という記述をしておく必要があります。

これは、このプラグインが、Choreonoid本体に含まれる「Bodyプラグイン」に依存していることをシステムに伝えるための記述です。Bodyプラグインはロボットに関する基本的な機能を実装しているプラグインです。実はChoreonoidでは、そのように基本的な機能であっても、プラグインとして実装されているものが多いです。ロボットに関わるプラグインとしては他に以下のようなものがあります。

* PoseSeqプラグイン: キーフレームによる動作作成機能を実装
* Balancerプラグイン: 二足歩行ロボットの動作をバランスのとれたものに補正する機能を実装

今回はロボットモデルのポーズを変えるということで、Bodyプラグインの機能が必要となりますので、上記のようにrequire関数でこれを明示しています。requireに与える名前については、プラグイン名の本体部分（最後の「プラグイン」を除いたもの）となります。

ちなみに、上記の3つのプラグインにも依存関係があります。これをツリー形式で書くと以下のようになります。

* Bodyプラグイン
 * PoseSeqプラグイン
  * Balancerプラグイン

PoseSeqプラグインはBodyプラグインに依存し、BalancerプラグインはPoseSeqプラグインとBodyプラグインに依存しています。このような依存関係にあるとき、requireで指定しなければならないのは、直近の依存プラグイン（ここではPoseSeqプラグイン）のみとなります。

ツールバーの作成
----------------

本プラグインでは２つのボタンを備えた独自のツールバーを作成しています。

ツールバーに対応するクラスはToolBarクラスとなっていますので、まずそのヘッダをインクルードしておきます。 ::

 #include <cnoid/ToolBar>

そして、ツールバーのインスタンスを生成します。 ::

 auto bar = new ToolBar("Sample1");

ToolBarのコンストラクタに与えているのはこのツールバーの名前で、これはプロジェクトファイルに状態を保存するときなどに識別名として使われます。

ToolBarはボタンを生成・追加する関数 "addButton" を備えており、 ::

 auto button1 = bar->addButton("Increment");

とすることで、"Increment" というキャプションのついたボタンを生成しています。この関数は追加したボタンをToolButtonクラスのオブジェクトポインタとして返します。ここではbutton1という変数にそれを格納しています。

クリック時に呼ばれる関数の結びつけ
----------------------------------

追加したボタンに対して以下の記述を行い、ボタンがクリックされたときに呼ばれる関数の設定をしています。  ::

 button1->sigClicked().connect([&](){ onButtonClicked(0.04); });

"sigClicked" はToolButtonが備えているシグナルのひとつで、ボタンがクリックされたことを知らせるためのシグナルです。これはToolButtonクラス（src/Base/Buttons.hにて定義）で ::

 SignalProxy<void()> sigClicked()

と定義されており、引数なしのシグナルであることが分かります。

今回これに関連付けたいのは、 ::

 void Sample1Plugin::onButtonClicked(double dq)

という関数です。ここでdqという引数に増減させる関節角度を与えるようになっています。これに対して、ラムダ式を ::

 [&](){ onButtonClicked(0.04); }

とすることで、dqに0.04という値を与えながらも、シグナルからは引数なしの関数として見えるようにしています。これによって、「Incrementボタンが押されると、onButtonClicked関数を引数0.04で呼び出す」という処理を記述できました。

あとはDecrementボタンに対しても、 ::

 auto button2 = bar->addButton("Decrement");
 button2->sigClicked().connect([&](){ onButtonClicked(-0.04); });

とすることで、ボタンの追加と関数の関連付けを行っています。Incrementボタンとは異なり、onButtonClickedに -0.04 という負の値を与えていることに注意してください。

onButtonClickedで増減値の引数をとるようにし、それをラムダ関数内で特定することで、2つのボタンの挙動をひとつの関数に実装することができています。

補足：シグナルが持つ引数を利用する場合
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ToolButtonのシグナル"sigClicked"は引数無しのシグナルでしたが、ToolButtonは他に ::

 SignalProxy<void(bool)> sigToggled()

というシグナルも持っています。これはボタンがトグルボタンであるときに、そのトグル状態の変化をbool値で教えてくれるシグナルとなっています。これは基本的には ::

 void function(bool)

という形式の関数と結びつけることが想定されています。

本サンプルでは利用しませんでしたが、このboolの引数を利用する場合は、ラムダ式の引数を利用します。例えば、 ::

 void onButtonToggled(bool on)

という関数が定義されていていて、この引数onにトグル状態を渡したいものとします。その場合は、 ::

 button->sigToggled().connect([&](bool on){ onButtonToggled(on); });

といった記述を行えばOKです。

ツールバーの登録
----------------

作成したツールバーに対して、 ::

 addToolBar(bar);

とすることで、ツールバーをChoreonoidのシステムに登録しています。

addToolBarはPluginクラスのメンバ関数（正確には基底クラスExtensionManagerのメンバ関数）で、ツールバー作成後はこの関数でツールバーを登録しておくことが必要です。

なお、 ::

 bar->setVisibleByDefault(true);

とすることで、ツールバーをデフォルトで表示するようにしています。

各ツールバーを表示するかどうかはユーザが設定できるのですが、新しく追加したツールバーについてはデフォルトでは表示されないようになっています。しかしサンプルではユーザの設定なしにすぐに試せるようにしたいので、この記述も入れています。

.. note:: 本サンプルでは素のToolBarクラスのインスタンスをまず生成して、それに対してaddButtonで外部からツールバーを構築していきました。簡単なツールバーの場合はこれでも良いのですが、ツールバーの内容が複雑になって来る場合は、ToolBarクラスを継承したクラスを新たに定義して、そのクラスの内部でツールバーの中身を実装していくというのが、一般的なやり方になるかと思います。

ボタンが押されたときの処理の記述
--------------------------------

ボタンが押されたときの処理は、 メンバ関数 ::

 void onButtonClicked(double dq)

内に記述しています。引数 dq は関節角度の変化量で、ボタンのシグナルsigClickedとの接続時に設定したものです。

以下では、この関数内の処理について説明します。

選択しているBodyアイテムの取得
------------------------------

まず、 ::

 auto bodyItems = ItemTreeView::instance()->selectedItems<BodyItem>();

として、アイテムツリービューにおいてユーザが選択状態としているBodyアイテムを取得しています。

ItemTreeView::instance()によってアイテムツリービューのインスタンスを取得しています。これはHelloWorldサンプルで説明したMessageViewの取得と同様です。

そして、ItemTreeViewのメンバ関数 "selectedItems" によって、選択しているアイテムのリスト（配列）を得ることができます。この関数はアイテムの型を引数にとるテンプレート関数で、選択されている全てのアイテムの中から、指定された型にマッチするもだけを返すようになっています。ここではBodyItem型を指定することで、Bodyアイテムのみを取得対象としています。

アイテムのリストは、ItemListというテンプレートクラスで返されるようになっています。こちらもアイテムの型がテンプレート引数になっており、その型のアイテムを格納する配列となります。selectedItems関数は自身のテンプレート引数と同じ型のItemListを返すようになっていますので、ここでは ::

 ItemList<BodyItem>

という型で結果が返ってきます。これをbodyItemsという変数に入れています。

.. note:: ItemTreeViewクラスには、他にも「チェックされているアイテムのリストを返す」"checkedItems"や、「あるアイテムが選択されているかどうかを調べる」"isItemSelected"、「アイテムの選択状態が変わったことを知らせるシグナル」"sigSelectionChanged"といったものが定義されており、それらを用いることで処理対象となるアイテムの取得を柔軟に行うことが可能です。

Bodyアイテムのリストが取得できたら、次にそこに含まれる各Bodyアイテムについて個別に処理を行っていきます。ItemListクラスは std::vectorをベースとしていますので、std::vectorと同様に扱うことができます。ここではC++11のRange-basedなfor文を使用して、 ::

 for(auto& bodyItem : bodyItems){
     ...
 }

とすることで、各BodyItemに対して処理を行うループを記述しています。

Bodyオブジェクトの取得
----------------------

各BodyItemに対する処理を行うループ内では、まず ::

 auto body = bodyItem->body();

として、BodyアイテムからBodyクラスのオブジェクトを取得しています。

Bodyクラスはロボットモデルのデータ構造や処理関数を実装しているクラスで、このオブジェクトがモデルの本体と言えるものです。このクラスはChoreonoidの中では「Bodyライブラリ」と呼ばれる部分で定義されています。（Choreonoidのソース中ではsrc/Body以下がこれに該当します。）一方で、BodyアイテムはBodyオブジェクトをChoreonoidのGUI上でアイテムとして扱えるようにするためのラッパであり、Choreonoid内部ではBodyItemクラスとして定義されています。BodyItemは対応するBodyオブジェクトを保有しており、これを返すのがbody関数というわけです。

このように、ロボットモデルをBodyクラスとBodyItemクラスに分けて定義しているのはなぜかと言うと、モデルのデータ構造や処理関数自体はGUIとは切り離して、様々なプログラムにおいて汎用的に使えるようにしておくことが望ましいからです。実際にBodyクラスを実装しているBodyライブラリはGUIには依存しないライブラリとなっており、ロボットの制御プログラムなどでも使用することが可能です。その一方で、GUIに依存する部分はBodyプラグインに実装されており、アイテム、ツールバー、ビューといったGUIの部分はそこでカバーされています。Choreonoid上で扱うデータは、このように、「GUIとは独立したクラス」がまずあって、それを「アイテムとしてラップ」して使うということがよくあります。もちろん、そのような切り分けをせずに、アイテム自体にデータの本体を実装することも可能です。

補足： Choreonoid本体のモジュール構造
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

BodyライブラリとBodyプラグインの関係について述べましたが、Choreonoidでは他にもこのような部分がありますので、以下に概要をまとめておきます。

* GUIには依存しない部分:

 * Utilライブラリ (src/Util) : 様々な部分から使われるクラスや関数を定義

 * Collisionライブラリ (src/Collision): ポリゴン(三角形）モデル間の干渉検出処理を定義

 * Bodyライブラリ (src/Body) : 物体／関節物体のモデル化と、それらの運動学・動力学関連処理を定義

 これらのモジュールは、Choreonoidのプラグインではない外部プログラムから利用することも可能です。

* GUIに依存する部分

 * Baseモジュール (src/Base) : ChoreonoidのGUIのベースとなる部分を定義

 * Bodyプラグイン (src/BodyPlugin): Bodyライブラリと関連するモデル関連処理のGUIを定義

 * その他プラグインの全て

これらのモジュール間の依存関係は、以下の図のようになっています。

.. figure:: module-dependencies.png

関節角の変更
------------

以下のコードで、ロボットモデルの関節角度を変更しています。 ::

 for(auto& joint : body->joints()){
     joint->q() += dq;
 }

body->joints()はBodyオブジェクトが保有する関節のリストを返します。この要素に対するループをrange-based forによって回すことで、各関節に対する処理を行います。ここでの要素はBodyライブラリで定義されているLinkクラスのオブジェクトです。これはロボットのリンクに対応するもので、親リンクからの関節の情報も含んでいます。関節角度はqという名前で定義されており、これにアクセスする関数qを用いて、関節角度の値をdq分だけ変化させています。

状態変更の通知
--------------

上のコードで行ったことは関節角度を格納している変数の更新のみであり、これだけではその結果をモデル全体に反映するには不十分です。これを行うため、最後に ::

 bodyItem->notifyKinematicStateChange(true);

を実行しています。

ここで使用しているnotifyKinematicStateChange関数は、モデルに対して運動学的な変更がなされたことをChoreonoidのシステムに伝え、GUI上での表示にも反映させるための関数です。この関数は以下のように定義されています。 ::

 void notifyKinematicStateChange(bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);

ここでは第一引数である"requestFK"にtrueを与えています。

関節角度の変数qの値を全リンクの位置姿勢に反映させるためには、順運動学計算を行う必要があります。これはBodyオブジェクトに対して ::

 body->calcForwardKinematics();

を実行すればよいのですが、requestFKにtrueを与えると、notifyKinematicStateChangeの中でこれを行なってくれます。また、速度や加速度の値も更新したい場合には、それぞれ引数requestVelFK、requestAccFKにtrueを与えるようにします。

requestFKにtrueを与えずにnotifyKinematicStateを実行した場合、順運動学の処理が行われないため、関節スライダビューなどに表示される関節角度は変わるものの、シーンビューなどで表示されるロボットのポーズは変わらないという結果となります。

.. note:: notifyKinematicStateChange が calcForwardKinematics も行なうことの理由は、単にcalcForwardKinematicsを実行しなくて済むためというわけではありません。Choreonoidは複数のオブジェクトが連携して機能するということを意識して設計しており、notifyKinematicStateChangeもある程度このことを考慮して提供されています。例えば、ロボットのモデルに対して、上半身の姿勢を扱うオブジェクトと、下半身の姿勢を扱うオブジェクトが、それぞれ独立して存在・機能することも考えられます。そして、両者が同じタイミングで機能することもあるでしょう。この場合、もしそれぞれが個別に関節角を変更して運動学計算を行い、GUIの更新をしてしまうと、結果として処理の重複が生じてしまいます。そうなるよりは、それぞれ関節角だけを更新して、両者の処理が終わった時点でまとめて運動学計算とGUIの更新を行うほうが、より効率的です。このため、notifyKinematicStateChangeはそれが呼ばれた時点で運動学計算とGUI更新を行うのではなく、それらが必要であることをイベントとしてポストし、同じタイミングで行われる全ての更新が行われた後で、まとめて１回だけ運動学計算とGUI更新を行うように記述されています。

notifyKinematicStateChange関数が実行されると、最終的にBodyItemクラスが備える"sigKinematicStateChanged" というシグナルが発行されます。従って、モデルの運動学的状態が変化した際に行いたい処理がある場合は、その処理関数をこのシグナルに接続するようにします。実際、関節スライダビューやシーンビューにモデルの状態が反映されるのは、このシグナルと接続することで実現されています。これにより、notifyKinematicStateChangeを呼ぶだけで、関連する全ての表示が更新されることになるわけです。

この仕組みにおいて、モデルの状態を更新する側は、更新した結果をどこにどう反映させるかについて全く気にする必要がありません。プラグインによって新たに表示機能が追加される場合でも、それをsigKinematicStateChangedと接続するだけで、既存の表示機能と同様に機能します。これにより、機能拡張を柔軟に行うことが可能となります。これはいわゆるModel-View-Controller(MVC)、Document-View、Publisher-Subscriberといった枠組みに該当するものであり、一般的な設計技法のひとつであると言えます。

ビルド方法
----------

:doc:`hello-world-sample` では :ref:`hello-world-build` について3つ紹介しました。本サンプルでも、この3つの方法に対応するビルド用ファイルの記述について紹介します。

.. highlight:: cmake

まず、 :ref:`hello-world-build-together` 場合は、CMakeLists.txtに以下のように記述します。 ::

  set(target CnoidSample1Plugin)
  add_cnoid_plugin(${target} SHARED Sample1Plugin.cpp)
  target_link_libraries(${target} CnoidBodyPlugin)
  apply_common_setting_for_plugin(${target})

記述内容はHelloWorldサンプルの場合とほぼ同様ですが、target_link_librariesの内容が少し異なります。今回のプラグインはBodyプラグインに依存しているため、依存ライブラリとしてCnoidBaseではなく、CnoidBodyPluginを指定するようにします。プラグインである以上、今回もCnoiBaseに依存はしているのですが、それは明示的に記述する必要はありません。これは、CnoidBodyPluginもCnoidBaseに依存しており、CnoidBodyPluginへの依存によってCnoidBaseにも依存することをCMakeが把握しているからです。

次に、 :ref:`hello-world-stand-alone-build` 場合は、以下のようなCMakeLists.txtを作成します。 ::

  cmake_minimum_required(VERSION 3.1.0)
  project(Sample1Plugin)
  find_package(Choreonoid REQUIRED)
  add_definitions(${CHOREONOID_DEFINITIONS})
  include_directories(${CHOREONOID_INCLUDE_DIRS})
  link_directories(${CHOREONOID_LIBRARY_DIRS})
  set(target CnoidSample1Plugin)
  add_library(${target} SHARED Sample1Plugin.cpp)
  target_link_libraries(${target} ${CHOREONOID_BODY_PLUGIN_LIBRARIES})
  install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})

こちらについても、HelloWorldサンプルと異なるのは、Bodyプラグインへの依存があるという点です。これについては、target_link_librariesにおいて、"CHOREONOID_BODY_PLUGIN_LIBRARIES" という変数を用いることで、Bodyプラグインへの依存で必要となる全てのライブラリをセットすることができます。

最後に、 :ref:`hello-world-makefile-build` 場合は、以下のようなMakefileを作成します。

.. code-block:: makefile

 CXXFLAGS += -fPIC `pkg-config --cflags choreonoid-body-plugin`
 PLUGIN = libCnoidSample1Plugin.so
 
 $(PLUGIN): Sample1Plugin.o
 	g++ -shared  -o $(PLUGIN) Sample1Plugin.o `pkg-config --libs choreonoid-body-plugin`
 
 install: $(PLUGIN)
	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
 clean:
	rm -f *.o *.so

ここでもBodyプラグインのライブラリをリンクすることがポイントです。pkg-configにおいては、choreonoid-body-pluginというモジュール名を用いることで、Bodyプラグインを利用する場合の情報を得ることができます。

これらのビルド用ファイルのサンプルについて、ChoreonoidソースのSample1Pluginのディレクトリ（sample/tutorial/Sample1Plugin）に格納してあります。
