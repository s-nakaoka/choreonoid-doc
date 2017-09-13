
ステップ3: ゲームパッドによる砲塔の制御
=======================================

ステップ2では最小限の制御を行うコントローラを作成し、コントローラの導入方法を学びました。ステップ3ではもう少し複雑なコントローラを作成してみましょう。

.. contents:: 目次
   :local:
   :depth: 2

.. highlight:: C++
   :linenothreshold: 7

.. _simulation-tank-tutorial-gamepad:

ゲームパッドの準備
------------------

このステップで作成するコントローラは、ゲームパッド（ジョイスティック）を操作することでTankモデルの砲塔を動かすというものです。このための準備として、ゲームパッドを用意してPCに接続しておきます。

実際には、 **ゲームパッドが無くても大丈夫** です。その場合はChoreonoidの「仮想ジョイスティックビュー」をいう機能が代用できますので、次節（:ref:`tank-tutorial-virtual-joystick-view`）に進むようにして下さい。以下はゲームパッドの実機を使う場合の説明になります。

本チュートリアルが想定しているのは、以下に図に示すような構成をもったゲームパッドです。

.. image:: images/f310.jpg

これは `ロジクールのF310 <http://gaming.logicool.co.jp/ja-jp/product/f310-gamepad>`_ という製品ですが、他に同様の構成をもつゲームパッドとして、プレイステーション3、4用のゲームパッド（DualShock3、4)や、XBox用のゲームパッドも使用することが可能です。

それら以外の機種を使う場合は、軸やボタンの配置が適切でなくなってしまい、操作しにくくなる場合があります。その場合はソースコードに書かれている軸やボタンのID値を変更するようにして下さい。本ステップ最後の :ref:`simulation-tank-tutorial-step3-implementation` にこれに関する解説があります。

ゲームパッドがOSに認識されているか、軸やボタンの配置はどうなっているか、といったことを確認するための便利なツールとして、"jstest" というコマンドがあります。ゲームパッドがうまく機能しない場合はこのコマンドで確認するようにして下さい。これはUbuntuでは、コマンドラインから ::

 sudo apt-get install joystick
  
を実行することでインストールされ、 ::
   
 jstest /dev/input/js0
  
などと入力することで実行されます。

コマンドの引数はゲームパッドのデバイスファイルを表しています。ここで "js0" というのは最初に接続されたIDが0番のゲームパッドということで、通常はこれを使います。2台目以降のゲームパッドを使いたい場合はjs1、js2といったデバイスを使いますが、その場合はソースコードの変更が必要になりますのでご注意下さい。

ゲームパッドを接続して、上記のコマンドを実行すると、 ::

 Driver version is 2.1.0.
 Joystick (Logitech Gamepad F310) has 8 axes (X, Y, Z, Rx, Ry, Rz, Hat0X, Hat0Y)
 and 11 buttons (BtnX, BtnY, BtnTL, BtnTR, BtnTR2, BtnSelect, BtnThumbL, BtnThumbR, ?, ?, ?).
 Testing ... (interrupt to exit)
 Axes:  0:     0  1:     0  2:-32767  3:     0  4:     0  5:-32767  6:     0  7:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off

といった情報がコンソールに出力されます。ここでゲームパッドの軸やボタンを操作すると出力内容が変わりますので、これで接続状態や軸・ボタンの配置を確認できます。このような出力がされなかったり、ゲームパッドを操作しても出力内容が変わらない場合は、うまく接続されていませんので、接続方法やゲームパッドの状態を確認するようにして下さい。

.. note:: 以下で使用するジョイスティック入力用のJoystickクラスでは、ゲームパッドの機種ごとの軸やボタンの配置（ID値の対応）について、共通の配置に補正する機能を備えています。これによって上記のゲームパッドに関しては同じプログラムが使えるようになるのですが、プログラム内で使われているID値はjstestで出力されるものとは必ずしも一致しませんので、ご注意ください。

.. _tank-tutorial-virtual-joystick-view:

仮想ジョイスティックビューの準備
--------------------------------

ゲームパッドがない場合は、「仮想ジョイスティックビュー」を使います。これはメインメニューの「表示」-「ビューの表示」から「仮想ジョイスティック」を選択すると表示されます。外観は以下の図のようになっています。

.. image:: images/joystickview.png

これは通常メインウィンドウ下部のメッセージビューと同じ領域に表示されます。このままではメッセージが見えなくなってしまいますので、メッセージビューと仮想ジョイスティックビューを同時に使えるように :ref:`basics_modify_view_layout` を行っておくとよいでしょう。例えば以下の図のようなレイアウトにします。

.. image:: images/joystickview-layout.png

ゲームパッドの実機が接続されているとそちらの入力が優先されますので、仮想ジョイスティックビューを使う場合はゲームパッドは接続しないようにして下さい。

これで準備は完了です。


コントローラのソースコード
--------------------------

今回作成するコントローラのソースコードを以下に示します。これはステップ2のTurretController1に対して、砲塔ヨー軸の制御とゲームパッド入力による指令値の変更を追加した内容となっています。 ::

 #include <cnoid/SimpleController>
 #include <cnoid/Joystick>
 
 using namespace cnoid;
 
 class TurretController2 : public SimpleController
 { 
     Link* joints[2];
     double q_ref[2];
     double q_prev[2];
     double dt;
     Joystick joystick;
 
 public:
     virtual bool initialize(SimpleControllerIO* io)
     {
         joints[0] = io->body()->link("TURRET_Y");
         joints[1] = io->body()->link("TURRET_P");
 
         for(int i=0; i < 2; ++i){
             Link* joint = joints[i];
             io->enableIO(joint);
             q_ref[i] = q_prev[i] = joint->q();
         }
 
         dt = io->timeStep();
         
         return true;
     }
 
     virtual bool control()
     {
         static const double P = 200.0;
         static const double D = 50.0;
         static const int axisID[] = { 3, 4 };
 
         joystick.readCurrentState();
 
         for(int i=0; i < 2; ++i){
             Link* joint = joints[i];
             double q = joint->q();
             double dq = (q - q_prev[i]) / dt;
             double dq_ref = 0.0;
 
             double pos = joystick.getPosition(axisID[i]);
             if(fabs(pos) > 0.25){
                 double deltaq = 0.002 * pos;
                 q_ref[i] += deltaq;
                 dq_ref = deltaq / dt;
             }
             
             joint->u() = P * (q_ref[i] - q) + D * (dq_ref - dq);
             q_prev[i] = q;
         }
 
         return true;
     }
 };
 
 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController2)
 

コントローラのコンパイル
------------------------

上記のソースコードを入力・保存し、コンパイルを行いましょう。

手順はステップ2で行ったのと同様です。ソースコードを "TurretController2.cpp" というファイル名でプロジェクトディレクトリに保存し、CMakeLists.txt に以下の記述を追加して下さい。 ::

 add_cnoid_simple_controller(TankTutorial_TurretController2 TurretController2.cpp)

これでChoreonoid本体のコンパイル操作を行うと、このコントローラも同時にコンパイルされ、コントローラディレクトリ内に "TankTutorial_TurretController2.so" というファイルが生成されます。

コントローラの置き換え
----------------------

今度はこのコントローラをTankモデルのコントローラとして使用するようにしましょう。

ステップ2で作成したプロジェクトがあるかと思いますので、そこのコントローラの設定だけを変えることにします。ステップ2で解説した :ref:`simulation-tank-tutorial-set-controller` を再度行って、今回作成した "TankTutorial_TurretController2.so" のコントローラファイルに置き換えるようにして下さい。

これでコントローラの準備は完了です。この状態でプロジェクトを "step3.cnoid" といったファイル名で保存し直しておくとよいでしょう。

砲塔の操作
----------

シミュレーションを実行しましょう。

今回はゲームパッドでTankモデルの砲塔を動かせるはずですので、動かしてみましょう。F310であれば、右下のアナログスティックを砲塔の操作に対応させていますので、この軸を操作して下さい。別の機種の場合は、どの軸が対応しているか、いろいろ動かして試してみて下さい。うまくいかない場合は、ソースコードの軸設定を変更しましょう。これは次節で解説します。

仮想ジョイスティックビューを使う場合は、キーボードで操作します。ビューに表示されているボタンは、それぞれゲームパッドの十字キーやアナログスティックの各軸や、各ボタンに対応しています。この対応関係を下図に示します。

.. image:: images/joystickview-mapping.png

この図とゲームパッドF310を見比べると、F310の主要な軸とボタンに対応していることが分かるかと思います。今回はキーボードの "J"、"L" で砲塔のヨー軸回転、"I"、"K" でピッチ軸回転を操作できることになります。

注意点として、 仮想ジョイスティックビューは **キーボードフォーカスが入っていないと機能しません。** このため、使用の際にはいったんこのビューをマウスでクリックするなどして、フォーカスを入れておく必要があります。操作している最中にシーンビューの視点を変えるなどの操作をした場合、フォーカスはそちらに行ってしまっているので、再度仮想ジョイスティックビューをクリックしてフォーカスを入れなおす必要があります。

Tankモデルの砲塔をうまく動かせましたでしょうか？このように、コントローラ次第で、様々な操作が可能となってきます。外部デバイスからの入力を取り込むことで、コントローラの幅も広がります。

.. _simulation-tank-tutorial-step3-implementation:

実装内容の解説
--------------

今回のTurretController2も、ステップ2で作成したTurretController1と同様に、PD制御で砲塔の軸を制御するというもので、その部分は基本的に変わりません。

ただ、これをベースとして、以下の２点を拡張した点が異なっています。

1. 砲塔ヨー軸に対応する "TURRET_Y" 関節に加えて、ピッチ軸に対応する "TURRET_P" 関節も制御するようにした。
2. PD制御の目標関節角について、モデルの初期角度に固定するのではなく、ゲームパッド（ジョイスティック）からの入力に応じて変化させるようにした。

1については関連する変数を配列化し、forループによってそれぞれに同じ処理を行うようにしただけです。

2についてはChoreonoidが提供する "Joysitick" クラスを用いてジョイスティックからの入力を取得するようにしました。これについて解説しましょう。

まず、 ::

 #include <cnoid/Joystick>

によってJoystickクラスが定義されているヘッダをインクルードしています。

JoystickクラスのオブジェクトはTurretController2のメンバ変数 ::

 Joystick joystick;

として定義しています。コンストラクタはデフォルトのものを使っており、この場合は "/dev/input/js0" のデバイスファイルがジョイスティックの入力元となります。また、このデバイスファイルが存在しない場合、仮想ジョイスティックビューがあればそちらを入力元とします。

ジョイスティックの状態を取得するにあたっては、まず ::

 joystick.readCurrentState();

を実行します。するとデバイスファイルや仮想ジョイスティックビューから、ジョイスティックの現在の状態が読み込まれます。

あとは ::

 joystick.getPosition(軸ID）

によって、軸の状態（どれだけ倒しているか）を -1.0 〜 +1.0 の値として取得できますし、 ::

 joystic.getButtonState(ボタンID)

によって、ボタンが押しているかどうかの値をbool血として取得できます。ボタンについてはステップ5以降のコントローラで利用します。

注意点として、アナログスティックの軸の状態値について、0が中立点となるのですが、スティックを倒していない場合でも常に値が0になるとは限りません。ですので、倒しているかどうかの判定として、一定の閾値をかませることが必要になります。この処理は上記ソースコードのcontrol関数内で ::

 if(fabs(pos) > 0.25){
  
というコードで行っています。
 
ジョイスティックの軸の対応は、control関数内の ::

 static const int axisID[] = { 3, 4 };

で設定しています。ここの3,4がそれぞれ砲塔ヨー軸、ピッチ軸に対応させる軸ID値で、F310の場合は右アナログスティックに対応しています。他のゲームパッドの場合も、jstestコマンドの出力を確認するなどして、適切な軸に対応させて下さい。

実際に目標関節角度を設定している箇所は、control関数内の ::

 double pos = joystick.getPosition(axisID[i]);
 if(fabs(pos) > 0.25){
     double deltaq = 0.002 * pos;
     q_ref[i] += deltaq;
     dq_ref = deltaq / dt;
 }

の部分になります。ここでq_ref[i]が目標関節角、dq_refが目標関節角速度に対応する変数です。あとはこれらの目標値を使って、パート1と同様のPD制御を行っています。
