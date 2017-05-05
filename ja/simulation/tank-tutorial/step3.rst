
Step 3: ゲームパッドによる砲塔の制御
====================================

Step 2では最小限の制御を行うコントローラを作成し、コントローラの導入方法を学びました。Step 3ではもう少し複雑なコントローラを作成してみましょう。

.. contents:: 目次
   :local:
   :depth: 2

.. highlight:: C++
   :linenothreshold: 6


ゲームパッドの準備
------------------

このステップで作成するコントローラは、ゲームパッド（ジョイスティック）を操作することでTankモデルの砲塔を動かすというものです。このための準備として、ゲームパッドを用意してPCに接続しておく必要があります。

ただし、ゲームパッドが無い場合は、その代わりにChoreonoidの「仮想ジョイスティックビュー」をいう機能を使うことができますので、次節（:ref:`tank-tutorial-virtual-joystick-view`）に進むようにしてください。以下はゲームパッドの実機を使う場合の説明になります。

本チュートリアルが想定しているゲームパッドは、 `ロジクールのF310 <http://gaming.logicool.co.jp/ja-jp/product/f310-gamepad>`_ という製品です。この機種であれば、掲載されたソースコードそのままで適切な軸・ボタンの配置となります。これ以外の機種を使う場合は、軸やボタンの配置が適切でなくなってしまい、操作しにくくなる場合があります。その場合はソースコードに書かれている軸やボタンのID値を変更するようにしてください。

ゲームパッドがOSに認識されているか、軸やボタンの配置はどうなっているか、といったことを確認するための便利なツールとして、"jstest" というコマンドがあります。ゲームパッドがうまく機能しない場合はこのコマンドで確認するようにしてください。これはUbuntuでは、コマンドラインから ::

 sudo apt-get install joystick
  
を実行することでインストールされ、 ::
   
 jstest /dev/input/js0
  
などと入力することで実行されます。

コマンドの引数はゲームパッドのデバイスファイルを表しています。ここで "js0" というのは最初に接続されたIDが0番のゲームパッドということで、通常はこれを使います。2台目以降のゲームパッドを使いたい場合はjs1、js2といったデバイスを使いますが、その場合はソースコードの変更が必要になりますのでご注意ください。

ゲームパッドを接続して、上記のコマンドを実行すると、 ::

 Driver version is 2.1.0.
 Joystick (Logitech Gamepad F310) has 8 axes (X, Y, Z, Rx, Ry, Rz, Hat0X, Hat0Y)
 and 11 buttons (BtnX, BtnY, BtnTL, BtnTR, BtnTR2, BtnSelect, BtnThumbL, BtnThumbR, ?, ?, ?).
 Testing ... (interrupt to exit)
 Axes:  0:     0  1:     0  2:-32767  3:     0  4:     0  5:-32767  6:     0  7:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off

といった情報がコンソールに出力されます。ここでゲームパッドの軸やボタンを操作すると出力内容が変わりますので、これで接続状態や軸・ボタンの配置を確認できます。このような出力がされなかったり、ゲームパッドを操作しても出力内容が変わらない場合は、うまく接続されていませんので、接続方法やゲームパッドの状態を確認するようにしてください。

.. _tank-tutorial-virtual-joystick-view:

仮想ジョイスティックビューの準備
--------------------------------

ゲームパッドがない場合は、「仮想ジョイスティックビュー」を使います。これはメインメニューの「表示」-「ビューの表示」から「仮想ジョイスティック」を選択すると表示されます。外観は以下の図のようになっています。

.. image:: images/joystickview.png

これは通常メインウィンドウ下部のメッセージビューと同じ領域に表示されます。このままではメッセージが見えなくなってしまいますので、メッセージビューと仮想ジョイスティックビューを同時に使えるように :ref:`basics_modify_view_layout` を行っておくとよいでしょう。例えば以下の図のようなレイアウトにします。

.. image:: images/joystickview-layout.png


コントローラのソースコード
--------------------------

今回作成するコントローラのソースコードは以下になります。これはStep 2のTurretController1に対して、砲塔ヨー軸の制御とゲームパッド入力による指令値の変更を追加した内容となっています。 ::

 #include <cnoid/SimpleController>
 #include <cnoid/Joystick>
 
 using namespace cnoid;
 
 class TurretController2 : public SimpleController
 { 
     Link* joints[2];
     double qref[2];
     double qold[2];
     double dt;
     Joystick joystick;
 
 public:
     virtual bool initialize(SimpleControllerIO* io)
     {
         joints[0] = io->body()->link("TURRET_Y");
         joints[1] = io->body()->link("TURRET_P");
 
         for(int i=0; i < 2; ++i){
             Link* joint = joints[i];
             qref[i] = qold[i] = joint->q();
             io->setLinkInput(joint, JOINT_ANGLE);
             io->setLinkOutput(joint, JOINT_TORQUE);
         }
 
         dt = io->timeStep();
         
         return true;
     }

     virtual bool control()
     {
         static const double P = 200.0;
         static const double D = 50.0;
         static const int cannonAxis[] = { 3, 4 };
 
         joystick.readCurrentState();
 
         for(int i=0; i < 2; ++i){
             Link* joint = joints[i];
             double q = joint->q();
             double dq = (q - qold[i]) / dt;
             double dqref = 0.0;
 
             double pos = joystick.getPosition(cannonAxis[i]);
             if(fabs(pos) > 0.25){
                 double deltaq = 0.002 * pos;
                 qref[i] += deltaq;
                 dqref = deltaq / dt;
             }
             
             joint->u() = P * (qref[i] - q) + D * (dqref - dq);
             qold[i] = q;
         }
 
         return true;
     }
 };
 
 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController2)



