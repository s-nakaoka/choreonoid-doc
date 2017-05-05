
Step 3: ゲームパッドによる砲塔の制御
====================================

Step 2では最小限の制御を行うコントローラを作成し、コントローラの導入方法を学びました。Step 3ではもう少し複雑なコントローラを作成してみましょう。

.. contents:: 目次
   :local:
   :depth: 2

.. highlight:: C++
   :linenothreshold: 5


ゲームパッドの準備
------------------

このステップで作成するコントローラは、ゲームパッド（ジョイスティック）を操作することでTankモデルの砲塔を動かすというものです。このための準備として、ゲームパッドを用意してPCに接続しておく必要があります。

ただし、ゲームパッドが無い場合は、その代わりにChoreonoidの「仮想ジョイスティックビュー」をいう機能を使うことができますので、本節は飛ばして次の節に進むようにしてください。以下はゲームパッドの実機を使う場合の説明になります。

本ステップでは砲塔の２軸を動かすだけなので、ゲームパッドは操作軸を最低2軸（スティック1本）を有するものであればOKです。ただし今後のステップで操作内容を増やしていきますので、操作軸を4軸（スティック２本）以上、ボタンを4つ以上有するものが望ましいです。と言っても最近のゲームパッドはもっと多くの軸やボタンを持っていますので、問題ないかと思われます。

本チュートリアルではコーディングは必要最小限に抑えているため、様々なゲームパッドの軸配置、ボタン配置に対応できるようにはなっていません。従って、ゲームパッドの機種によっては、軸やボタンの配置が適切でなくなってしまい、操作しにくくなる場合もあるかと思います。その場合はソースコードに書かれている軸やボタンのID値を変更するようにしてください。本チュートリアルが想定しているゲームパッドは、ロジクールのF310という製品です。この機種であれば、掲載されたソースコードそのままで適切な軸・ボタンの配置となります。

ゲームパッドがOSに認識されているか、軸やボタンの配置はどうなっているか、といったことを確認するための便利なツールとして、"jstest" というコマンドがあります。ゲームパッドがうまく機能しない場合はこのコマンドで確認するようにしてください。これはUbuntuでは、コマンドラインから ::

 sudo apt-get install joystick
  
を実行することでインストールされ、 ::
   
 jstest /dev/input/js0
  
などと入力することで実行されます。

コマンドの引数はゲームパッドのデバイスファイルを表しています。ここで "js0" というのは最初に接続されたIDが0番のゲームパッドということで、通常はこれを使います。2台目以降のゲームパッドを使いたい場合はjs1、js2といったデバイスを使いますが、その場合はソースコードの変更が必要になりますのでご注意ください。

上記コマンドを実行すると ::

 a0 b0 ...

といった情報がコンソールに出力されます。ここでゲームパッドの軸やボタンを操作すると出力内容が変わりますので、これで接続状態や軸・ボタンの配置を確認できます。このような出力がされなかったり、ゲームパッドを操作しても出力内容が変わらない場合は、うまく接続されていませんので、接続方法やゲームパッドの状態を確認するようにしてください。
 
仮想ジョイスティックビューの準備
--------------------------------


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



