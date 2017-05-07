
ステップ5: ライトの制御
=======================

Tankモデルはライト（光源）を搭載しています。ステップ5ではこのライトをコントローラから操作できるようにし、デバイスを制御する方法について学びます。

.. contents:: 目次
   :local:
   :depth: 2

.. highlight:: C++
   :linenothreshold: 7

環境設定
--------

ライトを使用するにあたって、その効果をより分かりやすくするための環境設定を行うことにしましょう。

シーンレンダラの変更
~~~~~~~~~~~~~~~~~~~~
Choreonoidの実装においては、モデルを3DCGとして描画する部分を「シーンレンダラ」と呼んでおり、この部分の実装によってシーンビューの描画能力も変わってきます。

これに関して、実はChoreonoidの開発版には新しいシーンレンダラの実装が含まれています。これは3DCG描画用APIである "OpenGL" のプログラマブルシェーダ機能を使うもので、内部的には 「GLSLシーンレンダラ」と命名されています。このレンダラを用いることにより、光源による物体の照射をより正確に再現できたり、それによって生じる影を再現することも可能となります。つまり、よりリアルなシーン描画を行うことができるというわけです。これはTankモデルに搭載されたライトの効果を高めるためにも有効ですので、今回この機能も使ってみましょう。

GLSLシーンレンダラは現在開発中のこともあり、デフォルトでは使われないようになっていますが、環境変数 "CNOID_USE_GLSL" を設定することにより利用可能となります。具体的には、Choreonoid起動時に、 ::

 CNOID_USE_GLSL=1 choreonoid

といったかたちで、この環境変数の設定を付与することにより、起動されたChoreonoidではGLSLシーンレンダラが使われるようになります。あるいは、あらかじめ ::

 export CNOID_USE_GLSL=1

というコマンドをどこかで実行しておけば、それ以降はこの環境変数の設定が有効となります。

ただし、GLSLシーンレンダラを利用するためには、OpenGLのバージョン3.3以降が利用可能となっている必要があります。

利用可能なOpenGLのバージョンは、"glxinfo" というコマンドを実行すると分かります。このコマンドはUbuntu上では ::

 sudo apt-get install mesa-utils

を実行するとインストールされます。そして ::

 glxinfo

を実行すると利用可能なOpenGLのバージョンや機能に関する情報が表示されます。この中に ::

 OpenGL core profile version string: 4.5.0 NVIDIA 375.39

といった表示があれば、OpenGLの4.5.0までサポートされていることになります。

ただし、OpenGLの3.3がサポートされていても、環境によってはうまく機能しないこともありますので、ご了承ください。その場合は上記の環境変数は設定せず、引き続きデフォルトのシーンレンダラを使っていただくことになります。

環境モデルの変更
~~~~~~~~~~~~~~~~

まず環境モデルを変更します。これまで使ってきた床のモデルだけだと、ライトが照射されてもあまり代り映えがしません。そこで今回は、Choreonoidのサンプルとして用意されている "Labo1" というモデルを使うことにします。これは以下に示すような、研究用プラントを想定したモデルとなっています。

.. image:: images/labo1.png

このモデルはChoreonoidのshareディレクトリの "model/Labo1" に "Labo1.body" というファイル名で格納されていますので、これを読み込んでください。配置は他のモデルと同様に、Worldアイテムの小アイテムとし、アイテムのチェックも入れておきましょう。そして、これまで読み込んでいた床のモデルである "Floor" アイテムは削除しておきます。アイテムを右クリックするとコンテキストメニューが表示されますので、そこから「カット」を選択することで削除できます。以上の作業を行うと、アイテムツリーは以下のようになっているかと思います。

.. image:: images/labo1item.png

なお、アイテム間の親子関係が同じであれば、アイテムの並び順はどうなっていてもOKです。なので例えばLabo1がAISTSimulatorの次に配置されたりしていてもかまいません。もし並び順が気になる場合は、アイテムをドラッグすることで並び順だけ変えることもできますので、そこは好きなように設定してください。

シーン設定の変更
~~~~~~~~~~~~~~~~

次にシーンの描画に関わる設定を変更しましょう。これを行うにあたっては、まず以下のシーンバーの設定ボタンを押します。

.. image:: images/scenebar-config.png

すると以下のような設定ダイアログが表示され、シーン描画に関する各種設定を行うことができます。

.. image:: images/scene-config.png

まず「床グリッド線の表示」のチェックを外して、グリッド線を表示しないようにしましょう。ちゃんとした環境モデルがある場合は、グリッド線を表示すると不自然になるからです。






ライトのコントローラ
--------------------

では本題に入りましょう。今回作成するのは、Tankモデルのライトを操作するためのコントローラで、これを "LightController" とします。このコントローラのソースコードを以下に示します。 ::

 #include <cnoid/SimpleController>
 #include <cnoid/SpotLight>
 #include <cnoid/Joystick>
 
 using namespace cnoid;
 
 class LightController : public SimpleController
 {
     SpotLight* light;
     Joystick joystick;
     bool prevButtonState;
 
 public:
     virtual bool initialize(SimpleControllerIO* io)
     {
         light = io->body()->findDevice<SpotLight>("Light");
         prevButtonState = false;
         return true;
     }
 
     virtual bool control()
     {
         static const int buttonID[] = { 0, 2, 3 };
 
         joystick.readCurrentState();
 
         bool changed = false;
 
         bool currentState = joystick.getButtonState(buttonID[0]);
         if(currentState && !prevButtonState){
             light->on(!light->on());
             changed = true;
         }
         prevButtonState = currentState;
 
         if(joystick.getButtonState(buttonID[1])){
             light->setBeamWidth(std::min(0.7854f, light->beamWidth() + 0.001f));
             changed = true;
         } else if(joystick.getButtonState(buttonID[2])){
             light->setBeamWidth(std::max(0.1f, light->beamWidth() - 0.001f));
             changed = true;
         }
 
         if(changed){
             light->notifyStateChange();
         }
 
         return true;
     }
 };
 
 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(LightController)
