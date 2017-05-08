
ステップ6: カメラ画像のシミュレーションと取得
=============================================

.. contents:: 目次
   :local:
   :depth: 2

.. highlight:: C++
   :linenothreshold: 7

本ステップの概要
----------------

本ページは現在作成中ですが、ステップ6の概要を記しておきます。

ステップ6は、カメラ等の視覚センサをシミュレーションし、シミュレートされたセンサのデータを入力するというものです。

以下に掲載のソースコードをこれまでと同様に入力、コンパイルして、プロジェクトに追加導入してください。

また、カメラ画像のシミュレーションを行うため、 「GLビジョンシミュレータアイテム」をプロジェクトに導入します。これはメインメニューの「ファイル」-「新規」-「GLビジョンシミュレータ」によって作成し、Worldアイテムの小アイテムとして配置してください。この状態でシミュレーションを実行すると、内部でカメラ画像がシミュレートされるようになります。

本ステップのコントローラは、そのようにして生成されたカメラ画像を取得します。ゲームパッドもしくは仮想ジョイスティックビューのBボタンを押すと、現在のカメラ画像がファイルに保存されます。保存先はカレントディレクトリで、ファイル名は "Camera.png" となります。うまく取得できているが、適当な画像ビューアで確認してみてください。Ubuntu上では標準の"eog"という画像ビューアがあります。コマンド"eog"で起動できます。

視覚センサのシミュレーションや入出力に関する詳細は、 :doc:`../vision-simulation` で解説していますので、そちらも参考にしてください。

コントローラのソースコード
--------------------------
::

 #include <cnoid/SimpleController>
 #include <cnoid/Camera>
 #include <cnoid/Joystick>
 
 using namespace cnoid;
 
 class CameraController : public SimpleController
 {
     Camera* camera;
     Joystick joystick;
     bool prevButtonState;
     std::ostream* os;
     
 public:
     virtual bool initialize(SimpleControllerIO* io)
     {
         camera = io->body()->findDevice<Camera>("Camera");
         io->enableInput(camera);
         prevButtonState = false;
         os = &io->os();
         return true;
     }
 
     virtual bool control()
     {
         static const int button[] = { 1 };
         
         joystick.readCurrentState();
 
         bool currentState = joystick.getButtonState(button[0]);
         if(currentState && !prevButtonState){
             const Image& image = camera->constImage();
             if(!image.empty()){
                 std::string filename = camera->name() + ".png";
                 camera->constImage().save(filename);
                 (*os) << "The image of " << camera->name() << " has been saved to \"" << filename << "\"." << std::endl;
             }
         }
         prevButtonState = currentState;
 
         return true;
     }
 };
 
 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)

