
コントローラの実装
==================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

.. contents:: 目次
   :local:

.. highlight:: cpp


コントローラの実装
------------------

ここでは、コントローラの実装方法について、その基本を解説します。

コントローラが行うことは、基本的には以下の３つで、これらを「制御ループ」として繰り返し実行します。

1. ロボットからの状態の入力
2. 制御計算
3. ロボットへの指令出力

これらの処理を単体のコントローラで行うこともあれば、複数のソフトウェアコンポーネントを組み合わせて行うこともあります。また、「2.制御計算」とひとくくりにした処理には、実際には各種認識・動作計画等の多様な処理がからむものですし、ロボット以外を対象とした入出力も含まれる可能性があります。しかし、ロボットを中心としてみると、最終的にコントローラがやっていることは上記の3つの処理に整理して考えることができます。

このように考えると、コントローラというものは、上記３つを行うためのインタフェースを備えたソフトウェアモジュールであると言えます。そのための実際のAPIはコントローラの形式によって異なってくるわけですが、本質的な部分は同じです。

以下では :doc:`howto-use-controller` でも用いた "SR1MinimumController"サンプルを通して解説を行います。コントローラの形式はChoreonoidのサンプル用に設計された"SimpleController"形式であり、制御の内容は関節に対するPD制御でロボットの姿勢を維持するというだけのものです。

実際にコントローラ開発する際には、「このサンプルを通して述べた基本的な事柄を、希望のコントローラ形式や制御内容に置き換える」という観点で取り組んでいただければよいかと思います。一般的に、ロボットのコントローラ開発では制御やプログラミング、ハードウェア等に関する様々な知識とスキルが必要となります。それらの多くについては本マニュアルの対象外となりますので、やりたいことに応じて、別途取り組むようにしてください。


SR1MinimumController
--------------------

まず、SR1MinimumControllerのソースコードを以下に示します。本ソースコードはChoreonoidソースの"sample/SimpleController"ディレクトリ内にある"SR1MinimumController.cpp"というファイルです。 ::

 #include <cnoid/SimpleController>
 #include <vector>
 
 using namespace cnoid;
 
 const double pgain[] = {
     8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
     3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
     8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
     3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
     8000.0, 8000.0, 8000.0 };
    
 const double dgain[] = {
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0 };
 
 class SR1MinimumController : public SimpleController
 {
     BodyPtr body;
     double dt;
     std::vector<double> qref;
     std::vector<double> qold;
 
 public:
 
     virtual bool initialize()
     {
         body = ioBody();
         dt = timeStep();
 
         for(int i=0; i < body->numJoints(); ++i){
             qref.push_back(body->joint(i)->q());
         }
         qold = qref;
         
         return true;
     }
 
     virtual bool control()
     {
         for(int i=0; i < body->numJoints(); ++i){
             Link* joint = body->joint(i);
             double q = joint->q();
             double dq = (q - qold[i]) / dt;
             double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
             qold[i] = q;
             joint->u() = u;
         }
         return true;
     }
 };
 
 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

コントローラの内容とは関係ありませんが、SimpleControllerについては、継承クラス定義後に以下の記述でそのファクトリ関数を定義しておく必要があります。

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

これにより、このソースからビルドされたバイナリファイルが、SimpleControllerアイテムから利用可能となります。

コンパイルについては、同じディレクトリにあるCMakeLists.txt内にある ::

 add_cnoid_simple_controller(SR1MinimumController SR1MinimumController.cpp)

という記述で行っています。この関数の詳細は"src/SimpleControllerPlugin/library/CMakeLists.txt"を参照してください。基本的には、"CnoidSimpleController" というライブラリとリンクすればOKです。(Linuxの場合、ライブラリは"libCnoidSimpleController.so"というファイル名になります。）


SimpleControllerクラス
----------------------

SimpleControllerは、SimpleControllerクラスを継承することで実装します。このクラスは ::

 #include <cnoid/SimpleController>

により、cnoid/SimpleControllerヘッダをインクルードすることで使えるようになります。

SimpleControllerクラスの定義において本節の解説に関連する部分を以下に示します。（実際のクラス定義はChoreonoidソースの"src/SimpleControllerPlugin/library/SimpleController.h" でされていますので、そちらを参照ください。） ::

 class SimpleController
 {
 public:
     virtual bool initialize() = 0;
     virtual bool control() = 0;

 protected:
     Body* ioBody();
     double timeStep() const;
     std::ostream& os() const;
 };


本クラスは、メンバ関数として以下の純粋仮想関数を有しています。

* **virtual bool initialize()**

 コントローラの初期化処理を行います。

* **virtual bool control()**

 コントローラの入力・制御・出力処理を行います。制御中この関数は制御ループとして繰り返し実行されることになります。

コントローラ実装の際には、まずSimpleControllerクラスを継承したクラスを定義します。その中で上記の関数をオーバーライドすることにより、コントローラの処理を実装します。

また、SimpleControllerクラスは以下のprotectedメンバ関数も備えています。

* **Body\* ioBody()**

 入出力に使うためのBodyオブジェクトを返します。

* **double timeStep() const**

 制御のタイムステップを返します。上記のcontrol関数は制御中にこの時間間隔で繰り返し呼ばれることになります。

* **std::ostream& os() const**

 テキスト出力用の出力ストリームを返します。このストリームに出力することで、Choreonoidのメッセージビュー上にテキストメッセージを表示できます。

これらのメンバ関数は上記のinitialize()、control()関数内で使用することができます。


Bodyオブジェクト
----------------

SimpleControllerでは、ioBody()が返す「Bodyオブジェクト」を介して入出力を行います。Bodyオブジェクトは、 :doc:`../handling-models/bodymodel` のChoreonoid内部での表現で、C++のソースコードにおいて「Bodyクラス」として定義されたものです。BodyクラスはBodyモデルの状態を格納するデータ構造なので、入出力対象となる関節角度やトルク、センサの状態といった要素も当然格納できます。そこで、SimpleControllerではこのBodyクラスのオブジェクトを介して入出力を行うこととしています。

.. note:: BodyクラスはBodyモデルに関する様々な情報と機能を有するので、入出力だけを行うためにはオーバースペックなクラスです。入出力のインタフェースとして通常はこのようなクラスは用いず、入出力の各要素だけをやりとりするのに最適化されたデータ構造を用いるのが一般的です。本節の内容を他のコントローラ形式に応用する際には、この点に注意して下さい。例えば、OpenRTMのRTコンポーネントでは通常「データポート」というインタフェースを用いて、データの種類ごとに分けて入出力を行います。

関節角度と関節トルク
--------------------

ロボットを制御する際に基本となる入出力要素として、関節角度と関節トルクが挙げられます。これらの要素により、各関節をPD制御で動かすことが可能となります。その場合、関節角度がロボットからの入力値となり、関節トルクがロボットへの出力指令となります。使用するコントローラ形式において、まずこれらの値をどのようにして入出力するかを確認するとよいでしょう。

SimpleControllerにおいては、これらの入出力を行うために、対応する関節の「Linkオブジェクト」を用います。LinkオブジェクトはBodyモデルの各リンクを表現する「Linkクラス」のオブジェクトで、Bodyオブジェクトから以下のメンバ関数を用いて取得できます。

* **int numJoints() const**

 モデルが有する関節の数を返します。

* **Link\* joint(int id)**

 関節番号に対応するLinkオブジェクトを返します。


取得したLinkオブジェクトに関して、以下のメンバ関数を用いて関節状態値へのアクセスが可能です。

* **double& q()**

 関節角度のdouble値への参照を返します。単位はラジアンです。参照なので、値を代入することも可能です。

* **double& u()**

 関節トルクのdouble値への参照を返します。単位は[N・m]です。参照なので、値を代入することも可能です。

SimpleContrllerにおいては、ioBody()から得られるLinkオブジェクトに対して、上記の関数を用いて入出力を行います。すなわち、q()を読み出すことにより現在の関節角度の入力を行い、u()に書き込むことによりトルク指令値をロボットへ出力します。

.. note:: 関節角度の入力に関して、上記のq()はモデルの真値を返しますが、実際のロボットで入力される値はエンコーダの精度に依存した値となります。シミュレーションにおいてもエンコーダの精度を反映したい場合には、そのための追加の処理が必要になります。また、指令値の出力に関して、実際のロボットでは関節角度や電流値等、様々な形態が有りますが、シミュレーションにおいては最終的にトルク値として出力する必要があります。ただし、目標角度を指令値として出力可能な「ハイゲインモード」が利用可能なシミュレータアイテムもあります。


初期化処理
----------

SimpleController継承クラスのinitialize()関数では、コントローラの初期化を行います。

サンプルでは、まず ::

 body = ioBody();

によって、入出力用のBodyオブジェクトを取得しています。このオブジェクトは繰り返しアクセスすることになるので、効率化と記述の簡略化のためこのようにbodyという変数に格納して使うようにしています。

同様に、制御計算では必要となるタイムステップ値について、 ::

 dt = timeStep();

によって値をdtという変数に格納しています。

次に ::

 for(int i=0; i < body->numJoints(); ++i){
     qref.push_back(body->joint(i)->q());
 }
 qold = qref;

によって目標関節角度を格納する qref という変数に、初期化時（シミュレーション開始時）のロボットの関節角度をセットしています。qoldは1ステップ前の関節角度を格納する変数で、こちらも制御計算で使います。qrefと同じ値に初期化しています。

ここでは、 ::

 body->joint(i)->q()

という記述で、i番目の関節の関節角度を入力しています。

最後にtrueを返すことで、初期化に成功したことをシミュレータに伝えます。

制御ループ
----------

SimpleControllerでは、control()関数に制御ループを記述します。

サンプルでは以下のfor文 ::

 for(int i=0; i < body->numJoints(); ++i){
     ...
 }

により、全ての関節に対して制御計算を行っています。この中身が各関節に対する処理コードです。

まず、 ::

 Link* joint = body->joint(i);

でi番目の関節に対応するLinkオブジェクトを取得しています。

次に現在の関節角度の入力を行います。 ::

 double q = joint->q();

PD制御によって関節トルクの指令値を計算します。まず、制御ループの前回の関節角度との差分から、関節角速度を算出します。 ::

 double dq = (q - qold[i]) / dt;

制御の目標は初期姿勢の維持ですので、関節角度は初期関節角度、角速度は0（静止状態）を目標として、トルク指令値を計算します。 ::

 double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];

ソースの冒頭で設定したpgain, dgainの配列から、各関節に関するゲイン値を取り出しています。ゲイン値についてはモデルごとに調整が必要ですが、その方法についてはここでは割愛します。

次回の計算用に、関節角度をqold変数に保存しておきます。 ::

 qold[i] = q;

計算したトルク指令値を出力します。これにより、関節が初期関節角度を維持するように制御されます。 ::

 joint->u() = u;

以上が全ての関節に対して適用されることにより、ロボット全体の姿勢も維持されることになります。

最後にcontrol()関節がtrueを返すことで、制御が継続している旨をシミュレータに伝えています。これにより、control()関数が繰り返し呼ばれます。

デバイス
--------

以上の例では、入出力の対象として、関節の状態量である関節角度と関節トルクを扱いました。一方で、関節とは独立した入出力の対象もあります。Choreonoidでは、それらを「デバイス」として定義しており、Bodyモデルの構成要素となります。

デバイスの例としては、まず

* 力センサ、加速度センサ、角速度センサ（ジャイロ）
* カメラ、レーザーレンジファインダ

といったセンサ類が挙げられます。これらは主に入力の対象となりますが、カメラのズーム変更等、操作指令を出力したい場合もあります。

また、主に出力の対象となるものとして、

* ライト（光源）

といったデバイスもあります。

.. * ディスプレイ
.. * プロジェクタ
.. * スピーカ
.. （※これらのうち、ライト以外はChoreonoidではまだサポートされていません。）

実用的なコントローラを開発するためには、デバイスに対しても入出力を行う必要があります。以下ではSimpleControllerにおける入出力方法を紹介しますが、他の形式の入出力方法についてはそのマニュアルを参照するようにしてください。

デバイスオブジェクト
--------------------

SimpleControllerでは、デバイスに対してもBodyオブジェクトを介して入出力を行います。正確に言うと、Bodyオブジェクトが所有する「Deviceオブジェクト」を用いて、対象となるデバイスへの入出力を行います。

デバイスは、「Deviceクラス」をベースとし、各デバイスはそれを継承したクラスとして定義されています。現在定義されているデバイスのクラス階層は以下のようになっています。 ::

 + Device
   + Sensor
     + ForceSensor
     + RateGyroSensor
     + AccelSensor
     + VisionSensor
       + Camera
       + RangeCamera
       + RangeSensor
   + ActiveDevice
     + Light
       + PointLight
       + SpotLight

これらのデバイスをBodyモデルに含める場合は、通常モデルファイルにそのための記述を追加します。OpenHRP形式のモデルファイルについては、 :doc:`../handling-models/modelfile/modelfile-openhrp` の :ref:`oepnrhp_modelfile_sensors` を記述します。

Bodyオブジェクトが有するDeviceオブジェクトの取得に関しては、Bodyクラスの以下の関数が利用できます。

* **int numDevices() const**

 デバイスの数を返します。

* **Device\* device(int i) const**

 i番目のデバイスを返します。デバイスの順番はモデルファイル中の記述順になります。（これは以下の２つの関数についても同じです。）

* **const DeviceList<>& devices() const**

 全デバイスのリストを返します。

* **template<class DeviceType> DeviceList<DeviceType> devices() const**

 指定した型のデバイスのリストを返します。

* **template<class DeviceType> DeviceType\* findDevice(const std::string& name) const**

 指定した型と名前を有するデバイスがあればそれを返します。

デバイスの型と名前が分かっている場合は、最後の関数を使うのがよいでしょう。例えばSR1モデルは "gsensor" という名前の加速度センサを有しています。これを取得するには、Bodyオブジェクトに対して ::

 AccelSensor* accelSensor = body->findDevice<AccelSensor>("gsensor");

とすればOKです。

SR1モデルが有するデバイスは以下のとおりです。

.. tabularcolumns:: |p{3.5cm}|p{3.5cm}|p{6.0}|

.. list-table::
 :widths: 30,30,40
 :header-rows: 1

 * - 名前
   - デバイスの型
   - 内容
 * - gsensor
   - AccelSensor
   - 腰リンクに搭載された加速度センサ
 * - gyrometer
   - RateGyroSensor
   - 腰リンクに搭載されたジャイロ
 * - VISION_SENSOR1
   - RangeCamera
   - 左目に対応する距離画像センサ
 * - VISION_SENSOR2
   - RangeCamera
   - 右目に対応する距離画像センサ




デバイスに対する入出力
----------------------

Deviceオブジェクトを介した入出力は、以下のようにして行います。

* **入力**

 対応するDeviceオブジェクトのメンバ関数を用いて値を取得する。

* **出力**

 対応するDeviceオブジェクトのメンバ関数を用いて値を設定した後、Deviceオブジェクトの "notifyStateChange()" 関数を実行する。

これらを行うためには、使用するデバイスのクラス定義を知っている必要があります。例えば加速度センサのクラスである"AccelSensor"に関しては、その状態にアクセスするための"dv()"というメンバ関数があります。これは加速度をVector3型の3次元ベクトルで返します。

従って、加速度を取得するためには、 ::

 AccelSensor* accelSensor = body->findDevice<AccelSensor>("gsensor");

として取得しておいたAccelSensorオブジェクトに対して、 ::

 accelSensor->dv();

とすればOKです。

同様に、ForceSensorやRateGyroSensorに関しても該当するメンバ関数を用いて状態の入力を行うことが可能です。

カメラやレンジセンサ等の視覚センサを使用する際には、そのための準備が必要になります。これについては次節で解説します。

デバイスへの出力については、ライトのオン・オフを行う "TankJoystickLight.cnoid" というサンプルを参考にしてください。

.. 立たせるのもやめて、倒れるシミュレーションにして、加速度が一定値以上のときだけ表示するようなサンプルを作る？
