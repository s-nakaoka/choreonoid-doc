
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

1. ロボットの状態を入力
2. 制御計算
3. ロボットへ指令を出力

これらの処理を単体のコントローラで行うこともあれば、複数のソフトウェアコンポーネントを組み合わせて行うこともあります。また、「制御計算」とひとくくりにした処理には、実際には各種認識・動作計画等の多様な処理がからむものですし、ロボット以外を対象とした入出力も含まれる可能性があります。しかし、ロボットを中心としてみると、最終的にコントローラがやっていることは上記の3つの処理に整理して考えることができます。

このように考えると、コントローラというものは、上記３つを行うためのインタフェースを備えたソフトウェアモジュールであると言えます。そのための実際のAPIはコントローラの形式によって異なってくるわけですが、本質的な部分は同じです。

以下では :doc:`howto-use-controller` でも用いた "SR1MinimumController"サンプルを通して解説を行います。コントローラの形式はChoreonoidのサンプル用に設計された「シンプルコントローラ」形式であり、制御の内容は関節に対するPD制御でロボットの姿勢を維持するというだけのものです。記述言語はC++です。

実際にコントローラ開発する際には、このサンプルを通して述べた基本的な事柄を希望のコントローラ形式や制御内容に置き換えて、取り組んでいただければよいかと思います。一般的に、ロボットのコントローラ開発では制御やプログラミング、ハードウェア等に関する様々な知識とスキルが必要となります。それらの多くについては本マニュアルの対象外となりますので、やりたいことに応じて、別途取り組むようにしてください。


サンプルコントローラのソースコード
----------------------------------

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
     BodyPtr ioBody;
     double dt;
     std::vector<double> qref;
     std::vector<double> qold;
 
 public:
 
     virtual bool initialize(SimpleControllerIO* io)
     {
         ioBody = io->body();
         dt = io->timeStep();

         io->setJointInput(JOINT_ANGLE);
         io->setJointOutput(JOINT_TORQUE);
 
         for(int i=0; i < ioBody->numJoints(); ++i){
             qref.push_back(ioBody->joint(i)->q());
         }
         qold = qref;
 
         return true;
     }
 
     virtual bool control()
     {
         for(int i=0; i < ioBody->numJoints(); ++i){
             Link* joint = ioBody->joint(i);
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

コンパイルについては、同じディレクトリにあるCMakeLists.txt内にある ::

 add_cnoid_simple_controller(SR1MinimumController SR1MinimumController.cpp)

という記述で行っています。この関数の詳細は"src/SimpleControllerPlugin/library/CMakeLists.txt"を参照してください。基本的には、"CnoidSimpleController" というライブラリとリンクすればOKです。(Linuxの場合、ライブラリは"libCnoidSimpleController.so"というファイル名になります。）


SimpleControllerクラス
----------------------

シンプルコントローラ形式のコントローラは、SimpleControllerクラスを継承することで実装します。このクラスは ::

 #include <cnoid/SimpleController>

により、cnoid/SimpleControllerヘッダをインクルードすることで使えるようになります。

このクラスは基本的には以下のような定義になっています。 ::

 class SimpleController
 {
 public:
     virtual bool initialize(SimpleControllerIO* io);
     virtual bool control() = 0;
 };

.. ※定義の詳細についてはそのソースコードである"src/SimpleControllerPlugin/library/SimpleController.h" にて確認してください。

このクラスのvirtual関数を継承先のクラスでオーバーライドすることにより、コントローラの処理内容を記述します。各関数の内容は以下のようになっています。

* **virtual bool initialize(SimpleControllerIO\* io)**

 コントローラの初期化処理を行います。引数 io を通して制御に関わるオブジェクトや情報を取得できます。

* **virtual bool control()**

 コントローラの入力・制御・出力処理を行います。制御中この関数は制御ループとして繰り返し実行されることになります。

SimpleControllerを継承したクラスを定義したら、そのファクトリ関数を定義しておく必要があります。これは以下のようにマクロを用いて記述すればOKです。 ::

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

これにより、このソースからコンパイルされた共有（ダイナミックリンク）ライブラリのファイルが、実際のコントローラとしてシンプルコントローラアイテムから利用可能となります。

.. _simulator-simple-controller-io:

SimpleControllerIO オブジェクト
-------------------------------

上記のinitialize関数の引数 io として渡されるSimpleControllerIOオブジェクトは、コントローラとロボットの間の入出力に必要な情報を扱うオブジェクトで、以下のようなメンバ関数を有しています。

* **Body\* body()**

 入出力に使うためのBodyオブジェクトを返します。

* **void setLinkInput(Link\* link, int stateTypes)**

 コントローラへ入力となる状態値の種類をリンクごとに指定します。

* **void setJointInput(int stateTypes)**

 コントローラへの入力となる状態値の種類について、全ての関節に対して一括で指定します。

* **void setLinkOutput(Link\* link, int stateTypes)**

 コントローラから出力される指令値の種類をリンクごとに指定します。
  
* **void setJointOutput(int stateTypes)**

 コントローラから出力される指令値の種類について、全ての関節に対して一括で指定します。
 
* **double timeStep() const**

 制御のタイムステップを返します。SimpleControllerのcontrol関数は制御中にこの時間間隔で繰り返し呼ばれることになります。

* **std::ostream& os() const**

 テキスト出力用の出力ストリームを返します。このストリームに出力することで、Choreonoidのメッセージビュー上にテキストメッセージを表示できます。

以下ではこのオブジェクトを「ioオブジェクト」と呼ぶことにします。

.. _simulator-io-by-body-object:

Bodyオブジェクトを介した入出力
------------------------------

シンプルコントローラでは、「Bodyオブジェクト」を介して入出力を行います。Bodyオブジェクトは、 :doc:`../handling-models/bodymodel` のChoreonoid内部での表現で、C++で定義された「Bodyクラス」のインスタンスです。Bodyクラスはロボットのモデルとその状態を格納するためのデータ構造なので、入出力対象となる関節角度やトルク、センサの状態に関する値も当然格納できます。そこで、シンプルコントローラではこのBodyオブジェクトを介して入出力を行うこととしています。このためのBodyオブジェクトはioオブジェクトのbody関数で取得することが可能です。

.. note:: Bodyクラスはモデルに関する様々な情報と機能を有するので、入出力だけを行うためには実はオーバースペックなデータ構造です。シンプルコントローラでは実装の簡便さを優先してこれを用いていますが、入出力のインタフェースとしては通常このようなデータ構造は用いずに、特定の入出力要素のやりとりに最適化されたデータ構造を用いるのが一般的です。例えば、OpenRTMのRTコンポーネントでは、特定のデータをやりとりする「データポート」というインタフェースを用いて入出力を行います。

実際にどの要素を入出力対象とするかは、ioオブジェクトのsetLinkInputやsetJointInputで入力対象を、setLinkOutputやsetJointOutputで出力対象を指定します。対象となる状態値の種類は以下のシンボルを引数stateTypesに渡すことで指定します。

.. list-table::
 :widths: 60,40
 :header-rows: 1

 * - シンボル
   - 内容
 * - JOINT_ANGLE
   - 関節角度
 * - JOINT_DISPLACEMENT
   - 関節変位
 * - JOINT_VELOCITY
   - 関節速度
 * - JOINT_ACCELERATION
   - 関節加速度
 * - JOINT_TORQUE
   - 関節トルク
 * - JOINT_FORCE
   - 関節(並進)力
 * - LINK_POSITION
   - リンク位置・姿勢
   
.. note:: JOINT_ANGLEとJOINT_DISPLACEMENTが実際に指す内容（シミュレータ内での変数）は同じです。関節には回転（ヒンジ）関節と直動関節があり、それらに合わせた表現をできるように２種類のシンボルが用意されています。JOINT_TORQUEとJOINT_FORCEについても同様です。

複数の要素を対象としたい場合は、それらのシンボルをビット演算子の '|' で列挙します。例えば、 ::

 JOINT_ANGLE | JOINT_VELOCITY

と指定することで、関節角度と関節速度の両方が対象となります。

Bodyオブジェクトにおいて、上記の要素はロボットのリンクをモデル化した「Linkクラス」のオブジェクトに格納されます。各関節に対応するLinkオブジェクトは、Bodyオブジェクトの以下のような関数を用いて取得できます。

* **int numJoints() const**

 モデルが有する関節の数を返します。

* **Link\* joint(int id)**

 関節番号(id)に対応するLinkオブジェクトを返します。
  
* **Link\* link(const std::string& name)**

 モデルファイルで定義されている名前がnameであるLinkオブジェクトを返します。
 
取得したLinkオブジェクトに関して、以下のメンバ関数を用いてJ関節状態値へのアクセスが可能です。

* **double& q()**

 関節変位値への参照を返します。JOINT_ANGLE, JOINT_DISPLACEMENTに対応します。単位は[rad]または[m]です。

* **double& dq()**

 関節速度値への参照を返します。JOINT_VELOCITYに対応します。単位は[rad/s]または[m/s]です。

* **double& ddq()**

 関節加速度値への参照を返します。JOINT_ACCELERATIONに対応します。単位は[rad/s^2]または[m/s^2]です。

* **double& u()**

 関節トルク（並進力）値への参照を返します。JOINT_TORQUE, JOINT_FORCEに対応します。単位は[N・m]または[N]です。


これらのメンバは対応する変数への参照を返すので、値を代入することも可能です。コントローラからの出力はそのようにして行います。

実際に入出力で利用可能な要素は、シミュレータアイテム（≒物理エンジン）のタイプや設定によって変わってきます。ほとんどのシミュレータアイテムでは関節角度（変位）の入力と関節トルク（並進力）の出力をサポートしており、これを用いて基本的なPD制御を行うことが可能です。その場合、各関節に対応するLinkオブジェクトに関して、q()の値を読むことで関節角度（変位）の入力を行い、その値をもとにPD制御の計算を行い、結果のトルク（力）をu()に代入することで出力します。

実はコントローラへの入力に関しては、多くのシミュレータアイテムで上に挙げた全ての要素の入力が可能となっています。それらはシミュレータ内部の物理計算において保持されている値なので、シミュレーションにおいてはその値を返すだけで入力を実現できるのです。ただし、対象がロボットの実機となると話が違ってきます。実機の場合、関節変位の入力にはエンコーダが、関節トルクの入力にはトルクセンサが必要となりますし、関節速度や加速度は関節変位の微分によって算出するのが一般的です。

出力に関しては、関節トルク（力）以外の要素は利用できる状況が限定されてきますが、いくつかの利用方法があります。例えば、AISTシミュレータアイテムでは「動力学モード」というプロパティがあり、ここで「ハイゲイン動力学」を指定すると、コントローラからの出力として関節角度、関節速度、関節加速度を受け付けるようになります。この場合、出力した関節姿勢を実現するように全体の運動が計算されます。ただし、これについてもロボット実機では利用できないと思ったほうがよいでしょう。

初期化処理
----------

SimpleController継承クラスのinitialize()関数では、コントローラの初期化を行います。

サンプルでは、まず ::

 ioBody = io->body();

によって、入出力用のBodyオブジェクトを取得し、メンバ変数ioBodyに格納しています。これにより、このオブジェクトをコントローラの他の関数内でも使えるようにしています。

同様に、制御計算で必要となるタイムステップ（デルタタイム）値について、 ::

 dt = io->timeStep();

によって値をdtというメンバ変数に格納しています。

次に、 ::

 io->setJointInput(JOINT_ANGLE);
 io->setJointOutput(JOINT_TORQUE);

によって、コントローラへの入力を関節角度とし、コントローラからの出力を関節トルクとしています。このような入出力要素の設定は初期化時に一回行っておけばOKです。

なお、ここでは全ての関節に対して入力・出力のタイプが同じになるため、setJointInput, setJointOutput関数で一括して設定しています。関節ごとに入出力のタイプを変えたい場合は、setLinkInput, setLinkOutput関数を使います。

そして ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     qref.push_back(ioBody->joint(i)->q());
 }
 qold = qref;

によって目標関節角度を格納する qref という変数に、初期化時（シミュレーション開始時）のロボットの関節角度を格納しています。qoldは1ステップ前の関節角度を格納する変数で、こちらも制御計算で使います。qrefと同じ値に初期化しています。

ここでは、 ::

 ioBody->joint(i)->q()

という記述で、i番目の関節の関節角度を取得しています。

最後に、initialize関数の戻り値としてtrueを返すことで、初期化に成功したことをシミュレータに伝えます。

制御ループ
----------

SimpleController継承クラスでは、そのcontrol()関数に制御ループを記述します。

サンプルでは以下のfor文 ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     ...
 }

により、全ての関節に対して制御計算を行っています。この中身が各関節に対する処理コードです。

まず、 ::

 Link* joint = ioBody->joint(i);

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

最後にこのcontrol関数がtrueを返すことで、制御が継続している旨をシミュレータに伝えています。これにより、control関数が繰り返し呼ばれることになります。

デバイス
--------

以上の例では関節角度を入力し、関節トルクを出力しました。これは関節に備え付けられたエンコーダ、アクチュエータといったデバイスを対象に入出力を行っていると考えることができます。

そのように入出力の対象となるデバイスは他にも様々なものが存在し得ます。例えば、エンコーダと同様に、センサとして主に入力の対象となるものとして、

.. 一般的にロボットは関節エンコーダ、アクチュエータ以外にも多様なデバイスを備えています。

.. 以上の例では、入出力の対象として、関節の状態量である関節角度と関節トルクを扱いました。一方で、関節とは独立した入出力要素もあります。Choreonoidでは、それらを「デバイス」として定義しており、Bodyモデルの構成要素となります。
.. デバイスの例としては、まず

.. その例として、

* 力センサ、加速度センサ、角速度センサ（レートジャイロ）
* カメラ、レーザーレンジファインダ
* マイク

といったデバイスが挙げられます。

.. が、カメラのズーム変更等、操作指令を出力したい場合もあります。
.. 主に出力の対象となるものとして、

また、アクチュエータと同様に、主に出力の対象として外界に働きかけるものとして、

* スピーカ
* ディスプレイ
* ライト

といったデバイスもあり得ます。

.. * ディスプレイ
.. * プロジェクタ
.. * スピーカ
.. （※これらのうち、ライト以外はChoreonoidではまだサポートされていません。）

実際のコントローラ開発においては、これらの多様なデバイスに対しても入出力を行う必要が出てきます。これを行うためには、

* モデルにおいてデバイスがどのように定義されているか
* 使用するコントローラ形式において所定のデバイスにどのようにアクセスするか

を把握している必要があります。

.. _simulation-device-object:


デバイスオブジェクト
--------------------

Choreonoidのボディモデルにおいて、デバイスの情報は「Deviceオブジェクト」として表現されます。これは「Deviceクラス」を継承した型のインスタンスで、デバイスの種類ごとにそれぞれ対応する型が定義されています。標準で定義されているデバイス型は以下のようになっています。 ::

 + Device
   + ForceSensor (力センサ)
   + RateGyroSensor (角速度センサ)
   + AccelerationSensor (加速度センサ)
   + Camera (カメラ）
     + RangeCamera (カメラ＋距離画像センサ）
   + RangeSensor (レンジセンサ）
   + Light
     + PointLight (点光源ライト）
     + SpotLight (スポットライト）

ロボットに搭載されているデバイスの情報は、通常はモデルファイルにおいて記述します。OpenHRP形式のモデルファイルについては、 :doc:`../handling-models/modelfile/modelfile-openhrp` の :ref:`oepnrhp_modelfile_sensors` を記述します。

シンプルコントローラでは、Bodyオブジェクトと同様に、デバイスに対してもChoreonoid内部表現であるDeviceオブジェクトをそのまま用いて入出力を行います。DeviceオブジェクトはBodyオブジェクトから以下の関数を用いて取得できます。

* **int numDevices() const**

 デバイスの数を返します。

* **Device\* device(int i) const**

 i番目のデバイスを返します。デバイスの順番はモデルファイル中の記述順になります。

* **const DeviceList<>& devices() const**

 全デバイスのリストを返します。

* **template<class DeviceType> DeviceList<DeviceType> devices() const**

 指定した型のデバイスのリストを返します。

* **template<class DeviceType> DeviceType\* findDevice(const std::string& name) const**

 指定した型と名前を有するデバイスがあればそれを返します。

特定の型のデバイスを取得するには、テンプレートクラスDeviceListを使用します。DeviceListは指定した型のデバイスオブジェクトを格納する配列であり、そのコンストラクタや抽出オペレータ(<<)等を用いて、他の型も含むDeviceListから対応する型のみを抽出できます。例えばBodyオブジェクト"ioBody"の保有する力センサを取得したい場合は、 ::

 DeviceList<ForceSensor> forceSensors(ioBody->devices());

としてもよいですし、既存のリストに対して ::

 forceSensors << ioBody->devices();

として追加することもできます。

DeviceListはstd::vectorと同様の関数や演算子を備えており、例えば ::

 for(size_t i=0; i < forceSensors.size(); ++i){
     ForceSensor* forceSensor = forceSensor[i];
     ...
 }

といったかたちで各オブジェクトにアクセスできます。

findDevice関数を用いることで、型と名前でデバイスを特定して取得することもできます。例えばSR1モデルは腰リンクに搭載された "WaistAccelSensor" という名前の加速度センサを有しています。これを取得するには、Bodyオブジェクトに対して ::

 AccelerationSensor* waistAccelSensor =
     ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");

とすればOKです。

SR1モデルが有するデバイスは以下のとおりです。

.. tabularcolumns:: |p{3.5cm}|p{3.5cm}|p{6.0}|

.. list-table::
 :widths: 30,30,40
 :header-rows: 1

 * - 名前
   - デバイスの型
   - 内容
 * - WaistAccelSensor
   - AccelerationSensor
   - 腰リンクに搭載された加速度センサ
 * - WaistGyro
   - RateGyroSensor
   - 腰リンクに搭載されたジャイロ
 * - LeftCamera
   - RangeCamera
   - 左目に対応する距離画像センサ
 * - RightCamera
   - RangeCamera
   - 右目に対応する距離画像センサ
 * - LeftAnkleForceSensor
   - ForceSensor
   - 左足首に搭載された力センサ
 * - RightAnkleForceSensor
   - ForceSensor
   - 右足首に搭載された力センサ


デバイスに対する入出力
----------------------

Deviceオブジェクトを介した入出力は、以下のようにして行います。

* **入力**

 対応するDeviceオブジェクトのメンバ関数を用いて値を取得する。

* **出力**

 対応するDeviceオブジェクトのメンバ関数を用いて値を設定した後、Deviceオブジェクトの "notifyStateChange()" 関数を実行する。

これらを行うためには、使用するデバイスのクラス定義を知っている必要があります。例えば加速度センサのクラスである"AccelerationSensor"に関しては、その状態にアクセスするための"dv()"というメンバ関数があります。これは加速度をVector3型の3次元ベクトルで返します。

従って、加速度センサ waistAccelSensor の加速度は、 ::

 Vector3 dv = waistAccelSensor->dv();

といったかたちで取得できます。

同様に、ForceSensorやRateGyroSensorに関しても該当するメンバ関数を用いて状態の入力を行うことが可能です。

カメラやレンジセンサ等の視覚センサを使用する際には、そのための準備が必要になります。これについては :doc:`vision-simulation` で解説します。

デバイスへの出力については、ライトのオン・オフを行う "TankJoystickLight.cnoid" というサンプルを参考にしてください。

.. 立たせるのもやめて、倒れるシミュレーションにして、加速度が一定値以上のときだけ表示するようなサンプルを作る？


リンク位置姿勢の入出力
----------------------

コントローラの入出力の対象としては、他にリンクの位置姿勢があります。ここで言う位置姿勢というのは関節角度のことではなく、リンクという剛体そのもののグローバル座標における位置と姿勢を意味します。この値は通常ロボット実機に対して入出力を行うことはできません。空間中に固定されていないロボットに対して、あるリンクの正確な位置と姿勢を知ることは（かなり性能のよいモーションキャプチャでも無ければ）困難ですし、あるリンクの位置姿勢をコントローラからの出力で直接変えることは物理的に不可能です。しかしながら、シミュレーションにおいてはそのようなことも可能となるため、シミュレーション限定での利用を想定してこの値の入出力機能も備えています。

これを行うためには、ioオブジェクトに対してsetLinkInput, setLinkOutput関数を用いて状態値シンボル"LINK_POSITION"を指定します。すると対象のLinkオブジェクトを介してそのリンクの位置姿勢の入出力を行うことが可能となります。

Linkオブジェクトにおいて、その位置姿勢はPosition型の値として格納されています。これはChoreonoidの実装に用いているEigenという行列・ベクトルライブラリの"Transform"型をカスタマイズしたもので、基本的には３次元の同次座標変換行列を格納したものとなっています。この値にはLinkクラスの以下のような関数を用いてアクセスできます。

* **Position& T(), Position& position()**

 位置姿勢に対応するPosition値への参照を返します。

* **Position::TranslationPart translation()**

 位置成分に対応する３次元ベクトルを返します。

* **void setTranslation(const Eigen::MatrixBase<Derived>& p)**
   
 位置成分を設定します。引数はEigenの3次元ベクトル相当の型が使えます。

* **Position::LinearPart rotation()**

 姿勢（回転）成分に対応する3x3行列を返します。

* **setRotation(const Eigen::MatrixBase<Derived>& R)**

 姿勢（回転）成分を設定します。引数はEigenの3x3行列相当の型が使えます。

* **setRotation(const Eigen::AngleAxis<T>& a)**

 姿勢（回転）成分を設定します。引数は回転軸と回転角度で回転を表現するEigenのAngleAxis型になります。


例として、ルートリンクの位置を入力する場合は、まずSimpleControllerのinitialize関数にて ::

  io->setLinkInput(io->body()->rootLink(), LINK_POSITION);

などとします。そして control 関数にて ::

 Position T = io->body()->rootLink()->position();
 Vector3 p = T.translation();
 Matrix3 R = T.rotation();

などとすることにより、ルートリンクの位置姿勢を取得できます。

リンク位置姿勢の出力については、これをサポートしたシミュレータが必要で、特殊な利用形態となります。例えばAISTシミュレータアイテムでは、「動力学モード」を「運動学」にすると、シミュレーションにおいて動力学計算を行わず、与えた位置姿勢を再現するだけのモードとなります。この場合、ロボットのルートリンクの位置姿勢を出力することで、ルートリンクがその位置姿勢へ移動します。また、関節角も出力しておけば、ルートリンクからの順運動学の結果となる姿勢が再現されます。


その他のサンプル
----------------
 
Choreonoidでは、本節で挙げた以外にも様々なコントローラのサンプルを用意しています。それらを用いたプロジェクトが :ref:`basics_sample_project` に挙げてありますので、参考にしていただければと思います。
