ODEPluginの解説
===================

Choreonoidでは、Plugin形式で、ユーザが開発したプログラムを組み込むことができることを、:doc:`hello-world-sample` や :doc:`sample1` にて説明しました。ここでは、シミュレーションの核となる動力学エンジンを組み込む方法を解説します。説明には、一般的な動力学エンジンODE（Open Dynamics Engine)をPlugin化したODEPluginを使用します。ソースファイルは "src/ODEPlugin" にあります。ODEの詳細は `http://www.ode.org/ <http://www.ode.org/>`_ をご参照ください。

.. contents:: 目次
   :local:


ODEプラグイン
-------------

.. highlight:: cpp

まずは、Pluginの基本構造のソースコードです。（サンプルのソースファイルはGAZEBO用のコードと共有しているため、以下の内容とは異なる部分があります。また、本解説では、解説しない機能を含んでいますので、その部分を省略して掲載しています） ::

 #include "ODESimulatorItem.h"
 #include <cnoid/Plugin>
 #include <ode/ode.h>
 #define PLUGIN_NAME "ODE"

 class ODEPlugin : public Plugin
 {
 public:
     ODEPlugin() : Plugin(PLUGIN_NAME)
     {
         require("Body");
     }
     virtual bool initialize()
     {
        ............
     }

     virtual bool finalize()
     {
        ...............
     }
 };
 CNOID_IMPLEMENT_PLUGIN_ENTRY(ODEPlugin);

ロボットモデルを扱うので、コンストラクタで、BodyPluginが必要であることを示しています。また、ODEのヘッダーファイルをincludeしています。

プラグインの初期化
~~~~~~~~~~~~~~~~~~

プラグインがメモリに読み込まれた後、一度だけ実行される初期化関数です。 ::

 virtual bool initialize()
 {
     dInitODE2(0);
     dAllocateODEDataForThread(dAllocateMaskAll);

     ODESimulatorItem::initializeClass(this);
             
     return true;
 }

ODEの初期化関数を呼び出しています。（関数名の先頭に’d’が付いているのは、ODEの関数です。）ODESimulatorItemは後で詳しく解説します。ここでは、その初期化関数を呼び出します。

プラグインの終了
~~~~~~~~~~~~~~~~~

プラグインが破棄される時（Choreonoidが終了するとき）に、一度だけ実行される関数です。 ::

 virtual bool finalize()
 {
     dCloseODE();
     return true;
 }

ODEの終了関数を呼び出しています。
これで、Pluginとしての形が整いました。

ODEシミュレータアイテム
-----------------------

動力学エンジンは、Choreonoidがアイテムとして扱えるように、 :ref:`simulation_simulator_item` として実装します。ODEPluginでは、ODEをChoreonoidで使うための ODESimulatorItem を実装しています。

ODESimulatorItemはSimulatorItemを継承したクラスとして定義します。これを行っているヘッダーファイルを以下に示します。::

 #include <cnoid/SimulatorItem>
 #include "exportdecl.h"

 namespace cnoid {
         
 class CNOID_EXPORT ODESimulatorItem : public SimulatorItem
 {
 public:
     static void initializeClass(ExtensionManager* ext);
    ..........................
 };
 }

プラグインの初期化時に呼び出されるスタティックな初期化関数です。アイテムを管理するitemManagerにODESimulatorItemを登録し、メニューからODESimulatorItemを作成できるようにしています。 ::

 void ODESimulatorItem::initializeClass(ExtensionManager* ext)
 {
     ext->itemManager().registerClass<ODESimulatorItem>(ITEM_NAME);
     ext->itemManager().addCreationPanel<ODESimulatorItem>();
 }

アイテムにODEシミュレータアイテムが追加されると、ODESimulatorItemクラスのオブジェクトが生成されます。コンストラクタでは、ユーザが変更可能なパラメータの初期値を設定したり、変数の初期化を行います。 ::

 ODESimulatorItem::ODESimulatorItem()
 {
     initialize();
     stepMode.setSymbol(ODESimulatorItem::STEP_ITERATIVE,  N_("Iterative (quick step)"));
     gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
     .............
 }

doDuplicate関数は、ODEシミュレータアイテムを新規に作成した場合に呼び出されます。新たなオブジェクトを作成し、そのポインタを返すように実装してください。 ::

 ItemPtr ODESimulatorItem::doDuplicate() const
 {
     return new ODESimulatorItem(*this);
 }

GUIでODEシミュレータアイテムが削除されると、ODESimulatorItemクラスのオブジェクトも破棄されます。デストラクタで、必要に応じてメモリの開放などを行ってください。 ::

 ODESimulatorItem::~ODESimulatorItem()
 {
     clear();
     if(contactJointGroupID){
         dJointGroupDestroy(contactJointGroupID);
     }
 }

プロパティビューにパラメータを表示するとき、またパラメータの値を変更した時に呼ばれる関数です。

.. code-block:: cpp
    :emphasize-lines: 4,7

    void ODESimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
    {
        SimulatorItem::doPutProperties(putProperty);
        //シミュレータアイテム共通のプロパティを設定しますので、必ず呼び出してください。
     
        putProperty(_("Step mode"), stepMode, changeProperty(stepMode));
        //パラメータ設定を行う関数です。パラメータの名前、変数、呼び出す関数を指定します。
    }

プロジェクトファイルにパラメータ設定を保存するための関数です。

.. code-block:: cpp
    :emphasize-lines: 4,7,10

    bool ODESimulatorItem::store(Archive& archive)
    {
        SimulatorItem::store(archive);
        //シミュレータアイテム共通のプロパティを保存しますので、必ず呼び出してください。
    
        archive.write("stepMode", stepMode.selectedSymbol());
        //保存するパラメータの名前、変数を指定します。
    
        write(archive, "gravity", gravity);
        //Vector型の変数は、この関数を使用します。
    }

プロジェクトファイルからパラメータ設定を読み出すための関数です。

.. code-block:: cpp
    :emphasize-lines: 4,7,10

    bool ODESimulatorItem::restore(const Archive& archive)
    {
        SimulatorItem::restore(archive);
        //シミュレータアイテム共通のプロパティを読み出しますので、必ず呼び出してください。

        archive.read("friction", friction);
        //読み出すパラメータの名前、変数を指定します。

        read(archive, "gravity", gravity);
        //Vector型の変数は、この関数を使用します。
    }

シミュレーションの実装
~~~~~~~~~~~~~~~~~~~~~~

次は、シミュレーションの中心部分の実装です。まずは、全体の流れを解説します。

ユーザがシミュレーションの開始ボタンを押すと、まずは、ODEモデルの作成を行う関数createSimulationBodyが、シミュレーション対象となるモデルの個数回呼ばれます。

動力学エンジンの多くは、それぞれ独自のモデルの記述方法を持っています。ODEもそうです。Choreonoidでは、ロボットや環境をBodyオブジェクトとして保持しています。これらのBodyオブジェクトからODE用のモデルを構築する必要があります。

引数orgBodyには、Bodyオブジェクトのポインタが入っていますので、これからODE用のODEBodyオブジェクトを作成し、そのポインタを返します。ここでは、まだODE用モデルの実体は作成していません。 ::

 SimulationBodyPtr ODESimulatorItem::createSimulationBody(BodyPtr orgBody)
 {
     return new ODEBody(*orgBody);
 }

ODEBodyクラスは、SimulationBodyクラスを継承して作成します。 ::

 class ODEBody : public SimulationBody
 {
 public:
     ..................
 }
 
 ODEBody::ODEBody(const Body& orgBody)
     : SimulationBody(new Body(orgBody))
 {
    worldID = 0;
    ...............
 }

次に初期化関数が一度だけ呼び出されます。引数simBodiesには、シミュレーション対象とする上で作成したODEBodyオブジェクトへのポインタが入っています。

.. code-block:: cpp
    :emphasize-lines: 4,10,13,18

    bool ODESimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
    {
         clear();
         //前回のシミュレーションの結果を破棄します。
    
         dRandSetSeed(0);
         dWorldSetGravity(worldID, g.x(), g.y(), g.z());
         dWorldSetERP(worldID, globalERP);
         .............
         //シミュレーション用パラメータを設定します。

         timeStep = self->worldTimeStep();
         //worldTimeStep()で、シミュレーションの刻み時間が取得できます。

         for(size_t i=0; i < simBodies.size(); ++i){
             addBody(static_cast<ODEBody*>(simBodies[i]));
         }
        //シミュレーションの世界にODEの用のモデルを構築します。対象モデルの個数回addBodyを呼び出してモデルを追加していきます。

         return true;
     }

その後は、シミュレーションを１ステップ進める関数が、シミュレーション終了まで、繰り返し呼び出されます。引数activeSimBodiesには、シミュレーション対象とするODEBodyオブジェクトへのポインタが入っています。

.. code-block:: cpp
    :emphasize-lines: 6,9,14,21,34
    
    bool ODESimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
    {
        for(size_t i=0; i < activeSimBodies.size(); ++i){
            ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);
            odeBody->body()->setVirtualJointForces();
            //BodyCustomizerの関数を呼び出します。

            odeBody->setTorqueToODE();
            //各ODEBodyオブジェクトに関節トルクを設定します。
        }
    
        dJointGroupEmpty(contactJointGroupID);
        dSpaceCollide(spaceID, (void*)this, &nearCallback);
        //衝突検出を行います。

        if(stepMode.is(ODESimulatorItem::STEP_ITERATIVE)){
            dWorldQuickStep(worldID, timeStep);
        } else {
            dWorldStep(worldID, timeStep);
        }
        //シミュレーションの時間を１ステップ進めます。

        for(size_t i=0; i < activeSimBodies.size(); ++i){
            ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);

            if(!odeBody->sensorHelper.forceSensors().empty()){
                odeBody->updateForceSensors(flipYZ);
            }
            odeBody->getKinematicStateFromODE(flipYZ);
            if(odeBody->sensorHelper.hasGyroOrAccelSensors()){
                odeBody->sensorHelper.updateGyroAndAccelSensors();
            }
        }
        //１ステップ進んだ結果を、各ODEBodyオブジェクトから読み込みます。

        return true;
    }

.. note:: 上にodeBody->body()->setVirtualJointForces()という記述があります。これは、BodyCustomizerと呼んでいる仕組みで、これを使用すると、モデル固有のプログラムを、動的に動力学計算ライブラリに組み込むことができます。このサンプルのプロジェクトが、CustomizedSpringModel.cnoidです。サンプルプログラムがsample/SpringModel/SpringModelCustomizer.cppです。このサンプルの解説が、OpenHRP３のホームページの `関節のバネダンパモデル化の方法 <http://www.openrtp.jp/openhrp3/jp/springJoint.html>`_ にありますので、参考にしてください。


BodyクラスとLinkクラス
~~~~~~~~~~~~~~~~~~~~~~~~

次にODEモデルの構築について解説する前に、Choreonoid内で物理的な物体を記述するためのBodyクラスとLinkクラスについて解説します。（VRMLモデルの記述方法については、OpenHRP３のホームページの `ロボット・環境モデル記述形式 <http://www.openrtp.jp/openhrp3/jp/create_model.html>`_ をご覧ください。）

Bodyオブジェクトは、木構造をなすLinkオブジェクトを管理しています。床のような環境モデルも、一つのLinkオブジェクトからなるBodyオブジェクトです。Bodyオブジェクトは必ず木構造の根であるルートリンクを持っています。

Bodyクラスは以下の関数を提供します。

.. list-table:: Bodyクラスの関数
   :widths: 30 60
   :header-rows: 1

   * - 関数
     - 機能
   * - int numJoints()
     - 全関節数を返します。
   * - Link* joint(int id) 
     - 関節idに対応するLinkオブジェクトのポインタを返します。
   * - int numLinks() 
     - 全リンク数を返します。
   * - Link* link(int index)
     - リンクidに対応するLinkオブジェクトのポインタを返します。
   * - Link* link(const std::string& name)
     - link名が一致するLinkオブジェクトのポインタを返します。
   * - Link* rootLink()
     - ルートリンクのポインタを返します。
   * - int numDevices()
     - 全デバイス数を返します。　Deviceクラスは力センサなどを記述するための親クラスです。
   * - Device* device(int index)
     - デバイスidに対応するDeviceオブジェクトを返します。
   * - template<class DeviceType> DeviceList<DeviceType> devices()
     - | デバイスリストを返します。
       | 例えば、力センサのデバイスリストを得るためには次の様にします。
       | DeviceList<ForceSensor> forceSensors = body->devices();
   * - template<class DeviceType> DeviceType* findDevice(const std::string& name)
     - デバイス名が一致するDeviceオブジェクトのポインタを返します。
   * - void initializeDeviceStates()
     - 全デバイスを初期状態にします。
   * - bool isStaticModel()
     - 床や壁など、動かない物体のときtrueを返します。
   * - bool isFixedRootModel()
     - ルートリンクが固定関節のときtrueを返します。
   * - double mass()
     - 全質量を返します。
   * - const Vector3& centerOfMass() const;
     - 重心ベクトルを返します
   * - void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false)
     - | 順運動学（ルートリンクの位置姿勢と全関節の角度からルートリンク以外のリンクの位置姿勢）を計算します。
       | calcVelocity,calcAccelerationをtrueにすると、関節角速度、角加速度からリンクの速度、加速度を計算します。
   * - void clearExternalForces()
     - 外力を０に設定します。
   * - numExtraJoints()
     - 仮想関節数を返します。
   * - ExtraJoint& extraJoint(int index)
     - 仮想関節idに対応する仮想関節を返します。


Linkクラスは以下の関数を提供します。

.. list-table:: Linkクラスの関数
   :widths: 30 60
   :header-rows: 1

   * - 関数
     - 機能
   * - Link* parent()
     - 親リンクのポインタを返します。
   * - Link* sibling()
     - 兄弟リンクのポインタを返します。
   * - Link* child()
     - 子リンクのポインタを返します。
   * - bool isRoot()
     - ルートリンクならばtrueを返します。
   * - | Position& T()
       | Position& position()
     - ワールド座標からみたリンク原点の位置姿勢行列の参照を返します。
   * - Position::TranslationPart p()
     - ワールド座標からみたリンク原点の位置ベクトルの参照を返します。
   * - Position::LinearPart R()
     - ワールド座標からみたリンクの姿勢行列の参照を返します。
   * - Position::ConstTranslationPart b()
     - 親リンク座標からみたリンク原点の位置ベクトルを返します。
   * - int jointId()
     - 関節idを返します。
   * - JointType jointType()
     - 関節の種類を返します。回転、並進、フリー、固定、（クローラ）があります。
   * - bool isFixedJoint()
     - 固定関節のときtrueを返します。
   * - bool isFreeJoint()
     - フリー関節のときtrueを返します。
   * - bool isRotationalJoint()
     - 回転関節のときtrueを返します。
   * - bool isSlideJoint()
     - 並進関節のときtrueを返します。
   * - | const Vector3& a()
       | const Vector3& jointAxis()
     - 回転関節の回転軸ベクトルを返します。
   * - const Vector3& d()
     - 並進関節の並進方向ベクトルを返します。
   * - double& q()
     - 関節角度の参照を返します。
   * - double& dq() 
     - 関節角速度の参照を返します。
   * - double& ddq() 
     - 関節角加速度の参照を返します。
   * - double& u() 
     - 関節トルクの参照を返します。
   * - const double& q_upper()
     - 関節稼働角の上限の参照を返します。
   * - const double& q_lower() 
     - 関節稼働角の下限の参照を返します。
   * - Vector3& v() 
     - ワールド座標からみたリンク原点の速度ベクトルの参照を返します。
   * - Vector3& w()
     - ワールド座標からみたリンク原点の角速度ベクトルの参照を返します。
   * - Vector3& dv()
     - ワールド座標からみたリンク原点の加速度ベクトルの参照を返します。
   * - Vector3& dw()
     - ワールド座標からみたリンク原点の角加速度ベクトルの参照を返します。
   * - | const Vector3& c()
       | const Vector3& centerOfMass()
     - 自リンク座標からみた重心ベクトルの参照を返します。
   * - | const Vector3& wc() 
       | const Vector3& centerOfMassGlobal() 
     - ワールド座標からみた重心ベクトルの参照を返します。
   * - | double m() 
       | double mass() 
     - 質量を返します。
   * - const Matrix3& I()
     - 自リンク座標からみた重心周りの慣性テンソル行列の参照を返します。
   * - const std::string& name()
     - リンク名の参照を返します。
   * - SgNode* shape()
     - リンクの形状オブジェクトのポインタを返します。
   * - Matrix3 attitude() 
     - ワールド座標からみたリンクの姿勢行列を返します。（オフセットあり）

.. note:: Choreonoidでは、各リンクの位置と姿勢を表すローカル座標系を次のように設定しています。座標原点は関節軸中心です。関節角がすべて０度の時の姿勢で、姿勢行列はワールド座標系と並行です。しかし、ロボットの構造によっては、ローカル座標系の姿勢にオフセットを持たせた方が便利な場合もあります。VRMLファイル上でのモデルの記述では、オフセットの設定が可能です。Choreonoidでは、オフセットが設定されていても、モデルファイルを読み込む時に、ローカル座標系を上記の様に変更する処理を行います。上の関数で得られるデータは、変更後の座標系で表したものです。ただし、attitude()関数で得られる姿勢行列は、変更前の座標系で表したものとなります。


ODEモデルの構築
~~~~~~~~~~~~~~~
次に、ODEモデルの構築について、詳しく解説していきます。

createSimulationBody関数が呼ばれたときには、ODEBodyオブジェクトを作成していますが、入れ物を用意しているだけで、まだ実体はありません。initializeSimulationの中で、addBodyが呼ばれた時に、実体を作成します。

addBodyのソースコードです。

.. code-block:: cpp
    :emphasize-lines: 4,7,12,20,21,24,26,29

    void ODESimulatorItemImpl::addBody(ODEBody* odeBody)
    {
         Body& body = *odeBody->body();
         //Bodyオブジェクトへのポインタを取得します。

         Link* rootLink = body.rootLink();
         //ルートリンクのポインタを取得します。
         rootLink->v().setZero();
         rootLink->dv().setZero();
         rootLink->w().setZero();
         rootLink->dw().setZero();
         //ルートリンクの速度、加速度、角速度、角加速度を０に設定しています。
    
         for(int i=0; i < body.numJoints(); ++i){
             Link* joint = body.joint(i);
             joint->u() = 0.0;
             joint->dq() = 0.0;
             joint->ddq() = 0.0;
         }
         //各関節のトルク、角速度、角加速度も０に設定しています。
         //ルートリンクの位置、姿勢、各関節の角度にはシミュレーションの初期値が設定されています。
         
         body.clearExternalForces();
         //外力を０にします。
         body.calcForwardKinematics(true, true);
         //各リンクの位置と姿勢を計算します。

         odeBody->createBody(this);
         //ODEのモデルの作成を行います。
     }

createBodyのソースコードです。

.. code-block:: cpp
    :emphasize-lines: 4,7,11,14,15,18,21,24,32

    void ODEBody::createBody(ODESimulatorItemImpl* simImpl)
    {
        Body* body = this->body();
        //Bodyオブジェクトのポインタを取得します。
    
        worldID = body->isStaticModel() ? 0 : simImpl->worldID;
        //モデルが床など、動かない物体か否かを判断し、その扱いを変えることができます。
    
        spaceID = dHashSpaceCreate(simImpl->spaceID);
        dSpaceSetCleanup(spaceID, 0);
        //ODEの準備です。

        ODELink* rootLink = new ODELink(simImpl, this, 0, Vector3::Zero(), body->rootLink());
        //モデルのルートのリンク（物体）を作成します。ルートリンクから手先、足先へとたどって、全体を構成します。
        //ルートリンクには親リンクがないので、親リンクのポインタは０を、位置はゼロベクトルを渡します。

        setKinematicStateToODE(simImpl->flipYZ);
        //ODEBodyオブジェクトに、位置姿勢を設定します。

        setExtraJoints(simImpl->flipYZ);
        //仮想関節を設定します。
       
        setTorqueToODE();
        //ODEBodyオブジェクトにトルクを設定します。

        sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);
        const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
        forceSensorFeedbacks.resize(forceSensors.size());
        for(size_t i=0; i < forceSensors.size(); ++i){
            dJointSetFeedback(odeLinks[forceSensors[i]->link()->index()]->jointID, &forceSensorFeedbacks[i]);
        }
        //力センサなど、センサ出力用の初期設定を行います。
    
    }

ODELinkのソースコードです。Linkオブジェクトの情報からODELinkオブジェクトを生成します。

.. code-block:: cpp
    :emphasize-lines: 7,12,15,20

    ODELink::ODELink
    (ODESimulatorItemImpl* simImpl, ODEBody* odeBody, ODELink* parent, const Vector3& parentOrigin, Link* link)
    {
        ...................
    
        Vector3 o = parentOrigin + link->b();
        //ワールド座標系からみたリンク原点位置ベクトルを計算します。parentOriginは親リンクの位置ベクトルです。
    
        if(odeBody->worldID){
            createLinkBody(simImpl, odeBody->worldID, parent, o);
        }
        //物理データを設定します。ODEでは動かない物体は物理データは必要ないので設定しません。
        
        createGeometry(odeBody);
        //形状データを設定します。
    
        for(Link* child = link->child(); child; child = child->sibling()){
            new ODELink(simImpl, odeBody, this, o, child);
        }
        //子リンクを順番にたどり、ODELinkを作成します。
    }

ODEの物理データを設定するcreateLinkBodyのソースコードです。

.. code-block:: cpp
    :emphasize-lines: 4,14,19,23,28,31,34,37,40,44,47,50,54,59,61,67,71

    void ODELink::createLinkBody(ODESimulatorItemImpl* simImpl, dWorldID worldID, ODELink* parent, const Vector3& origin)
    {
        bodyID = dBodyCreate(worldID);
        //ODEの物体（ODEではBodyと表現されます。ChoreonoidではLinkに相当します）を生成します。
    
        dMass mass;
        dMassSetZero(&mass);
        const Matrix3& I = link->I();
        dMassSetParameters(&mass, link->m(),
                           0.0, 0.0, 0.0,
                           I(0,0), I(1,1), I(2,2),
                           I(0,1), I(0,2), I(1,2));
        dBodySetMass(bodyID, &mass);
        //質量と慣性テンソル行列を設定します。

        ................
    
        dBodySetRotation(bodyID, identity);
        //リンクの姿勢を設定します。
        
        Vector3 p = o + c;
        dBodySetPosition(bodyID, p.x(), p.y(), p.z());
        //リンクの位置を設定します。ODEでは重心をリンク原点とします。

        dBodyID parentBodyID = parent ? parent->bodyID : 0;

        switch(link->jointType()){
        //関節の種類によって、使用するODEの関節を変えます。
        
            case Link::ROTATIONAL_JOINT:
            //回転関節の場合ヒンジジョイントを使います。
            jointID = dJointCreateHinge(worldID, 0);
            dJointAttach(jointID, bodyID, parentBodyID);
            //親リンクと自リンクをつなぎます。
        
            dJointSetHingeAnchor(jointID, o.x(), o.y(), o.z());
            //ヒンジジョイントの位置はLinkオブジェクトの原点になります。
        
            dJointSetHingeAxis(jointID, a.x(), a.y(), a.z());
            //ヒンジジョイントの回転軸を設定します。
            break;
        
            case Link::SLIDE_JOINT:
            //並進関節の場合はスライダジョイントを使います。
            jointID = dJointCreateSlider(worldID, 0);
            dJointAttach(jointID, bodyID, parentBodyID);
            //親リンクと自リンクをつなぎます。
        
            dJointSetSliderAxis(jointID, d.x(), d.y(), d.z());
            //スライドジョイントのスライド軸を設定します。
            break;

            case Link::FREE_JOINT:
            //フリー関節の場合は、何も設定しません。
            break;

            case Link::FIXED_JOINT:
            default:
            //上記以外、または固定関節の場合は
            if(parentBodyID){
                //親リンクがあれば、親リンクに固定ジョイントで接続します。
                jointID = dJointCreateFixed(worldID, 0);
                dJointAttach(jointID, bodyID, parentBodyID);
                dJointSetFixed(jointID);
                if(link->jointType() == Link::CRAWLER_JOINT){
                    simImpl->crawlerLinks.insert(make_pair(bodyID, link));
                    //クローラ関節は、ODEでは固定ジョイントとし、衝突検出で特殊なケースとして扱います。
                }
            } else {
                dBodySetKinematic(bodyID);
                //親リンクがない場合は、KinematicBody（衝突が起きても動かない物体）と設定します。
            }
            break;
        }
    }

次に形状データを設定するcreateGeometryのソースコードです。形状データは、Shapeオブジェクト内で、階層構造で記述されています。

.. code-block:: cpp
    :emphasize-lines: 4,7,10,11,18,21,24
    
    void ODELink::createGeometry(ODEBody* odeBody)
    {
        if(link->shape()){
        //Shapeオブジェクトを取得します。
        
            MeshExtractor* extractor = new MeshExtractor;
            //MeshExtractorは、階層をたどり、形状データを展開するためのユーティリティクラスです。
            
            if(extractor->extract(link->shape(), boost::bind(&ODELink::addMesh, this, extractor, odeBody))){
            //階層をたどり、Meshオブジェクトを見つける度に、ODELink::addMeshを呼び出すように指定します。
            //extractの呼び出しから戻ると、三角メッシュ形状は、verticesにデータが集められています。
            
                if(!vertices.empty()){
                    triMeshDataID = dGeomTriMeshDataCreate();
                    dGeomTriMeshDataBuildSingle(triMeshDataID,
                                            &vertices[0], sizeof(Vertex), vertices.size(),
                                            &triangles[0],triangles.size() * 3, sizeof(Triangle));
                    //ODEのデータ形式に変換します。
                    
                    dGeomID gId = dCreateTriMesh(odeBody->spaceID, triMeshDataID, 0, 0, 0);
                    //ODEの三角メッシュオブジェクトを生成します。
                    geomID.push_back(gId);
                    dGeomSetBody(gId, bodyID);
                    //ODEのBodyと結びつけます。
                }
            }
            delete extractor;
        }
    }

Choreonoidでは、モデルを読み込む時に、形状データは全て三角メッシュ形状に変換しますが、元の形状がプリミティブ型の場合には、その情報も保存されています。次のコードでは、ODEが対応できるプリミティブ型は、そのまま使用し、できない型は三角メッシュ型として作成しています。

addMeshのソースコードです。

.. code-block:: cpp
    :emphasize-lines: 4,7,12,13,19,22,24,28,29,32,36,39,42,46,49,52,55,60,67,73,79,89,92,96,99,107,109,118,121,124,129,131,135,138,144

    void ODELink::addMesh(MeshExtractor* extractor, ODEBody* odeBody)
    {
        SgMesh* mesh = extractor->currentMesh();
        //Meshオブジェクトのポインタを取得します。

        const Affine3& T = extractor->currentTransform();
        //Meshオブジェクトの位置姿勢行列が取得できます。

        bool meshAdded = false;

        if(mesh->primitiveType() != SgMesh::MESH){
            //mesh->primitiveType()で形状データのタイプが取得できます。MESH, BOX, SPHERE, CYLINDER, CONEがあります。
            //以下、形状データがプリミティブ型のときの処理です。

            bool doAddPrimitive = false;
            Vector3 scale;
            optional<Vector3> translation;
            if(!extractor->isCurrentScaled()){
            //スケールの変更がある場合trrueを返します。
                scale.setOnes();
                doAddPrimitive = true;
                //スケール変更がない場合はscaleベクトルの各要素は１とし、プリミティブ型として扱います。
            } else {
                //スケールの変更がある場合の処理です。

                Affine3 S = extractor->currentTransformWithoutScaling().inverse() *
                    extractor->currentTransform();
                //currentTransformWithoutScaling()でスケール変換行列を含まない座標変換行列が取得できます。
                //スケール変換の行列だけを抽出します。

                if(S.linear().isDiagonal()){
                    //スケール変換行列が対角行列のときだけ処理します。そうでない時はODEではプリミティブ型として扱えません。

                    if(!S.translation().isZero()){
                        translation = S.translation();
                        //スケール行列の中に位置変換がある場合は保存します。
                    }
                    scale = S.linear().diagonal();
                    //対角要素をscaleに代入します。

                    if(mesh->primitiveType() == SgMesh::BOX){
                        //プリミティブ型がBoxならば、プリミティブ型として扱います。
                        doAddPrimitive = true;
                    } else if(mesh->primitiveType() == SgMesh::SPHERE){
                        if(scale.x() == scale.y() && scale.x() == scale.z()){
                            //プリミティブ型がSphere、かつscaleの要素が同じ値ならばプリミティブ型として扱います。
                            doAddPrimitive = true;
                        }
                        //scaleの要素が同じ値でないならばプリミティブ型として扱えません。
                    } else if(mesh->primitiveType() == SgMesh::CYLINDER){
                        if(scale.x() == scale.z()){
                            //プリミティブ型がCylinder、かつscaleのx,z要素が同じ値ならばプリミティブ型として扱います。
                            doAddPrimitive = true;
                        }
                        //scaleのx,z要素が同じ値でないならばプリミティブ型として扱えません。
                    }
                }
            }
            if(doAddPrimitive){
                //プリミティブ型として扱う場合の処理です。ODEのプリミティブオブジェクトを生成します。

                bool created = false;
                dGeomID geomId;
                switch(mesh->primitiveType()){
                case SgMesh::BOX : {
                    const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                    //Boxのサイズが取得できます。
                    geomId = dCreateBox(odeBody->spaceID, s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
                    created = true;
                    break; }
                case SgMesh::SPHERE : {
                    SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                    //Sphereの半径が取得できます。
                    geomId = dCreateSphere(odeBody->spaceID, sphere.radius * scale.x());
                    created = true;
                    break; }
                case SgMesh::CYLINDER : {
                    SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                    //シリンダーのパラメータが取得できます。
                    geomId = dCreateCylinder(odeBody->spaceID, cylinder.radius * scale.x(), cylinder.height * scale.y());
                    created = true;
                    break; }
                default :
                    break;
                }
                if(created){
                    geomID.push_back(geomId);
                    dGeomSetBody(geomId, bodyID);
                    //ODEのプリミティブオブジェクトとODEのBodyを結びつけます。
                
                    Affine3 T_ = extractor->currentTransformWithoutScaling();
                    //スケール分を取り除いた変換行列を取得します。
                
                    if(translation){
                        T_ *= Translation3(*translation);
                        //スケール行列に含まれていた位置変換をかけます。
                    }
                    Vector3 p = T_.translation()-link->c();
                    //ODEではリンク原点は重心なので、その分を補正します。
                
                    dMatrix3 R = { T_(0,0), T_(0,1), T_(0,2), 0.0,
                                   T_(1,0), T_(1,1), T_(1,2), 0.0,
                                   T_(2,0), T_(2,1), T_(2,2), 0.0 };
                    if(bodyID){
                        dGeomSetOffsetPosition(geomId, p.x(), p.y(), p.z());
                        dGeomSetOffsetRotation(geomId, R);
                        //形状データの位置姿勢を設定します。
                    }else{
                        //動かない物体の場合は位置姿勢行列とidを関連付けておきます。
                        offsetMap.insert(OffsetMap::value_type(geomId,T_));
                    }
                    meshAdded = true;
                }
            }
        }

        if(!meshAdded){
            //元からプリミティブ型でない、またはプリミティブ型として扱えない場合の処理です。

            const int vertexIndexTop = vertices.size();
            //既に追加されている頂点座標の数を取得します。

            const SgVertexArray& vertices_ = *mesh->vertices();
            //Meshオブジェクト内の頂点座標の参照を取得します。
        
            const int numVertices = vertices_.size();
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Position::Scalar>() - link->c();
                //頂点ベクトルを座標変換します。
                vertices.push_back(Vertex(v.x(), v.y(), v.z()));
                //ODELinkオブジェクト内の頂点座標verticesに追加します。
            }

            const int numTriangles = mesh->numTriangles();
            //Meshオブジェクト内の三角形の総数を取得します。
            for(int i=0; i < numTriangles; ++i){
                SgMesh::TriangleRef src = mesh->triangle(i);
                //Meshオブジェクト内のi番目の三角形の頂点番号を取得します。
                Triangle tri;
                tri.indices[0] = vertexIndexTop + src[0];
                tri.indices[1] = vertexIndexTop + src[1];
                tri.indices[2] = vertexIndexTop + src[2];
                triangles.push_back(tri);
                //ODELinkオブジェクト内の三角形頂点番号に追加します。
            }
        }
    }

これで、ODEのモデルの構築は終了です。

次にODEのモデルとデータの受け渡しをする関数を説明します。ODEのBodyオブジェクトの位置姿勢、速度を設定するsetKinematicStateToODEのソースコードです。

.. code-block:: cpp
    :emphasize-lines: 4,7,14,18,21,25,29,32,35,41,49

    void ODELink::setKinematicStateToODE()
    {
        const Position& T = link->T();
        //リンクの位置姿勢行列を取得します。
    
        if(bodyID){
            //動く物体の場合の処理です。
        
            dMatrix3 R2 = { T(0,0), T(0,1), T(0,2), 0.0,
                            T(1,0), T(1,1), T(1,2), 0.0,
                            T(2,0), T(2,1), T(2,2), 0.0 };
    
            dBodySetRotation(bodyID, R2);
            //姿勢行列を設定します。
        
            const Vector3 lc = link->R() * link->c();
            const Vector3 c = link->p() + lc;
            //リンク原点を重心に変換します。
        
            dBodySetPosition(bodyID, c.x(), c.y(), c.z());
            //位置を設定します。
        
            const Vector3& w = link->w();
            const Vector3 v = link->v() + w.cross(lc);
            //リンク重心の速度を計算します。
        
            dBodySetLinearVel(bodyID, v.x(), v.y(), v.z());
            dBodySetAngularVel(bodyID, w.x(), w.y(), w.z());
            //速度と角速度を設定します。

        }else{
            //動かない物体の場合の処理です。形状データの位置を更新します。
            for(vector<dGeomID>::iterator it = geomID.begin(); it!=geomID.end(); it++){
                OffsetMap::iterator it0 = offsetMap.find(*it);
                //プリミティブ型の場合は、リンクローカル座標から見た位置姿勢行列がマッピングされているので、その行列を掛けます。
                Position offset(Position::Identity());
                if(it0!=offsetMap.end())
                    offset = it0->second;
                Position T_ = T*offset;
                Vector3 p = T_.translation() + link->c();
                //リンク原点を重心に変換します。
                
                dMatrix3 R2 = { T(0,0), T(0,1), T(0,2), 0.0,
                                T(1,0), T(1,1), T(1,2), 0.0,
                                T(2,0), T(2,1), T(2,2), 0.0 };

                dGeomSetPosition(*it, p.x(), p.y(), p.z());
                dGeomSetRotation(*it, R2);
                //形状データの位置姿勢情報を更新します。
            }
        }
    }

ODEのBodyオブジェクトにトルクを設定するsetTorqueToODEのソースコードです。

.. code-block:: cpp
    :emphasize-lines: 4,7

    void ODELink::setTorqueToODE()
    {
        if(link->isRotationalJoint()){
            //回転関節の場合です。
            dJointAddHingeTorque(jointID, link->u());
        } else if(link->isSlideJoint()){
            //並進関節の場合です。
            dJointAddSliderForce(jointID, link->u());
        }
    }


ODEのBodyオブジェクトから関節角度、角速度、リンク位置姿勢、速度を取得するgetKinematicStateFromODEのソースコードです。

.. code-block:: cpp
    :emphasize-lines: 4,6,10,17,23,28,31,34

    void ODELink::getKinematicStateFromODE()
    {
        if(jointID){
            //ジョインがある場合の処理です。
            if(link->isRotationalJoint()){
                //回転関節ならば、角度と角速度を取得します。
                link->q() = dJointGetHingeAngle(jointID);
                link->dq() = dJointGetHingeAngleRate(jointID);
            } else if(link->isSlideJoint()){
                //スライド関節ならば、位置と速度を取得します。
                link->q() = dJointGetSliderPosition(jointID);
                link->dq() = dJointGetSliderPositionRate(jointID);
            }
        }

        const dReal* R = dBodyGetRotation(bodyID);
        //ODEのBodyの姿勢行列を取得します。
    
        link->R() <<
            R[0], R[1], R[2],
            R[4], R[5], R[6],
            R[8], R[9], R[10];
        //Linkオブジェクトの姿勢行列に設定します。
    
        typedef Eigen::Map<const Eigen::Matrix<dReal, 3, 1> > toVector3;
        const Vector3 c = link->R() * link->c();
        link->p() = toVector3(dBodyGetPosition(bodyID)) - c;
        //ODEのBodyの位置を取得して、重心から関節位置に変換し、Linkオブジェクトの位置ベクトルに設定します。
    
        link->w() = toVector3(dBodyGetAngularVel(bodyID));
        //ODEのBodyの角速度を取得して、Linkオブジェクトの角速度ベクトルに設定します。
    
        link->v() = toVector3(dBodyGetLinearVel(bodyID)) - link->w().cross(c);
        //ODEのBodyの速度を取得して、関節位置の速度に変換し、Linkオブジェクトの速度ベクトルに設定します。
    }

衝突検出
------------

ODESimulatorItem::stepSimulation関数の中で、 ::

    dSpaceCollide(spaceID, (void*)this, &nearCallback);

という行があります。これは、衝突する可能性がある物体を探し、３番目の引数で指定したnearCallback関数を呼び出すODEの関数です。２番目の引数は、パラメータの受け渡しに使用します。ODEでは、このように衝突検出を行い、nearCallback関数の中で、接触する物体間に拘束力を生成します。 ここでは、ODEに関する詳しい説明は省略しますが、クローラリンクの扱いについて解説します。

nearCallback関数のソースコードです。　

.. code-block:: cpp
    :emphasize-lines: 6,10,19,27,30,32,34,37,46,48

    static void nearCallback(void* data, dGeomID g1, dGeomID g2)
    {
        ...............

        ODESimulatorItemImpl* impl = (ODESimulatorItemImpl*)data;
        //ODESimulatorItemImplの変数にアクセスできるようにします。

        ................
        if(numContacts > 0){
            //接触がある場合の処理です。
            dBodyID body1ID = dGeomGetBody(g1);
            dBodyID body2ID = dGeomGetBody(g2);
            Link* crawlerlink = 0;
            if(!impl->crawlerLinks.empty()){
                CrawlerLinkMap::iterator p = impl->crawlerLinks.find(body1ID);
                if(p != impl->crawlerLinks.end()){
                    crawlerlink = p->second;
                }
                //接触したリンクがクローラ型であるか否かを調べます。（今のところ、クローラリンク同士の接触は、想定していません。）
                ..............................
            }
            for(int i=0; i < numContacts; ++i){
                dSurfaceParameters& surface = contacts[i].surface;
                if(!crawlerlink){
                    surface.mode = dContactApprox1;
                    surface.mu = impl->friction;
                    //クローラリンクでない場合は、摩擦力を設定します。
                } else {
                    surface.mode = dContactFDir1 | dContactMotion1 | dContactMu2 | dContactApprox1_2;
                    //クローラリンクに対しては、摩擦１方向に表面速度を、摩擦２方向に摩擦力を設定します。
                    const Vector3 axis = crawlerlink->R() * crawlerlink->a();
                    //クローラリンクの回転軸ベクトルを計算します。
                    const Vector3 n(contacts[i].geom.normal);
                    //接触点の法線ベクトルを取得します。
                    Vector3 dir = axis.cross(n);
                    if(dir.norm() < 1.0e-5){
                        //この２つのベクトルが並行の時は、摩擦力だけ設定します。  
                        surface.mode = dContactApprox1;
                        surface.mu = impl->friction;
                    } else {
                        dir *= sign;
                        dir.normalize();
                        contacts[i].fdir1[0] = dir[0];
                        contacts[i].fdir1[1] = dir[1];
                        contacts[i].fdir1[2] = dir[2];
                        //２つのベクトルに対して垂直な方向を摩擦１方向の設定します。
                        surface.motion1 = crawlerlink->u();
                        //摩擦１方向に対して表面速度を設定します。

                ............................

センサ出力
-------------

次に力センサなどのセンサ出力について解説します。ロボットに取り付けられている加速度センサ、ジャイロ、力センサは、それぞれAccelSensorクラス、RateGyroSensorクラス、ForceSensorクラスで記述されています。BasicSensorSimulationHelperは、これらのセンサに関する処理をまとめたユーティリティクラスです。

モデルを構築するcreateBody関数の中の、センサに関する処理のソースコードです。

.. code-block:: cpp
    :emphasize-lines: 2,4,6,8,11,12
    
    sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);
    //初期化を行います。第２引数はシミュレーションの刻み時間、第３引数は重力ベクトルです。
    
    //そして、ODEから関節に係る力を取得するための設定をします。
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    //力センサオブジェクトのリストを取得します。
    forceSensorFeedbacks.resize(forceSensors.size());
    //力センサの個数分、データを格納する領域を確保します。
    for(size_t i=0; i < forceSensors.size(); ++i){
        dJointSetFeedback(odeLinks[forceSensors[i]->link()->index()]->jointID, &forceSensorFeedbacks[i]);
        //センサオブジェクトは、link()関数で、センサが取り付けられているリンクオブジェクトを返します。それから、ODEの関節idを取得します。
        //データの格納先をODEに対して指定します。
    
stepSimulation関数では、次の処理を行います。

.. code-block:: cpp
    :emphasize-lines: 6,13,14
    
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);

        if(!odeBody->sensorHelper.forceSensors().empty()){
            odeBody->updateForceSensors(flipYZ);
            //力センサがある場合には、updateForceSensorsクラスを呼び出します。
        }
        
        odeBody->getKinematicStateFromODE(flipYZ);
        
        if(odeBody->sensorHelper.hasGyroOrAccelSensors()){
            odeBody->sensorHelper.updateGyroAndAccelSensors();
            //ジャイロ、加速度センサがある場合には、updateGyroAndAccelSensors()を呼び出します。
            //この関数のなかで、Linkオブジェクトの速度、角速度からセンサの出力値が計算されます。
        }
    }

updateForceSensorsのソースコードです。

.. code-block:: cpp
    :emphasize-lines: 4,9,15,18,19,21,22,25,28,29,32
    
    void ODEBody::updateForceSensors(bool flipYZ)
    {
        const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
        //力センサのリストを取得します。
    
        for(int i=0; i < forceSensors.size(); ++i){
            ForceSensor* sensor = forceSensors.get(i);
            const Link* link = sensor->link();
            //センサが取り付けられているLinkオブジェクトのポインタが取得できます。
        
            const dJointFeedback& fb = forceSensorFeedbacks[i];
            Vector3 f, tau;
            f   << fb.f2[0], fb.f2[1], fb.f2[2];
            tau << fb.t2[0], fb.t2[1], fb.t2[2];
            //関節に係る力、トルクデータをODEから取得します。 
 
            const Matrix3 R = link->R() * sensor->R_local();
            //R_local()関数で、センサが取り付けられているリンク座標系からみたセンサの姿勢行列を取得できます。
            //リンクの姿勢行列を掛けて、ワールド座標系からみたセンサの姿勢行列に変換します。
            const Vector3 p = link->R() * sensor->p_local();
            //同様にp_local()関数で、センサの位置が取得できます。
            //ワールド座標系でみた、リンク原点からセンサ位置のベクトルを計算します。

            sensor->f()   = R.transpose() * f;
            //センサ座標系に変換して力データの変数に代入します。
        
            sensor->tau() = R.transpose() * (tau - p.cross(f));
            //tau - p.cross(f)で、リンク軸周りのトルクをセンサ位置周りのトルクに変換します。
            //さらにセンサ座標系に変換してトルクデータの変数に代入します。
        
            sensor->notifyStateChange();
            //センサの出力が更新されたことを知らせるシグナルを出す関数です。        
        }
    }


仮想関節とは
------------

２つのリンク間に仮想関節を設定すると、指定したリンク間に拘束力を発生させることができます。これを使用すると、閉リンク機構のシミュレーションを行うことができます。閉リンクモデルのサンプルは "share/model/misc/ClosedLinkSample.wrl" です。

このサンプルモデルには、仮想関節の定義が、以下のように書かれています。 ::

 DEF J1J3 ExtraJoint {
     link1Name "J1"
     link2Name "J3"
     link1LocalPos 0.2 0 0
     link2LocalPos 0 0.1 0
     jointType "piston"
     jointAxis 0 0 1
 }

J1J3は、仮想関節につける名前です。link1Name,link2Nameで拘束する２つのリンクの名前を指定します。link1LocalPos,link2LocalPosで、拘束位置をそれぞれのリンク座標系で指定します。jointTypeで拘束のタイプを指定します。"piston"か"ball"が指定できます。jointAxisでlink1のリンク座標系でみた拘束軸を指定します。

これらの情報は、BodyオブジェクトのExtraJoint構造体に保存されています。構造体の定義は ::

 struct ExtraJoint {
         ExtraJointType type;
         Vector3 axis;
         Link* link[2];
         Vector3 point[2];
 };

となっていて、モデルファイルで定義された値が保存されています。

次は、ODEBodyオブジェクトに仮想関節を設定するsetExtraJoint()のソースコードです。

.. code-block:: cpp
    :emphasize-lines: 5,9,15,21,33,36,40,42,44,46,49,51,53

    void ODEBody::setExtraJoints(bool flipYZ)
    {
        Body* body = this->body();
        const int n = body->numExtraJoints();
        //仮想関節の個数を取得します。

        for(int j=0; j < n; ++j){
            Body::ExtraJoint& extraJoint = body->extraJoint(j);
            //仮想関節の参照を取得します。

            ODELinkPtr odeLinkPair[2];
            for(int i=0; i < 2; ++i){
                ODELinkPtr odeLink;
                Link* link = extraJoint.link[i];
                //仮想関節で拘束するリンクのポインタが取得できます。
            
                if(link->index() < odeLinks.size()){
                    odeLink = odeLinks[link->index()];               
                    if(odeLink->link == link){
                        odeLinkPair[i] = odeLink;
                        //そのLinkオブジェクトに対応するODELinkオブジェクトを保存します。
                    }
                }
                if(!odeLink){
                    break;
                }
            }

            if(odeLinkPair[1]){
                dJointID jointID = 0;
                Link* link = odeLinkPair[0]->link;
                Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
                //Link1の拘束位置をワールド座標系に変換します。
            
                Vector3 a = link->attitude() * extraJoint.axis;
                //拘束軸をワールド座標系に変換します。
            
                if(extraJoint.type == Body::EJ_PISTON){
                    jointID = dJointCreatePiston(worldID, 0);
                    //ピストン関節を生成します。
                    dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                    //２つのリンクをその関節でつなぎます。
                    dJointSetPistonAnchor(jointID, p.x(), p.y(), p.z());
                    //関節の位置を指定します。
                    dJointSetPistonAxis(jointID, a.x(), a.y(), a.z());
                    //関節軸を指定します。
                } else if(extraJoint.type == Body::EJ_BALL){
                    jointID = dJointCreateBall(worldID, 0);
                    //ボールジョイントを生成します。
                    dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                    //２つのリンクをその関節でつなぎます
                    dJointSetBallAnchor(jointID, p.x(), p.y(), p.z());
                    //関節の位置を指定します。
                }
            }
        }
    }

ビルド
------

ビルドの方法については、 :doc:`hello-world-sample` や :doc:`sample1` を参照してください。
