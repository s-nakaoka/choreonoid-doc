
新YAML形式モデルファイルチュートリアル
==================================

.. contents::
   :local:
   :depth: 1

概要
----

現在github上で開発中のバージョンでは、YAMLをベースとした新形式のモデルファイルの導入を進めています。これは従来のOpenHRP形式のモデルファイルと比較して、より簡潔に記述でき、記述可能な情報の自由度も高い形式となっており、今後OpenHRP形式を置き換えるべく開発を進めています。YAMLの形式については :doc:`modelfile-yaml` でも用いていましたが、本形式は追加情報だけでなくモデル本体も含む全ての情報を記述可能なものとしています。

本節ではこの新しい形式のモデルファイルについて、チュートリアルの形態で解説を行います。これにより、新形式の仕様に加えて、モデルファイルの編集をどのように進めていったらよいかも学ぶことができます。題材とするモデルは以下に示す"Tank"モデルとなります。

.. image:: images/tank.png

これは２つのクローラと砲塔・砲身を動かす２軸の回転関節で構成されるモデルで、カメラとレーザーレンジセンサ、およびライトをデバイスとして備えています。このモデルはクローラ型モバイルロボットのサンプルとなるもので、これを用いたサンプルプロジェクトとして "TankJoystick.cnoid" や "OpenRTM-TankJoystick.cnoid" がChoreonoid本体に含まれています。


Tankモデルの基本構造
--------------------

Tankモデルは下図に示す5つの部位で構成されています。

.. image:: images/tank_decomposed.png

ベースとなる部分が車体です。この左右の側面にそれぞれクローラ機構が取り付けられます。また、車体の上部には砲塔・砲身が備わります。この部分は砲塔の土台となってヨー軸回転を行う部分と、その上部に砲身とともに取り付けられるピッチ軸回転を行う２つの部分からなります。

これら5つの部分が「リンク」としてモデリングされます。車体の部分はモデルの中心となる部分であり、これを「ルートリンク」としてモデリングします。クローラ部分は :doc:`../../simulation/pseudo-continuous-track` に対応する関節タイプのリンクとしてモデリングします。そして、砲塔の２リンクについてはそれぞれ回転関節としてモデリングします。

これらのリンクの間の階層構造（親子関係）は以下のようになります。 ::

 - 車体
     + 左クローラ
     + 右クローラ
     + 砲塔ヨー軸部
            + 砲塔ピッチ軸部


モデルファイルの用意
--------------------

モデルファイルはYAML形式のテキストファイルとして作成します。YAML形式のファイルは ".yaml" の拡張子をつけることが多いですが、モデルファイルについては他のyamlファイルと区別しやすいよう、".body" の拡張子をつけることにします。

モデルファイルの作成を開始するにあたって、まずはテキストエディタを用いて空のテキストを作成し、上記の拡張子をつけた適当なファイル名で保存しておきましょう今回は "tank.body" というファイル名で保存することにします。このファイルはChoreonoidのshareディレクトリの model/misc 以下に完成品が格納されています。今回はそのファイルの内容を解説しながら、完成に至るまでの作成手順の例を示すということになります。

ヘッダの記述
------------

まずモデルファイルのヘッダとして、YAMLのマッピングを用いて以下のように記述します。 ::

 format: ChoreonoidBody
 formatVersion: 1.0
 angleUnit: degree
 name: Tank

最初の行の記述により、このファイルがChoreonoidのモデルファイルとして認識されるようになります。formatVersionは現在のところ1.0となります。今後仕様に変更があった場合に、新しい仕様と区別するためにバージョン番号を明示しておきます。

モデルファイルにおける関節角度の単位を指定する項目として、"angleUnit" があります。今回は "degree" を指定しているので、角度を度数法で記述します。ラジアンで記述したい場合は、ここに "radian" を指定します。通常は degree の方が記述がしやすいのではないかと思います。

モデルの名前は"name"に記述します。

ルートリンクの記述
------------------

車体に対応するルートリンクを記述します。 ::

 links:
   -
     name: CHASSIS
     translation: [ 0, 0, 0.1 ]
     jointType: free
     centerOfMass: [ 0, 0, 0 ]
     mass: 8.0
     inertia: [
       0.1, 0,   0,
       0,   0.1, 0,
       0,   0,   0.5 ]
     elements:
       Shape:
         geometry:
           type: Box
           size: [ 0.4, 0.3, 0.1 ]
         appearance: &GREEN
           material:
             diffuseColor: [ 0, 0.6, 0 ]


まず、モデルが有するリンクの情報は、"links:" 以下にYAMLのリストとして記述します。ここではまずルートリンクのみを記述していますが、複数のリンクを記述する場合は、YAMLのリスト記号 "-" を使って、 ::

 links:
   -
     リンク1の記述
   -
     リンク2の記述
   -
     リンク3の記述
   ...

のように書いていきます。YAMLでは各行のインデントがデータの構造も規定することになりますので、同じレベルのデータに対してインデント量を揃えるように注意して下さい。

なお、複数のリンクが記述されている場合、最初に記述されたリンクがルートリンクとみなされます。

リンクの情報はYAMLのマッピングの形式で記述します。ここで使われるキーとその内容は以下のようになっています。

.. list-table::
 :widths: 20, 80
 :header-rows: 1

 * - キー
   - 内容
 * - name
   - リンク名
 * - translation
   - 本リンクローカルフレームの親リンクからの相対位置。ルートリンクの場合はモデル読み込み時のデフォルト位置として使われる。
 * - rotation
   - 本リンクローカルフレームの親リンクからの相対姿勢。姿勢は回転軸と回転角度に対応する4つの数値で表現(Axis-Angle形式）。ルートリンクの場合はモデル読み込み時のデフォルト位置として使われる。
 * - jointType
   - 関節タイプ。fixed (固定）、free (非固定ルートリンク）、 revolute (回転関節）、slide (並進関節）、pseudoContinousTrack (簡易無限軌道）のどれかを指定。
 * - jointId
   - 関節Id値を指定。関節でない場合（ルートリンクや固定関節等）や非公開の関節としたい場合は、指定しなくてもよい。
 * - centerOfMass
   - 重心位置。リンクローカル座標で指定。
 * - mass
   - 質量[kg]
 * - inertia
   - 慣性モーメント。慣性テンソルの9要素をリストとして列挙。
 * - elements
   - リンクの他の構成要素を階層的に記述。


リンクの形状やリンクに取り付けられたデバイス（各種センサやライト等）の情報は、elements以下に記述します。

編集中のモデルの確認
--------------------

これまでルートリンクの部分まで記述してきましたが、実はこの時点でモデルファイルとしては成立しています。そこで、編集中のファイルをChoreonoid上で読み込んで表示させ、正しく記述ができているか確認してみましょう。これまでのモデルファイルをボディアイテムとして読み込んでチェックを入れると、シーンビュー上に以下のように表示されるかと思います。

.. image:: images/tank_chassis.png

アイテム読み込み時にエラーが出たり、読み込めてもうまく表示できなかったりした場合は、これまでの記述内容を確認してください。

モデルファイルの修正後にそれを再度読み込む場合、修正前のファイルが既にボディアイテムとして読み込まれているのであれば、アイテムの「再読み込み機能」を用いて簡単に読み込み直すことができます。これを行うためには、アイテムツリービュー上で対象のアイテムを選択し、"Ctrl + R" キーを押します。すると更新されたファイルが読み込み直されて、（読み込みエラーがなければ）現在のアイテムがそれに置き換わります。更新したファイルに形状等の変化があれば、シーンビュー上の表示も即座にこれを反映します。この機能を使えば、テキストファイルで直接モデルファイルを編集しながら、比較的効率的にモデルファイルの編集を進めていくことが可能です。


リンク形状の記述
----------------

ルートリンクの形状は以下のように記述されています。 ::

 Shape:
   geometry:
     type: Box
     size: [ 0.4, 0.3, 0.1 ]
   appearance: &GREEN
     material:
       diffuseColor: [ 0, 0.6, 0 ]

形状の本体はShape以下に記述されます。ここのgeometryでどのような幾何形状かを指定し、appearanceで色などの要素を記述します。ここではgeometryにx, y, z軸方向の寸法がそれぞれ0.4[m], 0.3[m], 0.1[m]である直方体を設定し、appearanceに緑色のマテリアルを設定しています。先ほどChoreonoid上でモデルファイルを読み込んだ際にシーンビューに表示されたのが、この形状です。

今回はgeometryに与えた type: Box によって直方体を表現しましたが、他にも球(Sphere)、シリンダ(Cylinder)、円柱(Cone)といったプリミティブ形状を利用することができます。

このような形状の記述については、書き方は多少異なるものの、その構造や形状タイプ、パラメータ等についてVRML97で定義されているものを踏襲するようにしています。VRML97はOpenHRP形式のモデルファイルでベースとしていた形式なので、それの利用経験がある方でしたら勝手をつかみやすいのではないかと思います。

appearance の後の "&GREEN" は、YAMLの「アンカー」という機能で、このように記述しておくとこれ以下の部分を後で使いまわせるようになります。緑色は他の部位でも使いますので、ここでこのようにアンカーを入れています。


補足：ノードとelements
----------------------

モデルファイルにおいては、ある構成要素について同じ階層でまとまった情報を記述する部分を「ノード」と呼びます。例えば上記のリンクやShapeの定義がそれぞれひとつのノードに対応します。そして、あるノードにおいてその下層に位置するノードを記述するためのキーとして"elements"があります。これは下層のノードを持ち得る全てのノードに共通で使われるキーとなっていて、これを用いて階層的なノード構造を記述します。リンクノードとShapeノードの階層構造もこれで実現しています。

elementsは、基本的にはYAMLのリスト表現を用いて以下のように記述します。 ::

 elements:
   -
     type: ノードタイプ名
     key1: value1
     key2: value2
     ...
   - 
     type: ノードタイプ名
     key1: value1
     key2: value2
   ...

これにより、あるノードに対して複数のノードを下位に持たせることができます。

ただし、あるノードが有する下位ノードがひとつだけの場合は、以下のような簡略化記法が使用できます。 ::

 elements:
   ノードタイプ名:
      key1: value1
      key2: value2
      ...

大きな違いはありませんが、こちらの方がリスト表現を使わない分少しだけシンプルな記述になっています。

elementsが使用可能なノードとしては、他にTransformやRigidBodyといったノードがあります。それらについては以下の節で紹介します。

なお、モデルが複数のリンクを有する場合、それらも階層的な構造を有するものですが、本形式のモデルファイルではリンク間の階層構造の記述にはelementsは用いません。これは、リンクの階層が深い場合、elementsを用いるとモデルファイル上の記述も階層が深くなってしまい、テキストエディタを用いた編集がやりづらくなってしまうからです。リンクの階層構造の記述方法についても後ほど説明します。


クローラ部の記述
----------------

次はクローラの部分を記述しましょう。まずは左側から記述します。これまでの記述の下に以下を加えて下さい。 ::

 -
   name: CRAWLER_TRACK_L
   parent: CHASSIS
   translation: [ 0, 0.15, 0 ]
   jointType: pseudoContinuousTrack
   jointId: 0
   jointAxis: [ 0, 1, 0 ]
   centerOfMass: [ 0, 0, 0 ]
   mass: 1.0
   inertia: [
     0.02, 0,    0,
     0,    0.02, 0,
     0,    0,    0.02 ]
   elements:
     Transform:
       translation: [ 0, 0.05, 0 ]
       elements:
         Shape: &CRAWLER 
           geometry:
             type: Extrusion
             crossSection: [
               -0.2, -0.1,
               0.2, -0.1,
               0.3,  0.06,
               -0.3,  0.06,
               -0.2, -0.1
               ]
             spine: [ 0, -0.05, 0, 0, 0.05, 0 ]
           appearance:
             material:
               diffuseColor: [ 0.1, 0.1, 0.1 ]


左側も以下のように記述して追加します。 ::

 -
   name: CRAWLER_TRACK_R
   parent: CHASSIS
   translation: [ 0, -0.15, 0 ]
   jointType: pseudoContinuousTrack
   jointId: 1
   jointAxis: [ 0, 1, 0 ]
   centerOfMass: [ 0, 0, 0 ]
   mass: 1.0
   inertia: [
     0.02, 0,    0,
     0,    0.02, 0,
     0,    0,    0.02 ]
   elements:
     Transform:
       translation: [ 0, -0.05, 0 ]
       elements:
         Shape: *CRAWLER 


砲塔ヨー軸部の記述
------------------

::

 -
   name: CANNON_Y
   parent: CHASSIS
   translation: [ -0.05, 0, 0.08 ]
   jointType: revolute
   jointId: 2
   jointAxis: [ 0, 0, 1 ]
   elements:
     RigidBody:
       centerOfMass: [ 0, 0, 0.025 ]
       mass: 4.0
       inertia: [
         0.1, 0,   0,
         0,   0.1, 0,
         0,   0,   0.1 ]
       elements:
         Shape:
           geometry:
             type: Box
             size: [ 0.2, 0.2, 0.08 ]
           appearance: *GREEN


砲塔ピッチ軸部の記述
--------------------

::

 -
   name: CANNON_P
   parent: CANNON_Y
   translation: [ 0, 0, 0.04 ]
   jointType: revolute
   jointId: 3
   jointAxis: [ 0, 1, 0 ]
   elements:
     - 
       # Turnet
       type: RigidBody
       centerOfMass: [ 0, 0, 0 ]
       mass: 3.0
       inertia: [
         0.1, 0,   0,
         0,   0.1, 0,
         0,   0,   0.1 ]
       elements:
         Shape:
           geometry:
             type: Cylinder
             height: 0.1
             radius: 0.11
           appearance: *GREEN

砲身形状の記述
--------------

以下を砲塔ピッチ部のelementsに追加します。 ::

    - 
      # Cannon barrel
      type: RigidBody
      translation: [ 0.2, 0, 0 ]
      centerOfMass: [ 0.2, 0, 0 ]
      mass: 1.0
      inertia: [
        0.01, 0,   0,
        0,    0.1, 0,
        0,    0,   0.1 ]
      elements:
        Transform:
          rotation: [ 0, 0, 1, 90 ]
          elements:
            Shape:
              geometry:
                type: Cylinder
                height: 0.2
                radius: 0.02
              appearance: *GREEN


ライトの記述
------------

以下を砲身形状に続けて追加します。 ::

     -
       # Device Box
       type: Transform
       translation: [ 0.08, 0, 0.09 ]
       elements:
         -
           type: Transform
           rotation: [ 0, 0, 1, 90 ]
           elements:
             Shape:
               geometry:
                 type: Cone
                 height: 0.04
                 radius: 0.03
               appearance:
                 material:
                   diffuseColor: [ 1.0, 1.0, 0.4 ]
                   ambientIntensity: 0.3
                   emissiveColor: [ 0.8, 0.8, 0.3 ]
         -
           type: Transform
           translation: [ 0.02, 0, 0 ]
           elements:
             -
               type: SpotLight
               name: MainLight
               direction: [ 1, 0, 0 ]
               beamWidth: 36
               cutOffAngle: 40
               cutOffExponent: 6
               attenuation: [ 1, 0, 0.01 ]


カメラの記述
------------

以下をSpotLightノードと同階層に追加します。 ::

              - 
                type: Transform
                rotation: [ [ 0, 1, 0, -90 ], [ 0, 0, 1, -90 ] ]
                elements:
                  -
                    type: Camera
                    name: Camera
                    format: COLOR_DEPTH
                    width: 320
                    height: 240
                    id: 0
                    frameRate: 30


レーザーレンジセンサの記述
--------------------------

以下をCameraと同階層に追加します。 ::

                  -
                    type: RangeSensor
                    name: RangeSensor
                    id: 0
                    scanAngle: 90
                    scanStep:  0.5
                    scanRate:  10
                    maxDistance: 10


追加情報の記述
--------------

OpenHRP形式のモデルファイルにおいて任意の情報を追記する手段として、 :doc:`modelfile-yaml` がありましたが、これは本形式のモデルファイルに関してはファイルの追加なしに行うことができます。モデルファイル本体自体がYAML形式ですので、YAML形式の情報はこの中にいくらでも書くことが可能というわけです。


