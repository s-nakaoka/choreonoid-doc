
.. highlight:: yaml

YAMLによる追加情報の記述
========================

ChoreonoidではOpenHRP形式のモデルファイルを読み込むことができますが、さらにYAML形式のファイルと組み合わせることで、追加の情報を記述できるようになっています。モデルにこのファイルが用意されている場合は、そちらを読み込むようにすると、Choreonoidの機能を最大限に利用できるようになります。

.. contents::
   :local:
   :depth: 1

SR1サンプルモデルの追加情報
---------------------------

以下に示すのは、SR1サンプルモデルの追加情報ファイル (SR1.yaml) の内容です。この例を通して、追加情報の具体的な記述方法を説明したいと思います。 ::

 modelFile: SR1.wrl
 
 standardPose: [ 
     0, -30, 0,  60, -30, 0,
    20, -10, 0, -40,   0, 0, 0,
     0, -30, 0,  60, -30, 0,
    20,  10, 0, -40,   0, 0, 0,
     0,   0, 0 
 ]
 
 linkGroup:
   - name: UPPER-BODY
     links:
       - WAIST_P
       - WAIST_R
       - CHEST
       - name: ARMS
         links:
           - name: R-ARM
             links: [ RARM_SHOULDER_P, RARM_SHOULDER_R, RARM_SHOULDER_Y,
                      RARM_ELBOW, 
                      RARM_WRIST_Y, RARM_WRIST_P, RARM_WRIST_R ]
           - name: L-ARM
             links: [ LARM_SHOULDER_P, LARM_SHOULDER_R, LARM_SHOULDER_Y, 
                      LARM_ELBOW, 
                      LARM_WRIST_Y, LARM_WRIST_P, LARM_WRIST_R ]
   - WAIST
   - name: LEGS
     links:
       - name: R-LEG
         links: [ RLEG_HIP_R, RLEG_HIP_P, RLEG_HIP_Y, 
                  RLEG_KNEE, 
                  RLEG_ANKLE_P, RLEG_ANKLE_R ]
       - name: L-LEG
         links: [ LLEG_HIP_R, LLEG_HIP_P, LLEG_HIP_Y,
                  LLEG_KNEE, LLEG_ANKLE_P,
                  LLEG_ANKLE_R ]
 
 footLinks:
   - link: RLEG_ANKLE_R
     soleCenter: [ 0.05, 0.0, -0.055 ]
   - link: LLEG_ANKLE_R
     soleCenter: [ 0.05, 0.0, -0.055 ]

 defaultIKsetup:
   WAIST: [ RLEG_ANKLE_R, LLEG_ANKLE_R ]
   RLEG_ANKLE_R: [ WAIST ]
   LLEG_ANKLE_R: [ WAIST ]
 
 collisionDetection:
   excludeTreeDepth: 3
   excludeLinks: [ ]


モデルファイル本体の指定
------------------------

追加情報ファイルはあくまでモデルファイル本体に対して情報を追加するものであるため、まず本体となるモデルファイルが何であるかを明示しておく必要があります。

これを行っているのが以下の部分で、"modelFile" というキーにファイル名を記述します。 ::

 modelFile: SR1.wrl

追加情報ファイルと本体のファイルが同じディレクトリにある場合は、本体のファイル名のみでOKです。違うディレクトリにある場合は、そのディレクトリへの相対パスで記述します。

標準姿勢の設定
--------------

:doc:`../pose-editing` の :ref:`model_body_bar` で紹介した「標準姿勢」は、実際には追加情報ファイルに記述されています。これを行っているのが以下の部分です。 ::

 standardPose: [ 
     0, -30, 0,  60, -30, 0,
    20, -10, 0, -40,   0, 0, 0,
     0, -30, 0,  60, -30, 0,
    20,  10, 0, -40,   0, 0, 0,
     0,   0, 0 
 ]

このように "standardPose" というキーに標準姿勢に対応する関節角をリストとして記述します。関節角を並べる順番は関節IDの順で、関節角の単位は [degree] （直動関節の場合は [m]）になります。

リンクのグループ構造の設定
--------------------------

:doc:`../bodymodel` の :ref:`model_structure` で紹介した「リンクビュー」では、モデルが有するリンクの一覧が表示され、モデルの構造を確認することができました。また、ここで編集操作の対象となるリンクを選択することもできました。

このリンクビューではモデル構造の表示の仕方を上部のコンボボックスで切り替えることができるのですが、その中に「身体部位ツリー」という表示方法があります。これを選択するとSR1モデルの場合は以下のような表示になります。

.. image:: images/linkview_bodyparttree.png

ここでは、リンクが階層的にグループ化された身体部位ごとに分けられて表示されます。これを用いることで、リンクと身体部位の関係が把握しやすくなります。このため、この表示方法はキーポーズによる振り付け機能でも使われています。

このような階層グループ構造を記述しているのが、"linkGroup"というキーから始まる以下の部分です。 ::

 linkGroup:
   - name: UPPER-BODY
     links:
       - WAIST_P
       - WAIST_R
       - CHEST
       - name: ARMS
         links:
           - name: R-ARM
             links: [ RARM_SHOULDER_P, RARM_SHOULDER_R, RARM_SHOULDER_Y,
                      RARM_ELBOW, 
                      RARM_WRIST_Y, RARM_WRIST_P, RARM_WRIST_R ]
           - name: L-ARM
             links: [ LARM_SHOULDER_P, LARM_SHOULDER_R, LARM_SHOULDER_Y, 
                      LARM_ELBOW, 
                      LARM_WRIST_Y, LARM_WRIST_P, LARM_WRIST_R ]
   - WAIST
   - name: LEGS
     links:
       - name: R-LEG
         links: [ RLEG_HIP_R, RLEG_HIP_P, RLEG_HIP_Y, 
                  RLEG_KNEE, 
                  RLEG_ANKLE_P, RLEG_ANKLE_R ]
       - name: L-LEG
         links: [ LLEG_HIP_R, LLEG_HIP_P, LLEG_HIP_Y,
                  LLEG_KNEE, LLEG_ANKLE_P,
                  LLEG_ANKLE_R ]


ここでは、マップとリストの組み合わせでグループとそこに分類されるリンクを記述しています。"name"キーはグループ名を表していて、"links"以下にそこに所属するリンクや下位のグループを記述しています。

足リンクの設定
--------------

脚型のモデルについては、どのリンクが足のリンクであるかを明示し、さらに足の操作に関する情報を記述しておくことで、Choreonoidが提供する脚型モデルを対象とした機能を活用できるようになります。これを行っているのが以下の部分です。 ::

 footLinks:
   - link: RLEG_ANKLE_R
     soleCenter: [ 0.05, 0.0, -0.055 ]
   - link: LLEG_ANKLE_R
     soleCenter: [ 0.05, 0.0, -0.055 ]

このように、"footLinks"というキーに足に相当する（床と設置可能な足裏を有する）リンクの情報をリストで列挙します。各足リンクの情報は、"link"というキーにリンク名を記述し、"soleCenter"というキーに足裏の中心点を足リンクからの相対座標で記述します。これによって、例えば :ref:`model_legged_body_bar` の機能が使えるようになります。

.. note:: "soleCenter"に記述する中心点は、重心投影点やZMPがそこにあるときに一番安定となる点を想定したものであり、必ずしも幾何学的な中心である必要はありません。例えば制御上足首付近が安定点である場合は、仮に足首が足裏の中心から外れた位置に接続されている場合でも、soleCenterには足首の位置を設定しておきます。

.. _modelfile_yaml_preset_kinematics:

プリセット運動学の設定
----------------------

:doc:`../pose-editing` - :ref:`model_kinematics_mode` で述べた「プリセット運動学モード」では、ユーザが動かそうとしてるリンクに応じて自動的に順運動学と逆運動学が切り替わるようになっていました。この設定を行っているのが、追加情報ファイルにおける以下の部分です。 ::

 defaultIKsetup:
   WAIST: [ RLEG_ANKLE_R, LLEG_ANKLE_R ]
   RLEG_ANKLE_R: [ WAIST ]
   LLEG_ANKLE_R: [ WAIST ]

ここで行っている設定は以下の２つです。

* WAISTリンク（腰）を動かす際には、RLEG_ANKLE_Rリンク（右足）とLLEG_ANKLE_Rリンク（左足）の両方をベースリンクとして固定した逆運動学を行う
* RLEG_ANKLE_Rリンクを動かす際には、WAISTリンクをベースリンクとした逆運動学を行う
* LLEG_ANKLE_Rリンクを動かす際には、WAISTリンクをベースリンクとした逆運動学を行う

このように、プリセット運動学モード時に逆運動学としたいリンクと、その際のベースリンクを指定すればOKです。

干渉検出の設定
--------------

"collisionDetection"キーでは干渉検出に関わる設定が記述されています。 ::

 collisionDetection:
   excludeTreeDepth: 3
   excludeLinks: [ ]

"excludeTreeDepth"については、関節ツリーにおいて親子関係で隣接しているリンクを自己干渉から外す設定です。この値が 0 だと全てのリンクのペアについて干渉が無いかをチェックしますが、この値を 1 にすると、直接接続されているリンク同士では自己干渉チェックを行わなくなります。値を増やすと、その分だけ接続が離れているリンクも干渉チェック対象外に加えるようになります。

また、"excludeLinks"には、そもそも干渉チェックの対象外とするリンクをリンク名で指定できます。

他のリンクに埋め込まれた関節や、複数の回転軸を組み合わせた関節において、関節内部での干渉は可動範囲内では本来は起こさないように設計する必要がありますが、モデルファイルの形状をそこまで作りこむのには手間がかかることもあります。逆に、柔軟な表面で覆われたリンクでは設計上干渉が許容されることもあります。そのような場合に、上記の設定によって干渉チェックの対象外となるリンクを設定することで、Choreonoid上での操作を効率的に行うことが可能となります。


その他の情報の記述について
--------------------------

以上、SR1サンプルで記述されている主な情報について説明しましたが、追加情報ファイルにはYAML形式であればどのような情報を記述してもOKです。その内容はChoreonoid内部で読めるようになっており、各機能はこれによって必要な情報を得ることができます。これによって、新たに導入するプラグインが要求する情報を記述しておけば、そのプラグインの機能を使えるようになりますし、ユーザがプラグインを開発する場合でも、必要な情報をユーザが定義して利用することができます。このように、YAMLによる追加情報ファイルは柔軟に扱えるようになっており、Choreonoidの機能拡張においても重要な役割を果たす仕組みとなっています。
