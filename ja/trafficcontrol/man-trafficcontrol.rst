通信障害シミュレーションプラグイン
==================================
通信障害シミュレーションプラグインは、ロボットを遠隔操縦する際に発生しうる通信障害（通信遅延、帯域制限、パケットロス）を模擬するためのプラグインです。
本プラグインでは、Linux用のネットワーク設定ツールである“iproute2”を用いて、Ethernet接続された計算機間の通信を制御し、通信障害を模擬します。
ここでは、指定したパラメータに基づいてシミュレーション中に固定の通信障害の効果を与える通信障害シミュレーションと、シミュレーション中に動的に通信障害の効果を与える動的通信障害シミュレーションについて
以下の項目に分けて順に説明します。 

* 事前設定
* 通信障害シミュレーションプラグインの導入
* TrafficControlSimulatorItemの設定方法
* 動的通信障害シミュレーションプラグインの導入
* DynamicTrafficControlSimulatorItemの設定方法

事前設定
--------
通信障害シミュレーションプラグインでは、内向きの通信障害を模擬するためにifbモジュール（仮想通信ポート）を使用します。
（本マニュアルでは、「内向き」は本プラグインが動作している計算機がパケットを受信する方向、「外向き」は本プラグインが動作している計算機がパケットを送信する方向とします。）

.. note:: 仮想通信ポートは、「modprobe」および「ip」コマンドを使用して設定することができますが、通常これらの設定には管理者権限が必要です。管理者権限が付与されていない場合は、使用している計算機の管理者に確認してください。

ここでは、通信ポートを2つ使う場合を例に、仮想通信ポートの設定方法について説明します。

まず、通信制御を行う通信ポートと同数の仮想通信ポートを作成します。 ::

 $ modprobe ifb numifbs=2
 $ modprobe act_mirred

1行目と2行目のコマンドは、ifbモジュールを用いて「tc」コマンド（通信障害の効果を与えるために使用する）を使用する際に必要なモジュールを読み込んでいます。
ここで、1行目のnumifbs=2は、仮想通信ポートを2つ作成することを表しています。

次に、作成した通信ポートを有効にします。通常、生成されたifbの名前は、ifb0、ifb1のように0から昇順に番号を割り振られます。ここでは2つの通信ポートを使用するので、ifb0とifb1を有効にします。 ::

 $ ip link set dev ifb0 up
 $ ip link set dev ifb1 up

次に、コンフィグファイルを格納するディレクトリを作成します。/usr/local/share/以下にディレクトリcnoid-confを作成してください。
続いて、コンフィグレーションファイルを作成します。コンフィグレーションファイルは、内向きと外向きの通信ポートを1対1に紐付けるために必要となります。任意のテキストエディタを使用してファイル名を“tc.conf”として作成したディレクトリcnoid-conf内に以下を参考にファイルを作成してください。以下は、eth0とifb0、eth1とifb1をそれぞれ紐付ける設定をの記述例です。 ::

 Port,ifb
 eth0,ifb0
 eth1,ifb1

ここで、1行目はファイルヘッダを表しています。このファイルヘッダは必ず記述するようにしてください。2行目以降は、通信ポートを紐付けるための記述で、カンマ区切りで左に通信ポート、右に紐付ける仮想通信ポートを記述しています。

最後に、tcコマンドを使用するための権限設定を行います。visudoを実行して、以下の行を追記してください。なお以下は、ユーザ名：userを設定した記述例です。 ::

 user ALL=(ALL:ALL) NOPASSWD: /sbin/tc

事前設定の解除
----------------------
事前設定を解除する場合は、管理者権限で以下のコマンドを実行し、tcの設定と仮想通信ポートを削除してください。 ::

 // delete tc setting
 $ tc qdisc del dev eth0 root
 $ tc qdisc del dev eth1 root
 $ tc qdisc del dev eth0 ingress
 $ tc qdisc del dev eth1 ingress
 $ tc qdisc del dev ifb0 root
 $ tc qdisc del dev ifb1 root

 // unload ifb module
 $ rmmod ifb

最後に、事前設定で生成・編集したファイルおよびディレクトリを手動で削除・修正してください。

通信障害シミュレーションプラグインの導入
----------------------------------------
ビルド時のCMakeの設定で、以下のオプションをONにするようにしてください。

* BUILD_TRAFFIC_CONTROL_PLUGIN

TrafficControlSimulatorItemの設定方法
--------------------------------------
通信障害シミュレーションでは、TrafficControlSimulatorItemを使用します。
通信障害の効果はTrafficControlSimulatorItemのプロパティの設定に従って設定されます。なお、シミュレーション中の設定変更はできません。
メインメニューの「ファイル」-「新規」から「TrafficControlSimulator」を選択し、TrafficControlSimulatorItemを生成してください。デフォルトの名前は”TrafficControlSimulator”となります。これをアイテムツリービュー上でシミュレータアイテムの子アイテムとして配置してください。

TrafficControlSimulatorItemの設定例） ::

 [ ] - World
 [/]   + Tank
 [/]   + floor
 [ ]   + AISTSimulator
 [ ]     + TrafficControlSimulatorItem

TrafficControlSimulatorの設定項目
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
通信障害シミュレーションを行うには、TrafficControlSimulatorItemのプロパティの設定が必要です。各プロパティの内容を以下に示します。

.. csv-table::
    :header: "プロパティ", "単位", "意味"
    :widths: 16, 8, 32

    "EnableTrafficControl", "[-]", "通信障害の効果の有効／無効を指定します。"
    "Port", "[-]", "通信ポートを指定します。使用するポートを一覧から選択します。"
    "InboundDelay", "[ms]", "内向きの通信に与える遅延時間を指定します。0の場合は設定されません。"
    "InboundBandWidth", "[kbit/s]", "内向きの通信に与える通信速度の上限を指定します。0の場合は設定されません。"
    "InboundLoss", "[%]", "内向きの通信に与えるパケットロスの割合を指定します。0の場合は設定されません。"
    "OutboundDelay", "[ms]", "外向きの通信に与える遅延時間を指定します。0の場合は設定されません。"
    "OutboundBandWidth", "[kbit/s]", "外向きの通信に与える通信速度の上限を指定します。0の場合は設定されません。"
    "OutboundLoss", "[%]", "外向きの通信に与えるパケットロスの割合を指定します。0の場合は設定されません。"
    "IP Address", "[-]", "通信障害の効果を与える通信先の計算機のIPアドレスとサブネットマスクを指定します。通信先の計算機やネットワークを指定しない場合、設定は不要です。その場合、“Port”で指定した通信ポートを通るパケット全てに対して通信障害の効果を与えます。　入力例）192.168.0.1/24"

動的通信障害シミュレーションプラグインの導入
----------------------------------------
動的通信障害シミュレーションプラグインを使用するには、上記の通信障害シミュレーションプラグインが導入されている必要があります。
そのため、ビルド時のCMakeの設定で、BUILD_TRAFFIC_CONTROL_PLUGINがONになっていることを確認の上、以下のオプションをONにするようにしてください。

* BUILD_DYNAMIC_TRAFFIC_CONTROL_PLUGIN

DynamicTrafficControlSimulatorItemの設定方法
--------------------------------------------
動的通信障害シミュレーションでは、DynamicTrafficControlSimulatorItemと上述の通信障害シミュレーションプラグインのTrafficControlSimulatorItemを使用します。
メインメニューの「ファイル」-「新規」から「DynamicTrafficControlSimulator」を選択し、DynamicTrafficControlSimulatorItemを生成してください。デフォルトの名前は”DynamicTrafficControlSimulator”となります。これをアイテムツリービュー上でシミュレータアイテムの子アイテムとして配置してください。TrafficControlSimulatorItemをアイテムツリービューに登録していない場合は、上述のTrafficControlSimulatorItemの設定方法を参考にTrafficControlSimulatorItemをアイテムツリービューに登録してください。

DynamicTrafficControlSimulatorItemの設定例） ::

 [ ] - World
 [/]   + Tank
 [/]   + floor
 [ ]   + AISTSimulator
 [ ]     + TrafficControlSimulatorItem
 [ ]     + DynamicTrafficControlSimulatorItem

動的通信障害シミュレーションを行う際は、TrafficControlSimulatorItemのプロパティ“EnableTrafficControl”を“false”に設定し、TrafficControlSimulatorItemとDynamicTrafficControlSimulatorItemのプロパティ“Port”が一致するように設定してください。
シミュレーション中は、対象とするBodyモデルと基準点の距離に応じた通信障害の効果がDynamicTrafficControlSimulatorItemのプロパティで設定したタイムステップ毎に更新されます。なお、通信障害の効果はソースコードに直接記述してあるため、通信障害の効果の度合いを変更することも可能です。

DynamicTrafficControlSimulatorItemの設定項目
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
動的通信障害シミュレーションを行うには、DynamicTrafficControlSimulatorItemのプロパティの設定が必要です。各プロパティの内容を以下に示します。

.. csv-table::
    :header: "プロパティ", "単位", "意味"
    :widths: 16, 8, 32

    "Port", "[-]", "通信ポートを指定します。使用するポートを一覧から選択します。"
    "EnableDynamicTrafficControl", "[-]", "通信障害の効果の有効／無効を指定します。"
    "ReferencePoint", "[m, m, m]", "基準点の座標をグローバル座標で指定します。"
    "TargetBody", "[-]", "対象とするBodyモデルを指定します。"
    "TimeStep", "[s]", "通信障害の効果を更新する時間間隔を指定します。"

サンプル
--------
DynamicTrafficControlSimulatorItemにはサンプルとして、基準点(0,0,0)を中心とする半径10mの範囲で最大200msの通信遅延が動的に与えられるように設定されています。

.. figure:: image/dynamicsample.png

