競技会概要
==========

ここではWRS2018のロボット競技会「トンネル事故災害対応・復旧チャレンジ」の概要について紹介します。

.. contents::
   :local:

開催情報
--------

本競技はトンネル内で発生した災害や事故を想定したもので、その状況で必要とされるロボットのタスクの遂行能力を競うものです。競技は全部で6つのタスクから構成され、コンピュータ上でシミュレートされる仮想環境においてロボットがタスクを遂行するものとします。

開催に関する概要は以下のとおりです。

* 日程： 2018年10月17日（水）～21日（日）
* 会場： 東京ビッグサイト
* 参加形態: 参加希望者がチームを構成して書類応募。参加費は無料。書類審査を通過すると参加が可能となる。
* 参加チーム数：全9チーム
* `賞金 <http://worldrobotsummit.org/download/guideline/prize_money_for_the_wrc2018_ja.pdf>`_ ： 1位:1000万円、2位:300万円、3位:100万円

* 競技ルール:

  * `公式英語版 <http://worldrobotsummit.org/download/rulebook-en/rulebook-Tunnel_Disaster_Response_and_Recovery_Challenge.pdf>`_
  * `日本語参考版 <http://worldrobotsummit.org/download/detailed-rules/detailed-rules-tunnel-disaster-response-and-recovery-challenge-ja.pdf>`_

* 公式サイト案内ページ: http://worldrobotsummit.org/wrc2018/disaster/

※ 参加チームの応募受付は2018年1月〜3月にかけてWRS公式ページを通して行われました。現在応募は終了しており、参加チームも確定しています。

.. note:: WRSの競技会は今回の2018年大会に加えて、2020年にも開催される予定です。その際にも本競技と同様のシミュレーションによる競技が開催される見込みで、あらためて参加チームを募ることになりますので、興味のある方はその機会での参加もご検討ください。


.. * `渡航費、滞在費を支援 <http://worldrobotsummit.org/download/guideline/support_for_participating_teams_ja.pdf>`_

.. _wrs2018_simulator:

使用シミュレータ
----------------

本競技で使用するシミュレータの情報を以下にまとめます。


* シミュレータ本体: Choreonoid
* バージョン: Choreonoidの次期バージョンを使用

 * 現在「開発版」として開発・公開されているもので、 `github上のリポジトリ <https://github.com/s-nakaoka/choreonoid>`_ から取得可能

* 使用OS: Ubuntu Linux 16.04 64bit版

 * 希望チームに対しては Ubuntu 18.04 64bit版 で競技を行う可能性あり

* 物理エンジンとして AGX Dynamics を利用

 * 参加チームには競技用のランタイムライセンスを無償で提供
 * :doc:`../agxdynamics/index` によって導入

競技で使用するバージョンは、Choreonoidの次期バージョンとなります。これは現在「開発版」として開発・公開されているもので、 `github上のリポジトリ <https://github.com/s-nakaoka/choreonoid>`_ から取得することができます。

ChoreonoidはUbuntu LinuxとWindowsに対応していますが、競技で使用するOSはUbuntu Linuxになります。使用するUbuntuのバージョンは未定ですが、今年4月にリリースされる予定のUbuntu Linux 18.04の使用が有力となっています。現在はUbuntu Linux 16.04を用いてChoreonoidの改良や競技タスクの開発を進めています。

本競技では物理エンジンとして `AGX Dynamics <http://www.vmc-motion.com/14416057938792>`_ を使用します。AGX Dynamicsは商用の物理エンジンであり、使用にあたってはライセンスが必要となりますが、本競技の参加登録者には参加用のラインセンスが無償で提供されることになっています。 `Choreonoid用のライセンス <http://www.vmc-motion.com/15135605209828>`_ は `株式会社ブイエムシー <http://www.vmc-motion.com/>`_ から購入することも可能です。

競技会の本番で使用するシミュレーション用PCは運営側で用意します。競技会の参加準備のためのシミュレーションは、参加者各自のPCにChoreonoidをインストールして行ってください。

対象ロボット
------------

プロトタイプでは、`プラットフォームロボット <http://worldrobotsummit.org/download/201707/WRS_Disaster_Robotics_Category_A_standard_robot_platform_for_for_Simulation_Challenge_of_Tunnel_Disaster_Response_and_Recovery_Challenge-doc_jp.pdf>`_  のひとつである「双腕ロボット」（大阪大学開発）を使えるようにしています。（以下ではこれを「双腕重機ロボット」と呼ぶことにします。）このロボットモデルの外観を以下に示します。

.. image:: images/DoubleArmV7A.png

このモデルはクローラを使用していますが、クローラについて簡易的なシミュレーションを行うバージョンと、AGX Dynamicsを用いてより実機に近いシミュレーションを行うバージョンを用意しています。モデルのベース名は "DoubleArmV7" としており、これに "Simplified" の "S" または "AGX" の "A" を付与して、

* DoubleArmV7S（簡易クローラ版）
* DoubleArmV7A（AGXクローラ版）

というモデル名にしています。

モデルファイルは、Choreonoidソースの "share/model/DoubleArmV7" 以下に、"DoubleArmV7S.body"、"DoubleArmV7A.body" というファイル名で格納しています。

.. note:: 簡易クローラ版はChoreonoidの標準機能で利用可能で、プロトタイプではAISTシミュレータアイテムを使ってシミュレーションするように設定されています。AGXクローラ版を利用するためには、 :ref:`wrs2018_install_choreonoid` で述べたように、AGX DynamicsとAGX Dynamicsプラグインをインストールしておく必要があります。そちらはAGXシミュレータアイテムを使ってシミュレーションをします。

もうひとつのプラットフォームロボットとして、早稲田大学によって開発された "WAREC-1" も利用可能です。こちらの外観を以下に示します。

.. image:: images/WAREC1.png

このロボットは脚型ロボットの一種で、４脚型のロボットとして使うこともできますし、２脚で立たせてもう一方の２脚をアームとして使うことで、ヒューマノイドロボットのように使うことも可能となっています。

このロボットのモデルファイルは "share/model/WAREC1" 以下に "WAREC1.body" というファイル名で格納されています。ただし現状ではこのモデルを対象とした競技タスクプロトタイプは用意していません。

本競技では上記のプラットフォームロボットが設定されておりますが、ルール上はそれ以外のロボットでも参加可能です。競技タスクプロトタイプでは、会津大学と株式会社アイザックが共同で開発した "Aizu Spider" というロボットのモデルも利用できるようにしました。このロボットのモデルを以下に示します。

.. image:: images/AizuSpiderSA.png

このモデルは災害対応ロボットで一般的な構成となっており、メインクローラ２つと、前後のフリッパに搭載されるサブクローラを４つ備えています。またKinova社のJACO2アームを搭載しており、マニピュレーションも行えるようになっています。上の図に示したのはアームをひとつ搭載した単腕版で、プロトタイプでもこれを使用しています。双腕重機ロボットと同様に、クローラについて以下の２つのタイプがあります。

* AizuSpiderSS (簡易クローラ版）
* AizuSpiderSA (AGXクローラ版）

さらにアームの搭載数の違いから、以下のバリエーションモデルも用意されています。

* AizuSpiderNS (アーム無し、簡易クローラ版）
* AizuSpiderNA (アーム無し、AGXクローラ版）
* AizuSpiderDS (双腕、簡易クローラ版）
* AizuSpiderDA (双腕、AGXクローラ版）

これらのモデルのファイルは全てChoreonoidソースの "share/model/AizuSpider" 以下に収録されており、上記のモデル名に拡張子".body"がついたファイル名となっています。

タスク概要
----------



ロボットの行動／操作
--------------------

本競技では、ロボットの行動に遠隔操作を用いることができます。その場合、シミュレーション用PCとは別の機器（PC等）に遠隔操作端末を構築し、シミュレーション用PC内の仮想ロボットと遠隔操作端末をネットワークで接続します。参加者はこのための遠隔操作用端末を用意する必要があります。遠隔操作端末は、仮想ロボットとネットワーク接続できるものであれば、どのようなものを用いてもかまいません。

競技タスクのプロトタイプ
------------------------

Github上に上げているChoreonoid開発版をインストールすることで、現在開発中の競技タスクのプロトタイプを試せるようになっています。プロトタイプはAGX Dynamicsのライセンスがなくても実行可能なものも用意していますので、本競技への参加をご検討中のかたはぜひお試しください。

以下ではプロトタイプのシミュレーションを実行する方法について解説します。
