ロボットモデル
==============

:doc:`overview` で :ref:`wrs2018_overview_robots` の概要を紹介しました。ここでは各ロボットをChoreonoid上で扱うためのモデルデータについて解説します。

.. contents::
   :local:

モデルファイル
--------------

Choreonoidのモデルは通常拡張子が "body" の "Bodyフォーマット" のファイルとして記述されます。このフォーマットの仕様などは本マニュアルの :doc:`../handling-models/modelfile/index` にまとめていますので、詳細はそちらをご参照ください。

標準ロボットモデルにパラメータ調整や改造を行ったり、新たにロボットモデルを作成する場合は、この形式のモデルファイルを編集する必要があります。モデルファイルはYAML形式のテキストデータですので、通常はテキストエディタで編集します。編集の仕方については、 :doc:`../handling-models/modelfile/modelfile-newformat` をご覧ください。

モデルファイルは、「ボディアイテム」としてChoreonoid上に読み込むことができます。ボディアイテムの扱いについては、 :doc:`../handling-models/index` をご覧ください。

.. _wrs_standard_model_directory:

標準モデルディレクトリ
----------------------

Choreonoidには多数のモデルファイルがサンプルとして含まれています。これはChoreonoidの「標準モデルディレクトリ」以下に格納されています。標準モデルディレクトリは、ソースコードにおいては "share/model" になります。Choreonoidをmake installした場合は、インストール先の "share/choreonoid-x.x/model" になります。（ x.xはバージョン番号。:doc:`../install/directories` 参照）

WRS2018の標準ロボットモデルについても、Choreonoidのサンプルモデルとして含まれており、標準モデルディレクトリ以下に格納されています。

WAREC-1
-------

WAREC-1のモデルは、標準モデルディレクトリの "WAREC1" というディレクトリに格納されています。メインとなるファイルは "WAREC1.body" で、このファイルをChoreonoidから読み込むことにより、WAREC1のシミュレーションなどが可能になります。

双腕建機型ロボット
------------------

双腕建機型ロボットのモデルは標準モデルディレクトリの "DoubleArmV7" というディレクトリに格納されています。

このモデルはクローラを使用していますが、クローラについて簡易的なシミュレーションを行うバージョンと、AGX Dynamicsを用いてより実機に近いシミュレーションを行うバージョンを用意しています。モデルのベース名 "DoubleArmV7" に対して "Simplified" の "S" または "AGX" の "A" を付与して、

* 簡易クローラ版: DoubleArmV7S.body
* AGXクローラ版: DoubleArmV7A.body

というファイル名で格納されています。

簡易クローラ版はChoreonoidの標準機能で利用可能で、WRS2018のサンプルも用意しています。そちらを使えば、AGX Dynamicsのラインセンスをお持ちでない方でも、WRS2018のシミュレーションを試すことができます。ただし、クローラの動作が実機とは異なる点については予めご了承ください。

AGXクローラ版ではクローラの挙動が実機に近いものとなります。AGX Dynamicsのライセンスをお持ちの方は、こちらを使うようにしてください。競技においてもこちらを使用します。

Aizu Spider
-----------

Aizu Spiderのモデルは標準モデルディレクトリの "AizuSpider" というディレクトリに格納されています。

このモデルは全部で6つのバリエーションを用意しています。まず、ロボットに搭載されるアームに関して、アーム無し（N）、一本（S）、二本（D）の３つのバリーションを用意しています。また、このモデルもクローラを搭載しており、クローラに関して簡易版（S）とAGX版（A）があります。これらの組み合わせで、以下の6つのモデルとなります。

* アーム無し、簡易クローラ版: AizuSpiderNS.body
* アーム無し、AGXクローラ版: AizuSpiderNA.body
* 単腕、簡易クローラ版: AizuSpiderSS.body
* 単腕、AGXクローラ版: AizuSpiderSA.body
* 双腕、簡易クローラ版: AizuSpiderDS.body
* 双腕、AGXクローラ版: AizuSpiderDA.body

なお、このロボットに搭載されるアームは、既成品であるKinova社のJACO2アームとなっています。

Quadcopter
----------

Quadcopterのモデルは標準モデルディレクトリの "multicopter" というディレクトリに、"quadcopter.body" というファイル名で格納されています。このモデルの飛行シミュレーションを行うためには、 :doc:`../multicopter/index` が必要となります。
