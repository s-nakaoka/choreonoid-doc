
OpenRTMプラグインの使用準備
===========================

本ページではOpenRTMプラグインを使用するための準備について解説します。

.. contents::
   :local:

.. highlight:: sh

.. _openrtmplugin_install_openrtm:

OpenRTM-aistのインストール
--------------------------

OpenRTMプラグインを使用するためには、OSにOpenRTM-aistがインストールされている必要があります。

現在OpenRTM-aistの公式サイトや関連サーバが停止しておりますが、代わりに臨時の `OpenRTM-aist web on the github <http://openrtm.org/>`_ が設置されており、ここからOpenRTM-aistをダウンロードすることができます。現在ダウンロードできる最新版はバージョン 1.1.2 となっているので、このバージョンをダウンロードしてインストールしましょう。

Ubuntu 16.04 へのインストール
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ubuntu16.04の場合は、上記ページの説明に従って、コマンドラインから以下のように入力することにより、C++版をインストールすることができます。 ::

 git clone https://github.com/n-ando/xenial_package.git
 cd xenial_package/xenial/main/binary-amd64/
 sudo dpkg -i openrtm-aist_1.1.2-0_amd64.deb
 sudo dpkg -i openrtm-aist-example_1.1.2-0_amd64.deb
 sudo dpkg -i openrtm-aist-dev_1.1.2-0_amd64.deb

OpenRTM-aist関連のパッケージとしては、他にPython版やRTSystemEditor/RTCBuilderといったツールもあります。それらのインストール方法も上記ページにありますので、も必要に応じてインストールしてください。

バージョン1.1.2の不具合への対処
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

OpenRTM-aist 1.1.2にはバグがあり、この影響でChoeronoid上でOpenRTM関連機能を使用する際にChoreonoidが落ちてしまうことがあるようです。

この問題への対処として、OpenRTM-aistのヘッダファイル "OutPort.h" の修正版が、OpenRTM-aist開発チームより提案されています。これを導入することにより、この件が原因で落ちている症状については、改善されるようです。

OutPort.hは、Ubuntu 16.04上で上記の方法でインストールすると、 /usr/include/openrtm-1.1/rtm/ ディレクトリにあります。このディレクトリに、以下からダウンロードした修正版の OutPort.h をコピーして、上書きしてください。（OpenRTMプラグインやサンプルのビルド前にこれを行っておく必要があります。）

* :download:`OpenRTM-aist 1.1.2 用 修正版OutPort.h <files/OutPort.h>`

OpenRTMプラグインのビルド
-------------------------

OpenRTMプラグインはChoreonoidのソースコードに同梱されております。Choreonoidのビルド前に行う :ref:`build-ubuntu-cmake` のcmakeオプションで以下のオプションを **ON** にすることでビルドすることができます。

* **ENABLE_CORBA**            : CORBA機能 ON/OFF - CORBA通信を有効にするためのオプション
* **BUILD_CORBA_PLUGIN**      : CORBAプラグイン - CORBA通信を使用するためのプラグイン。RTコンポーネントと通信するために使用
* **BUILD_OPENRTM_PLUGIN**    : OpenRTMプラグイン - OpenRTM-aistと連携するためのプラグイン

また、必要に応じて以下のオプションを **ON** に設定します。

* **BUILD_OPENRTM_SAMPLES**   : OpenRTMプラグイン向けサンプル - OpenRTM-aist上で動作するRTコンポーネントのサンプル

この設定をした上で、Choreonoidのビルドを行います。

* :doc:`../install/build-ubuntu`
* :doc:`../install/build-windows`

.. _openrtmplugin_setup_corba:

CORBAの設定
-----------

omniORBの最大メッセージサイズの設定
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

OpenRTMを使用する際には、omniORBの最大メッセージサイズを増やしておいた方がよいです。omniORBというのはOpenRTM-aistの実装で使用されているCORBAライブラリで、OpenRTMのインストールの際にはこれもインストールされます。この設定ファイル /etc/omniORB.cfg がありますので、ルート権限でこのファイルを編集します。設定ファイルの中に ::

 giopMaxMsgSize = 2097152   # 2 MBytes.

という記述があるかと思うのですが、これが最大メッセージサイズを表しています。

デフォルトでは2MBとなっているのですが、この場合、例えば画像データやポイントクラウドデータの通信などで一度に2MB以上のサイズのデータを送信しようとすると、うまく送信できないことになってしまいます。2MBという値は小さいので、この値を増やしておきましょう。例えばこれを20MBにする場合、 ::

 giopMaxMsgSize = 20971520

に修正します。

.. _openrtm_install_clear_omninames_cache:

omniNamesのキャッシュのクリア
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

OpenRTMがベースとして利用しているCORBAという通信規格では、「ネームサーバ」というものを使用します。これはCORBAで扱う「CORBAオブジェクト」のネットワーク上でのアドレスを登録するためのものです。omniORBをインストールすると、omniNamesというネームサーバもインストールされ、デフォルトで使用されるようになっています。

このomniNamesについて、登録されていたオブジェクトの情報をOSの再起動時に復帰するという「キャッシュ」の機能があります。このキャッシュによって、存在しないオブジェクトの情報が蓄積してしまい、これがシステムの挙動に影響を与えることがあります。CORBAオブジェクトのアドレスはIPアドレスも含むものなので、ネットワーク上のPC構成が変わったり、ネットワーク自体が変わったりすると、容易にこの問題が発生します。

この問題を避けるため、ネットワーク構成が変わる度にキャッシュをクリアした方がよいです。キャッシュのクリアは

Linuxをご利用の場合は、 **reset-omninames.sh** というシェルスクリプトによってキャッシュをクリアすることができます。これはChoreonoidのビルドディレクトリやインストール先の **bin** ディレクトリにあります。このスクリプトをコマンドラインから、 ::

 reset-omninames.sh

として実行します。（binにパスが通っていない場合はパスもつけるようにしてください。）

このスクリプトの実行には管理者権限が必要です。実行時にはそのためのパスワードを求められた場合は、パスワードを入力して実行してください。

OpenRTM関連の動作がうまくいかない場合、キャッシュが悪さをしていることもありますので、その場合は一度システムを全て止めてから、このスクリプトを実行するとよいかと思います。
