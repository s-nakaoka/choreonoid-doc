
OpenRTMプラグインのビルド方法
----------------------------------------

| OpenRTMプラグインはChoreonoidのソースコードに同梱されております。
| Choreonoidのビルド前に行う :ref:`build-ubuntu-cmake` のcmakeオプションで以下のオプションを **ON** にすることでビルドすることができます。

* **BUILD_OPENRTM_PLUGIN**    : OpenRTMプラグイン - OpenRTM-aistと連携するためのプラグイン
* **BUILD_CORBA_PLUGIN**      : CORBAプラグイン - CORBA通信を使用するためのプラグイン。RTコンポーネントと通信するために使用
* **ENABLE_CORBA**            : CORBA機能 ON/OFF - CORBA通信を有効にするためのオプション

| また、必要に応じて以下のオプションを **ON** に設定します。

* **BUILD_OPENRTM_SAMPLES**   : OpenRTMプラグイン向けサンプル - OpenRTM-aist上で動作するRTコンポーネントのサンプル

| 以下にビルド、インストール方法の詳細を説明します。
| まず、choreonoidのソースディレクトリに移動し、ccmakeまでコマンドを実行します。

.. code-block:: txt

   cd choreonoid
   cmake .
   ccmake .

ccmakeで以下のオプションを有効にし、configure、generateを実行をします。

* BUILD_CORBA_PLUGIN      ON
* BUILD_OPENRTM_PLUGIN    ON
* ENABLE_CORBA            ON
* BUILD_OPENRTM_SAMPLES   ON (オプション)

Cmake Errorがでていないことを確認し、以下のようにmake、make installを実行しビルド、インストールをします。

.. code-block:: txt

   make -j4
   make install

.. note::

   ccmake configureを実行するとOpenRTM-aistのパス(OPENRTM_DIR)が自動的に設定されます。もしも、設定されていない場合には、手動で設定をしてください。

