
AGXDynamicsプラグインのビルドとインストール(Windows編)
-----------------------------------------------------------

| AGXDynamicsプラグインはChoreonoidのソースコードに同梱されております。
| Choreonoidのビルド前に行う cmakeオプションで
| 以下のオプションを **ON** にすることでビルドすることができます。

* **BUILD_AGX_DYNAMICS_PLUGIN**      : AGXDynamicsプラグイン - AGX Dynamicsのシミュレーションプラグイン
* **BUILD_AGX_BODYEXTENSION_PLUGIN** : AGXBodyExtensionプラグイン - 専用モデルプラグイン(ワイヤーなど)

| 以下にビルド、インストール方法の詳細を説明します。
| CMakeを起動します。

.. figure:: images/AGXplugin1.png

| **BUILD_AGX_DYNAMICS_PLUGIN** にチェックを入れ、Configureを押します。AGXのライブラリが自動で検出されます。
| 検出されない場合は、 **AGX_DIR** にインストール先のディレクトリを設定してください。
| **BUILD_AGX_BODYEXTENSION_PLUGIN** にチェックを入れ、もう一度configureを押します。

図のように、" *** Errors might occur during runtime! "という表示がでますが、 :ref:`install-agx-windows-setenv` を行っていれば大丈夫です。

.. note::

   AGXBodyExtensionプラグインはAGXDynamicsプラグインに依存しているため、BUILD_AGX_DYNAMICS_PLUGINがONにならないとccmakeで表示されません。
   一度BUILD_AGX_DYNAMICS_PLUGINをONにしてconfigureを実行してみてください。

Generate を押して、ソリューションファイルを生成します。後は、 :ref:`build-windows-visualstudio` と同様にビルド、インストールを行ってください。