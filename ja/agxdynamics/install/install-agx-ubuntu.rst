
AGX Dynamicsのインストール(Ubuntu Linux編)
============

| AGX Dynamics Ubuntu版のインストール方法について説明します。
| AGX Dynamicsダウンロードサイトからdebパッケージをダウンロードし、/var/tmpディレクトリに保存している状態から始めます。
| 本稿では説明のためにagx-setup-2.19.1.2-x64-ubuntu_16.04-double.debをインストールすることとします。

インストール
----------------------------

| AGX Dynamicsのインストールは下記のコマンドを実行することで行います。デフォルトでは/opt/Algoryx/AgX-<version>ディレクトリにインストールされます。
| 次にAGX実行ライセンスファイルをインストールディレクトリに配置し、AGX Dynamicsを実行できるようにします。
| 最後に.profileファイルに環境変数設定スクリプトの実行を記述し、OSログイン時に自動的に環境変数が設定されるようにします。

.. code-block:: txt

   # インストール
   cd /var/tmp
   sudo dpkg -i agx-setup-2.19.1.2-x64-ubuntu_16.04-double.deb    // パッケージインストール
   ls -al /opt/Algoryx/AgX-2.19.1.2                               // インストールディレクトリにファイルが配置されていることを確認

   # AGX実行ライセンスファイルの配置
   sudo cp -i agx.lic  /opt/Algoryx/AgX-2.19.1.2

   # 環境変数の設定
   cd ~                                                           // ホームディレクトリに移動
   cp -p .profile .profile_20171010                               // .profileファイルのバックアップ
   echo "source /opt/Algoryx/AgX-2.19.1.2/setup_env.bash" >> .profile
   diff .profile .profile_20171010                                // 変更部分以外に差分がないか確認
   source .profile                                                // ログインシェルに環境変数を設定
   env | grep -i agx                                              // AGX_DIRやAGX_BINARY_DIRなどが表示されること


動作確認
----------------------------

実行ライセンスファイルが配置されているか、環境変数が設定されているかを確認するために、
AGX Dynamicsのサンプルを実行して動作確認をします。

.. code-block:: txt

   # AGX Dynamicsの動作確認
   cd /opt/Algoryx/AgX-2.19.1.1/bin
   ./tutorial_trackedVehicle


.. note::
   VMWareなどの仮想マシン上ではウィンドウが開かず失敗する場合があります。

   .. code-block:: txt

      $ ./tutorial_trackedVehicle
         AGX Library 64Bit AgX-2.19.1.1-81db33e Algoryx(C)
         Tutorial Tracked Vehicle (agxVehicle::Track)
        --------------------------------
      Caught exception: Failed to find window with number: 0
   ..

   この場合は以下のコマンドで確認をしてください。--agxOnlyは描画なし、--stopAt 5は5秒シミュレーションをしたのち終了するオプションです。

   .. code-block:: txt

      ./tutorial_trackedVehicle --agxOnly --stopAt 5
         AGX Library 64Bit AgX-2.19.1.1-81db33e Algoryx(C)
         Tutorial Tracked Vehicle (agxVehicle::Track)
         --------------------------------
      Loading scene took 0.236783 sec
      Stepping 301 timesteps (5.01667 sec simulated time) took 1.69487 sec
