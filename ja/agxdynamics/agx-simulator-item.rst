
AGXSimulatorアイテム
=======================

AGXSimulatorアイテムでは以下のプロパティが追加で利用できます。

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 10,9,4,4,75
  :header-rows: 1

  * - パラメータ
    - デフォルト値
    - 単位
    - 型
    - 意味
  * - NumThreads
    - 1
    - スレッド
    - unsigned int
    - AGX Dynamicsで利用するスレッド数。内部で並列計算できる部分(接触判定、ソルバ)を並列化します。topなどでCPU usageを見ると有効になっているか確認できます。
  * - ContactReduction
    - true
    - \-
    - bool
    - 接触点削減機能の有効化、無効化。true、falseを指定します。不要な接触点を削減することにより、ソルバの計算量を減らします。
  * - ContactReductionBinResolution
    - 3
    - 個
    - unsigned int
    - 接触点削減ビン数。1-10を指定します。6次元bin packingアルゴリズムで利用するビン数です。
  * - ContactReductionThreshhold
    - 12
    - 個
    - unsigned int
    - 接触点削減開始閾値。Link同士の接触点が指定閾値以上になると、接触点の削減をします。
  * - ContactWarmstarting
    - false
    - \-
    - bool
    - 接触状態が前回のステップと変わらない場合、前回のソルバの解を利用して収束計算の高速化を行います。
  * - AMOR
    - false
    - \-
    - bool
    - 静止している剛体同士を一体化させ、ソルバの計算量を減らします。true、falseを指定します。合わせて各リンクにを設定する必要があります。詳細は  :doc:`agx-body` をご確認ください。
  * - AutoSleep(非推奨)
    - false
    - \-
    - bool
    - 静止している剛体をソルバから除き、計算量を減らします。true、falseを指定します。合わせて各リンクにautoSleepを設定する必要があります。詳細は  :doc:`agx-body` をご確認ください。
  * - SaveToAGXFileOnStart
    - false
    - \-
    - bool
    - シミュレーション開始時にAGXDynamicsのファイル形式.agxにシーンの保存をします。保存場所はchoreonoidの実行バイナリが置かれているディレクトリまたは実行時のカレントディレクトリです。AGXDynamics単体でのデバック、性能確認に利用することができます。
