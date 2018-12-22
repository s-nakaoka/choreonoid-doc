
About items used with OpoenRTM plug-in
=======================

The OpenRTM plug-in uses the following items.

RTSystem Item
----------------------------

| In OpenRTM, the RT function element is called an RT component (RT-Component: RTC), and a robot system (RT system) is constructed using multiple RTCs.
| "RT system item" is a project item for managing the RT system.It is used to manage the components (RTC) of the robot system on the Choreonoid.

The following properties are additionally available for RT system items.

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 15,12,4,75
  :header-rows: 1

  * - Parameters
    - Default value
    - Type
    - Detail
  * - Auto connection
    - True
    - bool
    - Specify whether to automatically execute the connection between the RTCs that make up the RT system when reading items.
  * - Vendor name
    - \-
    - string
    - Set the name of the vendor that created the target RT system.The value set here is used as "VendorName" of RTSProfile when saving the system.The default value can be specified in "Preferences".
  * - Version number
    - \-
    - string
    - Set the target RT system version number.The value set here is used as "Version" of RTSProfile when saving the system.The default value can be specified in "Setting screen".
  * - Profile save destination
    - \-
    - string
    - With the OpenRTM plug-in, information on the RT system is saved using RTSProfile format (format standardized by RT middleware).In this case, specify the save destination of RTSProfile.
  * - Polling Cycle
    - 500
    - int
    - Set the cycle for checking the state of the RTC that constitutes the RT system.The unit is ms.
  * - HeartBeat Period
    - 500
    - int
    - Available in OpenRTM-aist-1.2.0 or later.Sets the heartbeat reception cycle for confirming the presence of RTC.The unit is ms.If the heartbeat signal does not arrive within this cycle, it is judged that some error has occurred in the target RTC.

ControllerRTC Item
----------------------------

It is an item for defining the RTC for controlling the system defined by the RT system item.We will control the target system according to instructions from Choreonoid.

For the controller RTC item the following properties are additionally available:

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 15,12,75
  :header-rows: 1

  * - パラメータ
    - 型
    - 意味
  * - RTCモジュール
    - string
    - 実際に使用するコントローラRTCの名称を設定します。プロパティ編集時にファイル選択ダイアログが表示されますので、使用するコントローラRTCを選択して指定することも可能です。
  * - ベースディレクトリ
    - string
    - 使用するコントローラRTCが存在するディレクトリを設定します。
  * - RTCインスタンス名
    - string
    - コントローラRTCを識別するためのインスタンス名を設定します。OpenRTMでは同じ型のRTCを複数起動することが可能です。このような場合に、実際に使用するコントローラのインスタンスを識別するために指定します。
  * - 実行コンテキスト
    - string
    - コントローラRTCで使用する実行コンテキストを指定します。選択可能な実行コンテキストの種類は、使用しているOpenRTM-aistのバージョンによって異なりますので、詳細はOpenRTM-aistのサイトを参照してください。
  * - 実行周波数
    - int
    - コントローラRTCの実行周期を指定します。




BodyIoRTC Item
----------------------------



RTC Item
----------------------------


BodyRTC Item (Deprecated)
----------------------------

