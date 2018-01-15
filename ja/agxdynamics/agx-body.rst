AGXボディ
===========================

AGXDynamicsプラグインでは下記の機能を追加で利用することができます。

干渉検出の設定
--------------

.. code-block:: txt

  collisionDetection:
    excludeTreeDepth: 3
    excludeLinks: [ ]
    excludeLinksDynamic: [ ]
    excludeLinkGroups:
      -
        name: groupA
        links: [ linkA, linkB, linkC, ... ]
      -
        name: groupB
        links: [ linkZ, linkY, linkX, ... ]
    excludeSelfCollisionLinks: [ linkP ]
    enableAGXWireContact: true
    excludeLinksWireContact: [ linkQ, linkR, ... ]

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - パラメータ
    - デフォルト値
    - 単位
    - 型
    - 意味
  * - excludeLinksDynamic
    - \-
    - \-
    - string
    - 指定のリンクの干渉を無効化します
  * - excludeLinkGroups
    - \-
    - \-
    - string
    - グループに登録されているリンク間の干渉を無効化します
  * - excludeSelfCollisionLinks
    - \-
    - \-
    - string
    - 指定のリンクとボディ間の自己干渉を無効化します
  * - (未実装)enableWireContact
    - true
    - \-
    - bool
    - ボディとAGXWireとの干渉を有効、無効化します
  * - (未実装)excludeLinksWireContact
    - \-
    - \-
    - string
    - 指定のリンクとAGXWireとの干渉を無効化します
