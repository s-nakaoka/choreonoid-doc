高DPIディスプレイへの対応
=========================

.. contents::
   :local:
   :depth: 1

.. highlight:: sh

高DPIディスプレイ使用時の問題
-----------------------------

高DPIディスプレイ（High DIP Display）とは、ピクセル密度が従来の標準的なディスプレイよりも大幅に細かいディスプレイを意味します。

例えば、ノートPCに4K解像度のディスプレイが搭載されている場合は、高DPIに該当することになるでしょう。同じ4Kディスプレイでも、デスクトップ用に28インチのものを使う場合は、高DPIには当てはまらないかもしれません。

問題は、従来標準的な解像度を想定して開発されたアプリケーションを、高DPIのディスプレイに表示して使用する場合に発生します。この場合、アプリケーションの文字やアイコン、画像などがとても小さく表示されてしまい、見づらかったり操作しにくかったりする可能性があります。

これに対処するためには、実際のDPIに応じて文字やアイコン、画像の大きさを変えることが必要となります。しかしながら、従来のアプリケーションでは、そのような設計にはなっていないことが多いです。

Choreonoidについてもまだこの点を考慮した実装になっていないため、高DPIのディスプレイ上でそのまま使用するのは少々つらいところがあります。

Qtの高DPIディスプレイ対応機能
-----------------------------

Choreonoidで使用しているGUIライブラリのQtでは、バージョン5.6から高DPIディスプレイに対応するための機能が備わりました。以下にその説明があります。

* `High DIP Displays <https://doc.qt.io/qt-5/highdpi.html>`_ 

この機能により、高DPIディスプレイへの対応をしていないアプリケーションであっても、高DPIディスプレイ環境ではQtの側で自動的に表示サイズを拡大し、標準的な解像度で使用する場合と同様の表示や操作が可能となっています。Ubuntuの場合、Ubuntu 16.04（のパッケージで入るQt）ではこの機能はありませんが、Ubuntu 18.04で使用する場合はこの機能が有効となるようです。

これで問題なく表示や操作ができる場合はよいのですが、もし表示や操作が想定どおりにならない場合は、以下の環境変数でこの機能の制御を行うことができます。

* QT_AUTO_SCREEN_SCALE_FACTOR
* QT_SCALE_FACTOR
* QT_SCREEN_SCALE_FACTORS

これらの変数はGnomeなどのデスクトップ環境によって自動で設定されるようですが、手動で上書きを行うことで、挙動を変えることができます。基本的には QT_SCALE_FACTOR に表示の拡大率が入っていて、通常はこれが 1 になりますが、高DPIディスプレイに対しては2になります。後者の場合、アプリケーションの表示が元の２倍のサイズに拡大して表示されます。もしこれが問題になる場合は、QT_SCALE_FACTOR に 1 を設定すれば、この機能を無効にできます。

各変数の詳細については上記のページをご参照ください。

なお、Windowsについては、Choreonoid開発者の環境では、高DPIディスプレイへの対応がうまく機能しませんでした。高DPIディスプレイでそのまま表示するとやはりテキストやアイコンなどが小さくなってしまいますが、QT_SCALE_FACTOR に 2 を設定しても、依然として小さいままの部分があり、全体としても正常な表示にはなりませんでした。これについては環境によるところもあるかもしれません。

高DPI対応時のChoreonoidの挙動について
-------------------------------------

Qtのこの機能で高DPIディスプレイに対応している場合に、Choreonoidの一部の機能が正常に動作しない可能性があります。というのも、この機能ではプログラム側からみたら従来と同じサイズ（解像度）で処理しているように見えるのですが、実際には拡大された（高解像度の）処理がなされるので、従来のコードのままでは両者の間で整合性がとれなくなることがあるのです。

例えばシーンビュー上での操作について、従来のコードではオブジェクトをマウスで操作できなくなる不具合があります。従いまして、Choreonoidのリリース済みのバージョン(バージョン1.7まで）では、高DPIディスプレイ対応機能をオフにしてお使いいただく必要があります。

なお、Choreonoidの開発版ではシーンビューの問題は修正してありますので、開発版であれば高DPIディスプレイ対応機能を使用可能です。ただしシーンビュー以外の部分でまだ同様の不具合が残っている可能性があります。もし高DPI環境で表示や操作がおかしくなる不具合が見つかりましたら、とりあえず QT_SCALE_FACTOR を1に設定して使用してください。その上で、不具合の内容を掲示板などでお知らせいただければと思います。
