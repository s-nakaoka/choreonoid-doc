ROSのインストール
=================

.. highlight:: sh

ROSがまだインストールされていない場合h、 `ROS.org <http://wiki.ros.org>`_ - `ROS/Installation <http://wiki.ros.org/ROS/Installation>`_ の記述に従ってインストールを行ってください。

ROSのバージョンについては、Ubuntu 16.04上でKinetic、Ubuntu 18.04上でMelodicでの動作を確認をしています。

これらのOSに関しては以下のコマンドでROS環境をインストールできます。

Ubuntu 16.04 (ROS Kinetic Kame) の場合 ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
 sudo apt-get update
 sudo apt-get install ros-kinetic-desktop-full
 sudo rosdep init
 rosdep update
 echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc

Ubuntu 18.04 (ROS Melodic Morenia) の場合 ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
 curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
 sudo apt update
 sudo apt install ros-melodic-desktop-full
 sudo rosdep init
 rosdep update
 echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
 source ~/.bashrc

.. note:: 最後の source コマンドは、setup.bash の内容を現在のシェルに反映させるためのもので、インストール（上記設定）直後に続けて同じシェルで作業する場合に必要となるものです。インストール後にあらためてシェルを起動する場合は、上記の設定によりsetup.bashの内容が反映されますので、このコマンドは必要ありません。

.. note:: apt-keyコマンドで取得する鍵には通常有効期限が設定されていて、期限が切れるとリポジトリへのアクセスなどができなくなってしまうようです。その場合は上記のapt-keyコマンドを再度実行し鍵を更新することで、リポジトリへアクセスできるようになります。

ChoreonoidをROSで使う場合、ビルドツールCatkinの新しいバージョン ( `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_）を使用します。これは ::

 sudo apt install python-catkin-tools

でインストールすることができます。

