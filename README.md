# raspike-ros-firmware

raspike-ros-firmware is a micro-ROS module for SPIKE Prime.

This repository was extracted from the original [raspike-ros](https://github.com/Hiyama1026/raspike-ros) repository for modular reuse.

## Build and Update firmware

```bash
git clone https://github.com/owhinata/raspike-ros-firmware.git
cd raspike-ros-firmware
make -f setup_micro-ros.mk setup_micro-ros  # libmicroros.aのビルド
make asp.dfu

# update firmware
python pydfu.py -u asp.dfu --vid 0x0694 --pid 0x0008
```
- ⚠️ make asp.dfu は初回ビルド時にリンクエラーが発生する場合があります。その場合は、再度 make asp.dfu を実行してください。
- 👀 libmicroros.a のビルドは、ROS2のメッセージ定義を変更しない限り1回のみビルドすればよいです
- 👀 asp.dfuの書き込みはpyusbのインストールとsudoでの実行が必要です。

## デバッグ用UART出力

普段は、PortA~Fはデバイスを接続する設定になっているのですが、さすがにデバッグで困るので、以下の箇所のコメントアウトを外すと、PortFをUARTとして利用し、ログ出力を可能にすることができます。

https://github.com/emtechstars/raspike-ros-firmware/blob/48148ecd10c7b7550e687f06c62f314a21eec4f2/pybricks.mk#L17

## IMUキャリブ結果の反映方法

IMUのキャリブレーションで得られた .calibファイル(後述)は、[calib/](https://github.com/emtechstars/raspike-ros-firmware/tree/main/calib) ディレクトリに以下のようにリネームして配置します。

- test_imu_acc.calib → spike1_test_imu_acc.calib
- test_imu_gyro.calib → spike1_test_imu_gyro.calib

<details><summary>その後、pybricks.mk の PYBRICKS_SPIKE_ID にプレフィックス（例：spike1_）を指定することで、これらのキャリブ結果がビルドに反映されます。</summary>

```bash
# (1) .calibファイルを配置する
$ ls -l calib
合計 16
-rw-rw-r-- 1 ouwa ouwa 221  4月 29 17:15 spike1_test_imu_acc.calib
-rw-rw-r-- 1 ouwa ouwa 226  4月 29 17:15 spike1_test_imu_gyro.calib
-rw-rw-r-- 1 ouwa ouwa  51  4月 29 17:15 test_imu_acc.calib
-rw-rw-r-- 1 ouwa ouwa  51  4月 29 17:15 test_imu_gyro.calib

# (2) 参考: .calibファイルの例
$ cat calib/spike1_test_imu_acc.calib
          1 -0.00173681  0.00131414
          0           1 0.000770331
         -0           0           1

 1.00391        0        0
       0 0.994432        0
       0        0 0.997833

0.127158
0.102372
-0.0812641

# (3) pybricks.mkでcalibファイルを指定する
# PYBRICKS_SPIKE_ID := とした場合は、キャリブ結果は反映せずにセンサー値のパススルーとなる
$ cat pybricks.mk
...
# PYBRICKS_SPIKE_ID :=
PYBRICKS_SPIKE_ID := spike1_
```
</details>

## IMUキャリブレーション方法

<details><summary>IMUのキャリブレーションには、以下のツールを使用します。</summary>

👉 [imu_tk](https://bitbucket.org/alberto_pretto/imu_tk/src/master/)

このツールはやや古いため、最近のUbuntu環境ではビルドが難しく、Dockerコンテナで用意しました。以下の手順でセットアップできます。

```bash
git clone https://github.com/owhinata/ubuntu-imu_tk.git
cd ubuntu-imu_tk
docker build --network host -t ubuntu-imu_tk .
```
</details>

**キャリブレーションの実施方法**

<details><summary>以下は実際のキャリブレーション風景です。SPIKE Hubを三脚や雲台を使って固定しています。</summary>

![20250430_01](https://github.com/user-attachments/assets/3b8d66f2-f66b-48b3-916a-ced1410ff9ac)

右の図のように、SPIKE Hubをさまざまな向きに回転させて、各姿勢で10秒ほど静止させる操作を繰り返します。 回転は以下のような手順で行います：

- Hubを垂直方向に約30度ずつ回転
- 各位置で球面上を約45度ずつ回転
- 天頂・天底方向を含め、合計で42通りの姿勢で静止
</details>

**IMUデータの収集**

<details><summary>SPIKE HubをPCにUSB接続し、micro-ROS Agentを起動します。</summary>


micro-ROS Agentのビルド方法
```bash
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo rosdep init
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash
```

micro-ROS Agentの起動
```bash
$ cd ros2_ws
$ . install/setup.bash
$ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
[1745997538.646518] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1745997539.318963] info     | Root.cpp           | delete_client            | delete                 | client_key: 0x5BDB56D5
[1745997539.318998] info     | SessionManager.hpp | destroy_session          | session closed         | client_key: 0x5BDB56D5, address: 0
[1745997539.319010] info     | Root.cpp           | create_client            | create                 | client_key: 0x7C9D0577, session_id: 0x81
[1745997539.319014] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x7C9D0577, address: 0
[1745997539.325627] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x7C9D0577, participant_id: 0x000(1)
[1745997539.338152] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7C9D0577, topic_id: 0x000(2), participant_id: 0x000(1)
[1745997539.343176] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x7C9D0577, publisher_id: 0x000(3), participant_id: 0x000(1)
[1745997539.353553] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x7C9D0577, datawriter_id: 0x000(5), publisher_id: 0x000(3)
[1745997539.373980] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7C9D0577, topic_id: 0x001(2), participant_id: 0x000(1)
[1745997539.378333] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x7C9D0577, publisher_id: 0x001(3), participant_id: 0x000(1)
[1745997539.389057] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x7C9D0577, datawriter_id: 0x001(5), publisher_id: 0x001(3)
[1745997539.408259] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7C9D0577, topic_id: 0x002(2), participant_id: 0x000(1)
[1745997539.413184] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x7C9D0577, publisher_id: 0x002(3), participant_id: 0x000(1)
[1745997539.424758] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x7C9D0577, datawriter_id: 0x002(5), publisher_id: 0x002(3)
[1745997539.443261] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7C9D0577, topic_id: 0x003(2), participant_id: 0x000(1)
[1745997539.448509] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x7C9D0577, subscriber_id: 0x000(4), participant_id: 0x000(1)
[1745997539.463753] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x7C9D0577, datareader_id: 0x000(6), subscriber_id: 0x000(4)
[1745997539.478427] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7C9D0577, topic_id: 0x004(2), participant_id: 0x000(1)
[1745997539.483889] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x7C9D0577, subscriber_id: 0x001(4), participant_id: 0x000(1)
[1745997539.498595] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x7C9D0577, datareader_id: 0x001(6), subscriber_id: 0x001(4)
[1745997539.513808] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7C9D0577, topic_id: 0x005(2), participant_id: 0x000(1)
[1745997539.518421] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x7C9D0577, subscriber_id: 0x002(4), participant_id: 0x000(1)
[1745997539.533618] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x7C9D0577, datareader_id: 0x002(6), subscriber_id: 0x002(4)
[1745997539.553612] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7C9D0577, topic_id: 0x006(2), participant_id: 0x000(1)
[1745997539.558513] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x7C9D0577, subscriber_id: 0x003(4), participant_id: 0x000(1)
[1745997539.573587] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x7C9D0577, datareader_id: 0x003(6), subscriber_id: 0x003(4)
```
</details>

その後、IMUのデータを記録するノードを起動します

```bash
$ cd ros2_ws
$ . install/setup.bash
$ ros2 run raspike_ros_imu_recorder imu_recorder
```

このノードにより、IMUのデータが xsens_acc.mat（加速度）と xsens_gyro.mat（角速度）として保存されます。

**キャリブレーション結果の生成**

最後に、imu_tkを使ってキャリブレーションパラメータを生成します。

```bash
docker run -it --rm --network host \
-u `id -u`:`id -g` \
-v /etc/passwd:/etc/passwd \
-v /etc/group:/etc/group \
-v $PWD:$PWD -w $PWD \
ubuntu-imu_tk \
test_imu_calib xsens_acc.mat xsens_gyro.mat
```

(参考) 👉 https://emtechs-blog.blogspot.com/2025/04/et-8.html
