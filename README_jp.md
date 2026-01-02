# Spresense IMU Localizer ROS 2 ノード

[English](./README.md) | 日本語

このROS 2パッケージは、[cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino)を実行しているSpresenseボードからのシリアル出力をROSメッセージに変換するノードを提供します。IMUベースの自己位置推定データをROSエコシステムとリアルタイムで可視化・統合することができます。

## 概要

本ノードは、Spresenseボードからシリアルデータを読み取り、位置推定出力を解析して、標準的なROS 2メッセージとしてパブリッシュします：

- **IMUデータ** (sensor_msgs/Imu): 角速度、線形加速度、姿勢
- **Poseデータ** (geometry_msgs/Pose): 位置と姿勢
- **TF変換**: センサの位置姿勢をTF変換としてブロードキャスト

## 特徴

- **シリアル通信:** Spresenseから115200ボーでhexエンコードされたデータを読み取り
- **ROS 2統合:** 標準ROSメッセージ型をパブリッシュし、容易な統合を実現
- **TFブロードキャスト:** RVizでの可視化のためにセンサの位置姿勢をTF変換としてパブリッシュ
- **設定可能なパラメータ:** シリアルポート、ボーレート、TFフレーム名をROSパラメータで設定可能

## ハードウェア要件

- **Sony Spresense メインボード**（CXD5602プロセッサ搭載）
- **Spresense IMUアドオンボード**（CXD5602PWBIMU）
- **USB接続**（SpresenseとROS 2ホスト間）

## ソフトウェア要件

- **ROS 2**（Humble以降推奨）
- **Python 3**
- **pyserial** ライブラリ
- Spresenseで動作する **[cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino)** ファームウェア

## インストール

### 1. リポジトリのクローン

```bash
cd ~/ros2_ws/src
git clone https://github.com/hijimasa/cxd5602pwbimu_localizer_node.git
```

### 2. 依存関係のインストール

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install pyserial
```

### 3. パッケージのビルド

```bash
cd ~/ros2_ws
colcon build --packages-select cxd5602pwbimu_localizer_node
source install/setup.bash
```

## 使い方

### ノードの実行

```bash
ros2 run cxd5602pwbimu_localizer_node localizer_node
```

### カスタムパラメータでの実行

```bash
ros2 run cxd5602pwbimu_localizer_node localizer_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p tf_parent_frame:=world \
  -p tf_child_frame:=sensor
```

## パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `serial_port` | `/dev/ttyUSB0` | Spresenseに接続されたシリアルポート |
| `baud_rate` | `115200` | シリアル通信のボーレート |
| `tf_parent_frame` | `world` | TFブロードキャストの親フレーム |
| `tf_child_frame` | `sensor` | TFブロードキャストの子フレーム |

## パブリッシュするトピック

| トピック | 型 | 説明 |
|---------|-----|------|
| `/imu/data_raw` | `sensor_msgs/Imu` | IMUデータ（角速度、線形加速度、姿勢） |
| `/pose` | `geometry_msgs/Pose` | センサの位置姿勢（位置と向き） |
| `/tf` | `tf2_msgs/TFMessage` | 親フレームからセンサフレームへの変換 |

## RVizでの可視化

1. RVizを起動:
   ```bash
   rviz2
   ```

2. ディスプレイを追加:
   - **TF** ディスプレイを追加してセンサフレームを可視化
   - **Pose** ディスプレイを追加し、トピックを `/pose` に設定
   - **Imu** ディスプレイを追加（rviz_imu_pluginがインストールされている場合）し、トピックを `/imu/data_raw` に設定

3. Fixed Frameを `world`（または設定した `tf_parent_frame`）に設定

## データフォーマット

本ノードは、以下のhexエンコードされたカンマ区切りフォーマットのシリアルデータを期待します：

```
タイムスタンプ, 温度, gx, gy, gz, ax, ay, az, qw, qx, qy, qz, vx, vy, vz, px, py, pz
```

このフォーマットは [cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino) ファームウェアから出力されます。

## トラブルシューティング

### よくある問題

1. **シリアルポートのアクセス権限エラー:**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # または、dialoutグループにユーザーを追加（恒久的な解決策）:
   sudo usermod -a -G dialout $USER
   ```

2. **シリアルポートが見つからない:**
   - Spresenseが接続されているか確認: `ls /dev/ttyUSB*`
   - パラメータで正しいポートが設定されているか確認

3. **データが受信されない:**
   - Spresenseでcxd5602pwbimu_localizer_arduinoファームウェアが動作していることを確認
   - ボーレートが一致しているか確認（115200）

## ライセンス

このプロジェクトはMITライセンスの下で配布されています。詳細は `LICENSE` ファイルをご確認ください。

## 関連プロジェクト

- [cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino) - Spresense IMU自己位置推定用Arduinoファームウェア

---

ご質問や問題がございましたら、GitHub Issueを作成するか、リポジトリ管理者にお問い合わせください。
