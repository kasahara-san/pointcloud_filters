# pointcloud_filters

D37PXiの搭載LiDARから機体自身の点を除き、周囲の点群を間引いて表示するROS 2 Humbleパッケージです。

## 起動

このディレクトリで実行します。入力トピックは配信済みのものを使います。

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log build --base-paths . --symlink-install --packages-select pointcloud_filters
source install/setup.bash
QT_QPA_PLATFORM=xcb ros2 launch pointcloud_filters d37pxi.launch.py
```

フィルタとRVizを起動します。bagの追加再生は行いません。

## 入出力

| 用途 | トピック |
| --- | --- |
| 入力点群 | `/lidar_points` |
| 機体位置・姿勢（map基準） | `/d37pxi/global_pose` |
| 関節角度 | `/d37pxi/joint_states` |
| 処理後の点群 | `/d37pxi/lidar_points_filtered` |

## 現在の設定

[config/d37pxi.yaml](config/d37pxi.yaml)で変更できます。変更後はフィルタを再起動してください。

| 項目 | 設定 |
| --- | --- |
| 抽出範囲 | 機体中心からmap水平面で半径10 m |
| 更新周期 | 0.5秒（約2 Hz） |
| 間引き幅 `voxel_size` | **0.23 m**。大きくすると点数が減ります |
| 自己除去 | 各リンクの凸包内部と周辺を除去 |
| 除去範囲の拡大 | 形状1.2倍＋余裕0.35 m |

検証データでは1スキャン約2,700〜3,100点で、間引き幅0.12 mのときの約1/3です。
自己点の除去を優先するため、機体直近の地形も少し広めに除去します。

## RVizの見方

- **緑**：処理後の点群。
- **黄**：関節角に追従する機体CAD。
- **赤**：除去済みの点。初期状態はオフ。
- **灰**：元の点群。初期状態はオフ。

左のチェックボックスで表示を切り替えます。
出力座標系は `hesai_lidar`、RVizのFixed Frameは `d37pxi/filter_base` です。

## 別のデータで使う場合

LiDARの取付位置と時刻補正は、2026-11-20の記録に合わせた暫定値です。
取付や記録を変えた場合は `sensor_xyz`・`sensor_rpy`・`cloud_time_offset_sec` を見直してください。
観測していない領域の補完や、過去スキャンの蓄積は行いません。

[較正・検証・Mesh出力の詳細](docs/d37pxi_details.md)
