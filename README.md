# pointcloud_filters

更新された `rosbag/rosbag2_2026_10_25-08_46_47` を既定で再生し、バックホウ周辺の点群を抽出・自己点群除去・外れ値除去して発行する ROS 2 Humble パッケージです。PointCloud2 と三角形 Mesh の出力を選べます。必要な ZX200 の URDF・メッシュは本パッケージの [robot/](robot/README.md) に同梱しています。

**付属bagに合わせた暫定較正を既定にしています。** `/zx200/global_pose` の XYZ・姿勢はbase_linkとして保持し、CAD原点への変換だけを `XYZ = [-0.17, 0.30, -3.21] m`、`RPY = [0, 0, 0]` としています。補正なしではモデルが実点群より上に浮くため、ユーザーの指示に基づき点群とのロバストな位置合わせから求めました。各リンク形状を中心から元モデルの1.5倍に膨張し、除去余裕は0.30 mです。関節位置やGNSS位置は拡大しません。測量済みの外部較正ではなく、このbag向けの暫定値です。[補正前後の重ね図と検証結果](docs/validation.md) を参照してください。

## 起動

```bash
source /opt/ros/humble/setup.bash
cd /home/kasahara/SIP_PROJECTS/ros2_tms_ws
rosdep install --from-paths src/pointcloud_filters --ignore-src -r -y
colcon build --symlink-install --packages-select pointcloud_filters
source install/setup.bash
ros2 launch pointcloud_filters pointcloud_filter.launch.py
```

既定でフィルタ、更新bag、RViz2を起動します。モデルと購読の準備が完了してからbagを起動し、さらにDDS接続用に2秒の再生開始遅延を設け、必要な `/merged_pointcloud`、`/zx200/global_pose`、`/zx200/joint_states`、`/tf_static` だけを再生します。既定では不要な動的 `/tf` を再生・購読しません。独自メッセージを含む他トピックの再生は不要です。`--clock` を発行し、ノードとRViz2はシミュレーション時刻を使います。フィルタの処理周期は実時間で10秒です。

```bash
# GUIを開かずに再生・処理
ros2 launch pointcloud_filters pointcloud_filter.launch.py use_rviz:=false

# Mesh出力、解像度3（1が粗く5が細かい）
ros2 launch pointcloud_filters pointcloud_filter.launch.py output_format:=mesh mesh_resolution:=3

# 実機など、すでに配信中のトピックを使用（実時間へ自動切替）
ros2 launch pointcloud_filters pointcloud_filter.launch.py play_bag:=false

# 別bag、繰り返し再生、再生速度
ros2 launch pointcloud_filters pointcloud_filter.launch.py \
  bag_path:=/absolute/path/to/recording bag_loop:=true bag_rate:=1.0
```

bagは約2.5 GBのため install へ複製しません。source、symlink-install、通常のcolcon workspace内installから source 配下の既定bagを探します。workspace外へコピーしたinstallでは `bag_path` を指定してください。**手動の `ros2 bag play` と自動再生を同時に実行しないでください。** 手動再生する場合はlaunchに `play_bag:=false use_sim_time:=true` を指定し、手動側は `ros2 bag play <bag> --clock` を使います。`ros2 run` はフィルタのみを起動し、YAMLを自動読込みしません。同じ暫定較正を使う場合は `--ros-args --params-file config/pointcloud_filter.yaml` をsourceディレクトリで指定してください。

## 座標系と関節

測定Poseと関節角度からロボット形状を直接計算するので、`world→base_link` や各関節のTF、`robot_state_publisher` は不要です。点群座標系と測定Pose座標系を結ぶTFは必要です。

```text
world（平面直角座標系IX系の原点）
 ├─ map（実験フィールド原点）     /tf_static
 └─ base_link                    /zx200/global_pose の位置・姿勢
     └─ CADモデル原点             model_origin_xyz/rpy、付属bag用の暫定補正
         └─ crawler/body/boom/arm/bucket  URDF + /zx200/joint_states
```

変換は次の順序です。`T_A_B` はB座標をA座標へ変換する行列です。

```text
T_map_base  = inverse(T_world_map) × T_world_base
T_map_mesh = T_map_base × T_base_model × FK(joint_positions) × collision_origin
```

- bagの `/tf_static` にある `world→map` は並進 `(21395.1661, 14034.4613, 28.558)` m、回転identityです。逆変換が必要で、単純にこの並進を加える処理ではありません。
- bagの `/zx200/global_pose` は `geometry_msgs/msg/PoseStamped` ですが、全件 `header.frame_id` が空です。空の場合に限り `pose_frame: world` を適用します。明記されたframeはそのまま尊重します。
- 初回のmap上のbase位置は約 `(20.2339, 21.5367, 3.9380)` mです。XYだけでなくZとクォータニオンによる姿勢も使います。緯度経度への再変換、XY軸交換、アンテナ高への変換は行いません。CAD原点との差は別の固定変換として扱います。
- bag内の `map→base_link_3d` TFは高さを1.20 mに固定しており、この測定Poseと異なります。自己除去に流用しません。
- 関節角度は `sensor_msgs/msg/JointState` の名前で対応付けます。必須関節は `swing_joint`、`boom_joint`、`arm_joint`、`bucket_joint`。省略関節を0とみなす処理はありません。collisionのない `bucket_end_joint` は必須ではありません。部分的なJointStateも関節ごとの時刻で管理します。
- `model_origin_xyz`（base座標でのm）と `model_origin_rpy`（固定XYZ順、rad）は **base_link→CAD原点** です。同梱YAMLは付属bagから推定した `[-0.17, 0.30, -3.21] m` / `[0,0,0] rad` を使います。較正済みの値がある場合は置き換えてください。別の機体・実験フィールドではこの値をそのまま流用せず、原点関係を確認してください。抽出半径の中心は測定base位置を維持します。

### TF警告と動的TFの利用

付属bagの `map→base_link_3d` は自己位置の計算に使いません。既定では `/tf_static` のみを購読・再生します。別の配信元やbagの再生し直しによって、同じchildに新旧の動的TFが混在すると `TF_OLD_DATA` が大量に出るため、不要なTFをフィルタに取り込みません。

点群とPose間の座標変換自体に動的TFが必要な構成だけ、`use_dynamic_tf:=true` を指定してください。このモードではROS時計の巻き戻りで動的TFの履歴をクリアし、static TFを保持します。異なる配信元が同じchildを同時発行する構成は避けてください。

### 計測時刻

位置・各関節の30秒分の履歴から、点群の **header.stamp** に最も近い実測値を選びます。補間や未来予測は行いません。受信が遅れた古い点群に最新姿勢をそのまま当てることを防ぎます。pose/jointとも点群との最大許容差は既定0.5秒です。

bagの記録時刻・`/clock` とセンサのheader時刻には約146330秒の差があります。状態同期とTF問い合わせにはheader時刻を使い、記録時刻を代用しません。必要な `world→map` はstaticなのでこの差で外挿エラーにはなりません。

状態不足・古い関節・無効な姿勢・TF不足では警告し、その点群の出力を見送って再試行します。仮の原点や仮の関節姿勢では発行しません。1秒を超える計測時刻の巻き戻りでは対応する履歴をリセットし、bagのloopに対応します。

## 処理内容

1. NaN/Infを除き、測定baseのXY位置から半径 `radius_m`（既定10 m）以内を抽出。境界を含み、高さ方向は制限しません。
2. 全主要collisionメッシュを測定姿勢に配置します。各形状のローカル境界の中心から `self_filter_scale`（同梱YAMLでは1.5）倍に拡大し、原形から0.1刻みの中間形状と指定倍率の形状の除去範囲を合成し、拡大で開いた面が移動した際の抜けを抑えます。関節間距離は変えません。三角形表面から `self_filter_padding`（同梱YAMLでは0.30 m）以内の点を除去。水密な連結成分は内部も除去します。開いたCAD面は内外判定が定義できないため表面距離だけを使います。[モデルの制約](robot/README.md) を参照してください。
3. `voxel_size`（既定0.1 m）ごとに元の点を1点残します。自己除去を先に行うため、自己点が代表になって地形点を失うことを防ぎます。
4. Statistical Outlier Removal。近傍20点との3D平均距離が全体の「平均＋標準偏差×2」を超える点を除去します。3点未満ではSORを省略します。
5. 選択された元の点をPointCloud2へ、またはOpen3DでMeshへ変換して発行します。

地面除去、最大クラスタだけの選択は行いません。半径内の離れた領域も保持します。

Meshは専用voxelの平均位置から生成するため、平均頂点が自己モデルに入り直した場合も除去します。三角形の各辺の中点と重心を必ず検査し、辺と内部を最大0.075 m間隔（paddingが小さい場合はさらに細分化）でサンプルして、自己モデルにかかる面も除去します。これは有限間隔の検査であり、連続的な交差判定を保証するものではありません。膨張した形状ではバケット接触部周辺の低い点も追加で除去されます。[膨張前後の比較](docs/validation.md) に点数を記録しています。

| mesh_resolution | Mesh用voxel幅 |
| ---: | ---: |
| 1 | 0.8 m |
| 2 | 0.6 m |
| 3 | 0.4 m |
| 4 | 0.2 m |
| 5 | 0.1 m |

平面は平面内でDelaunay三角形化し、3D面はBall Pivotingで再構成します。法線・球半径は実点間隔に合わせて調整し、過大な長辺や面積ゼロの三角形を除きます。点不足なら空Meshになります。水密性や観測されていない領域の補完は保証しません。

## 出力と表示

| トピック（既定） | 型・用途 |
| --- | --- |
| `/merged_pointcloud_filtered` | PointCloud2、または `output_format:=mesh` なら `shape_msgs/msg/Mesh` |
| `/merged_pointcloud_filtered/self_removed` | PointCloud2。自己モデルで除去した点、voxel/SOR前 |
| `/merged_pointcloud_filtered/header` | Meshモード用Header |
| `/merged_pointcloud_filtered/marker` | MeshモードのRViz用TRIANGLE_LIST Marker |

PointCloud2は元のHeader・全フィールド・point_step・各点の属性/パディングを保持し、height=1で発行します。MeshはXYZと三角形だけでHeaderを持たないため補助トピックを使います。Markerは元Headerを持ち、空MeshではDELETEを発行します。

RViz2では入力が灰色、出力が緑色、自己除去点が赤色です。Fixed Frameは `map`、視点は更新bagの中心付近です。`fixed_frame`、`rviz_config`、`output_topic` で変更できます。赤はvoxel/SORによる間引き点ではなく、自己モデルで除去した点です。緑の点は出力に残っています。[以前の1.3倍設定での画面](docs/rviz_scale13.png) も保存しています。

```bash
ros2 topic info /merged_pointcloud_filtered
ros2 topic hz /merged_pointcloud_filtered
ros2 topic echo /merged_pointcloud_filtered/header --once  # Meshモード
```

最新点群を実時間で10秒ごとに処理します。10秒分の蓄積ではありません。入力が止まった後は最後の未処理点群だけを処理します。処理と受信は別スレッドです。初回出力にはモデル読込み・初回形状検索の時間も加わります。

入力はBest Effort/Volatile。点群はKeep Last 1、位置・関節は100です。出力PointCloud2/Mesh/HeaderはReliable/Volatile/Keep Last 1、MarkerのみTransient Localです。

## 設定

[config/pointcloud_filter.yaml](config/pointcloud_filter.yaml) を編集して再ビルドするか、`params_file:=/absolute/path/to/config.yaml` を指定してください。パラメータは起動時固定です。

| パラメータ | 既定値 | 意味 |
| --- | --- | --- |
| `input_topic` | `/merged_pointcloud` | 入力点群 |
| `pose_topic` | `/zx200/global_pose` | world等でのbase位置・姿勢 |
| `pose_frame` | `world` | Poseのframe_idが空の場合の座標系 |
| `joint_states_topic` | `/zx200/joint_states` | 関節角度 |
| `self_filter` | `true` | 自己除去を有効化 |
| `self_filter_padding` | `0.30` | 拡大後もm単位で保つ除去余裕 |
| `self_filter_scale` | `1.50` | 各リンク形状の膨張倍率、1以上 |
| `use_dynamic_tf` | `false` | `true`なら `/tf` も購読・bag再生 |
| `robot_description_path` | 空 | 空なら同梱URDF |
| `model_origin_xyz` | `[-0.17, 0.30, -3.21]` | 測定base→CAD原点の暫定並進、m |
| `model_origin_rpy` | `[0,0,0]` | 測定base→CAD原点の固定XYZ回転、rad |
| `max_pose_time_diff_sec` | `0.5` | poseとcloudの最大時刻差、0は無制限 |
| `max_joint_time_diff_sec` | `0.5` | 各関節とcloudの最大時刻差、0は無制限 |
| `state_history_sec` | `30.0` | 履歴保持時間 |
| `radius_m` | `10.0` | XY円形抽出半径、m |
| `voxel_size` | `0.1` | フィルタvoxel幅、m、0で無効 |
| `sor_mean_k` / `sor_stddev_mul` | `20` / `2.0` | SOR近傍数・標準偏差倍率 |
| `output_topic` | `/merged_pointcloud_filtered` | 出力先 |
| `output_format` | `pointcloud2` | `pointcloud2` / `mesh` |
| `mesh_resolution` | `3` | Mesh解像度、整数1〜5 |
| `publish_period_sec` | `10.0` | 実時間での処理周期、秒 |

主な設定は同名launch引数でも指定できます。`model_origin_*`、時刻差、履歴長などはYAMLで指定してください。`self_filter:=false` なら従来の位置中心の抽出を行い、関節と姿勢は不要です。この場合だけ、固定中心用のゼロstamp Poseも使用できます。

主な依存はrclpy、tf2、標準ROSメッセージ、NumPy、SciPy、PyYAML、Open3Dです。自己除去には `RaycastingScene` 対応のOpen3Dが必要で、本環境は0.16.0で検証しています。自己除去とMesh出力を両方無効にした場合はOpen3Dを読み込みません。

## 検証の再実行

```bash
source /opt/ros/humble/setup.bash
cd /home/kasahara/SIP_PROJECTS/ros2_tms_ws
colcon test --packages-select pointcloud_filters
colcon test-result --verbose
```

全bagの検証はパッケージのsourceディレクトリから実行します。実機トピックへの発行は行いません。

```bash
cd /home/kasahara/SIP_PROJECTS/ros2_tms_ws/src/pointcloud_filters
PYTHONPATH=.:$PYTHONPATH OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  python3 scripts/validate_bag.py \
  --report /tmp/pointcloud_filters_validation.json \
  --preview /tmp/pointcloud_filters_overlay.png
```

全163フレームのHeader・全点属性保持、有限性、半径、指定モデル上の除去残存0、最寄り状態の時刻差を確認し、先頭・中央・末尾でMesh全5段階も検査します。`--bag`、`--params` でデータ/較正設定を変更できます。単体テスト・ROS実通信・bag再生の結果と、暫定較正の根拠・制約は [docs/validation.md](docs/validation.md) に記録しています。
