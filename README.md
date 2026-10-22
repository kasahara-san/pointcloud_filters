# pointcloud_filters

ROS 2の`/merged_pointcloud`（`sensor_msgs/msg/PointCloud2`）から、バックホウの位置を中心に**XY平面の半径`radius_m`以内**の点を抽出し、外れ値を除去して`/merged_pointcloud_filtered`に発行します。中心位置は`/zx200/global_pose`（`geometry_msgs/msg/PoseStamped`）から取得します。出力は起動時パラメータでPointCloud2または`shape_msgs/msg/Mesh`を選べます。どちらも既定で10秒周期です。ROS 2 Humble / Python 3.10で動作確認しています。

## 処理内容

1. 最新の点群と最新のバックホウ位置を取得。位置の座標系が異なる場合は、TFで点群の座標系に変換。
2. XYZにNaN・Infを含む点を除去し、`hypot(x - center_x, y - center_y) <= radius_m`を満たす点を残す。半径の既定値は10 mで、境界上の点も含みます。
3. 0.1 mのボクセルごとに元の点を1点残す。同じ場所に重なる点の影響と処理量を抑えます。`voxel_size: 0.0`で無効にできます。
4. Statistical Outlier Removal（SOR）を適用。各点から近い20点への3次元平均距離を求め、その値が全体の「平均 + 標準偏差 × 2.0」を超える点を除去。

半径判定にはZ座標やバックホウの向きは使いません。点群の座標系のXY平面における円形範囲で、高さ方向の制限はありません。SORは3次元距離を使うため、半径内でも周囲から離れた点は除去される場合があります。SORの近傍数は点数が少ない場合は自動で減らし、3点未満ではSORを省略します。

密度による選別・クラスタ分類・最大領域の選択は行いません。円内に複数の離れた領域があれば、SORで残ったものをまとめて出力します。地面除去は行いません。

## バックホウ位置・座標系・時刻

- `pose_topic`の既定値は`/zx200/global_pose`、型は`geometry_msgs/msg/PoseStamped`です。既存のZX200ナビゲーション側の購読コードに合わせています。実機・元bag側でも`ros2 topic info /zx200/global_pose`で確認できます。
- 点群と位置の両方に、有効な`header.frame_id`が必要です。同じ座標系なら直接XYを使います。例えば点群が`map`、位置が`world`なら、`world`から`map`へのTFを用意してください。位置メッセージの計測時刻でTFを取得します。バックホウ自体の姿勢クォータニオンは使いません。
- 最新の点群・位置を処理周期の開始時に取り出します。厳密な時刻同期や位置の補間は行いません。両方のスタンプが非ゼロの場合は、その差が`max_pose_time_diff_sec`（既定2秒）以内か確認します。超えた場合は発行を見送ります。`0.0`にすると時刻差チェックを無効にできます。
- どちらかのスタンプがゼロの場合は時刻差を判定できないため、このチェックを省略します。位置のスタンプがゼロでTFが必要な場合は、最新のTFを使います。
- 位置が未受信・非有限値・座標系不明・時刻差超過・TF未取得の場合は警告を出し、原点などの仮の位置では処理しません。最新の点群を保持し、次の周期で再確認します。

## ビルド

```bash
source /opt/ros/humble/setup.bash
cd /home/common/SIP_PROJECTS/M2/SIMIZU_PROJECT/ros2_ws
rosdep install --from-paths src/pointcloud_filters --ignore-src -r -y
colcon build --symlink-install --packages-select pointcloud_filters
source install/setup.bash
```

主な依存は`rclpy`、`tf2_ros_py`、`sensor_msgs`、`sensor_msgs_py`、`shape_msgs`、`geometry_msgs`、`std_msgs`、`visualization_msgs`、RViz2、PyYAML、NumPy、SciPy、Open3Dです。`rosdep`で解決できます。Open3DはMeshモードで使用し、PointCloud2モードでは読み込みません。Ubuntu 22.04では`python3-open3d`パッケージを利用できます。

## 起動

ターミナル1でフィルタを起動します。既定の出力は従来どおりPointCloud2で、動作確認用のRViz2も同時に開きます。

```bash
source /opt/ros/humble/setup.bash
source /home/common/SIP_PROJECTS/M2/SIMIZU_PROJECT/ros2_ws/install/setup.bash
ros2 launch pointcloud_filters pointcloud_filter.launch.py radius_m:=10.0
```

Mesh出力にする場合は、代わりに次のように起動します。

```bash
ros2 launch pointcloud_filters pointcloud_filter.launch.py \
  output_format:=mesh mesh_resolution:=3 radius_m:=15.0
```

`output_format:=pointcloud2`を指定すればPointCloud2出力になります。`radius_m`・`pose_topic`・`output_format`・`mesh_resolution`・`output_topic`のlaunch引数を省略した場合は、YAMLの値を使います。指定したlaunch引数はYAMLより優先します。

どちらのモードも出力トピックの既定名は`/merged_pointcloud_filtered`で、型が選択したモードに変わります。両モードを比較するため別のトピックで出したい場合は、例えば`output_topic:=/merged_pointcloud_mesh`を追加してください。

GUIを使わない場合は`use_rviz:=false`を追加します。

```bash
ros2 launch pointcloud_filters pointcloud_filter.launch.py use_rviz:=false
```

`ros2 run`で直接起動する場合、RViz2は自動起動しません。

ターミナル2で付属rosbagを再生します。すでに入力トピックが発行されている場合は、フィルタの起動だけで構いません。

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/common/SIP_PROJECTS/M2/SIMIZU_PROJECT/ros2_ws/src/pointcloud_filters/rosbag/rosbag2_2026_10_20-15_55_57_merged4
```

**付属の`merged4`バッグは点群だけを含み、`/zx200/global_pose`とTFは含みません。** 実際のバックホウ位置で動かすには、位置を含む元bagなどから、点群と時刻・座標系が対応する位置トピックも発行してください。

付属bagだけで表示を試す場合は、ターミナル3から明示的な仮の位置を発行できます。これは**実際のバックホウ位置ではなく、確認用の固定中心`(25, 40)` m**です。スタンプを省略してゼロにしているため、時刻差チェックは省略されます。

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --rate 1 /zx200/global_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 25.0, y: 40.0, z: 0.0}, orientation: {w: 1.0}}}'
```

繰り返す場合は`ros2 bag play`に`--loop`を付けます。`--clock`で再生し、他のノードとシミュレーション時刻を使う場合は、launchに`use_sim_time:=true`を指定できます。フィルタの10秒周期は実時間で動きます。

launchを使わず起動することもできます。

```bash
ros2 run pointcloud_filters pointcloud_filter --ros-args \
  -p publish_period_sec:=10.0 \
  -p radius_m:=10.0 \
  -p pose_topic:=/zx200/global_pose
```

```bash
ros2 run pointcloud_filters pointcloud_filter --ros-args \
  -p output_format:=mesh \
  -p mesh_resolution:=5
```

## Mesh出力と5段階の解像度

Meshモードでは、バックホウ位置からの半径抽出とSORまで同じ処理を行い、その結果をメッシュ化します。メッシュ専用のボクセル処理、法線推定、[Open3DのBall Pivoting法](https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html)による三角形生成を行います。平面に対しては、その平面上の2次元座標に投影してDelaunay三角形化することで、規則的な点配置でも安定して面を作ります。水平な地形だけでなく、垂直面や曲面にも対応します。

`mesh_resolution`は**1が最も粗く、5が最も細かい**設定です。既定値は3です。

| `mesh_resolution` | メッシュ用ボクセルの一辺 | 傾向 |
| ---: | ---: | --- |
| 1 | 0.8 m | 最も粗い・軽量 |
| 2 | 0.6 m | 粗い |
| 3 | 0.4 m | 標準 |
| 4 | 0.2 m | 細かい |
| 5 | 0.1 m | 最も細かい・処理量が多い |

ここでのボクセル処理は、フィルタ用の`voxel_size`とは別です。各ボクセル内の点の平均座標をメッシュの候補頂点にします。設定値は頂点間隔の目安であり、点群の密度や形状によって三角形の大きさ・数は変わります。入力やフィルタ後の点間隔より細かいディテールを新しく作る処理ではないため、レベルを上げても結果があまり変わらない場合があります。

法線推定半径とBall Pivotingの球半径は、レベルと実際の点間隔に合わせて自動調整します。点間隔の基準値は「メッシュ用ボクセル幅」と「最近傍距離の中央値」の大きい方で、法線推定半径はその3倍、球半径は1.2倍・2倍です。長辺が基準値の3倍を超える三角形と面積ゼロの三角形を除去します。範囲内で分断された面ができても、最大の面だけを選ばず、すべての有効な面を発行します。

表面の点が不足している場所は穴になる場合があります。閉じた立体や水密なメッシュを保証する処理ではありません。3点未満・一直線上の点など、面を作れない場合は空のMeshを発行します。

`shape_msgs/msg/Mesh`は`vertices`と`triangles`だけを持ち、**Headerがありません**。頂点座標は入力と同じ座標系で、時刻・座標系を確認するために`<output_topic>/header`（`std_msgs/msg/Header`）も同じ処理周期で発行します。既定名は`/merged_pointcloud_filtered/header`です。MeshとHeaderは別メッセージのため、厳密な時刻同期を保証する形式ではありません。MeshにはXYZと三角形の頂点インデックスを格納し、intensity・RGBなどの点属性は含みません。

## 出力の確認

```bash
ros2 topic info /merged_pointcloud_filtered
ros2 topic hz /merged_pointcloud_filtered
```

入力が継続している間の出力は約0.1 Hzです。最初の発行はノードの初期化完了から10秒後のタイマー処理が完了した時点です。`hz`は複数回の発行を待つため、結果表示まで数十秒かかります。

同時起動するRViz2は、入力点群を灰色、半径抽出・外れ値除去後の結果を緑色で重ねて表示します。PointCloud2モードでは点群を、Meshモードでは三角形の面を表示する設定を自動で選びます。点群と有効な位置が受信できていれば、初回の処理結果は初期化完了から約10秒と処理時間が経過してから表示されます。

Fixed Frameの既定は付属bagに合わせて`map`です。異なる座標系の入力を使う場合は、その`header.frame_id`を指定してください。

```bash
ros2 launch pointcloud_filters pointcloud_filter.launch.py fixed_frame:=odom
```

Displaysのチェックボックスで入力と出力の表示を切り替えられます。初期の視点は付属bag向けです。対象が見えない場合はマウスで視点を調整してください。

Meshモードでは`ros2 topic info`の型が`shape_msgs/msg/Mesh`になります。座標系・時刻は次のように確認できます。

```bash
ros2 topic echo /merged_pointcloud_filtered/header --once
```

Meshモードでは、ノードがRViz2向けの`visualization_msgs/msg/Marker`（TRIANGLE_LIST）も`<output_topic>/marker`へ発行します。既定名は`/merged_pointcloud_filtered/marker`です。Meshと同じ頂点・三角形を使い、元のHeaderを直接付けるため、補助Headerとの組み合わせ処理は不要です。空のMeshに対してはDELETEマーカーを発行し、前回の面を消します。MarkerはTransient Local / Reliable / Keep Last 1で保持するため、RViz2が後から接続しても最後の表示状態を取得できます。

同梱の[PointCloud2用設定](rviz/pointcloud_filter.rviz)と[Mesh用設定](rviz/pointcloud_filter_mesh.rviz)を編集するか、独自の設定を指定することもできます。YAMLやlaunch引数で変更した入出力トピック名は、RViz2側にもリマップされます。

```bash
ros2 launch pointcloud_filters pointcloud_filter.launch.py \
  rviz_config:=/absolute/path/to/custom.rviz fixed_frame:=map
```

| RViz用launch引数 | 既定値 | 意味 |
| --- | --- | --- |
| `use_rviz` | `true` | RViz2を同時起動 |
| `fixed_frame` | `map` | RViz2のFixed Frame |
| `rviz_config` | 空文字 | 空なら出力形式に対応した同梱設定を使用 |

## 設定

既定値は[config/pointcloud_filter.yaml](config/pointcloud_filter.yaml)にあります。変更後は再ビルドして再起動するか、設定ファイルをコピーして指定してください。

```bash
ros2 launch pointcloud_filters pointcloud_filter.launch.py \
  params_file:=/absolute/path/to/pointcloud_filter.yaml
```

| パラメータ | 既定値 | 意味 |
| --- | ---: | --- |
| `input_topic` | `/merged_pointcloud` | 入力PointCloud2 |
| `pose_topic` | `/zx200/global_pose` | バックホウ位置のPoseStamped |
| `radius_m` | `10.0` | バックホウ位置を中心としたXY抽出半径、m。正の値 |
| `max_pose_time_diff_sec` | `2.0` | 点群と位置の最大スタンプ差、秒。0でチェック無効 |
| `output_topic` | `/merged_pointcloud_filtered` | 選択した型の出力先 |
| `output_format` | `pointcloud2` | `pointcloud2`または`mesh` |
| `mesh_resolution` | `3` | メッシュの解像度。整数1〜5。Meshモードで使用 |
| `publish_period_sec` | `10.0` | 実時間での処理周期、秒 |
| `voxel_size` | `0.1` | ボクセルの一辺、m。0で無効 |
| `sor_mean_k` | `20` | SORで距離を平均する近傍点数 |
| `sor_stddev_mul` | `2.0` | SORの標準偏差倍率。小さくすると厳しくなる |

距離パラメータは入力座標がm単位である前提です。各設定は起動時に読み込み、実行中の変更は受け付けません。

- 抽出範囲を変える場合は、`radius_m`を変更します。
- 出力がない場合は、位置の受信、双方の`header.frame_id`、座標系間のTF、スタンプ差を確認してください。
- 外れ値が残る場合は、`sor_stddev_mul`を小さくします。
- 処理が重い場合は、`voxel_size`を大きくして点数を減らします。
- Meshモードの処理量を減らしたい場合は、`mesh_resolution`を小さくします。

## 周期・メッセージ・QoSの仕様

- 入力コールバックは最新の1メッセージだけを保持し、タイマーが10秒ごとに取り出して処理します。10秒分の点群を蓄積・合成する処理ではありません。
- 点群・位置・TFの受信とフィルタ処理は2スレッドで動作します。処理中に届いた最新入力は次の周期で処理します。
- 未処理の入力がなければ発行をスキップします。再生終了後、残った最新入力を1回処理した後は発行しません。
- 有効な位置が取得でき、円内に点が残らない場合は、選択した型の空メッセージを発行します。Meshモードではメッシュ生成に失敗した場合も空のMeshと対応するHeaderを発行します。位置やTFの待機中は発行を見送るため、RViz2に前回の結果が表示されている場合はそのまま残ります。
- PointCloud2モードでは、元の`header.frame_id`と`header.stamp`を保持します。出力は選択した元の点のXYZ、intensity、RGBなど全フィールドと`point_step`を保持し、`height=1`の非構造化点群になります。ボクセル代表点も元の点を選びます。Meshモードでは上記の補助Headerを使います。
- 点群・位置の入力QoSはBest Effort / Volatile / Keep Last 1です。ReliableおよびBest Effortの入力に接続できます。出力QoSはReliable / Volatile / Keep Last 1です。後から接続した購読側は次回の発行を待ちます。
- 処理周期はsteady clockを使うため、bagの停止・時刻の巻き戻り・`use_sim_time`によって停止しません。処理時間が周期を超える場合は警告を表示します。
- 入出力に同じトピックを指定すると起動エラーになります。XYZフィールドやデータ長などが不正なメッセージはエラーを記録してスキップします。

## 動作確認

```bash
cd /home/common/SIP_PROJECTS/M2/SIMIZU_PROJECT/ros2_ws
colcon test --packages-select pointcloud_filters
colcon test-result --verbose
```

単体テストでは、円形のXY半径判定・境界点・Z方向の非制限・中心の移動・半径内の離れた領域の保持・SOR・NaN/Inf・空入力・ボクセル処理・不正パラメータを確認します。位置については同一座標系、実際のTFバッファによる回転と平行移動、計測時刻でのTF補間、TF不足、時刻差超過、座標系や位置の不正を検証します。PointCloud2の行パディング・エンディアン・追加属性の保持、Mesh全5段階・垂直面・曲面・複数面の保持、RViz Markerの頂点順序・Header・空の面の削除も確認します。

付属bagは約60秒・308フレームです。確認用の仮中心`(25, 40)` m・半径10 m・他は既定値で、先頭・中間・末尾を処理した結果です。実際のバックホウ位置を使った計測ではありません。処理時間は実行環境によって変わります。

| フレーム（0始まり） | 入力点数 | 出力点数 | 処理時間の目安 |
| --- | ---: | ---: | ---: |
| 0 | 104,348 | 5,913 | 0.02秒 |
| 154 | 101,248 | 6,615 | 0.04秒 |
| 307 | 101,401 | 6,280 | 0.02秒 |

ROSトピックを使った通信試験でも、付属bagの点群と明示的な検証用PoseStamped・TFを送信して確認しています。位置未受信時の待機、位置やTFが後から届いた際の処理再開、不正・時刻差超過の位置の拒否、中心位置の移動、launch引数による半径・位置トピックの上書き、PointCloud2とMesh全5段階、出力が円内に収まること、空の範囲でのMeshとRViz Markerの削除を検証しました。実機の位置データを使った試験ではありません。

`colcon build`および`colcon test`で検証しています。単体テストは53件成功・既存の著作権テスト1件スキップです。

ノードは各回の入力・半径抽出後・ボクセル処理後・SOR後・最終出力の点数、使用した中心XY、処理時間をログに表示します。目的の範囲を選べているかはRViz2で確認して調整してください。
