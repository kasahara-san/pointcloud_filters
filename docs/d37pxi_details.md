# D37PXiの処理・較正・検証

[簡潔な起動手順と現在の設定](../README.md)

## D37PXi：オンボードLiDARとRViz

このディレクトリでビルド・起動できます。すでに再生されているトピックを購読し、bagの追加再生は行いません。

```bash
source /opt/ros/humble/setup.bash
cd /home/common/SIP_PROJECTS/M2/SIMIZU_PROJECT/ros2_ws/src/pointcloud_filters
colcon --log-base log build --base-paths . --symlink-install --packages-select pointcloud_filters
source install/setup.bash
QT_QPA_PLATFORM=xcb ros2 launch pointcloud_filters d37pxi.launch.py
```

RVizでは次の表示を切り替えられます。

- **Filtered point cloud (green)**：機体の自己点を除去し、10 m以内を0.23 m voxelで間引いた点群。
- **Removed robot points (red)**：自己フィルタで除去済みの点。初期状態はオフ。比較するときだけチェックを入れてください。
- **D37PXi articulated model (yellow)**：実測のlift・tilt・angleに追従する機体CAD。
- **Input point cloud (gray)**：元の点群。初期状態はオフ。チェックを入れると比較できます。

機体は観測していない側もCADで表示しますが、その側の地形点は生成しません。機体が動いても過去のスキャンを重ねません。搭載LiDARは機体前方と周辺の見えている範囲だけを取得するので、環境設置型センサのように全周を埋めることはありません。

設定は [config/d37pxi.yaml](../config/d37pxi.yaml)。出力は `/d37pxi/lidar_points_filtered`、除去点はその `/self_removed`、CADは `/robot_model` です。0.5秒周期で最新の1スキャンを処理します。Header・XYZ・intensity・ring・timestampを含む元の全点属性を保持し、出力座標系は入力と同じ `hesai_lidar` です。RVizのFixed Frameは専用の `d37pxi/filter_base` で、推定取付変換から生成します。既存の機体TFを変更する必要はありません。

```bash
# Mesh出力（RVizでは面を表示）
ros2 launch pointcloud_filters d37pxi.launch.py output_format:=mesh mesh_resolution:=4

# GUIなし、設定を差し替え
ros2 launch pointcloud_filters d37pxi.launch.py use_rviz:=false params_file:=/absolute/config.yaml
```

### 取付位置・時刻・オンボード用の処理

2026-11-20の配信元 `rosbag2_2026_11_20-13_35_23` に含まれる1501スキャンから、均等に15フレームを抽出しました。冒頭・中間・末尾の3フレームの可視機体点を、実測関節角で配置した全7 collisionメッシュへロバストに位置合わせしています。

- 暫定の **base_link→LiDAR**：XYZ `[0.8460634, 0.1506704, 2.7640916]` m、固定XYZ順RPY `[-1.5762310, -0.0010623, -1.5048309]` rad。横倒しのLiDAR軸を補正します。[較正結果](d37pxi_calibration.json)。
- 上流の [D37PXiモデル](https://github.com/pwri-opera/d37pxi_ros2/tree/fd0a90aaaabe4878d24b221ad9e14ed04e1fc177/d37pxi_description) を同梱。`lift→lift_joint`、`tilt→tilt_joint`、`angle→angle_joint` と対応付け、シリンダー等はURDFのmimicに従います。[出典と変更点](../robot/d37pxi/README.md)。
- この記録では点群のstampが起動後秒、Pose/JointStateはUNIX時刻です。**状態照合時だけ** `cloud_time_offset_sec: 1795148958.7430618` を加えます。点群Headerや各点timestampは変更しません。これは当該記録の受信時刻から推定した差で、時計同期の測定値ではありません。別の記録・実機で時計条件が変わったら再設定し、ドライバがUNIX時刻へ同期済みなら `0.0` にしてください。差が許容範囲を超える状態では出力しません。
- 0.5 m未満の点を除き、Hesaiの未検出点 `(0,0,0)` を排除します。半径はLiDARのXY面ではなく、機体中心からの**map水平面**で判定します。大きなmap座標の並進は相対計算で相殺し、姿勢を使って水平軸へ変換します。
- 自己除去は各collisionメッシュの凸包を1.20倍に膨張し、0.35 mの余裕で行います（`self_filter_geometry: convex_hull`）。開いたCAD面の内側も除去します。各リンクを個別に処理するため関節追従を保ち、機体全体を単一の凸包では囲みません。取付推定と黄色のCAD表示には原形を使います。機体近傍の地形も少し広めに削ることで自己点群の除去を優先します。従来の表面形状による判定は `self_filter_geometry: mesh` で選べます。単一視点の距離依存密度を保つため、D37PXiでは全体統計によるSORを既定で無効にしています。必要なら `sor_enabled: true` で有効化できます。

これは測量による外部較正ではなく、この記録に対する暫定値です。機体・センサ取付を変更した場合は再較正してください。未観測領域や土砂に隠れた機体の判定精度は保証しません。

### D37PXiの検証

15フレームで、元の点レコードのバイト一致、Header・属性保持、有限性、10 m半径、原点点の排除、設定した自己モデルとの除去残存0を確認しました。冒頭・中間・末尾のMesh生成も確認済みです。[強化後の数値結果](d37pxi_validation_strong.json)。

初期設定（原形メッシュ、余裕0.18 m、倍率1.0）から自己除去を強化し、同じ15フレームで1スキャン当たり1,586〜3,414点を追加除去しました。車体周辺の高い機体候補領域に残る点は合計757点から0点になりました。機体中心から3.5 m超・CAD基準高さ±0.25 mの地面候補510,586点のうち293点は追加で除去され、4.5 m超の地面候補は失われていません。候補領域は幾何条件によるもので、人手の正解ラベルではありません。[設定比較](d37pxi_self_filter_comparison.json)、[初期設定の結果](d37pxi_validation.json)。

```bash
PYTHONPATH=.:$PYTHONPATH OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  python3 scripts/validate_onboard.py \
  --bag /path/to/recording \
  --check-mesh --report /tmp/d37pxi_validation.json

# 同じ可視範囲・探索境界で取付位置の推定も再実行
# --fitを追加（設定ファイルは書き換えません）
```

スクリプトはSQLiteの単一db3 bagに対応しています。実通信テストでは、時刻差のあるセンサ、横倒し取付、関節名変換、map水平面の抽出、元レコードの保持も検証しています。

D37PXi専用に整理した現行コードでは、単体・launch・lint等92件とROS実通信7件が成功しています（著作権ヘッダ検査1件はスキップ）。保存済みの実機データ15フレームで、元の点属性の保持・自己除去・Mesh出力と、整理前と同じ出力点数を確認しています。[現行コードの検証結果](d37pxi_validation_current.json)。

以前のライブ確認でも約2 Hzの更新とbagループ後の復帰を確認しています。[ライブ検証結果](d37pxi_live_validation.json)。

強化設定でも[ライブ出力](d37pxi_live_validation_strong.json)とオンボード実通信を再確認しています。凸包の内部除去と、離れたcollision間の空間を保持するテストも追加しました。

点群密度はvoxel幅を0.23 mに設定しました。0.12 m時の約8,000〜8,700点に対して、同じ15フレームの出力は平均で約1/3になります。自己除去する点は変更していません。[密度変更後の検証](d37pxi_validation_sparse.json)。

