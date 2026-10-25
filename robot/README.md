# ZX200 collision geometry

このディレクトリは自己点群除去に必要なモデルを同梱します。外部の
`zx200_description` パッケージや `robot_state_publisher` は不要です。

出典はワークスペース内の
`opera/zx200/zx200_ros2/zx200_description`、リポジトリコミット
`5433c8c59e8bf9cb86bc5cc7baab660a869a009f` です。
上流リポジトリの Apache-2.0 ライセンスを [LICENSE](LICENSE) に同梱しています。
上流 package.xml の license 欄は `-` ですが、リポジトリ直下の LICENSE を保持しています。

- `meshes/` の5個の COLLADA ファイルは上流の原本です。
- `zx200.urdf` は上流 `urdf/zx200.xacro` を xacro で展開し、メッシュ参照を
  このディレクトリからの相対パスに変更したものです。
- モデルを変更した箇所は URDF 内のコメントにも記載しています。
- crawler、body、boom、arm、bucket の collision geometry をすべて使います。
  上流のURDFに結び付けられていないシリンダー・リンク補助メッシュを、
  誤った固定姿勢で追加する処理は行いません。
- 関節の必須値は `swing_joint`, `boom_joint`, `arm_joint`, `bucket_joint` です。
  `bucket_end_joint` は collision を持たない末端リンクのため省略可能です。

COLLADA の単位、up_axis、シーングラフ内の回転・並進を読み込みます。
行列テキストの並び順は [Khronos COLLADA 1.4仕様](https://www.khronos.org/files/collada_spec_1_4.pdf)
に従い、数学的な列ベクトルに作用する行列を行ごとに読み込みます。
Open3D による DAE 読み込みへの対応状況に依存せず、三角形配列を直接
`RaycastingScene` に渡します。

除去対象は、関節姿勢を反映した全三角形から `self_filter_padding` m 以内の点、
および水密な連結成分の内部です。独立した成分の内部判定を OR で合成するため、
重なった立体の内部が偶奇反転で残ることを防ぎます。CAD由来の原本には開いた面や
非多様体の面があるため、その部分では未定義の内外判定を適用せず、表面距離のみを使います。
凸包による近似は使いません。開いたメッシュの奥深くの内部まで一律に除去する保証はありません。
これは [Open3D の内外判定が要求する水密性](https://www.open3d.org/docs/release/python_api/open3d.t.geometry.RaycastingScene.html)
に合わせた扱いです。

`/zx200/global_pose` は測定base_linkの位置・姿勢として保持します。
同梱YAMLでは、測定baseからこのURDFの原点への変換に付属bagから推定した
`model_origin_xyz: [-0.17, 0.30, -3.21]` m、`model_origin_rpy: [0,0,0]` radを使います。
これはユーザー指示に基づく暫定位置合わせで、測量による外部較正ではありません。
別TFの `base_link_3d` の固定高さは使用しません。
同梱YAMLの padding は0.30 mです。汎用の `SelfFilter` とYAMLなしのノードは
原点補正identity・padding 0.15 mを既定にし、bag固有の補正を暗黙に適用しません。
[補正前後と検証の制約](../docs/validation.md) を確認してください。

暫定推定を再現するには、パッケージのルートで次を実行します。計測baseからの
絶対的なCAD原点変換を推定するため、YAMLの補正を二重に適用しません。

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=.:$PYTHONPATH python3 scripts/calibrate_model_origin.py \
  --output /tmp/model_origin_calibration.json \
  --preview /tmp/model_origin_comparison.png
```

最初の点群だけで推定し、中間・最後の点群で検証します。探索範囲と点群選択条件は
付属bag向けの暫定条件です。JSONには各初期値からの推定値、独立フレームの距離分布、
padding比較を記録します。設定ファイルは自動変更しません。

## 元モデルの1.5倍の膨張

同梱YAMLの `self_filter_scale` は1.50です。1.3倍した形状をさらに1.5倍するのではなく、元モデルから直接1.5倍にします。各collision形状のローカル境界中心を基準に拡大し、URDFのjoint origin、GNSS姿勢、関節角度は維持します。原形、1.1/1.2/1.3/1.4倍の中間形状、指定した最終1.5倍の除去範囲をORで合成します。これにより、開いた面が外側へ移動しても、以前の1.3倍で除去していた点が再び残ることを防ぎます。表面距離の余裕0.30 mは拡大後も同じm単位です。

原点の較正ツールは形状の原寸（scale=1.0）で位置合わせを行います。実際の除去範囲の検査には、設定済みのscale=1.5を反映する `scripts/validate_bag.py` を使用してください。
