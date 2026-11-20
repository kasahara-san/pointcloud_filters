# D37PXi collision model

出典：[pwri-opera/d37pxi_ros2](https://github.com/pwri-opera/d37pxi_ros2/tree/fd0a90aaaabe4878d24b221ad9e14ed04e1fc177/d37pxi_description)、コミット `fd0a90aaaabe4878d24b221ad9e14ed04e1fc177`。リポジトリのApache-2.0ライセンスを [LICENSE](LICENSE) に同梱しています。

`meshes/` の7個のDAEとテクスチャPNGは原本です。`d37pxi.urdf` は上流の `d37pxi_description/urdf/d37pxi24.xacro` にある空の `tf_prefix` を展開し、先頭の `/` を外し、メッシュ参照を相対パスへ変更したものです。リンク寸法、関節原点、軸、mimic係数は変更していません。上流の古い `d37pxi24.urdf` と異なり、Xacroの `base_joint` の暫定原点 `[-0.086289, 0, 0]` と `lift_joint / tilt_joint / angle_joint` の名前を保持しています。

全7 collision（車体、ブレードアーム、アームシリンダー、アームピストン、左右角度ピストン、ブレード）を使います。同梱メッシュと黄色のCAD表示は原形のままです。現在のD37PXiフィルタ設定では実行時に各collisionの凸包を個別に作り、1.20倍の膨張と0.35 mの余裕を加えて内部も除去します。関節原点や関節間距離は変えません。原形モード（`self_filter_geometry: mesh`）では閉じた連結成分は内部も除去し、開いたCAD面では表面距離のみを使います。

LiDARは `base_link` に対する固定取付として別パラメータで扱います。較正値と制約は [README](../../README.md) と [較正結果](../../docs/d37pxi_calibration.json) を参照してください。
