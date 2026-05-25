# pbl_ros2

## data download
[Google Drive](https://drive.google.com/drive/folders/1Dwz5-Qbh3jfCdc-tpFJd9m5-tmM1SvO4?usp=sharing) からrosbagやPCDをダウンロードできます

rosbagは rosbag/ に、PCDは pbl_launch/map/map.pcd に保存してください

## setup
mappingを行う場合は [point_lio_ros2](https://github.com/kazu-321/point_lio_ros2.git) をcloneしてビルドしてください

navigationを行う場合は navigation2をapt installしてください

## simulation
differentialロボットの簡易シミュレーションができます

当たり判定や点群はないので主にnav2などの動作確認用です

```bash
ros2 launch pbl_launch simulation.launch.xml
```

## rosbag_mapping
rosbagから点群を再生して地図を作成することができます

```bash
./scripts/rosbag_mapping.sh <rosbag_file>
```

PCDは環境変数 `PBL_MAP_DIR` に保存されます

`PBL_MAP_DIR`が未定義の場合はinstall/pbl_launch/map/ に保存されます

おすすめはsrcなので、 /home/username/ros2_ws/src/pbl_ros2/pbl_launch/map などです

cloud compareなどで確認し、そのマップを使用したいならばmap.pcdにファイル名を変更してください
