include "map_builder.lua"
include "trajectory_builder.lua"


options = {
  map_builder = MAP_BUILDER,
    -- >  [string ?] / ???
  trajectory_builder = TRAJECTORY_BUILDER,
    -- >  [???] / ???
  map_frame = "map",
    -- >  [string ?] / The ROS frame ID to use for publishing submaps, the parent
    -- >  frame of poses, usually “map”.
    -- >       サブマップの publish に使用するROSフレームID。
    -- >       poses の親フレームで、通常は "map"。
  tracking_frame = "unilidar_imu",
    -- >  [string ?] / The ROS frame ID of the frame that is tracked by the SLAM
    -- >  algorithm. If an IMU is used, it should be at its position, although
    -- >  it might be rotated. A common choice is “imu_link”.
    -- >       SLAM時にトラッキングされるROSフレームID。
    -- >       IMUを使用する場合は、回転するとしてもその位置であるようにする。
    -- >       通常は "imu_link "。
  published_frame = "base_link",
    -- >  [string ?] / The ROS frame ID to use as the child frame for publishing
    -- >  poses. For example “odom” if an “odom” frame is supplied by a different
    -- >  part of the system. In this case the pose of “odom” in the map_frame will
    -- >  be published. Otherwise, setting it to “base_link” is likely appropriate.
    -- >       poses を publish するための子フレームとして使用する ROS フレーム ID。
    -- >       例えば "odom" フレームが他から提供されている場合は、"odom" となります。
    -- >       この場合、map_frame 内の "odom" の pose が publish されます。
    -- >       それ以外の場合は、"base_link" に設定するのが適切と思われます。
  odom_frame = "odom",
    -- >  [string ?] / Only used if provide_odom_frame is true. The frame between
    -- >  published_frame and map_frame to be used for publishing the
    -- >  (non-loop-closed) local SLAM result. Usually “odom”.
    -- >       provide_odom_frame の設定が true の場合のみ使用される。
    -- >       （非ループクローズドの）Local SLAM の結果を Publish するために使用する
    -- >       published_frame と map_frame の間のフレーム。
    -- >       通常は "odom"。
  provide_odom_frame = true,
    -- >  [bool] / If enabled, the local, non-loop-closed, continuous pose will be
    -- >  published as the odom_frame in the map_frame.
    -- >       有効にすると、Local の non-loop-closed かつ連続的な pose が、map_frame 内
    -- >       での odom_frame として publish されます。
  publish_frame_projected_to_2d = false,
    -- >  [bool] / If enabled, the published pose will be restricted to a pure 2D
    -- >  pose (no roll, pitch, or z-offset). This prevents potentially unwanted
    -- >  out-of-plane poses in 2D mode that can occur due to the pose extrapolation
    -- >  step (e.g. if the pose shall be published as a ‘base-footprint’-like frame)
    -- >       有効にすると、publish されるpose は純粋な2Dポーズに制限されます
    -- >      （ロール、ピッチ、z-オフセットなし）。これは、pose extrapolation step によって
    -- >       発生する可能性のある、2Dモードでの不要なポーズが面外になる事を防止します。
    -- >       （例：ポーズが 'base-footprint' 等のフレームとしてパブリッシュされる場合）
  use_pose_extrapolator = true,
    -- >  [bool] / ???
  use_odometry = true,
    -- >  [bool] / If enabled, subscribes to nav_msgs/Odometry on the topic “odom”.
    -- >  Odometry must be provided in this case, and the information will be
    -- >  included in SLAM.
    -- >       有効にすると、nav_msgs/Odometry 型の "odom" というトピックを subscribe する。
    -- >       この場合、オドメトリデータが提供されている必要があり、その情報はSLAMに利用される。
  use_nav_sat = false,
    -- >  [bool] / If enabled, subscribes to sensor_msgs/NavSatFix on the topic “fix”.
    -- >  Navigation data must be provided in this case, and the information will
    -- >  be included in the global SLAM.
    -- >       有効にすると、sensor_msgs/NavSatFix の "fix" というトピックを Subscribeする。
    -- >       この場合、ナビゲーションデータが提供されている必要があり、その情報はグローバルSLAMに利用される。
  use_landmarks = false,
    -- >  [bool] / If enabled, subscribes to cartographer_ros_msgs/LandmarkList on
    -- >  the topic “landmarks”. Landmarks must be provided, as
    -- >  cartographer_ros_msgs/LandmarkEntry within
    -- >  cartographer_ros_msgs/LandmarkList.
    -- >  If cartographer_ros_msgs/LandmarkEntry data is provided the information
    -- >  will be included in the SLAM accoding to the ID of the
    -- >  cartographer_ros_msgs/LandmarkEntry.
    -- >  The cartographer_ros_msgs/LandmarkList should be provided at a sample
    -- >  rate comparable to the other sensors. The list can be empty but has to
    -- >  be provided because Cartographer strictly time orders sensor data in
    -- >  order to make the landmarks deterministic. However it is possible to
    -- >  set the trajectory builder option “collate_landmarks” to false and
    -- >  allow for a non-deterministic but also non-blocking approach.
    -- >       有効にすると、cartographer_ros_msgs/LandmarkList 型のトピック "landmarks" を
    -- >       Subscribe するようになります。
    -- >       ランドマークは cartographer_ros_msgs/LandmarkList 内の
    -- >       cartographer_ros_msgs/LandmarkEntry として提供される必要があります。
    -- >       cartographer_ros_msgs/LandmarkEntry データが提供された場合、情報は
    -- >       cartographer_ros_msgs/LandmarkEntry の ID に従って SLAM に利用されます。
    -- >       cartographer_ros_msgs/LandmarkList は他のセンサーと同等のサンプルレートで
    -- >       提供される必要があります。リストは空でも構いませんが、Cartographer はランドマーク
    -- >       を決めるにあたりセンサーデータを厳密に時間順で並べるので、必ず提供されていなければ
    -- >       なりません。しかし、Trajectory Builderのオプション "collate_landmarks "をfalseに
    -- >       設定することで、非決定的な、しかしノンブロッキングなアプローチを可能にすることができます。
  num_laser_scans = 0,
    -- >  [int] / Number of laser scan topics to subscribe to. Subscribes to
    -- >  sensor_msgs/LaserScan on the “scan” topic for one laser scanner, or
    -- >  topics “scan_1”, “scan_2”, etc. for multiple laser scanners.
    -- >       Subscribe するレーザースキャントピックの数。
    -- >       1台のレーザースキャナの場合は "scan "トピック、複数のレーザースキャナの場合は
    -- >       "scan_1", "scan_2 "などのトピックで、sensor_msgs/LaserScan 型として Subscribe します。
  num_multi_echo_laser_scans = 0,
    -- >  [int] / Number of multi-echo laser scan topics to subscribe to.
    -- >  Subscribes to sensor_msgs/MultiEchoLaserScan on the “echoes” topic
    -- >  for one laser scanner, or topics “echoes_1”, “echoes_2”, etc. for
    -- >  multiple laser scanners.
    -- >       Subscribe するマルチエコーレーザースキャンのトピック数。
    -- >       sensor_msgs/MultiEchoLaserScan 型で、1台のレーザースキャナの場合は
    -- >       "echoes" トピック、複数台の場合は "echoes_1", "echoes_2" 等のトピックを Subscribe します。
  num_subdivisions_per_laser_scan = 1,
    -- >  [int] / Number of point clouds to split each received (multi-echo)
    -- >  laser scan into. Subdividing a scan makes it possible to unwarp scans
    -- >  acquired while the scanners are moving. There is a corresponding
    -- >  trajectory builder option to accumulate the subdivided scans into a
    -- >  point cloud that will be used for scan matching.
    -- >       受信した（マルチエコー）レーザースキャンを分割する点群の数。
    -- >       スキャンを分割することで、スキャナーの移動中に取得したスキャンを歪まないように
    -- >       することが可能になる。また、スキャンマッチングに使用する点群に分割されたスキャンを
    -- >       蓄積する Trajectory ビルダーオプションも用意されています。
  num_point_clouds = 1,
    -- >  [int] / Number of point cloud topics to subscribe to. Subscribes to
    -- >  sensor_msgs/PointCloud2 on the “points2” topic for one rangefinder,
    -- >  or topics “points2_1”, “points2_2”, etc. for multiple rangefinders.
    -- >       Subscribe する点群トピックの数。
    -- >       sensor_msgs/PointCloud2 型で、1台のレンジファインダーの場合は "points2 "トピック、
    -- >       複数台の場合は "points2_1", "point2_2" 等のトピックを Subscribe します。
  lookup_transform_timeout_sec = 0.2,
    -- >  [double] / Timeout in seconds to use for looking up transforms
    -- >  using tf2.
    -- >       tf2 の transform を検索する際のタイムアウト (秒)。
  submap_publish_period_sec = 0.3,
    -- >  [double] / Interval in seconds at which to publish the submap poses,
    -- >  e.g. 0.3 seconds.
    -- >       サブマップ poses を publish する間隔（秒）。例： 0.3 秒。
  pose_publish_period_sec = 5e-3,
    -- >  [double] / Interval in seconds at which to publish poses, e.g. 5e-3
    -- >  for a frequency of 200 Hz.
    -- >       poses を publish する間隔（秒） 例：200Hzの場合 5e-3
  trajectory_publish_period_sec = 30e-3,
    -- >  [double] / Interval in seconds at which to publish the trajectory
    -- >  markers, e.g. 30e-3 for 30 milliseconds.
    -- >       Trajectory マーカーを publish する間隔（秒）、例：30ミリ秒なら30e-3。
  rangefinder_sampling_ratio = 1.,
    -- >  [double] / Fixed ratio sampling for range finders messages.
    -- >       レンジファインダーメッセージ用のサンプリング比。
  odometry_sampling_ratio = 1.,
    -- >  [double] / Fixed ratio sampling for odometry messages.
    -- >       オドメトリメッセージ用のサンプリング比。
  fixed_frame_pose_sampling_ratio = 1.,
    -- >  [double] / ???
  imu_sampling_ratio = 0.1,
    -- >  [double] / Fixed ratio sampling for IMU messages.
    -- >       IMU メッセージ用のサンプリング比。
  landmarks_sampling_ratio = 1.,
    -- >  [double] / Fixed ratio sampling for landmarks messages.
    -- >       ランドマークメッセージ用のサンプリング比。

  -- 下記ページから見つけた差分です。
  -- (https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html)

  publish_to_tf = true, -- true/false, (デフォルト値は不明)
    -- >  [bool] / Enable or disable providing of TF transforms.
    -- >       TF 変換の提供の有効・無効を設定します。
  publish_tracked_pose = true, -- true/false, (デフォルト値は不明)
    -- >  [bool] / Enable publishing of tracked pose as a
    -- >  geometry_msgs/PoseStamped to topic “tracked_pose”.
    -- >       トラッキングした pose を geometry_msgs/PoseStamped 型として、
    -- >       "tracked_pose " トピックに publish できるようにします。
  -- fixed_frame_sampling_ratio = 1., -- , (デフォルト値は不明) (使い方わからず)
    -- >  [double ?] / Fixed ratio sampling for fixed frame messages.
    -- >       fixed frame メッセージ用のサンプリング比。
}


MAP_BUILDER.use_trajectory_builder_2d = false
  -- >  [bool] / Not yet documented.
MAP_BUILDER.use_trajectory_builder_3d = true
  -- >  [bool] / Not yet documented.
MAP_BUILDER.num_background_threads = 4
  -- >  [int32] / Number of threads to use for background computations.
  -- >       バックグラウンドでの計算に使用するスレッド数。
MAP_BUILDER.pose_graph = POSE_GRAPH
  -- >  [cartographer.mapping.proto.PoseGraphOptions] / Not yet documented.
MAP_BUILDER.collate_by_trajectory = false
  -- >  [bool] / ???
  -- >  (https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/map_builder.cc)


POSE_GRAPH.optimize_every_n_nodes = 100
  -- >  [int32] / Online loop closure: If positive, will run the loop closure
  -- >  while the map is built.
  -- >       オンライン ループクロージャ：　正の値の場合、地図生成時に
  -- >       ループクロージャが実行されます。


POSE_GRAPH.constraint_builder.sampling_ratio = 0.8
  -- >  [double] / A constraint will be added if the proportion of added constraints
  -- >  to potential constraints drops below this number.
  -- >       潜在的な制約に対する追加された制約の割合が、この設定値より低くなると、
  -- >       制約が追加されることになります。
POSE_GRAPH.constraint_builder.max_constraint_distance = 20.
  -- >  [double] / Threshold for poses to be considered near a submap.
  -- >       サブマップと近い poses について、考慮する距離の閾値。
POSE_GRAPH.constraint_builder.min_score = 0.50
  -- >  [double] / Threshold for the scan match score below which a match is not
  -- >  considered. Low scores indicate that the scan and map do not look similar.
  -- >       スキャンマッチスコアの閾値で、これ以下ではマッチングが考慮されない。
  -- >       低いスコアは、スキャンとマップが似ていないことを示す。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.50
  -- >  [double] / Threshold below which global localizations are not trusted.
  -- >       global localization を信頼しないようにするスコア閾値。
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.8e4
  -- >  [double] / Weight used in the optimization problem for the translational
  -- >  component of loop closure constraints.
  -- >       ループクロージャー制約の"並進"成分に対する最適化問題で使用される重み。
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.8e5
  -- >  [double] / Weight used in the optimization problem for the rotational
  -- >  component of loop closure constraints.
  -- >       ループクロージャー制約の"回転"成分に対する最適化問題で使用される重み。
POSE_GRAPH.constraint_builder.log_matches = true
  -- >  [bool] / If enabled, logs information of loop-closing constraints for debugging.
  -- >       有効にすると、ループクロージャー制約の情報をデバッグログに出力します。


POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
  -- >  [double] / Minimum linear search window in which the best possible scan
  -- >  alignment will be found.
  -- >       最適なスキャンアライメントを見つけるための最小の線形探索 Window。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
  -- >  [double] / Minimum angular search window in which the best possible scan
  -- >  alignment will be found.
  -- >       最適なスキャンアライメントが得られる最小の角度探索 Window。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
  -- >  [int32] / Number of precomputed grids to use.
  -- >       事前計算に使用するグリッドの数。


POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
  -- >  [double] / Scaling parameters for each cost functor.
  -- >       各コスト functor のスケーリング値
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
  -- >  [double] / Not yet documented.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
  -- >  [double] / Not yet documented.


POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
  -- >  [bool] / Configure the Ceres solver. See the Ceres documentation for more
  -- >  information: https://code.google.com/p/ceres-solver/
  -- >       Ceres ソルバーの設定。
  -- >       おそらくここが参考になりそう
  -- >       http://ceres-solver.org/nnls_solving.html
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
  -- >  [int32] / Not yet documented.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1
  -- >  [int32] / Not yet documented.


POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 9
  -- >  [int32] / Number of precomputed grids to use.
  -- >       事前計算に使用するグリッドの数。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 3
  -- >  [int32] / Number of full resolution grids to use, additional grids will reduce
  -- >  the resolution by half each.
  -- >       フル解像度グリッドに使う設定値。グリッドを追加すると、それぞれ解像度が半分になります。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.77
  -- >  [double] / Minimum score for the rotational scan matcher.
  -- >       回転のためのスキャンマッチャーの最小スコア。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.55
  -- >  [double] / Threshold for the score of the low resolution grid below which a match
  -- >  is not considered. Only used for 3D.
  -- >       低解像度でのグリッドにおける、これ以下ではマッチング考慮されなくなるスコア閾値。
  -- >       3D にのみ使用される。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 12.
  -- >  [double] / Linear search window in the plane orthogonal to gravity in which the
  -- >  best possible scan alignment will be found.
  -- >       重力に直交する平面において、最適なスキャンアライメントが得られるための線形探索 window の数。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 3.
  -- >  [double] / Linear search window in the gravity direction in which the best possible
  -- >  scan alignment will be found.
  -- >       重力方向において、最適なスキャンアライメントが得られるための線形探索 window の数。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(20.)
  -- >  [double] / Minimum angular search window in which the best possible scan
  -- >  alignment will be found.
  -- >       最適なスキャンアライメントが得られる最小の角度探索 Window。


POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 5.
  -- >  [double] / Scaling parameters for each cost functor.
  -- >       各コスト functor のスケーリング値
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 30.
  -- >  [double] / Scaling parameters for each cost functor.
  -- >       各コスト functor のスケーリング値
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10.
  -- >  [double] / Not yet documented.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1.
  -- >  [double] / Not yet documented.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = false
  -- >  [bool] / Whether only to allow changes to yaw, keeping roll/pitch constant.
  -- >       ロール/ピッチを一定にして、ヨーのみを変化させるかどうか。


POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false
  -- >  [bool] / Configure the Ceres solver. See the Ceres documentation for more
  -- >  information: https://code.google.com/p/ceres-solver/
  -- >       Ceres ソルバーの設定。
  -- >       おそらくここが参考になりそう
  -- >       http://ceres-solver.org/nnls_solving.html
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50
  -- >  [int32] / Not yet documented.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 2
  -- >  [int32] / Not yet documented.


POSE_GRAPH.matcher_translation_weight = 5e2
  -- >  [double] / Weight used in the optimization problem for the translational component of
  -- >  non-loop-closure scan matcher constraints.
  -- >       非ループクロージャー制約の"並進"成分に対する最適化問題で使用される重み。
POSE_GRAPH.matcher_rotation_weight = 1.6e3
  -- >  [double] / Weight used in the optimization problem for the rotational component of
  -- >  non-loop-closure scan matcher constraints.
  -- >       非ループクロージャー制約の"回転"成分に対する最適化問題で使用される重み。


POSE_GRAPH.optimization_problem.huber_scale = 1e1
  -- >  [double] / Scaling parameter for Huber loss function.
  -- >       Huber loss function のスケーリング値
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
  -- >  [double] / Scaling parameter for the IMU acceleration term.
  -- >       IMU acceleration 期間のスケーリング値
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4
  -- >  [double] / Scaling parameter for the IMU rotation term.
  -- >       IMU 回転の期間のスケーリング値
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
  -- >  [double] / Scaling parameter for translation between consecutive nodes based
  -- >  on the local SLAM pose.
  -- >       ローカル SLAM の pose に基づく連続したノード間の"並進"のためのスケーリング値。
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
  -- >  [double] / Scaling parameter for rotation between consecutive nodes based on
  -- >  the local SLAM pose.
  -- >       ローカル SLAM の pose に基づく連続したノード間の"回転"のためのスケーリング値。
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
  -- >  [double] / Scaling parameter for translation between consecutive nodes based
  -- >  on the odometry.
  -- >       オドメトリに基づく連続したノード間の"並進"のためのスケーリング値。
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
  -- >  [double] / Scaling parameter for rotation between consecutive nodes based on
  -- >  the odometry.
  -- >       オドメトリに基づく連続したノード間の"回転"のためのスケーリング値。
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
  -- >  [double] / Scaling parameter for the FixedFramePose translation.
  -- >       FixedFramePose の"並進"のためのスケーリング値。
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
  -- >  [double] / Scaling parameter for the FixedFramePose rotation.
  -- >       FixedFramePose の"回転"のためのスケーリング値。
POSE_GRAPH.optimization_problem.fixed_frame_pose_use_tolerant_loss = false
  -- >  [bool] / ??? (https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/optimization_problem_3d.cc)
POSE_GRAPH.optimization_problem.fixed_frame_pose_tolerant_loss_param_a = 1
  -- >  [int32 ?] / ??? (https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/optimization_problem_3d.cc)
POSE_GRAPH.optimization_problem.fixed_frame_pose_tolerant_loss_param_b = 1
  -- >  [int32 ?] / ??? (https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/optimization_problem_3d.cc)
POSE_GRAPH.optimization_problem.log_solver_summary = false
  -- >  [bool] / If true, the Ceres solver summary will be logged for every optimization.
  -- >       有効なら、Ceres ソルバーのサマリーはすべての最適化についてログに記録します。
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true
  -- >  [bool] / ??? (https://github.com/cartographer-project/cartographer_ros/issues/1486)
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
  -- >  [bool] / ??? (https://github.com/cartographer-project/cartographer/issues/1160)


POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
  -- >  [bool] / Configure the Ceres solver. See the Ceres documentation for more
  -- >  information: https://code.google.com/p/ceres-solver/
  -- >       Ceres ソルバーの設定。
  -- >       おそらくここが参考になりそう
  -- >       http://ceres-solver.org/nnls_solving.html
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
  -- >  [int32] / Not yet documented.
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 7
  -- >  [int32] / Not yet documented.


POSE_GRAPH.max_num_final_iterations = 200
  -- >  [int32] / Number of iterations to use in 'optimization_problem_options'
  -- >  for the final optimization.
  -- >       'optimization_problem options' で最終的な最適化に使用する反復回数
POSE_GRAPH.global_sampling_ratio = 0.01
  -- >  [double] / Rate at which we sample a single trajectory’s nodes for global
  -- >  localization.
  -- >       グローバルな Localization 時における、1つのトラジェクトリーのノードがサンプリングする速さ。
POSE_GRAPH.log_residual_histograms = true
  -- >  [bool] / Whether to output histograms for the pose residuals.
  -- >       pose 残差のヒストグラムを出力するかどうか。


POSE_GRAPH.global_constraint_search_after_n_seconds = 10.
  -- >  [double] / If for the duration specified by this option no global contraint
  -- >  has been added between two trajectories, loop closure searches will be performed
  -- >  globally rather than in a smaller search window.
  -- >       このオプションで指定された期間中に、2つの Trajectory 間にグローバルな拘束が追加
  -- >       されない場合、ループクロージャー検索は小さな検索窓ではなく、グローバルに実行されます。


TRAJECTORY_BUILDER_3D.min_range = 1.
  -- >  [float] / Rangefinder points outside these ranges will be dropped.
  -- >       この範囲外のレンジファインダーポイントのデータは破棄されます。
TRAJECTORY_BUILDER_3D.max_range = MAX_3D_RANGE
  -- >  [float] / Not yet documented.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
  -- >  [int32] / Number of range data to accumulate into one unwarped, combined range
  -- >  data to use for scan matching.
  -- >       スキャンマッチングに使用するための、歪みが無く結合された１個の範囲データに
  -- >       蓄積する範囲データの数。
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
  -- >  [float] / Voxel filter that gets applied to the range data immediately
  -- >  after cropping.
  -- >       切り出し直後の範囲データに適用されるボクセルフィルタ。


TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.
  -- >  [float] / ‘max_length’ of a voxel edge.
  -- >       ボクセルフィルタの 'max_length' 。
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
  -- >  [float] / If there are more points and not at least ‘min_num_points’
  -- >  remain, the voxel length is reduced trying to get this minimum number of points.
  -- >       点群数が多くありつつも、 'min_num_points' より少ない場合、この最小点数を得るために
  -- >       ボクセルフィルタ長が減少される。
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.
  -- >  [float] / Points further away from the origin are removed.
  -- >       原点から遠く削除される点の距離。


TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.
  -- >  [float] / ‘max_length’ of a voxel edge.
  -- >       ボクセルフィルタの 'max_length' 。
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
  -- >  [float] / If there are more points and not at least ‘min_num_points’
  -- >  remain, the voxel length is reduced trying to get this minimum number of points.
  -- >       点群数が多くありつつも、 'min_num_points' より少ない場合、この最小点数を得るために
  -- >       ボクセルフィルタ長が減少される。
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = MAX_3D_RANGE
  -- >  [float] / Points further away from the origin are removed.
  -- >       原点から遠く削除される点の距離。


TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
  -- >  [bool] / Whether to solve the online scan matching first using the correlative
  -- >  scan matcher to generate a good starting point for Ceres.
  -- >       Ceres の良い出発点を生成するために、相関スキャンマッチャーを使用して、
  -- >       最初にオンラインスキャンマッチングを解くかどうか。


TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
  -- >  [double] / Minimum linear search window in which the best possible scan
  -- >  alignment will be found.
  -- >       最適なスキャンアライメントを見つけるための最小の線形探索 Window。
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)
  -- >  [double] / Minimum angular search window in which the best possible scan
  -- >  alignment will be found.
  -- >       最適なスキャンアライメントが得られる最小の角度探索 Window。
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
  -- >  [double] / Weights applied to each part of the score.
  -- >       スコアの各パートに適用される重み。
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
  -- >  [double] / Not yet documented.


TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.
  -- >  [double] / Scaling parameters for each cost functor.
  -- >       各コスト functor のスケーリング値
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.
  -- >  [double] / Scaling parameters for each cost functor.
  -- >       各コスト functor のスケーリング値


TRAJECTORY_BUILDER_3D.ceres_scan_matcher.intensity_cost_function_options_0.weight = 0.5
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.intensity_cost_function_options_0.huber_scale = 0.3
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.intensity_cost_function_options_0.intensity_threshold = INTENSITY_THRESHOLD
  -- >  [int32 ?] / ??? 


TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.
  -- >  [double] / Not yet documented.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2
  -- >  [double] / Not yet documented.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false
  -- >  [bool] / Whether only to allow changes to yaw, keeping roll/pitch constant.
  -- >       ロール/ピッチを一定にして、ヨーのみを変化させるかどうか。


TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
  -- >  [bool] / Configure the Ceres solver. See the Ceres documentation for
  -- >  more information: https://code.google.com/p/ceres-solver/
  -- >       Ceres ソルバーの設定。
  -- >       おそらくここが参考になりそう
  -- >       http://ceres-solver.org/nnls_solving.html
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12
  -- >  [int32] / Not yet documented.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 1
  -- >  [int32] / Not yet documented.


TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
  -- >  [double] / Threshold above which range data is inserted based on time..
  -- >       これ以上の時間の時に、範囲データが挿入される閾値（秒）。
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
  -- >  [double] / Threshold above which range data is inserted based on linear motion.
  -- >       これ以上の距離の時に、線形移動に基づき範囲データが挿入される閾値。
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004
  -- >  [double] / Threshold above which range data is inserted based on rotational motion.
  -- >       これ以上の角度の時に、回転に基づいて範囲データが挿入される閾値（ラジアン）。


TRAJECTORY_BUILDER_3D.rotational_histogram_size = 120
  -- >  [int32] / Number of histogram buckets for the rotational scan matcher.
  -- >       回転スキャンマッチャーのための、ヒストグラムのバケット数。



TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 5.0
  -- >  [double] / Time constant in seconds for the orientation moving average
  -- >  based on observed gravity via the IMU. It should be chosen so that the error 1.
  -- >  from acceleration measurements not due to gravity
  -- >  (which gets worse when the constant is reduced) and 2. from integration of angular velocities
  -- >  (which gets worse when the constant is increased) is balanced.
  -- >       IMU で観測された重力に基づく方位の移動平均の時定数（秒）。
  -- >       error 1. 重力によらない加速度測定による誤差（定数を小さくすると悪化する）と、
  -- >       error 2. 角速度の積分による誤差（定数を大きくすると悪化する）がバランスよくなるように
  -- >       する必要があります。


TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false
  -- >  [bool] / ??? 


TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 10.
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.001
  -- >  [double ?] / ??? 


-- TODO(wohe): Tune these parameters on the example datasets.
-- >       TODO: これらのパラメータをサンプルデータセットで調整します。

TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_queue_duration = 5.
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.gravity_constant = 9.806
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_translation_weight = 1.
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_rotation_weight = 1.
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight = 1.
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight = 1.
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.odometry_translation_weight = 1.
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.odometry_rotation_weight = 1.
  -- >  [double ?] / ??? 


TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.use_nonmonotonic_steps = false;
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.max_num_iterations = 10;
  -- >  [double ?] / ??? 
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.num_threads = 1;
  -- >  [double ?] / ??? 


TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
  -- >  [double] / Resolution of the ‘high_resolution’ map in meters used for
  -- >  local SLAM and loop closure.
  -- >       Local SLAM とループクロージャーに使用する 'high_resolution' 
  -- >       マップの解像度 （メートル）。
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.
  -- >  [double] / Maximum range to filter the point cloud to before insertion
  -- >  into the ‘high_resolution’ map.
  -- >       'high_resolution' マップに挿入する前に、点群をフィルタする最大距離範囲。
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
  -- >  [double] / Resolution of the ‘low_resolution’ version of the map in
  -- >  meters used for local SLAM only.
  -- >       Local SLAM のみに使用される 'low_resolution' 版のマップの解像度（メートル単位）。
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100
  -- >  [int32] / Number of range data before adding a new submap. Each submap will
  -- >  get twice the number of range data inserted: First for initialization without
  -- >  being matched against, then while being matched.
  -- >       新しいサブマップを追加する前の範囲データの数。各サブマップには2倍のレンジデータが挿入されます。
  -- >       最初はマッチングされずに初期化され、次にマッチングされます。


TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
  -- >  [double] / Probability change for a hit (this will be converted to odds and
  -- >  therefore must be greater than 0.5).
  -- >       hit した場合の確率の変化（オッズに変換されるため、0.5以上でなければならない）。
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
  -- >  [double] / Probability change for a miss (this will be converted to odds and
  -- >  therefore must be less than 0.5).
  -- >       miss した場合の確率の変化（オッズに変換されるため、0.5未満でなければならない）。
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 2
  -- >  [int32] / Up to how many free space voxels are updated for scan matching.
  -- >  0 disables free space.
  -- >       スキャンマッチングで更新されるフリーな空間のボクセルの最大数。
  -- >       0はフリーな空間を無効にします。
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.intensity_threshold = INTENSITY_THRESHOLD
  -- >  [int32 ?] / ??? 


TRAJECTORY_BUILDER_3D.use_intensities = false
  -- >  [bool] / When setting use_intensites to true, the intensity_cost_function_options_0
  -- >  parameter in ceres_scan_matcher has to be set up as well or otherwise
  -- >  CeresScanMatcher will CHECK-fail.
  -- >       use_intensites の設定を true にする場合、ceres_scan_matcher の
  -- >       intensity_cost_function_options_0 パラメータも設定しないと CeresScanMatcher は
  -- >       CHECK-fail するようになります。


return options
