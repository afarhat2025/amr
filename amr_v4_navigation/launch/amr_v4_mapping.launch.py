import os
import time
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():

    #os.system("sudo systemctl restart ptp4l")
    #time.sleep(1.0)
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')
    deskewing_slerp = LaunchConfiguration('deskewing_slerp')
    mapping = LaunchConfiguration('mapping')
    qos = LaunchConfiguration('qos')
    robot_name = (os.getenv('ROBOT_MODEL','amr_x'))

    parameters = {
            'frame_id': robot_name+'/base_link',
            'map_frame_id': 'map',
            'publish_tf':True,
            'subscribe_depth':False,
            'subscribe_scan':False,
            'subscribe_scan_cloud':True,
            'subscribe_rgb':False,
            'subscribe_rgbd': True,
            'subscribe_stereo':False,
            'approx_sync':True,
            'rgbd_cameras':1,
            'wait_for_transform':0.2,
            'odom_frame_id': robot_name+'/odom_icp',
            'tf_prefix':'',
            'use_sim_time':use_sim_time,
            'qos':qos,
            'qos_image':qos,
            'qos_camera_info':qos,
            'qos_odom':qos,
            'wait_imu_to_init': False,
            #'landmark_linear_variance':0.1,
            #'landmark_angular_variance': 0.05,
            'odom_tf_linear_variance':0.0005, #0.0005
            'odom_tf_angular_variance':0.00007569, #c 0.005
            'odom_sensor_sync': True,
            #RGBD
            'RGBD/MarkerDetection':'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '3',
            'RGBD/Enabled':'true',
            'RGBD/ProximityPathFilteringRadius':'0',
            'RGBD/MaxOdomCacheSize':'50',
            'RGBD/StartAtOrigin':'false',
            'RGBD/ForceOdom3DoF':'true',
            'Reg/Strategy':'1',
            'Reg/Force3DoF':'true',
            'Reg/RepeatOnce': 'false',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate':'0.01', 
            'RGBD/CreateOccupancyGrid':'true',
            'map_empty_ray_tracing': True,            
            #optimizer
            'Optimizer/GravitySigma':'0.0',
            'Optimizer/Strategy':'2',
            'Optimizer/Iterations':'10',
            'GTSAM/Optimizer':'2',
            #ICP
            'Icp/CCFilterOutFarthestPoints':'true',
            'Icp/PointToPlane':'true',
            'Icp/VoxelSize':'0.15',
            'Icp/Epsilon':'0.001', 
            'Icp/DownsamplingStep':'1',
            'Icp/PointToPlaneK':'20',
            'Icp/PointToPlaneRadius':'0',
            'Icp/MaxTranslation':'0.5',#0.04
            'Icp/MaxCorrespondenceDistance':'1.2', #c #1.0
            'Icp/CorrespondenceRatio':'0.2',
            'Icp/PointToPlaneGroundNormalsUp':'0.9',
            'Icp/Strategy':'1',
            'Icp/ReciprocalCorrespondences':'true',
            'Icp/OutlierRatio':'0.4',
            'Icp/PointToPlaneMinComplexity':'0.04', #c0.04
            'Icp/Iterations':'10',
            #memory
            'Mem/STMSize':'60',
            'Mem/LaserScanNormalK':'20',
            'Mem/NotLinkedNodesKept':'false',
            'Mem/UseOdomFeatures':'true',
            #loop closures
            'Kp/DetectorStrategy':'11',
            #'Kp/RoiRatios': '0.0 0.0 0.1 0.2',
            #'Vis/RoiRatios': '0.0 0.0 0.1 0.2',
            'Vis/FeatureType':'11',
            'Vis/CorGuessWinSize':'0',
            'Vis/CorNNType':'1',
            'Vis/CorNNDR':'0.6',
            'Vis/MaxFeatures':'600', #600 max
            #localization
            'SuperPoint/ModelPath':'/home/amr/AMRMAIN/src/ROS_DRIVERS_Readme/rtabmap/archive/2022-IlluminationInvariant/scripts/superpoint_v1.pt',
            #'PyMatcher/Path':'/home/amr/AMRMAIN/src/ROS_DRIVERS_Readme/rtabmap/archive/2022-IlluminationInvariant/scripts/SuperGluePretrainedNetwork/rtabmap_superglue.py',
            #marker
            'Marker/Dictionary':'20',
            'Marker/CornerRefinementMethod':'3',
            'Marker/Length':'0.1',
            'Marker/MaxRange':'1.2',
            'Marker/MinRange':'0.8',
            #'Marker/PriorsVarianceAngular':'9999',
            #'Marker/PriorsVarianceLinear':'0.1',
            'Marker/VarianceAngular': '9999',
            'Marker/VarianceLinear':'0.1',
            'Marker/VarianceOrientationIgnored':'true',
            #grid
            'Grid/Sensor':'0',
            'Grid/3D':'false',
            'Grid/CellSize':'0.05',
            'Grid/RangeMin':'0.1',
            'Grid/RangeMax':'7.0',
            'Grid/ClusterRadius':'1',
            'Grid/MinGroundHeight':'0.05',
            'Grid/MaxGroundHeight':'0.1',
            'Grid/MaxObstacleHeight':'2.0',
            'Grid/NoiseFilteringRadius':'0.0',
            'Grid/NoiseFilteringMinNeighbors':'2',
            'Grid/NormalsSegmentation':'false',
            'Grid/FootprintHeight':'0.171',
            'Grid/FootprintLength':'0.6604',
            'Grid/FootprintWidth':'0.6096',
            'Grid/VoxelSize':'0.05',
            'Grid/RayTracing':'true',
            
    }


    remappings = [
        ('/'+robot_name+'/scan_cloud', '/'+robot_name+'/odom_filtered_input_scan'),
        ('/'+robot_name+'/scan', '/'+robot_name+'/scan_not_used'),
        ('/'+robot_name+'/grid_map','/'+robot_name+'/map'),
        ('/tf','tf'),
        ('/tf_static','tf_static'),
        ('/diagnostics','/'+robot_name+'/diagnostics')
    ]

    stereo_sync = ComposableNode(
        package='rtabmap_sync', 
        plugin='rtabmap_sync::StereoSync',
        #executable='stereo_sync',
        namespace=robot_name,
        #output='screen',
        name = 'rtabmap_stereo_sync',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[{
            'log_to_rosout_level': 4,
            'approx_sync':False,
            'sync_queue_size': 5,
            'topic_queue_size': 10,
            'qos':qos,
            'qos_camera_info': qos,
        }],
        #arguments=['--uwarn'],
        remappings=[
            ('/'+robot_name+'/left/image_rect', '/'+robot_name+'/zed_node/left/image_rect_color'),
            ('/'+robot_name+'/left/camera_info', '/'+robot_name+'/zed_node/left/camera_info'),
            ('/'+robot_name+'/right/image_rect', '/'+robot_name+'/zed_node/right/image_rect_color'),
            ('/'+robot_name+'/right/camera_info', '/'+robot_name+'/zed_node/right/camera_info'),
            ('/diagnostics','/'+robot_name+'/diagnostics')
        ])
        
    icp_odometry = ComposableNode(
        package='rtabmap_odom',
        #executable='icp_odometry',
        plugin='rtabmap_odom::ICPOdometry',
        #prefix="xterm -e gdb -ex run --args",
        #output='screen',
        name = 'icp_odom_container',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace=robot_name,
        parameters=[{
            'log_to_rosout_level': 4,
            'frame_id': robot_name+'/base_link',
            'odom_frame_id': robot_name+'/odom_icp',
            'guess_frame_id': robot_name+'/odom_ekf',
            'publish_tf':True,
            'approx_sync':False,
            'tf_prefix':'', 
            'wait_for_transform':0.2,
            'sync_queue_size':5,
            'topic_queue_size': 5,
            'publish_null_when_lost':False,
            'expected_update_rate':25.0,
            'use_sim_time':use_sim_time,
            'qos':qos,
            'deskewing':deskewing,
            'deskewing_slerp':deskewing_slerp,
            'wait_imu_to_init': False,
            'Icp/CCFilterOutFarthestPoints':'true',
            'Icp/PointToPlane':'true',
            'Icp/VoxelSize':'0.15',
            'Icp/Epsilon':'0.001', 
            'Icp/DownsamplingStep':'1',
            'Icp/PointToPlaneK':'20',
            'Icp/PointToPlaneRadius':'0',
            'Icp/MaxTranslation':'0.5',#0.4
            'Icp/Iterations':'10', 
            'Icp/MaxCorrespondenceDistance':'1.2', #c #1.0
            'Icp/CorrespondenceRatio':'0.01',
            'Icp/PointToPlaneGroundNormalsUp':'0.9',
            'Icp/Strategy':'1',
            'Icp/ReciprocalCorrespondences':'true',
            'Icp/OutlierRatio':'0.5',
            'Icp/PointToPlaneMinComplexity':'0.1', #c0.04
            #Odom
            'Odom/ScanKeyFrameThr':'0.5',
            'OdomF2M/ScanSubtractRadius':'0.15',
            'OdomF2M/ScanMaxSize':'20000',
            'OdomF2M/BundleAdjustment':'false',
            'Odom/ResetCountdown':'5',
            'Reg/Force3DoF':'true',
            #optimizer
            'Optimizer/GravitySigma':'0.0',
            'Optimizer/Strategy':'2',
            'GTSAM/Optimizer':'2',
            'Optimizer/PriorsIgnored':'false',
            'Optimizer/Iterations':'10',
            'Reg/RepeatOnce': 'false',
        }],
        remappings=[
            ('/'+robot_name+'/scan_cloud', '/'+robot_name+'/hesai_points'),
            ('/'+robot_name+'/odom','/'+robot_name+'/odom/icp'),
            ('/tf','tf'),
            ('/tf_static','tf_static'),
            ('/diagnostics','/'+robot_name+'/diagnostics')
        ],
        #arguments=[
        #    '--uerror'
            #'--uwarn'
        #]
    )

    #this is for mapping node only!
    Mapping = ComposableNode(
        condition=UnlessCondition(mapping),
        package='rtabmap_slam',
        plugin='rtabmap_slam::CoreWrapper',
        name = 'mapping_container',
        #executable='rtabmap',
        #output='screen',
        #extra_arguments=[{'use_intra_process_comms': True}],
        namespace=robot_name,
        #prefix="xterm -e gdb -ex run --args",
        #arguments = ['-d','--udebug'],
        parameters=[parameters,
                    {
                        'log_to_rosout_level': 1,
                        'delete_db_on_start': True,
                        'Rtabmap/DetectionRate':'2',
                        'sync_queue_size': 5,
                        'topic_queue_size':5,

                    }],
        remappings=remappings,
    )

    #this is for localization node only !
    Localization = ComposableNode(
        condition=IfCondition(mapping),
        package='rtabmap_slam',
        #executable='rtabmap',
        plugin='rtabmap_slam::CoreWrapper',
        name = 'localization_container',
        #extra_arguments=[{'use_intra_process_comms': True}],
        #output='screen',
        namespace=robot_name,

        #prefix="xterm -e gdb -ex run --args",
        parameters=[parameters,
        {
            'log_to_rosout_level': 4,
            'delete_db_on_start': False,
            'map_empty_ray_tracing': False,
            'sync_queue_size': 10,
            'topic_queue_size': 10,
            'Rtabmap/DetectionRate':'2',
            'Rtabmap/StartNewMapOnLoopClosure':'true',
            #'Rtabmap/TimeThr':'500',
            'RGBD/OptimizeFromGraphEnd':'false',
            'RGBD/OptimizeMaxError':'1.5',
            'RGBD/CreateOccupancyGrid':'false',
            'Optimizer/Robust':'false',
            'Optimizer/PriorsIgnored':'false',
            #'Kp/RoiRatios': '0.0 0.0 0.1 0.2',
            #'Vis/RoiRatios': '0.0 0.0 0.1 0.2',
            #memory
            'Mem/IncrementalMemory':'false',
            'Mem/InitWMWithAllNodes':'true',
            #'Mem/RehearsalSimilarity':'0.586',
            #'Mem/BinDataKept':'false', production only, not for testing
            #'Marker/Priors':'"1 13.77 -22.54 0.61 -0.13 0.08 -0.05|2 -15.01 9.91 0.66 -0.15 0.01 -1.50|3 -7.75 1.45 0.56 -2.12 0.15 -1.50|4 -22.59 12.09 0.51 -0.08 0.05 -3.11"',
            'Marker/Priors':'"1 13.78 -22.53 0.62 -0.14 0.07 -0.05|2 -15.04 9.89 0.66 -0.15 0.01 -1.57|3 -7.78 1.45 0.59 -0.14 0.07 -1.63|4 -22.58 12.05 0.55 -0.07 0.01 -3.10"',

            #'Stereo/OpticalFlow':'False',
            'Stereo/Gpu':'true',
            #'Stereo/MaxDisparity':'64',
            'log_level': 'warn'
           

        }],
        remappings=remappings,
        #arguments=['--uwarn'
                   #'--udebug'
        #           ]
    )

    stereo_container = ComposableNodeContainer(
    name='stereo_container',
    namespace=robot_name,
    package='rclcpp_components',
    executable='component_container_isolated',
    composable_node_descriptions=[
        stereo_sync
    ],
    output='screen',
    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/diagnostics', f'/{robot_name}/diagnostics')]
    )

    icp_container = ComposableNodeContainer(
    name='icp_container',
    namespace=robot_name,
    package='rclcpp_components',
    executable='component_container_isolated',
    composable_node_descriptions=[icp_odometry],
    output='screen',
    remappings=[('/tf','tf'),('/tf_static','tf_static'),('/diagnostics','/'+robot_name+'/diagnostics')]
    )  

    mapping_container = ComposableNodeContainer(
        name='mapping_container',
        namespace=robot_name,
        package='rclcpp_components',
        #prefix='xterm -e gdb -ex run --args',
        executable='component_container_isolated',
        composable_node_descriptions=[Mapping],
        output='screen',
        remappings=[('/tf','tf'),('/tf_static','tf_static'),('/diagnostics','/'+robot_name+'/diagnostics')] 
    )

    
    local_container = ComposableNodeContainer(
        name='local_container',
        namespace=robot_name,
        package='rclcpp_components',
        #prefix='xterm -e gdb -ex run --args',
        executable='component_container_isolated',
        composable_node_descriptions=[Localization],
        output='screen',
        remappings=[('/tf','tf'),('/tf_static','tf_static'),('/diagnostics','/'+robot_name+'/diagnostics')] 
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices = ['true','false'], description='Simulation purposes only'),
        DeclareLaunchArgument('deskewing', default_value='false', choices = ['true','false'], description='Lidar slicing'),
        DeclareLaunchArgument('deskewing_slerp', default_value='true', choices = ['true','false'], description='Lidar slicing'),
        DeclareLaunchArgument('mapping', default_value='true', choices = ['true','false'], description='True = localization, false = create new map'),
        DeclareLaunchArgument('qos', default_value='1', choices= ['1','2'], description='QoS 1 = reliable, 2 = best effort'),
        stereo_container,
        icp_container,
        mapping_container,
        local_container
    ])
