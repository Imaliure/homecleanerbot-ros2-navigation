# Ev modelinin yolunu belirle
    world_path = os.path.join(pkg_path, 'worlds', 'house.world')

    # Gazebo Launch (Ev modeli ile başlat)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )
