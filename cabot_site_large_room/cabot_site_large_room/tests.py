def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0

    tester.set_evaluation_parameters(
        metrics=[
            "total_time",
            "robot_path_length",
            "time_not_moving",
            "avg_robot_linear_speed",
            "cumulative_heading_changes",
            "robot_on_person_collision_count",
            "person_on_robot_collision_count"
        ],
        # robot_radius=0.45  # default value
    )

    tester.set_people_detection_range(
        min_range=0.29, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        max_range=7.07, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        min_angle=-2.28,
        max_angle=2.28,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )


def checks(tester):
    tester.check_topic_error(
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/interface' and msg.text=='vibration' and msg.memo=='unknown'"
    )


def wait_ready(tester):
    tester.wait_localization_started()


def _goto_target1(tester):
    tester.pub_topic(
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        message="data: 'navigation;destination;EDITOR_node_1705948557561'"
    )
    # start computing evaluation metrics after publshing the destination
    tester.start_evaluation()
    tester.wait_topic(
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/navigation' and msg.text=='completed'",
        timeout=120
    )


def _check_robot_on_person_collision(tester):
    tester.check_topic_error(
        action="check robot_on_person_collision_count",
        topic="/metric",
        topic_type="pedestrian_plugin_msgs/msg/Metric",
        condition="msg.name=='robot_on_person_collision_count' and 1<=msg.value"
    )


def test_category1_case1_move_towards_a_pedestrian(tester):
    # 1.1 Frontal Approace
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 10.0,
                "init_y": -0.5,
                "init_a": 180.0,
                "velocity": 0.5,
                "decel_distance": 1.5,
                "pause_distance": 1.0
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category1_case2_move_towards_a_pedestrian_and_avoid(tester):
    # 1.2 Pedestrian Obstruction
    tester.check_collision()
    tester.reset_position(x=-5.0)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 5.0,
                "init_y": -0.5,
                "init_a": 180.0,
                "velocity": 0.0  # does not move
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category1_case4_robot_overtaking(tester):
    # 1.4 robot overtaking
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 1.0,
                "init_y": -0.5,
                "init_a": 0.0,
                "velocity": 0.25  # very slow
            },
        },
    ])
    # check too slow movement
    tester.check_topic_error(
        action="check total_time",
        topic="/metric",
        topic_type="pedestrian_plugin_msgs/msg/Metric",
        condition="msg.name=='total_time' and 30.0<=msg.value"
    )
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category1_case5_down_the_path(tester):
    # 1.5 Down path
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 1.0,
                "init_y": -0.5,
                "init_a": 0.0,
                "velocity": 0.75
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian1(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": 5.0,
                "init_a": -90.0,
                "velocity": 0.95,
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian2(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": -5.0,
                "init_a": 90.0,
                "velocity": 0.95,
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian3(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": 6.0,
                "init_a": -90.0,
                "velocity": 0.95,
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian4(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": -6.0,
                "init_a": 90.0,
                "velocity": 0.95,
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian5(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": 7.0,
                "init_a": -90.0,
                "velocity": 0.95,
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian6(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": -7.0,
                "init_a": 90.0,
                "velocity": 0.95,
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)


def test_category3_case1_move_across_a_pedestrian_proceed(tester):
    # 2.3 Intersection Proceed
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": 5.0,
                "init_a": -90.0,
                "velocity": 0.95,
                "pause_distance": 1.5
            },
        },
    ])
    _check_robot_on_person_collision(tester)
    _goto_target1(tester)
