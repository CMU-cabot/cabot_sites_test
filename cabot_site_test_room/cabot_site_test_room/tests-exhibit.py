def config(tester):
    tester.config['init_x'] = 9.0
    tester.config['init_y'] = 9.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 90.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    # tester.wait_localization_started()
    tester.wait_ready()


def test8_navigation_to_an_exhibit_and_pause_and_move_manually_and_resume(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1710807829757')
    tester.wait_for(12)
    tester.button_down(3)
    tester.reset_position(x=10.0, y=7.0)
    tester.button_down(4)
    tester.wait_navigation_arrived(timeout=60)


def test7_navigation_to_an_exhibit(tester):
    tester.reset_position(x=0.0, y=-4.0, z=10.0, a=180.0)
    tester.goto_node("EDITOR_node_1711930827334")
    tester.wait_navigation_arrived(timeout=45)


def test6_navigation_to_an_exhibit_without_social_announce(tester):
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 11.0,
                "init_a": -90.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 12.0,
                "init_a": 0.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
    ])
    tester.goto_node('EDITOR_node_1710807829757')
    cancel = tester.check_topic_error(
        action="check_no_social",
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category == 'cabot/interface' and msg.text == 'social'"
    )
    tester.check_topic(
        action="check_stop_reason",
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category == 'cabot/interface' and msg.text == 'stop-reason'"
    )
    tester.wait_for(30)
    cancel()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 11.0,
                "init_a": -90.0,
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 12.0,
                "init_a": 0.0,
            },
        },
    ])
    tester.wait_navigation_arrived(timeout=30)


def test5_navigation_to_an_exhibit_complete_stop(tester):
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 11.0,
                "init_a": -90.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 12.0,
                "init_a": 0.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
    ])
    tester.goto_node('EDITOR_node_1710807829757')
    tester.wait_for(10)
    cancel = tester.check_topic_error(
        action="check_complete_stop",
        topic="/cabot/cmd_vel",
        topic_type="geometry_msgs/msg/Twist",
        condition="msg.linear.x == 0 and msg.angular.z > 0"
    )
    tester.wait_for(5)
    cancel()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 11.0,
                "init_a": -90.0
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 12.0,
                "init_a": 0.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
    ])
    tester.wait_for(10)
    cancel = tester.check_topic_error(
        action="check_complete_stop",
        topic="/cabot/cmd_vel",
        topic_type="geometry_msgs/msg/Twist",
        condition="msg.linear.x == 0 and msg.angular.z > 0"
    )
    tester.wait_for(5)
    cancel()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 11.0,
                "init_a": -90.0,
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 12.0,
                "init_a": 0.0,
            },
        },
    ])

    tester.wait_navigation_arrived(timeout=60)


def test4_navigation_to_an_exhibit_and_then_elevator(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899235671')
    cancel = tester.check_topic_error(
        action="check_last_pose",
        topic="/path",
        topic_type="nav_msgs/msg/Path",
        condition="math.sqrt((msg.poses[-1].pose.position.x - 10.2)**2 + (msg.poses[-1].pose.position.y - 8.0)**2) > 0.5"
    )
    tester.wait_goal("NarrowGoal")
    cancel()
    tester.wait_navigation_arrived(timeout=60)


def test3_navigation_to_an_exhibit_and_then_elevator(tester):
    tester.reset_position(y=4.0)
    tester.goto_node('EDITOR_node_1707899235671')
    tester.check_topic_error(
        action="check_last_pose",
        topic="/path",
        topic_type="nav_msgs/msg/Path",
        condition="math.sqrt((msg.poses[-1].pose.position.x - 10.8)**2 + (msg.poses[-1].pose.position.y - 8.4)**2) > 0.1"
    )
    tester.wait_navigation_arrived(timeout=60)



def test2_2_navigation_to_an_exhibit_check_social_navigation(tester):
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor1',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 12.0,
                "init_a": 0.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
    ])
    tester.goto_node('EDITOR_node_1710807829757')
    tester.wait_topic(
        action="check_social_navigation",
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/interface' and msg.text=='Message' and msg.memo=='PERSON_AHEAD'"
    )
    cancel = tester.check_topic_error(
        action="check_social_navigation",
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/interface' and msg.text=='Message' and msg.memo=='PERSON_AHEAD'"
    )
    tester.wait_for(28)
    cancel()
    tester.wait_topic(
        action="check_social_navigation",
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/interface' and msg.text=='Message' and msg.memo=='PERSON_AHEAD'"
    )
    tester.setup_actors(actors=[
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 12.0,
                "init_a": 0.0,
            },
        },
    ])
    tester.wait_navigation_arrived(timeout=60)


def test2_navigation_to_an_exhibit(tester):
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 11.0,
                "init_a": -90.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 12.0,
                "init_a": 0.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
    ])
    tester.goto_node('EDITOR_node_1710807829757')
    tester.wait_for(10)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 11.0,
                "init_a": -90.0
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.stand_with_noise",
            "params": {
                "init_x": 10.0,
                "init_y": 12.0,
                "init_a": 0.0,
                "std_x": 0.01,
                "std_y": 0.01,
            },
        },
    ])
    tester.wait_for(10)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 11.0,
                "init_a": -90.0,
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 12.0,
                "init_a": 0.0,
            },
        },
    ])

    tester.wait_navigation_arrived(timeout=60)


def test1_navigation_to_an_exhibit(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1710807829757')
    tester.wait_navigation_arrived(timeout=60)
