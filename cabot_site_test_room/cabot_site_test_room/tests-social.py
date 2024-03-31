def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def test1_test_social_announce(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_topic(
        action="check_event",
        topic="/cabot/event",
        topic_type="std_msgs/msg/String",
        condition="msg.data == 'navigation;event;navigation_start'"
    )
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 5.0,
                "init_y": 0.0,
                "init_a": -90.0
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 5.0,
                "init_y": 1.0,
                "init_a": 0.0
            },
        },
        {
            "name": 'actor2',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 5.0,
                "init_y": -1.0,
                "init_a": 0.0
            },
        },
    ])
    tester.wait_topic(
        action="check_social_navigation",
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category == 'cabot/interface' and  msg.text == 'Message' and msg.memo == 'PERSON_AHEAD'"
    )
    tester.wait_navigation_arrived(timeout=30)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 25.0,
                "init_y": 0.0,
                "init_a": -90.0
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 25.0,
                "init_y": 1.0,
                "init_a": 0.0
            },
        },
        {
            "name": 'actor2',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 25.0,
                "init_y": -1.0,
                "init_a": 0.0
            },
        },
    ])


def test2_avoid_obstacle(tester):
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_topic(
        action="check_event",
        topic="/cabot/event",
        topic_type="std_msgs/msg/String",
        condition="msg.data == 'navigation;event;navigation_start'"
    )
    tester.wait_for(3)
    tester.spawn_obstacle(name="f1_door", x=6.0, y=0.5, z=0.0, yaw=0.0, width=0.25, height=2.5, depth=1.0)
    tester.wait_topic(
        action="check_social_navigation",
        topic="/cabot/event",
        topic_type="std_msgs/msg/String",
        condition="msg.data == 'navigation_sound;OBSTACLE_AHEAD'"
    )
    tester.wait_navigation_arrived(timeout=60)
