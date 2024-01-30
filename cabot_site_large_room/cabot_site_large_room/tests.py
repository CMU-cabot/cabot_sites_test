import random


def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0


def checks(tester):
    tester.check_topic_error(
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/interface' and msg.text=='vibration' and msg.memo=='unknown'"
    )


def wait_ready(tester):
    tester.wait_ready()


def _goto_target1(tester):
    tester.pub_topic(
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        message="data: 'navigation;destination;EDITOR_node_1705948557561'"
    )
    tester.wait_topic(
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/navigation' and msg.text=='completed'",
        timeout=120
    )


def test1_move_towards_a_pedestrian(tester):
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 5.0,
                "init_y": 0.0,
                "init_a": 180.0,
                "velocity": 0.5,
            },
        },
    ])
    _goto_target1(tester)


def test2_move_across_a_pedestrian(tester):
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
    _goto_target1(tester)


def test0_sfm_actors(tester):
    tester.check_collision()
    bound = 5.0
    actors = []
    for i in range(0, 10):
        actors.append({
            "name": f"actor{i}",
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": random.uniform(-bound, bound),
                "init_y": random.uniform(-bound, bound),
                "init_a": random.uniform(-180.0, 180.0),
                "velocity": 1.0,
            },
        })
    tester.reset_position(x=-6.0)
    tester.setup_actors(actors=actors)
    _goto_target1(tester)
