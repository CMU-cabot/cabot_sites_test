def config(tester):
    tester.config['init_x'] = 9.0
    tester.config['init_y'] = 9.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 90.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()

def test4_navigation_to_an_exhibit_and_then_elevator(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899235671')
    cancel = tester.check_topic_error(
        action="check_last_pose",
        topic="/path",
        topic_type="nav_msgs/msg/Path",
        condition="math.sqrt((msg.poses[-1].pose.position.x - 10.804997353263879)**2 + (msg.poses[-1].pose.position.y - 8.395464385083699)**2) < 0.1"
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
        condition="math.sqrt((msg.poses[-1].pose.position.x - 10.804997353263879)**2 + (msg.poses[-1].pose.position.y - 8.395464385083699)**2) > 0.1"
    )
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
