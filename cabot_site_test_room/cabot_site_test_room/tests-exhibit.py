def config(tester):
    tester.config['init_x'] = 9.0
    tester.config['init_y'] = 9.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 90.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def test1_navigation_to_an_exhibit(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1710807829757')
    tester.wait_navigation_arrived(timeout=60)


def test2_navigation_to_an_exhibit(tester):
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 10.0,
                "init_y": 11.0,
                "init_a": -90.0
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 10.0,
                "init_y": 12.0,
                "init_a": 0.0
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
            "module": "pedestrian.pool",
            "params": {
                "init_x": 10.0,
                "init_y": 12.0,
                "init_a": 0.0
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
                "init_a": -90.0
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 14.0,
                "init_y": 12.0,
                "init_a": 0.0
            },
        },
    ])

    tester.wait_navigation_arrived(timeout=60)

