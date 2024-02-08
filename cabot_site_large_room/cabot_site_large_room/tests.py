import random
import time

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
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian(tester):
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
    _goto_target1(tester)


def test_category5_allocate_actors_preparation(tester):
    nx = 10
    ny = 5

    init_x = 0.0
    init_y = 10.0
    for j in range(0, ny):
        actors = []
        for i in range(0, nx):
            index = i + nx * j
            print(f"index={index}")
            actor = {
                "name": f'actor{index}',
                "module": "pedestrian.pool",
                "params": {
                    "init_x": init_x + i,
                    "init_y": init_y + j,
                    "init_a": -180.0,
                    "velocity": 0.95,
                },
            }
            actors.append(actor)
        tester.setup_actors(actors=actors)
        time.sleep(1)


def test_category5_case2_move_parallel_traffic(tester):
    # 5.2 parallel traffic
    tester.check_collision()
    tester.reset_position(x=-3.0)

    actors = []

    nx = 5
    ny = 5

    interval_x = 4.0
    interval_y = 3.0

    x_0 = 0.0
    y_0 = 0.0
    shift_y = -0.5

    # forward flow
    for i in range(0, nx):
        for j in range(0, ny):
            index = j + ny * i
            init_x = x_0 - interval_x * nx + interval_x * i
            init_y = y_0 - interval_y * (ny/2) + interval_y * j
            actors.append({
                "name": f'actor{index}',
                "module": "pedestrian.walk_across",
                "params": {
                    "init_x": init_x,
                    "init_y": init_y,
                    "init_a": 0.0,
                    "velocity": 1.0,
                },
            })

    # inverse flow
    for i in range(0, nx):
        for j in range(0, ny):
            index = j + ny * i + ny * nx
            init_x = x_0 + interval_x * nx - interval_x * i
            init_y = y_0 - interval_y * (ny/2) + interval_y * j + shift_y
            actors.append({
                "name": f'actor{index}',
                "module": "pedestrian.walk_across",
                "params": {
                    "init_x": init_x,
                    "init_y": init_y,
                    "init_a": 180.0,
                    "velocity": 1.0,
                },
            })

    tester.setup_actors(actors=actors)
    _goto_target1(tester)


def test_category5_case3_move_perpendicular_traffic(tester):
    # 5.3 Perpendicular traffic
    tester.check_collision()
    tester.reset_position(x=-10.0)

    actors = []

    interval_x = 3.0
    interval_y = 3.0

    init_x = -5.0
    init_y = 0.0
    for i in range(0, 5):
        actor = {
            "name": f'actor{i}',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": -90.0,
                "velocity": 0.95,
            },
        }
        actors.append(actor)
        init_y += interval_y

    init_x = -5.0 + interval_x
    init_y = -interval_x
    for i in range(5, 10):
        actor = {
            "name": f'actor{i}',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": 90.0,
                "velocity": 0.95,
            },
        }
        actors.append(actor)
        init_y -= interval_y

    init_x = -5.0 + interval_x + interval_x
    init_y = interval_x + interval_x
    for i in range(10, 15):
        actor = {
            "name": f'actor{i}',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": -90.0,
                "velocity": 0.95,
            },
        }
        actors.append(actor)
        init_y += interval_y

    init_x = -5.0 + 3 * interval_x
    init_y = - 3 * interval_x
    for i in range(15, 20):
        actor = {
            "name": f'actor{i}',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": 90.0,
                "velocity": 0.95,
            },
        }
        actors.append(actor)
        init_y -= interval_y

    tester.setup_actors(actors=actors)
    _goto_target1(tester)


def test_category6_case1_sfm_actors(tester):
    # 6.1 crowd navigation
    tester.check_collision()
    bound = 5.0
    actors = []
    n_actors = 10
    for i in range(0, n_actors):
        actors.append({
            "name": f"actor{i}",
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": random.uniform(-bound, bound),
                "init_y": random.uniform(-bound, bound),
                "init_a": random.uniform(-180.0, 180.0),
                "velocity": 1.0,
                "n_actors": n_actors,
                "min_x": -bound,
                "max_x": bound,
                "min_y": -bound,
                "max_y": bound,
            },
        })
    tester.reset_position(x=-6.0)
    tester.setup_actors(actors=actors)
    _goto_target1(tester)


def test_category6_case2_sfm_parallel_traffic(tester):
    # 6.2 parallel traffic
    tester.check_collision()
    actors = []

    nx = 5
    ny = 5

    interval_x = 3.0
    interval_y = 2.0

    x_0 = 0.0
    y_0 = 0.0
    shift_y = -0.5

    n_actors = nx * ny * 2

    std_x = 0.5
    std_y = 0.5

    # forward flow
    for i in range(0, nx):
        for j in range(0, ny):
            index = j + ny * i
            init_x = x_0 - interval_x * nx + interval_x * i + random.normalvariate(0.0, std_x)
            init_y = y_0 - interval_y * (ny//2) + interval_y * j + random.normalvariate(0.0, std_y)
            actors.append({
                "name": f"actor{index}",
                "module": "pedestrian.walk_sfm",
                "params": {
                    "init_x": init_x,
                    "init_y": init_y,
                    "init_a": 0.0,
                    "velocity": 1.0,
                    "n_actors": n_actors,
                    "goal_x": 14.0,
                    "goal_y": init_y
                },
            })

    # inverse flow
    for i in range(0, nx):
        for j in range(0, ny):
            index = j + ny * i + ny * nx
            init_x = x_0 + interval_x * nx - interval_x * i + random.normalvariate(0.0, std_x)
            init_y = y_0 - interval_y * (ny//2) + interval_y * j + shift_y + random.normalvariate(0.0, std_y)
            actors.append({
                "name": f"actor{index}",
                "module": "pedestrian.walk_sfm",
                "params": {
                    "init_x": init_x,
                    "init_y": init_y,
                    "init_a": 180.0,
                    "velocity": 1.0,
                    "n_actors": n_actors,
                    "goal_x": -14.0,
                    "goal_y": init_y
                },
            })

    tester.reset_position(x=-3.0)
    tester.setup_actors(actors=actors)
    _goto_target1(tester)


def test_category6_case3_sfm_perpendicular_traffic(tester):
    # 6.3 Perpendicular traffic
    tester.check_collision()
    tester.reset_position(x=-10.0)

    actors = []

    interval_x = 3.0
    interval_y = 3.0
    n_actors = 20

    init_x = -5.0
    init_y = 0.0
    for i in range(0, 5):
        actor = {
            "name": f"actor{i}",
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": -90.0,
                "velocity": 0.95,
                "n_actors": n_actors,
                "goal_x": init_x,
                "goal_y": -15.0
            }
        }
        actors.append(actor)
        init_y += interval_y

    init_x = -5.0 + interval_x
    init_y = -interval_x
    for i in range(5, 10):
        actor = {
            "name": f"actor{i}",
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": 90.0,
                "velocity": 0.95,
                "n_actors": n_actors,
                "goal_x": init_x,
                "goal_y": 15.0
            }
        }
        actors.append(actor)
        init_y -= interval_y

    init_x = -5.0 + interval_x + interval_x
    init_y = interval_x + interval_x
    for i in range(10, 15):
        actor = {
            "name": f"actor{i}",
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": -90.0,
                "velocity": 0.95,
                "n_actors": n_actors,
                "goal_x": init_x,
                "goal_y": -15.0
            }
        }
        actors.append(actor)
        init_y += interval_y

    init_x = -5.0 + 3 * interval_x
    init_y = - 3 * interval_x
    for i in range(15, 20):
        actor = {
            "name": f"actor{i}",
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": 90.0,
                "velocity": 0.95,
                "n_actors": n_actors,
                "goal_x": init_x,
                "goal_y": 15.0
            }
        }
        actors.append(actor)
        init_y -= interval_y

    tester.setup_actors(actors=actors)
    _goto_target1(tester)