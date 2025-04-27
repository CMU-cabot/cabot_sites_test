# ******************************************************************************
#  Copyright (c) 2024  Carnegie Mellon University and Miraikan
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# ******************************************************************************

import random
import time
import math


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
    # tester.wait_localization_started()
    tester.wait_ready()


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

    # Send navigation cancellation in case it ends due to timeout
    tester.pub_topic(
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        message="data: 'navigation;cancel'"
    )


def _add_metric_condition_lt(tester, metric_name, success_threshold):
    """
    Adds a metric condition to the tester where the metric value must be less than the specified threshold.
    :param tester: The tester object that handles the metric conditions.
    :param metric_name: The name of the metric to evaluate.
    :param success_threshold: The threshold value that the metric must be less than to meet the condition.
    """
    condition = f"{success_threshold} > value"
    tester.add_metric_condition({"name": metric_name, "condition": condition})

# global variable
n_max_actors = 10


def _setup_actors_with_allocation(tester, actors):
    global n_max_actors

    if n_max_actors < len(actors):
        nx = 10
        ny = math.ceil(len(actors)/nx)

        init_x = 0.0
        init_y = 10.0
        for j in range(0, ny):
            actors_alloc = []
            for i in range(0, nx):
                index = i + nx * j
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
                actors_alloc.append(actor)
            tester.setup_actors(actors=actors_alloc)
            time.sleep(1)
        n_max_actors = len(actors)

    time.sleep(5)  # sleep to wait actor spawn
    tester.setup_actors(actors=actors)


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
            init_y = y_0 - interval_y * (ny//2) + interval_y * j
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
            init_y = y_0 - interval_y * (ny//2) + interval_y * j + shift_y
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

    _setup_actors_with_allocation(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 120) # 120 is the test case timeout value.
    _add_metric_condition_lt(tester, "robot_path_length", 43.7) # 43.7 is the average value of 5 runs (actor's speed is set to 0 and nx is set to 2) with a 200% margin added.
    _add_metric_condition_lt(tester, "time_not_moving", 14.6) # 14.6 is the average value of 5 runs (actor's speed is set to 0 and nx is set to 2) with a 200% margin added.
    _add_metric_condition_lt(tester, "cumulative_heading_changes", 9.6) # 9.6 is the average value of 5 runs (actor's speed is set to 0 and nx is set to 2) with a 200% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category5_case2_move_parallel_traffic_repeat(tester):
    # 5.2 parallel traffic (repeat)
    tester.check_collision()
    tester.reset_position(x=-6.0)

    actors = []

    nx = 5
    ny = 5

    interval_x = 4.0
    interval_y = 3.0

    x_0 = 0.0
    y_0 = 0.0
    shift_y = -0.5

    goal_x_dist = 12

    # forward flow
    for i in range(0, nx):
        for j in range(0, ny):
            index = j + ny * i
            init_x = x_0 - 8.0
            init_y = y_0 - interval_y * (ny//2) + interval_y * j
            goal_x = x_0 + goal_x_dist
            goal_y = init_y
            actors.append({
                "name": f'actor{index}',
                "module": "pedestrian.walk_across",
                "params": {
                    "init_x": init_x,
                    "init_y": init_y,
                    "init_a": 0.0,
                    "velocity": 1.0,
                    "goal_x": goal_x,
                    "goal_y": goal_y,
                    "repeat": 1,
                    "start_after": i * interval_x
                },
            })

    # inverse flow
    for i in range(0, nx):
        for j in range(0, ny):
            index = j + ny * i + ny * nx
            init_x = x_0 + 8.0
            init_y = y_0 - interval_y * (ny//2) + interval_y * j + shift_y
            goal_x = x_0 - goal_x_dist
            goal_y = init_y
            actors.append({
                "name": f'actor{index}',
                "module": "pedestrian.walk_across",
                "params": {
                    "init_x": init_x,
                    "init_y": init_y,
                    "init_a": 180.0,
                    "velocity": 1.0,
                    "goal_x": goal_x,
                    "goal_y": goal_y,
                    "repeat": 1,
                    "start_after": i * interval_x
                },
            })

    _setup_actors_with_allocation(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 120) # 120 is the test case timeout value.
    _add_metric_condition_lt(tester, "robot_path_length", 43.7) # 43.7 is set to the same value as test_category5_case2_move_parallel_traffic.
    _add_metric_condition_lt(tester, "time_not_moving", 14.6) # 14.6 is set to the same value as test_category5_case2_move_parallel_traffic.
    _add_metric_condition_lt(tester, "cumulative_heading_changes", 9.6) # 9.6 is set to the same value as test_category5_case2_move_parallel_traffic.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
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

    _setup_actors_with_allocation(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 120) # 120 is the test case timeout value.
    _add_metric_condition_lt(tester, "robot_path_length", 107.6) # 107.6 is the average value of 5 runs (actor's speed is set to 0 and actor is set to advance 8m) with a 400% margin added.
    _add_metric_condition_lt(tester, "time_not_moving", 26.7) # 26.7 is the average value of 5 runs (actor's speed is set to 0 and actor is set to advance 8m) with a 400% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category5_case3_move_perpendicular_traffic_repeat(tester):
    # 5.3 Perpendicular traffic (repeat)
    tester.check_collision()
    tester.reset_position(x=-10.0)

    actors = []

    interval_x = 3.0
    interval_y = 3.0

    init_x = -2.5
    init_y = 7.5
    goal_x = init_x
    goal_y = -init_y
    for i in range(0, 5):
        actor = {
            "name": f'actor{i}',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": -90.0,
                "velocity": 0.95,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "repeat": 1,
                "start_after": i * interval_y
            },
        }
        actors.append(actor)

    init_x = -2.5 + interval_x
    init_y = -7.5
    goal_x = init_x
    goal_y = -init_y
    for i in range(0, 5):
        actor = {
            "name": f'actor{i+5}',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": 90.0,
                "velocity": 0.95,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "repeat": 1,
                "start_after": i * interval_y
            },
        }
        actors.append(actor)

    init_x = -2.5 + 2 * interval_x
    init_y = 7.5
    goal_x = init_x
    goal_y = -init_y
    for i in range(0, 5):
        actor = {
            "name": f'actor{i+10}',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": -90.0,
                "velocity": 0.95,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "repeat": 1,
                "start_after": i * interval_y
            },
        }
        actors.append(actor)

    init_x = -2.5 + 3 * interval_x
    init_y = -7.5
    goal_x = init_x
    goal_y = -init_y
    for i in range(0, 5):
        actor = {
            "name": f'actor{i+15}',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": init_x,
                "init_y": init_y,
                "init_a": 90.0,
                "velocity": 0.95,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "repeat": 1,
                "start_after": i * interval_y
            },
        }
        actors.append(actor)

    _setup_actors_with_allocation(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 120) # 120 is the test case timeout value.
    _add_metric_condition_lt(tester, "robot_path_length", 107.6) # 107.6 is set to the same value as test_category5_case3_move_perpendicular_traffic.
    _add_metric_condition_lt(tester, "time_not_moving", 26.7) # 26.7 is set to the same value as test_category5_case3_move_perpendicular_traffic.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
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
    _setup_actors_with_allocation(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 120) # 120 is the test case timeout value.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category6_case1_sfm_actors_variant1_perfect_people_detection(tester):
    # 6.1 crowd navigation
    tester.set_people_detection_range(
        min_range=0.0,
        max_range=100.0,
        min_angle=-3.142,
        max_angle=3.142,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )
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
    _setup_actors_with_allocation(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 120) # 120 is the test case timeout value.
    _add_metric_condition_lt(tester, "time_not_moving", 3.6) # 3.6 is the average value of 5 runs in test_category6_case1_sfm_actors with a 100% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)
    tester.set_people_detection_range(
        min_range=0.29, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        max_range=7.07, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        min_angle=-2.28,
        max_angle=2.28,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )


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
    _setup_actors_with_allocation(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 120) # 120 is the test case timeout value.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
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

    _setup_actors_with_allocation(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 120) # 120 is the test case timeout value.
    _add_metric_condition_lt(tester, "robot_path_length", 107.6) # 107.6 is set to the same value as test_category5_case3_move_perpendicular_traffic.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)
