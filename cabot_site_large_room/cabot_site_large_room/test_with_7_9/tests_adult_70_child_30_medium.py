# ******************************************************************************
#  Copyright (c) 2024, 2025  Carnegie Mellon University and Miraikan
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

import time


def config(tester):
    # original config
    tester.config['init_x'] = -9.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0

    tester.set_evaluation_parameters(
        robot_radius=0.25,
        metrics=[
            "total_time",
            "robot_path_length",
            "time_not_moving",
            "avg_robot_linear_speed",
            "cumulative_heading_changes",
            "minimum_distance_to_people",
            "minimum_distance_to_child",
            "minimum_distance_to_adult",
            "maximum_distance_to_people",
            "robot_on_person_collision_count",
            "person_on_robot_collision_count",
            "collision",
            "proximity_violation"
        ],
    )

    tester.set_people_detection_range(
        min_range=0.29,
        max_range=7.07,
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
    condition = f"{success_threshold} > value"
    tester.add_metric_condition({"name": metric_name, "condition": condition})


def _add_metric_condition_gt(tester, metric_name, success_threshold):
    condition = f"{success_threshold} < value"
    tester.add_metric_condition({"name": metric_name, "condition": condition})

def _setup_actors(tester, actors):
    tester.setup_actors(actors=actors)
    time.sleep(5)

def tests_adult_70_child_30_test_case_01_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.80,
                "init_y": -4.90,
                "init_a": 144.90,
                "velocity": 1.11,
                "goals": [[5.80, -4.90], [6.27, -4.18], [-3.55, 1.17], [4.63, 4.73]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.97,
                "init_y": -4.35,
                "init_a": 144.90,
                "velocity": 1.11,
                "goal_x": 5.80,
                "goal_y": -4.90,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.01,
                "init_y": 1.79,
                "init_a": -35.67,
                "velocity": 1.03,
                "goal_x": -2.01,
                "goal_y": 1.79,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.79,
                "init_y": 1.16,
                "init_a": -35.67,
                "velocity": 1.03,
                "goals": [[-2.01, 1.79], [-7.53, 4.58], [3.12, -5.94], [-2.29, 1.23]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.19,
                "init_y": 2.85,
                "init_a": 75.06,
                "velocity": 1.17,
                "goal_x": 3.44,
                "goal_y": 1.87,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.79,
                "init_y": -3.75,
                "init_a": 0.71,
                "velocity": 1.16,
                "goal_x": 6.82,
                "goal_y": -5.77,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.04,
                "init_y": 1.64,
                "init_a": -33.61,
                "velocity": 0.85,
                "goal_x": 5.80,
                "goal_y": -2.95,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_02_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.00,
                "init_y": 3.33,
                "init_a": 23.06,
                "velocity": 1.15,
                "goal_x": 6.16,
                "goal_y": 1.18,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.00,
                "init_y": 3.34,
                "init_a": 23.06,
                "velocity": 1.15,
                "goals": [[6.16, 1.18], [0.65, -1.37], [-1.89, 2.74], [4.82, 2.84]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.72,
                "init_y": 0.15,
                "init_a": 177.69,
                "velocity": 0.82,
                "goal_x": 4.38,
                "goal_y": 4.53,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.74,
                "init_y": 0.32,
                "init_a": 177.69,
                "velocity": 0.82,
                "goals": [[4.38, 4.53], [0.28, 3.41], [-3.28, 4.18], [7.17, -2.26]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.76,
                "init_y": -3.90,
                "init_a": -37.15,
                "velocity": 0.95,
                "goal_x": 0.24,
                "goal_y": 0.89,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.64,
                "init_y": -5.50,
                "init_a": 14.01,
                "velocity": 0.81,
                "goal_x": -2.39,
                "goal_y": 5.83,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.56,
                "init_y": 5.06,
                "init_a": 34.64,
                "velocity": 1.20,
                "goal_x": 4.60,
                "goal_y": -2.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.81,
                "init_y": -1.39,
                "init_a": -21.06,
                "velocity": 0.82,
                "goal_x": -3.37,
                "goal_y": -1.33,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.38,
                "init_y": 3.74,
                "init_a": -150.00,
                "velocity": 0.85,
                "goal_x": -5.25,
                "goal_y": -5.43,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_03_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.67,
                "init_y": 3.62,
                "init_a": 170.56,
                "velocity": 0.94,
                "goal_x": -5.67,
                "goal_y": 3.62,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.82,
                "init_y": 3.09,
                "init_a": 170.56,
                "velocity": 0.94,
                "goal_x": -5.67,
                "goal_y": 3.62,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.68,
                "init_y": -2.60,
                "init_a": -41.49,
                "velocity": 1.12,
                "goal_x": -2.68,
                "goal_y": -2.60,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.33,
                "init_y": -3.36,
                "init_a": -41.49,
                "velocity": 1.12,
                "goals": [[-2.68, -2.60], [-4.23, -5.97], [6.29, -2.87], [-4.30, -4.81]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.61,
                "init_y": -4.93,
                "init_a": 35.45,
                "velocity": 1.18,
                "goal_x": 4.60,
                "goal_y": 0.13,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.99,
                "init_y": 4.32,
                "init_a": 98.92,
                "velocity": 1.19,
                "goals": [[-3.83, 4.86], [5.87, 3.78], [-0.50, 0.61], [6.28, -4.41]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.73,
                "init_y": -0.90,
                "init_a": -57.60,
                "velocity": 1.17,
                "goal_x": -4.83,
                "goal_y": 5.29,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.82,
                "init_y": -1.69,
                "init_a": 83.20,
                "velocity": 0.95,
                "goal_x": 1.67,
                "goal_y": 2.40,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_04_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.18,
                "init_y": -0.11,
                "init_a": -0.28,
                "velocity": 1.14,
                "goals": [[-1.18, -0.11], [-6.65, 1.57], [6.92, -2.30], [3.31, -4.93]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.81,
                "init_y": -1.04,
                "init_a": -0.28,
                "velocity": 1.14,
                "goal_x": -1.18,
                "goal_y": -0.11,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.13,
                "init_y": 5.30,
                "init_a": 28.19,
                "velocity": 1.11,
                "goal_x": 4.13,
                "goal_y": 5.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.17,
                "init_y": 6.30,
                "init_a": 28.19,
                "velocity": 1.11,
                "goals": [[4.13, 5.30], [3.06, -1.25], [-1.21, 1.66], [-7.53, 3.64]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.74,
                "init_y": -0.64,
                "init_a": 62.90,
                "velocity": 1.11,
                "goal_x": -5.83,
                "goal_y": 2.41,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.78,
                "init_y": -4.93,
                "init_a": 37.63,
                "velocity": 1.18,
                "goal_x": 2.10,
                "goal_y": -1.79,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.68,
                "init_y": 0.57,
                "init_a": 106.34,
                "velocity": 1.15,
                "goal_x": -3.02,
                "goal_y": -1.83,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.10,
                "init_y": 3.43,
                "init_a": -42.69,
                "velocity": 0.94,
                "goal_x": 5.73,
                "goal_y": 1.04,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_05_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.05,
                "init_y": 2.86,
                "init_a": 23.79,
                "velocity": 0.85,
                "goal_x": 2.05,
                "goal_y": 2.86,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.75,
                "init_y": 3.58,
                "init_a": 23.79,
                "velocity": 0.85,
                "goal_x": 2.05,
                "goal_y": 2.86,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.17,
                "init_y": 0.49,
                "init_a": -12.95,
                "velocity": 1.08,
                "goals": [[1.17, 0.49], [0.12, 1.38], [0.73, -0.02], [-2.82, -2.61]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.52,
                "init_y": 1.43,
                "init_a": -12.95,
                "velocity": 1.08,
                "goal_x": 1.17,
                "goal_y": 0.49,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.68,
                "init_y": -5.41,
                "init_a": 82.34,
                "velocity": 1.16,
                "goals": [[0.40, 0.79], [4.09, 3.40], [-1.27, -2.29], [6.23, -3.93]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.94,
                "init_y": -3.31,
                "init_a": -128.96,
                "velocity": 1.01,
                "goal_x": 7.95,
                "goal_y": 3.69,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.86,
                "init_y": -1.01,
                "init_a": 104.27,
                "velocity": 0.83,
                "goal_x": -3.89,
                "goal_y": -4.20,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.94,
                "init_y": -4.63,
                "init_a": 114.71,
                "velocity": 1.16,
                "goal_x": -4.39,
                "goal_y": 1.09,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.60,
                "init_y": 1.83,
                "init_a": 105.30,
                "velocity": 0.95,
                "goal_x": -2.42,
                "goal_y": 2.88,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_06_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.28,
                "init_y": 5.18,
                "init_a": 115.33,
                "velocity": 1.06,
                "goal_x": -2.54,
                "goal_y": -5.99,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.23,
                "init_y": 6.03,
                "init_a": 115.33,
                "velocity": 1.06,
                "goals": [[-2.54, -5.99], [5.35, 1.75], [-6.28, -0.48], [-0.91, -2.30]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.75,
                "init_y": -1.75,
                "init_a": 85.54,
                "velocity": 1.06,
                "goal_x": 4.82,
                "goal_y": 2.34,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.33,
                "init_y": -0.94,
                "init_a": 85.54,
                "velocity": 1.06,
                "goal_x": 4.82,
                "goal_y": 2.34,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.08,
                "init_y": 1.93,
                "init_a": 16.11,
                "velocity": 1.16,
                "goal_x": -3.12,
                "goal_y": -0.43,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.33,
                "init_y": -3.83,
                "init_a": 126.00,
                "velocity": 0.85,
                "goals": [[-5.04, -1.36], [-3.03, -4.91], [2.21, 2.14], [-7.99, -4.04]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.67,
                "init_y": 5.84,
                "init_a": 11.02,
                "velocity": 0.83,
                "goal_x": -5.00,
                "goal_y": 3.06,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_07_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.96,
                "init_y": 4.07,
                "init_a": 69.71,
                "velocity": 0.85,
                "goal_x": 6.96,
                "goal_y": 4.07,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.27,
                "init_y": 4.79,
                "init_a": 69.71,
                "velocity": 0.85,
                "goal_x": 6.96,
                "goal_y": 4.07,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.10,
                "init_y": -3.72,
                "init_a": -3.82,
                "velocity": 1.18,
                "goal_x": -6.10,
                "goal_y": -3.72,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.50,
                "init_y": -2.91,
                "init_a": -3.82,
                "velocity": 1.18,
                "goal_x": -6.10,
                "goal_y": -3.72,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.29,
                "init_y": -2.36,
                "init_a": -63.89,
                "velocity": 1.00,
                "goals": [[2.70, -5.86], [-1.90, -0.89], [-5.09, 4.19], [7.56, -0.57]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.74,
                "init_y": -5.15,
                "init_a": 69.68,
                "velocity": 0.91,
                "goals": [[-7.54, 2.22], [-7.07, -3.97], [2.76, -4.68], [-6.65, 3.55]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.60,
                "init_y": 5.41,
                "init_a": -166.35,
                "velocity": 0.98,
                "goal_x": 4.27,
                "goal_y": 0.76,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_08_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.65,
                "init_y": -0.06,
                "init_a": 159.35,
                "velocity": 1.06,
                "goal_x": -4.33,
                "goal_y": 1.17,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.80,
                "init_y": -0.57,
                "init_a": 159.35,
                "velocity": 1.06,
                "goal_x": -4.33,
                "goal_y": 1.17,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.79,
                "init_y": -4.42,
                "init_a": -68.62,
                "velocity": 1.12,
                "goal_x": -3.67,
                "goal_y": -5.49,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.79,
                "init_y": -4.47,
                "init_a": -68.62,
                "velocity": 1.12,
                "goals": [[-3.67, -5.49], [-1.52, -4.23], [0.28, -1.07], [-4.40, -5.22]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.26,
                "init_y": 1.75,
                "init_a": 14.41,
                "velocity": 0.88,
                "goal_x": -2.16,
                "goal_y": -0.09,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.12,
                "init_y": -1.92,
                "init_a": 79.02,
                "velocity": 0.88,
                "goal_x": 6.02,
                "goal_y": -4.57,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.39,
                "init_y": -4.02,
                "init_a": 3.39,
                "velocity": 1.06,
                "goal_x": 5.03,
                "goal_y": -0.86,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.42,
                "init_y": -1.79,
                "init_a": 64.83,
                "velocity": 1.20,
                "goal_x": 4.49,
                "goal_y": -0.30,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.49,
                "init_y": 4.70,
                "init_a": -51.15,
                "velocity": 1.09,
                "goals": [[7.74, 5.10], [-2.19, 1.23], [-6.06, 1.41], [5.34, 4.68]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_09_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.95,
                "init_y": 0.77,
                "init_a": 24.94,
                "velocity": 1.04,
                "goal_x": -3.95,
                "goal_y": 0.77,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.95,
                "init_y": 0.69,
                "init_a": 24.94,
                "velocity": 1.04,
                "goal_x": -3.95,
                "goal_y": 0.77,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.40,
                "init_y": 4.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goals": [[3.40, 4.71], [5.05, -1.28], [5.71, -0.71], [5.88, -3.19]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.46,
                "init_y": 3.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goal_x": 3.40,
                "goal_y": 4.71,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.69,
                "init_y": -2.51,
                "init_a": 31.90,
                "velocity": 0.83,
                "goal_x": -0.89,
                "goal_y": 3.23,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.74,
                "init_y": 2.87,
                "init_a": -38.87,
                "velocity": 1.12,
                "goal_x": -4.04,
                "goal_y": 4.58,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.11,
                "init_y": -4.04,
                "init_a": 48.60,
                "velocity": 1.01,
                "goals": [[0.32, -3.93], [-6.16, -0.46], [-4.05, 4.12], [4.15, 1.28]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_10_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.29,
                "init_y": -5.58,
                "init_a": 154.03,
                "velocity": 0.81,
                "goals": [[7.33, 2.68], [-1.81, 3.35], [-5.10, 3.87], [-2.39, 5.76]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.83,
                "init_y": -6.42,
                "init_a": 154.03,
                "velocity": 0.81,
                "goals": [[7.33, 2.68], [7.30, 4.90], [0.02, -4.78], [3.70, -4.86]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.32,
                "init_y": 1.46,
                "init_a": 78.72,
                "velocity": 0.94,
                "goal_x": 6.95,
                "goal_y": 2.73,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.34,
                "init_y": 1.29,
                "init_a": 78.72,
                "velocity": 0.94,
                "goal_x": 6.95,
                "goal_y": 2.73,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.96,
                "init_y": 1.52,
                "init_a": 104.30,
                "velocity": 0.86,
                "goal_x": 1.44,
                "goal_y": 5.72,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.63,
                "init_y": -5.88,
                "init_a": 121.01,
                "velocity": 0.85,
                "goal_x": -6.05,
                "goal_y": 4.07,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.47,
                "init_y": 0.69,
                "init_a": -70.82,
                "velocity": 1.15,
                "goal_x": -2.04,
                "goal_y": -4.32,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.95,
                "init_y": 5.78,
                "init_a": 74.95,
                "velocity": 0.82,
                "goal_x": 4.23,
                "goal_y": -3.82,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_11_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.32,
                "init_y": 0.33,
                "init_a": -46.10,
                "velocity": 1.02,
                "goal_x": -0.32,
                "goal_y": 0.33,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.13,
                "init_y": -0.65,
                "init_a": -46.10,
                "velocity": 1.02,
                "goal_x": -0.32,
                "goal_y": 0.33,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.78,
                "init_y": -2.39,
                "init_a": -175.17,
                "velocity": 1.16,
                "goals": [[-7.78, -2.39], [2.72, -5.21], [0.07, -2.54], [-6.59, 1.05]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.68,
                "init_y": -1.40,
                "init_a": -175.17,
                "velocity": 1.16,
                "goal_x": -7.78,
                "goal_y": -2.39,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.33,
                "init_y": -2.60,
                "init_a": 77.31,
                "velocity": 1.03,
                "goal_x": 3.89,
                "goal_y": -4.21,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.11,
                "init_y": -4.28,
                "init_a": -58.15,
                "velocity": 1.15,
                "goals": [[-7.65, -1.20], [4.37, -3.37], [-2.08, -2.83], [-4.84, 0.90]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.10,
                "init_y": -4.34,
                "init_a": -78.76,
                "velocity": 0.97,
                "goal_x": -4.00,
                "goal_y": 2.99,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_12_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.54,
                "init_y": -5.96,
                "init_a": -14.91,
                "velocity": 0.86,
                "goal_x": 3.54,
                "goal_y": -5.96,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.96,
                "init_y": -5.06,
                "init_a": -14.91,
                "velocity": 0.86,
                "goal_x": 3.54,
                "goal_y": -5.96,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.87,
                "init_y": -2.86,
                "init_a": -53.96,
                "velocity": 0.98,
                "goals": [[1.87, -2.86], [-7.71, 1.61], [3.99, 1.85], [2.33, 4.73]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.15,
                "init_y": -3.56,
                "init_a": -53.96,
                "velocity": 0.98,
                "goal_x": 1.87,
                "goal_y": -2.86,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.42,
                "init_y": -3.29,
                "init_a": 75.07,
                "velocity": 1.02,
                "goal_x": -4.53,
                "goal_y": -5.57,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.80,
                "init_y": -5.33,
                "init_a": -136.61,
                "velocity": 0.91,
                "goals": [[-1.13, 2.94], [-1.44, -4.45], [-2.96, -4.16], [-3.63, -1.57]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.61,
                "init_y": 5.43,
                "init_a": -146.85,
                "velocity": 1.13,
                "goal_x": 2.27,
                "goal_y": -5.92,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.95,
                "init_y": -0.94,
                "init_a": 111.46,
                "velocity": 0.94,
                "goal_x": -0.73,
                "goal_y": -3.27,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.20,
                "init_y": -2.15,
                "init_a": 159.86,
                "velocity": 0.92,
                "goal_x": -0.98,
                "goal_y": 2.83,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_13_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.25,
                "init_y": 5.63,
                "init_a": 164.71,
                "velocity": 0.96,
                "goal_x": 5.82,
                "goal_y": -2.88,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.67,
                "init_y": 4.81,
                "init_a": 164.71,
                "velocity": 0.96,
                "goal_x": 5.82,
                "goal_y": -2.88,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.33,
                "init_y": -5.08,
                "init_a": -162.91,
                "velocity": 1.20,
                "goals": [[5.03, -2.07], [-6.91, 5.63], [-3.89, -1.81], [5.22, 0.96]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.18,
                "init_y": -5.60,
                "init_a": -162.91,
                "velocity": 1.20,
                "goals": [[5.03, -2.07], [-2.12, -0.49], [-0.30, 3.91], [-1.52, -4.05]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.73,
                "init_y": -2.15,
                "init_a": 126.91,
                "velocity": 1.10,
                "goal_x": -6.54,
                "goal_y": -4.97,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.84,
                "init_y": -4.51,
                "init_a": -95.80,
                "velocity": 1.02,
                "goal_x": 7.14,
                "goal_y": 1.64,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.41,
                "init_y": 5.35,
                "init_a": -6.24,
                "velocity": 1.05,
                "goal_x": 4.34,
                "goal_y": 0.88,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.05,
                "init_y": -5.21,
                "init_a": 120.15,
                "velocity": 1.12,
                "goal_x": -2.46,
                "goal_y": -1.97,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_14_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.96,
                "init_y": -3.64,
                "init_a": -29.65,
                "velocity": 0.86,
                "goal_x": -3.91,
                "goal_y": -4.23,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.65,
                "init_y": -4.35,
                "init_a": -29.65,
                "velocity": 0.86,
                "goal_x": -3.91,
                "goal_y": -4.23,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.55,
                "init_y": 0.34,
                "init_a": 10.47,
                "velocity": 1.17,
                "goals": [[-0.96, 1.37], [-2.62, 2.31], [4.05, 0.02], [-5.01, 2.40]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.00,
                "init_y": -0.49,
                "init_a": 10.47,
                "velocity": 1.17,
                "goal_x": -0.96,
                "goal_y": 1.37,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.40,
                "init_y": -0.17,
                "init_a": 103.05,
                "velocity": 0.96,
                "goal_x": -1.09,
                "goal_y": 4.27,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.82,
                "init_y": -3.44,
                "init_a": -147.27,
                "velocity": 0.91,
                "goal_x": -3.70,
                "goal_y": 1.42,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.41,
                "init_y": -0.93,
                "init_a": 25.66,
                "velocity": 0.88,
                "goals": [[5.62, -4.80], [-3.64, 4.67], [-6.43, -5.38], [2.80, 0.74]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_15_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.61,
                "init_y": 2.37,
                "init_a": 127.26,
                "velocity": 1.07,
                "goal_x": 4.61,
                "goal_y": 2.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.15,
                "init_y": 3.21,
                "init_a": 127.26,
                "velocity": 1.07,
                "goal_x": 4.61,
                "goal_y": 2.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.41,
                "init_y": -1.01,
                "init_a": 149.29,
                "velocity": 0.99,
                "goals": [[1.41, -1.01], [-1.48, 0.86], [-6.49, 1.60], [-7.15, -0.78]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.18,
                "init_y": -1.64,
                "init_a": 149.29,
                "velocity": 0.99,
                "goal_x": 1.41,
                "goal_y": -1.01,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.99,
                "init_y": -5.30,
                "init_a": -52.63,
                "velocity": 0.97,
                "goal_x": -6.40,
                "goal_y": 4.68,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.79,
                "init_y": -0.02,
                "init_a": -127.88,
                "velocity": 1.13,
                "goal_x": -6.98,
                "goal_y": -0.12,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.24,
                "init_y": 1.70,
                "init_a": -75.90,
                "velocity": 1.12,
                "goal_x": -0.63,
                "goal_y": -4.67,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.01,
                "init_y": -3.88,
                "init_a": 92.36,
                "velocity": 1.09,
                "goal_x": 5.87,
                "goal_y": 4.22,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.29,
                "init_y": -0.66,
                "init_a": -166.12,
                "velocity": 1.11,
                "goals": [[-0.38, 3.76], [6.74, 3.79], [0.70, 1.29], [1.21, 4.40]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_16_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.88,
                "init_y": -4.81,
                "init_a": -147.50,
                "velocity": 1.02,
                "goal_x": -6.16,
                "goal_y": -2.70,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.71,
                "init_y": -5.38,
                "init_a": -147.50,
                "velocity": 1.02,
                "goals": [[-6.16, -2.70], [-1.84, 2.06], [2.06, -5.90], [5.05, 1.57]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.11,
                "init_y": 3.77,
                "init_a": 50.90,
                "velocity": 1.08,
                "goal_x": 3.47,
                "goal_y": -4.97,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.24,
                "init_y": 4.26,
                "init_a": 50.90,
                "velocity": 1.08,
                "goal_x": 3.47,
                "goal_y": -4.97,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.50,
                "init_y": 4.32,
                "init_a": 48.32,
                "velocity": 1.11,
                "goals": [[-1.94, 3.03], [-7.86, 0.15], [-0.18, 2.77], [-6.40, -5.50]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.25,
                "init_y": -3.84,
                "init_a": -176.02,
                "velocity": 1.03,
                "goal_x": 3.25,
                "goal_y": -2.60,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.19,
                "init_y": -0.30,
                "init_a": 55.36,
                "velocity": 1.00,
                "goal_x": 3.48,
                "goal_y": 4.95,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.13,
                "init_y": 0.63,
                "init_a": 135.09,
                "velocity": 1.14,
                "goal_x": 7.20,
                "goal_y": -4.59,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_17_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.40,
                "init_y": -0.79,
                "init_a": -145.87,
                "velocity": 1.13,
                "goal_x": -6.97,
                "goal_y": -2.49,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.30,
                "init_y": -1.21,
                "init_a": -145.87,
                "velocity": 1.13,
                "goal_x": -6.97,
                "goal_y": -2.49,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.76,
                "init_y": 3.02,
                "init_a": -13.14,
                "velocity": 0.90,
                "goal_x": -2.68,
                "goal_y": -5.19,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.76,
                "init_y": 2.96,
                "init_a": -13.14,
                "velocity": 0.90,
                "goals": [[-2.68, -5.19], [-2.33, -5.82], [2.61, 5.01], [-6.49, 4.60]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.98,
                "init_y": 2.54,
                "init_a": -116.10,
                "velocity": 1.10,
                "goal_x": 5.68,
                "goal_y": -0.28,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.31,
                "init_y": 1.19,
                "init_a": 104.55,
                "velocity": 0.97,
                "goals": [[3.73, 2.84], [2.65, 3.08], [-0.67, 4.73], [3.44, -2.33]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.62,
                "init_y": -2.70,
                "init_a": 167.61,
                "velocity": 1.20,
                "goal_x": -0.72,
                "goal_y": 3.98,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.20,
                "init_y": 2.57,
                "init_a": -114.47,
                "velocity": 0.97,
                "goal_x": -4.25,
                "goal_y": -3.98,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_18_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.31,
                "init_y": -0.60,
                "init_a": -84.34,
                "velocity": 1.01,
                "goal_x": 0.31,
                "goal_y": -0.60,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.95,
                "init_y": 0.17,
                "init_a": -84.34,
                "velocity": 1.01,
                "goals": [[0.31, -0.60], [-5.49, -2.65], [7.69, 4.48], [0.38, -2.23]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.35,
                "init_y": 1.81,
                "init_a": -69.12,
                "velocity": 0.81,
                "goals": [[3.35, 1.81], [-6.36, 1.69], [-4.64, -4.13], [-6.04, 3.87]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.36,
                "init_y": 1.96,
                "init_a": -69.12,
                "velocity": 0.81,
                "goal_x": 3.35,
                "goal_y": 1.81,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.30,
                "init_y": 5.99,
                "init_a": 23.99,
                "velocity": 0.92,
                "goal_x": -3.81,
                "goal_y": -4.17,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.39,
                "init_y": 4.85,
                "init_a": 70.57,
                "velocity": 0.82,
                "goal_x": -5.02,
                "goal_y": 0.17,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.93,
                "init_y": 1.89,
                "init_a": 133.26,
                "velocity": 0.91,
                "goal_x": 2.02,
                "goal_y": 4.88,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_19_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.29,
                "init_y": -3.00,
                "init_a": -0.90,
                "velocity": 1.01,
                "goal_x": 3.29,
                "goal_y": -3.00,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.72,
                "init_y": -2.18,
                "init_a": -0.90,
                "velocity": 1.01,
                "goals": [[3.29, -3.00], [1.09, 4.31], [-1.38, -2.67], [0.76, 5.14]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.04,
                "init_y": -5.18,
                "init_a": 1.05,
                "velocity": 0.83,
                "goal_x": -1.04,
                "goal_y": -5.18,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.94,
                "init_y": -5.60,
                "init_a": 1.05,
                "velocity": 0.83,
                "goal_x": -1.04,
                "goal_y": -5.18,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.39,
                "init_y": 5.43,
                "init_a": 0.93,
                "velocity": 1.18,
                "goals": [[6.62, 1.26], [3.42, 1.36], [0.48, 5.60], [-0.01, 5.77]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.01,
                "init_y": -2.28,
                "init_a": 87.85,
                "velocity": 1.08,
                "goal_x": -1.40,
                "goal_y": 0.87,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.67,
                "init_y": 3.01,
                "init_a": 111.63,
                "velocity": 0.99,
                "goal_x": 6.14,
                "goal_y": -5.92,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_20_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.01,
                "init_y": -0.02,
                "init_a": 23.00,
                "velocity": 0.99,
                "goals": [[-1.01, -0.02], [5.39, -4.74], [2.31, -0.76], [6.44, 2.86]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.91,
                "init_y": -0.46,
                "init_a": 23.00,
                "velocity": 0.99,
                "goal_x": -1.01,
                "goal_y": -0.02,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.23,
                "init_y": -1.65,
                "init_a": 125.73,
                "velocity": 1.08,
                "goals": [[1.23, -1.65], [4.67, -2.52], [-3.23, -4.04], [6.07, 1.84]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.15,
                "init_y": -0.65,
                "init_a": 125.73,
                "velocity": 1.08,
                "goal_x": 1.23,
                "goal_y": -1.65,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.56,
                "init_y": 1.95,
                "init_a": 133.43,
                "velocity": 0.98,
                "goal_x": 5.90,
                "goal_y": -1.13,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.53,
                "init_y": 3.64,
                "init_a": 54.42,
                "velocity": 0.88,
                "goal_x": 4.04,
                "goal_y": 3.23,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.96,
                "init_y": 2.02,
                "init_a": -169.07,
                "velocity": 1.16,
                "goal_x": 0.74,
                "goal_y": 4.24,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.20,
                "init_y": -5.99,
                "init_a": -58.94,
                "velocity": 0.90,
                "goal_x": -4.91,
                "goal_y": 1.40,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_21_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.51,
                "init_y": 0.91,
                "init_a": 74.64,
                "velocity": 0.84,
                "goal_x": 5.12,
                "goal_y": -2.32,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.23,
                "init_y": 1.87,
                "init_a": 74.64,
                "velocity": 0.84,
                "goal_x": 5.12,
                "goal_y": -2.32,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.09,
                "init_y": 1.79,
                "init_a": 142.01,
                "velocity": 1.05,
                "goals": [[0.13, -4.10], [6.79, 1.18], [-2.76, -4.14], [-0.11, -1.78]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.38,
                "init_y": 0.84,
                "init_a": 142.01,
                "velocity": 1.05,
                "goal_x": 0.13,
                "goal_y": -4.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.90,
                "init_y": 5.63,
                "init_a": 173.36,
                "velocity": 0.88,
                "goals": [[-6.73, 4.83], [-7.57, 3.48], [-4.77, -1.41], [-3.18, 1.97]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.67,
                "init_y": 0.70,
                "init_a": -55.53,
                "velocity": 1.02,
                "goal_x": -7.04,
                "goal_y": -1.40,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.33,
                "init_y": 4.47,
                "init_a": 112.27,
                "velocity": 0.83,
                "goal_x": -2.19,
                "goal_y": -4.16,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.32,
                "init_y": -5.29,
                "init_a": 75.70,
                "velocity": 0.98,
                "goal_x": 0.15,
                "goal_y": -3.75,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_22_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.75,
                "init_y": 1.15,
                "init_a": -62.86,
                "velocity": 1.02,
                "goal_x": -0.12,
                "goal_y": -5.67,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.10,
                "init_y": 2.08,
                "init_a": -62.86,
                "velocity": 1.02,
                "goal_x": -0.12,
                "goal_y": -5.67,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.89,
                "init_y": -5.67,
                "init_a": 91.46,
                "velocity": 1.08,
                "goals": [[-1.87, 1.12], [6.09, 3.99], [4.86, 2.10], [-6.83, 1.85]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.35,
                "init_y": -6.56,
                "init_a": 91.46,
                "velocity": 1.08,
                "goal_x": -1.87,
                "goal_y": 1.12,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.17,
                "init_y": 0.70,
                "init_a": -150.22,
                "velocity": 1.11,
                "goal_x": 3.01,
                "goal_y": -0.47,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.78,
                "init_y": -1.27,
                "init_a": -128.07,
                "velocity": 0.88,
                "goal_x": 3.60,
                "goal_y": 1.88,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.28,
                "init_y": 3.41,
                "init_a": -123.21,
                "velocity": 1.03,
                "goals": [[6.26, -3.32], [0.13, -2.60], [-5.47, 0.06], [4.63, 0.34]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_23_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.57,
                "init_y": 1.92,
                "init_a": 102.76,
                "velocity": 0.86,
                "goal_x": 1.57,
                "goal_y": 1.92,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.57,
                "init_y": 2.92,
                "init_a": 102.76,
                "velocity": 0.86,
                "goal_x": 1.57,
                "goal_y": 1.92,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.94,
                "init_y": 4.89,
                "init_a": -9.30,
                "velocity": 0.97,
                "goals": [[-3.94, 4.89], [6.35, -0.67], [3.33, 3.45], [-2.76, 3.38]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.17,
                "init_y": 5.86,
                "init_a": -9.30,
                "velocity": 0.97,
                "goal_x": -3.94,
                "goal_y": 4.89,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.46,
                "init_y": 2.56,
                "init_a": -21.04,
                "velocity": 0.99,
                "goal_x": -0.56,
                "goal_y": -2.55,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.80,
                "init_y": -4.14,
                "init_a": 51.56,
                "velocity": 1.08,
                "goals": [[5.27, -3.37], [-7.89, -1.17], [4.80, 2.43], [-2.18, 3.12]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.98,
                "init_y": 2.33,
                "init_a": -102.76,
                "velocity": 1.12,
                "goal_x": 7.98,
                "goal_y": -2.01,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.50,
                "init_y": -3.12,
                "init_a": -125.37,
                "velocity": 0.94,
                "goal_x": 1.67,
                "goal_y": -5.77,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.64,
                "init_y": -6.00,
                "init_a": 4.40,
                "velocity": 0.99,
                "goal_x": -5.46,
                "goal_y": -3.60,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_24_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.98,
                "init_y": -1.49,
                "init_a": -157.41,
                "velocity": 0.85,
                "goal_x": 3.12,
                "goal_y": 0.02,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.98,
                "init_y": -1.50,
                "init_a": -157.41,
                "velocity": 0.85,
                "goal_x": 3.12,
                "goal_y": 0.02,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.15,
                "init_y": 5.85,
                "init_a": -93.65,
                "velocity": 0.93,
                "goal_x": -1.54,
                "goal_y": -1.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.32,
                "init_y": 4.87,
                "init_a": -93.65,
                "velocity": 0.93,
                "goal_x": -1.54,
                "goal_y": -1.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.01,
                "init_y": 5.22,
                "init_a": 28.78,
                "velocity": 1.05,
                "goals": [[6.34, 4.38], [-6.28, 4.07], [-2.52, -3.35], [7.57, 5.23]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.89,
                "init_y": -0.98,
                "init_a": 47.75,
                "velocity": 1.15,
                "goal_x": -7.70,
                "goal_y": 1.71,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.49,
                "init_y": -1.87,
                "init_a": -14.55,
                "velocity": 0.91,
                "goal_x": -1.22,
                "goal_y": -4.48,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.91,
                "init_y": 3.99,
                "init_a": -60.64,
                "velocity": 0.98,
                "goal_x": 0.34,
                "goal_y": 0.72,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.37,
                "init_y": -0.36,
                "init_a": 64.44,
                "velocity": 1.02,
                "goals": [[7.66, 1.79], [-1.71, -3.25], [-6.76, -5.68], [-3.97, 2.44]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_25_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.98,
                "init_y": 0.61,
                "init_a": 119.93,
                "velocity": 0.87,
                "goal_x": 1.98,
                "goal_y": 0.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.50,
                "init_y": -0.27,
                "init_a": 119.93,
                "velocity": 0.87,
                "goal_x": 1.98,
                "goal_y": 0.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.45,
                "init_y": -2.13,
                "init_a": -100.62,
                "velocity": 0.84,
                "goals": [[-0.45, -2.13], [5.00, -5.08], [-2.19, 3.00], [2.88, -3.92]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.96,
                "init_y": -2.99,
                "init_a": -100.62,
                "velocity": 0.84,
                "goal_x": -0.45,
                "goal_y": -2.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.83,
                "init_y": -3.75,
                "init_a": -121.25,
                "velocity": 1.04,
                "goal_x": -4.95,
                "goal_y": -2.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.26,
                "init_y": -5.25,
                "init_a": 99.04,
                "velocity": 0.84,
                "goal_x": -7.89,
                "goal_y": -4.06,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.23,
                "init_y": -1.59,
                "init_a": 46.51,
                "velocity": 0.89,
                "goal_x": 5.63,
                "goal_y": -3.67,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.42,
                "init_y": 5.15,
                "init_a": -78.10,
                "velocity": 1.13,
                "goals": [[1.13, -2.94], [-6.18, -1.93], [-3.08, 0.67], [-7.27, 5.99]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.39,
                "init_y": -5.56,
                "init_a": 103.82,
                "velocity": 1.15,
                "goal_x": -3.59,
                "goal_y": -5.29,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_26_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.64,
                "init_y": 0.98,
                "init_a": 22.56,
                "velocity": 0.92,
                "goals": [[-0.24, 1.05], [-6.66, 4.44], [0.80, 5.26], [6.16, 0.53]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.22,
                "init_y": 1.88,
                "init_a": 22.56,
                "velocity": 0.92,
                "goal_x": -0.24,
                "goal_y": 1.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.47,
                "init_y": -0.85,
                "init_a": -53.33,
                "velocity": 1.09,
                "goal_x": -0.05,
                "goal_y": 0.07,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.37,
                "init_y": -1.29,
                "init_a": -53.33,
                "velocity": 1.09,
                "goal_x": -0.05,
                "goal_y": 0.07,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.99,
                "init_y": 0.11,
                "init_a": -69.24,
                "velocity": 0.98,
                "goal_x": 6.90,
                "goal_y": -0.14,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.17,
                "init_y": 3.30,
                "init_a": 109.81,
                "velocity": 1.09,
                "goal_x": 4.50,
                "goal_y": -5.67,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.68,
                "init_y": -1.87,
                "init_a": -4.14,
                "velocity": 0.83,
                "goals": [[-5.08, 0.98], [-3.16, 3.49], [-7.47, 4.68], [-7.86, 3.61]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.69,
                "init_y": 0.38,
                "init_a": -109.84,
                "velocity": 1.10,
                "goal_x": 5.59,
                "goal_y": -2.72,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.13,
                "init_y": -5.11,
                "init_a": 57.07,
                "velocity": 0.85,
                "goal_x": -1.79,
                "goal_y": 0.87,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_27_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.65,
                "init_y": 5.95,
                "init_a": -174.00,
                "velocity": 0.93,
                "goals": [[2.09, -0.92], [5.07, -5.47], [5.50, -3.27], [-5.80, -2.82]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.67,
                "init_y": 6.18,
                "init_a": -174.00,
                "velocity": 0.93,
                "goal_x": 2.09,
                "goal_y": -0.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.36,
                "init_y": 5.49,
                "init_a": 64.23,
                "velocity": 1.17,
                "goal_x": -2.03,
                "goal_y": -4.57,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.62,
                "init_y": 4.53,
                "init_a": 64.23,
                "velocity": 1.17,
                "goal_x": -2.03,
                "goal_y": -4.57,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.09,
                "init_y": -4.75,
                "init_a": -166.89,
                "velocity": 0.80,
                "goal_x": -6.11,
                "goal_y": 2.08,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.54,
                "init_y": -2.62,
                "init_a": 121.59,
                "velocity": 0.97,
                "goals": [[-3.08, 0.68], [2.15, -0.45], [-3.59, -2.99], [6.00, 2.29]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.86,
                "init_y": 1.46,
                "init_a": -61.77,
                "velocity": 0.96,
                "goal_x": -4.06,
                "goal_y": 3.71,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.98,
                "init_y": 3.77,
                "init_a": -85.03,
                "velocity": 0.94,
                "goal_x": 3.70,
                "goal_y": 0.17,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_28_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.13,
                "init_y": 2.22,
                "init_a": -131.64,
                "velocity": 1.17,
                "goal_x": 7.97,
                "goal_y": -3.35,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.93,
                "init_y": 2.82,
                "init_a": -131.64,
                "velocity": 1.17,
                "goal_x": 7.97,
                "goal_y": -3.35,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.31,
                "init_y": -3.52,
                "init_a": 0.96,
                "velocity": 0.84,
                "goal_x": 7.13,
                "goal_y": 3.79,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.23,
                "init_y": -2.68,
                "init_a": 0.96,
                "velocity": 0.84,
                "goal_x": 7.13,
                "goal_y": 3.79,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.00,
                "init_y": 1.90,
                "init_a": 6.86,
                "velocity": 0.91,
                "goal_x": 5.91,
                "goal_y": 0.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.61,
                "init_y": 4.57,
                "init_a": -31.19,
                "velocity": 1.02,
                "goals": [[6.86, -1.43], [2.07, -1.00], [-7.32, 4.98], [-4.04, -0.62]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.95,
                "init_y": 1.66,
                "init_a": 52.24,
                "velocity": 1.02,
                "goal_x": -4.69,
                "goal_y": 2.62,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.43,
                "init_y": -0.13,
                "init_a": -37.55,
                "velocity": 0.94,
                "goals": [[6.27, 5.21], [6.98, -4.89], [-3.90, 0.21], [-2.07, 1.38]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_29_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.78,
                "init_y": -4.87,
                "init_a": 107.23,
                "velocity": 0.97,
                "goal_x": -4.89,
                "goal_y": 0.56,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.79,
                "init_y": -3.87,
                "init_a": 107.23,
                "velocity": 0.97,
                "goal_x": -4.89,
                "goal_y": 0.56,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.30,
                "init_y": -0.17,
                "init_a": -176.10,
                "velocity": 0.99,
                "goals": [[2.76, 2.61], [7.54, 5.92], [7.84, 3.23], [-5.56, 4.16]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.45,
                "init_y": 0.35,
                "init_a": -176.10,
                "velocity": 0.99,
                "goals": [[2.76, 2.61], [-7.87, -5.51], [7.32, -5.61], [5.05, 2.41]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.36,
                "init_y": 5.71,
                "init_a": 121.57,
                "velocity": 0.99,
                "goal_x": -4.09,
                "goal_y": 2.16,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.25,
                "init_y": -3.25,
                "init_a": -91.75,
                "velocity": 0.82,
                "goal_x": -1.68,
                "goal_y": -1.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.55,
                "init_y": -5.72,
                "init_a": 39.05,
                "velocity": 0.93,
                "goal_x": -5.65,
                "goal_y": -0.63,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.17,
                "init_y": -1.93,
                "init_a": -136.42,
                "velocity": 0.99,
                "goal_x": -0.85,
                "goal_y": 3.41,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_30_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.49,
                "init_y": -0.96,
                "init_a": -14.21,
                "velocity": 1.10,
                "goal_x": -5.84,
                "goal_y": -5.51,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.15,
                "init_y": -1.90,
                "init_a": -14.21,
                "velocity": 1.10,
                "goal_x": -5.84,
                "goal_y": -5.51,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.92,
                "init_y": 4.71,
                "init_a": -109.88,
                "velocity": 0.83,
                "goal_x": -3.80,
                "goal_y": 0.36,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.81,
                "init_y": 5.17,
                "init_a": -109.88,
                "velocity": 0.83,
                "goals": [[-3.80, 0.36], [7.12, 3.30], [-7.31, -0.19], [-3.83, 3.31]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.03,
                "init_y": -2.63,
                "init_a": -62.33,
                "velocity": 1.10,
                "goal_x": -1.92,
                "goal_y": -2.70,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.03,
                "init_y": -2.47,
                "init_a": 32.18,
                "velocity": 0.98,
                "goal_x": 2.42,
                "goal_y": 4.06,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.93,
                "init_y": -4.45,
                "init_a": 2.21,
                "velocity": 1.07,
                "goals": [[4.33, 1.13], [-1.93, 0.32], [-3.85, 3.03], [4.63, -5.13]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_31_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.04,
                "init_y": -2.91,
                "init_a": 172.22,
                "velocity": 1.19,
                "goals": [[7.04, -2.91], [-0.95, 1.10], [-7.95, -1.37], [-4.28, -1.52]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.24,
                "init_y": -3.89,
                "init_a": 172.22,
                "velocity": 1.19,
                "goal_x": 7.04,
                "goal_y": -2.91,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.59,
                "init_y": -2.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goal_x": 1.59,
                "goal_y": -2.71,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.52,
                "init_y": -1.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goal_x": 1.59,
                "goal_y": -2.71,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.07,
                "init_y": -4.24,
                "init_a": -170.53,
                "velocity": 1.13,
                "goal_x": 3.23,
                "goal_y": 2.89,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.46,
                "init_y": 4.54,
                "init_a": -162.87,
                "velocity": 1.13,
                "goal_x": 7.42,
                "goal_y": -5.70,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.13,
                "init_y": -3.15,
                "init_a": 52.77,
                "velocity": 0.84,
                "goals": [[-6.95, 1.01], [-4.36, -0.26], [-7.56, 2.60], [3.37, 2.43]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_32_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.51,
                "init_y": -1.87,
                "init_a": 76.89,
                "velocity": 1.18,
                "goal_x": -0.98,
                "goal_y": 0.29,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.52,
                "init_y": -1.70,
                "init_a": 76.89,
                "velocity": 1.18,
                "goals": [[-0.98, 0.29], [-3.87, -0.66], [-3.97, 1.76], [-4.04, 5.33]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.91,
                "init_y": -1.13,
                "init_a": 121.66,
                "velocity": 0.99,
                "goal_x": 5.31,
                "goal_y": -0.65,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.13,
                "init_y": -0.50,
                "init_a": 121.66,
                "velocity": 0.99,
                "goals": [[5.31, -0.65], [0.47, -5.99], [7.91, -1.61], [6.75, 1.06]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.20,
                "init_y": 4.35,
                "init_a": 128.58,
                "velocity": 0.83,
                "goal_x": -1.27,
                "goal_y": 5.80,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.48,
                "init_y": -4.41,
                "init_a": 59.41,
                "velocity": 1.12,
                "goal_x": -5.68,
                "goal_y": -4.75,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.12,
                "init_y": -2.68,
                "init_a": 136.70,
                "velocity": 1.08,
                "goal_x": 6.97,
                "goal_y": -5.17,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.88,
                "init_y": -5.17,
                "init_a": -6.13,
                "velocity": 1.03,
                "goal_x": -5.75,
                "goal_y": 0.95,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.23,
                "init_y": 5.04,
                "init_a": -124.46,
                "velocity": 1.08,
                "goal_x": -0.39,
                "goal_y": 4.01,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_33_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.00,
                "init_y": 5.47,
                "init_a": -77.91,
                "velocity": 1.04,
                "goals": [[6.00, 5.47], [5.73, -3.38], [6.09, -4.14], [-5.23, -5.73]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.38,
                "init_y": 6.39,
                "init_a": -77.91,
                "velocity": 1.04,
                "goal_x": 6.00,
                "goal_y": 5.47,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.78,
                "init_y": 4.94,
                "init_a": -171.83,
                "velocity": 0.87,
                "goal_x": 5.78,
                "goal_y": 4.94,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.41,
                "init_y": 4.02,
                "init_a": -171.83,
                "velocity": 0.87,
                "goal_x": 5.78,
                "goal_y": 4.94,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.38,
                "init_y": -3.94,
                "init_a": -33.94,
                "velocity": 1.07,
                "goals": [[6.19, 4.16], [-6.78, -1.91], [2.14, 5.42], [2.83, 1.14]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.94,
                "init_y": 0.43,
                "init_a": 164.76,
                "velocity": 0.89,
                "goal_x": 5.30,
                "goal_y": -5.56,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.46,
                "init_y": 1.94,
                "init_a": -98.62,
                "velocity": 1.08,
                "goal_x": -5.18,
                "goal_y": 5.63,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_34_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.29,
                "init_y": 0.59,
                "init_a": 37.87,
                "velocity": 0.93,
                "goal_x": 7.29,
                "goal_y": 0.59,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 8.22,
                "init_y": 0.24,
                "init_a": 37.87,
                "velocity": 0.93,
                "goals": [[7.29, 0.59], [-2.48, 5.20], [-6.30, 1.52], [-6.60, -1.26]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.07,
                "init_y": 0.15,
                "init_a": 51.97,
                "velocity": 0.85,
                "goal_x": -0.07,
                "goal_y": 0.15,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.62,
                "init_y": 0.87,
                "init_a": 51.97,
                "velocity": 0.85,
                "goal_x": -0.07,
                "goal_y": 0.15,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.16,
                "init_y": 1.66,
                "init_a": 16.28,
                "velocity": 0.92,
                "goals": [[5.80, -1.24], [-4.68, 4.29], [1.39, -1.31], [4.81, -3.56]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.24,
                "init_y": -5.96,
                "init_a": -148.42,
                "velocity": 0.99,
                "goal_x": 4.55,
                "goal_y": -1.00,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.68,
                "init_y": -3.74,
                "init_a": 27.48,
                "velocity": 1.14,
                "goal_x": 6.72,
                "goal_y": -0.69,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.74,
                "init_y": -4.19,
                "init_a": 137.42,
                "velocity": 1.20,
                "goal_x": 0.79,
                "goal_y": -2.01,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_35_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.61,
                "init_y": 0.41,
                "init_a": 131.72,
                "velocity": 1.13,
                "goal_x": 4.61,
                "goal_y": 0.41,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.91,
                "init_y": -0.54,
                "init_a": 131.72,
                "velocity": 1.13,
                "goal_x": 4.61,
                "goal_y": 0.41,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.93,
                "init_y": 3.07,
                "init_a": 83.88,
                "velocity": 0.93,
                "goals": [[-5.93, 3.07], [-0.31, 2.69], [-7.66, -1.25], [-1.33, -2.72]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.86,
                "init_y": 3.44,
                "init_a": 83.88,
                "velocity": 0.93,
                "goal_x": -5.93,
                "goal_y": 3.07,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.96,
                "init_y": -5.06,
                "init_a": 104.22,
                "velocity": 1.04,
                "goal_x": 1.45,
                "goal_y": -4.68,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.86,
                "init_y": 2.78,
                "init_a": 151.85,
                "velocity": 0.90,
                "goal_x": 0.18,
                "goal_y": -5.87,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.53,
                "init_y": 0.65,
                "init_a": -46.43,
                "velocity": 1.12,
                "goals": [[2.38, -3.10], [-6.38, -3.64], [-6.47, 0.47], [-2.95, -2.70]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.06,
                "init_y": -5.40,
                "init_a": -123.89,
                "velocity": 1.13,
                "goal_x": -4.51,
                "goal_y": -1.89,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.92,
                "init_y": 5.72,
                "init_a": -117.69,
                "velocity": 0.80,
                "goal_x": -7.93,
                "goal_y": 4.58,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_36_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.52,
                "init_y": 2.32,
                "init_a": 110.55,
                "velocity": 0.89,
                "goals": [[0.54, -3.91], [4.68, -0.40], [-7.79, -2.05], [-7.37, -0.37]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.54,
                "init_y": 2.51,
                "init_a": 110.55,
                "velocity": 0.89,
                "goal_x": 0.54,
                "goal_y": -3.91,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.07,
                "init_y": -1.82,
                "init_a": 36.02,
                "velocity": 1.18,
                "goals": [[-4.56, 4.52], [-1.97, 0.01], [7.42, -0.46], [0.52, 1.66]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.07,
                "init_y": -1.83,
                "init_a": 36.02,
                "velocity": 1.18,
                "goal_x": -4.56,
                "goal_y": 4.52,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.71,
                "init_y": 4.55,
                "init_a": -131.55,
                "velocity": 0.89,
                "goal_x": -4.73,
                "goal_y": 3.13,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.36,
                "init_y": 1.62,
                "init_a": -100.53,
                "velocity": 1.14,
                "goal_x": 5.23,
                "goal_y": -0.57,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.25,
                "init_y": -5.69,
                "init_a": 29.69,
                "velocity": 0.88,
                "goal_x": 6.38,
                "goal_y": 1.11,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_37_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.02,
                "init_y": 0.90,
                "init_a": 102.07,
                "velocity": 1.19,
                "goal_x": -5.02,
                "goal_y": 0.90,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.46,
                "init_y": 0.00,
                "init_a": 102.07,
                "velocity": 1.19,
                "goals": [[-5.02, 0.90], [-3.71, -4.82], [-3.19, -3.24], [3.78, -0.83]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.28,
                "init_y": 2.94,
                "init_a": 59.29,
                "velocity": 0.97,
                "goal_x": -7.28,
                "goal_y": 2.94,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.35,
                "init_y": 2.58,
                "init_a": 59.29,
                "velocity": 0.97,
                "goal_x": -7.28,
                "goal_y": 2.94,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.76,
                "init_y": -2.95,
                "init_a": -30.08,
                "velocity": 1.03,
                "goal_x": -6.22,
                "goal_y": -1.79,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.77,
                "init_y": -1.62,
                "init_a": -60.49,
                "velocity": 0.85,
                "goals": [[-2.91, 4.58], [-5.18, 2.49], [6.37, 0.92], [-2.32, 5.59]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.52,
                "init_y": 0.25,
                "init_a": 126.27,
                "velocity": 0.88,
                "goal_x": -7.76,
                "goal_y": -1.67,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_38_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.49,
                "init_y": 2.40,
                "init_a": -69.41,
                "velocity": 1.05,
                "goal_x": 4.49,
                "goal_y": 2.40,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.04,
                "init_y": 3.23,
                "init_a": -69.41,
                "velocity": 1.05,
                "goal_x": 4.49,
                "goal_y": 2.40,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.17,
                "init_y": 1.61,
                "init_a": 172.86,
                "velocity": 1.16,
                "goal_x": 5.17,
                "goal_y": 1.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.10,
                "init_y": 1.98,
                "init_a": 172.86,
                "velocity": 1.16,
                "goals": [[5.17, 1.61], [-6.86, 0.37], [0.38, 2.73], [-4.83, -4.83]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.78,
                "init_y": -0.10,
                "init_a": 97.54,
                "velocity": 0.96,
                "goals": [[0.17, 1.64], [1.77, 5.33], [-1.33, 5.35], [2.17, -5.37]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.18,
                "init_y": 2.18,
                "init_a": 59.07,
                "velocity": 0.99,
                "goal_x": 2.19,
                "goal_y": -4.41,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.68,
                "init_y": -2.56,
                "init_a": -100.52,
                "velocity": 0.92,
                "goal_x": 1.65,
                "goal_y": 3.36,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.46,
                "init_y": 2.45,
                "init_a": -39.44,
                "velocity": 0.87,
                "goal_x": -1.23,
                "goal_y": -5.42,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.07,
                "init_y": 0.59,
                "init_a": 47.54,
                "velocity": 0.92,
                "goal_x": 6.65,
                "goal_y": -3.74,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_39_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.38,
                "init_y": -2.33,
                "init_a": 0.15,
                "velocity": 1.07,
                "goals": [[1.38, -2.33], [-7.00, 5.42], [-5.71, 3.06], [5.73, 0.76]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.15,
                "init_y": -1.36,
                "init_a": 0.15,
                "velocity": 1.07,
                "goals": [[1.38, -2.33], [1.98, -5.20], [1.74, -4.41], [-4.85, -5.33]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.80,
                "init_y": -4.06,
                "init_a": -161.85,
                "velocity": 1.12,
                "goal_x": 5.80,
                "goal_y": -4.06,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.82,
                "init_y": -3.83,
                "init_a": -161.85,
                "velocity": 1.12,
                "goal_x": 5.80,
                "goal_y": -4.06,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.39,
                "init_y": -1.09,
                "init_a": 97.81,
                "velocity": 1.15,
                "goal_x": -4.19,
                "goal_y": -4.10,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.84,
                "init_y": 1.30,
                "init_a": 153.54,
                "velocity": 0.90,
                "goal_x": 0.87,
                "goal_y": 1.56,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.97,
                "init_y": 3.81,
                "init_a": 49.81,
                "velocity": 1.20,
                "goal_x": -5.73,
                "goal_y": -1.01,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_40_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.87,
                "init_y": 3.87,
                "init_a": -124.82,
                "velocity": 1.11,
                "goal_x": 6.87,
                "goal_y": 3.87,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.65,
                "init_y": 2.89,
                "init_a": -124.82,
                "velocity": 1.11,
                "goal_x": 6.87,
                "goal_y": 3.87,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.86,
                "init_y": 1.23,
                "init_a": 59.64,
                "velocity": 1.16,
                "goal_x": -6.86,
                "goal_y": 1.23,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.40,
                "init_y": 2.12,
                "init_a": 59.64,
                "velocity": 1.16,
                "goals": [[-6.86, 1.23], [-0.48, 2.41], [-6.71, 5.62], [-6.97, -1.70]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.32,
                "init_y": 1.57,
                "init_a": -165.67,
                "velocity": 1.03,
                "goal_x": -6.97,
                "goal_y": -5.11,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.78,
                "init_y": 5.11,
                "init_a": -139.55,
                "velocity": 0.82,
                "goals": [[-4.59, 2.13], [-0.60, -2.89], [-7.60, -5.83], [0.38, 3.73]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.61,
                "init_y": -3.01,
                "init_a": 29.82,
                "velocity": 0.89,
                "goal_x": -6.62,
                "goal_y": -4.67,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_41_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.79,
                "init_y": -3.22,
                "init_a": -132.76,
                "velocity": 0.88,
                "goals": [[-7.88, 3.45], [-7.72, 2.88], [-5.54, -2.45], [-4.40, -4.79]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.82,
                "init_y": -3.50,
                "init_a": -132.76,
                "velocity": 0.88,
                "goal_x": -7.88,
                "goal_y": 3.45,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.60,
                "init_y": 0.49,
                "init_a": -8.20,
                "velocity": 0.96,
                "goal_x": -2.82,
                "goal_y": -2.38,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.73,
                "init_y": -0.01,
                "init_a": -8.20,
                "velocity": 0.96,
                "goal_x": -2.82,
                "goal_y": -2.38,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.47,
                "init_y": -0.21,
                "init_a": -76.59,
                "velocity": 1.03,
                "goal_x": 5.38,
                "goal_y": -4.28,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.49,
                "init_y": -5.63,
                "init_a": 28.58,
                "velocity": 1.16,
                "goal_x": 5.00,
                "goal_y": -2.36,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.03,
                "init_y": 3.17,
                "init_a": -26.91,
                "velocity": 0.84,
                "goals": [[5.49, -2.47], [0.96, 2.98], [0.05, -1.89], [-5.09, 0.45]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.91,
                "init_y": 5.17,
                "init_a": -136.96,
                "velocity": 0.92,
                "goal_x": 3.50,
                "goal_y": -1.06,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_42_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.48,
                "init_y": -2.73,
                "init_a": -52.09,
                "velocity": 0.92,
                "goal_x": 2.37,
                "goal_y": 5.22,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.48,
                "init_y": -2.71,
                "init_a": -52.09,
                "velocity": 0.92,
                "goal_x": 2.37,
                "goal_y": 5.22,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.76,
                "init_y": 1.80,
                "init_a": -27.05,
                "velocity": 1.08,
                "goal_x": 7.65,
                "goal_y": 3.40,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.38,
                "init_y": 2.73,
                "init_a": -27.05,
                "velocity": 1.08,
                "goals": [[7.65, 3.40], [-6.65, -5.11], [-3.06, 1.76], [5.53, 2.82]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.24,
                "init_y": 1.73,
                "init_a": 137.96,
                "velocity": 0.97,
                "goals": [[-7.48, -1.91], [0.52, 2.70], [7.12, 5.00], [-4.81, 5.00]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.73,
                "init_y": -5.77,
                "init_a": -153.13,
                "velocity": 1.06,
                "goal_x": -2.54,
                "goal_y": 4.18,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.68,
                "init_y": -5.24,
                "init_a": 11.42,
                "velocity": 1.14,
                "goal_x": -5.03,
                "goal_y": 2.17,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.76,
                "init_y": 0.58,
                "init_a": 88.37,
                "velocity": 1.09,
                "goal_x": 4.50,
                "goal_y": 4.39,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.57,
                "init_y": -0.62,
                "init_a": -134.04,
                "velocity": 1.05,
                "goal_x": -4.23,
                "goal_y": 5.54,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_43_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.02,
                "init_y": 3.19,
                "init_a": -51.83,
                "velocity": 0.92,
                "goals": [[6.02, 3.19], [2.66, -4.98], [-7.21, -3.01], [4.43, 3.85]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.94,
                "init_y": 2.81,
                "init_a": -51.83,
                "velocity": 0.92,
                "goal_x": 6.02,
                "goal_y": 3.19,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.74,
                "init_y": 5.47,
                "init_a": -5.71,
                "velocity": 0.88,
                "goals": [[0.74, 5.47], [4.31, -0.86], [1.86, 4.31], [-5.86, -5.02]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.20,
                "init_y": 6.36,
                "init_a": -5.71,
                "velocity": 0.88,
                "goal_x": 0.74,
                "goal_y": 5.47,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.72,
                "init_y": -5.79,
                "init_a": 143.12,
                "velocity": 0.98,
                "goal_x": -1.83,
                "goal_y": -2.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.48,
                "init_y": -3.08,
                "init_a": -120.31,
                "velocity": 0.82,
                "goal_x": 4.02,
                "goal_y": 1.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.02,
                "init_y": 0.17,
                "init_a": 116.73,
                "velocity": 0.88,
                "goal_x": 4.59,
                "goal_y": 0.20,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.00,
                "init_y": -5.59,
                "init_a": -19.32,
                "velocity": 1.06,
                "goal_x": 7.41,
                "goal_y": -0.38,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.98,
                "init_y": 2.53,
                "init_a": -4.92,
                "velocity": 0.98,
                "goal_x": -7.31,
                "goal_y": -3.40,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_44_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.76,
                "init_y": -1.11,
                "init_a": 62.70,
                "velocity": 1.02,
                "goals": [[-2.76, -1.11], [3.38, 1.88], [-5.20, 3.84], [-1.92, 5.99]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.09,
                "init_y": -0.36,
                "init_a": 62.70,
                "velocity": 1.02,
                "goal_x": -2.76,
                "goal_y": -1.11,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.80,
                "init_y": 3.42,
                "init_a": 104.72,
                "velocity": 0.88,
                "goal_x": 7.80,
                "goal_y": 3.42,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.27,
                "init_y": 4.27,
                "init_a": 104.72,
                "velocity": 0.88,
                "goals": [[7.80, 3.42], [-5.64, 0.96], [-2.28, 0.69], [-4.28, -2.21]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.36,
                "init_y": 0.85,
                "init_a": 88.60,
                "velocity": 1.04,
                "goal_x": -3.59,
                "goal_y": -0.02,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.63,
                "init_y": -1.61,
                "init_a": -32.34,
                "velocity": 1.05,
                "goal_x": 4.79,
                "goal_y": 1.72,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.05,
                "init_y": 5.53,
                "init_a": 127.45,
                "velocity": 1.09,
                "goal_x": 5.39,
                "goal_y": 1.06,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.33,
                "init_y": 1.11,
                "init_a": -119.66,
                "velocity": 0.82,
                "goal_x": -7.51,
                "goal_y": 0.54,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_45_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.31,
                "init_y": 1.60,
                "init_a": -133.51,
                "velocity": 1.17,
                "goals": [[6.75, 0.40], [-5.30, -3.03], [1.24, 2.57], [-1.09, -3.25]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.58,
                "init_y": 0.63,
                "init_a": -133.51,
                "velocity": 1.17,
                "goal_x": 6.75,
                "goal_y": 0.40,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.91,
                "init_y": 2.01,
                "init_a": -61.08,
                "velocity": 1.00,
                "goal_x": 1.43,
                "goal_y": 1.81,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.32,
                "init_y": 1.10,
                "init_a": -61.08,
                "velocity": 1.00,
                "goal_x": 1.43,
                "goal_y": 1.81,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.93,
                "init_y": 0.15,
                "init_a": 83.14,
                "velocity": 1.01,
                "goals": [[-1.29, 2.63], [-6.63, -5.71], [5.28, -2.74], [5.35, -0.52]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.92,
                "init_y": -2.49,
                "init_a": -178.31,
                "velocity": 1.08,
                "goal_x": 6.35,
                "goal_y": -5.18,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.83,
                "init_y": -1.06,
                "init_a": -59.37,
                "velocity": 1.18,
                "goal_x": 7.97,
                "goal_y": 2.06,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.96,
                "init_y": -0.63,
                "init_a": 13.29,
                "velocity": 1.00,
                "goal_x": -2.68,
                "goal_y": 5.16,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_46_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.98,
                "init_y": -3.63,
                "init_a": 99.95,
                "velocity": 1.00,
                "goal_x": -1.98,
                "goal_y": -3.63,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.76,
                "init_y": -3.01,
                "init_a": 99.95,
                "velocity": 1.00,
                "goals": [[-1.98, -3.63], [5.90, -3.15], [-6.81, 1.95], [-2.98, -3.16]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.24,
                "init_y": -3.16,
                "init_a": 38.68,
                "velocity": 0.97,
                "goal_x": -7.24,
                "goal_y": -3.16,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.64,
                "init_y": -4.07,
                "init_a": 38.68,
                "velocity": 0.97,
                "goal_x": -7.24,
                "goal_y": -3.16,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.95,
                "init_y": 1.95,
                "init_a": 19.32,
                "velocity": 0.95,
                "goals": [[-0.87, -3.60], [-0.54, 5.43], [-0.82, -3.40], [1.75, 1.30]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.33,
                "init_y": -2.42,
                "init_a": -176.73,
                "velocity": 0.83,
                "goal_x": -1.41,
                "goal_y": 5.90,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.17,
                "init_y": -4.68,
                "init_a": -89.79,
                "velocity": 1.14,
                "goal_x": 1.17,
                "goal_y": 3.21,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_47_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.67,
                "init_y": 0.17,
                "init_a": 144.19,
                "velocity": 0.96,
                "goals": [[-0.67, 0.17], [-7.95, -4.61], [5.56, -2.84], [0.64, -3.01]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.62,
                "init_y": 0.48,
                "init_a": 144.19,
                "velocity": 0.96,
                "goal_x": -0.67,
                "goal_y": 0.17,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.45,
                "init_y": 5.92,
                "init_a": -61.07,
                "velocity": 1.16,
                "goal_x": -7.45,
                "goal_y": 5.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.77,
                "init_y": 5.18,
                "init_a": -61.07,
                "velocity": 1.16,
                "goal_x": -7.45,
                "goal_y": 5.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.42,
                "init_y": 3.37,
                "init_a": 38.20,
                "velocity": 1.03,
                "goal_x": 7.93,
                "goal_y": 5.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.78,
                "init_y": 4.46,
                "init_a": -88.04,
                "velocity": 1.05,
                "goals": [[-4.74, 0.90], [-3.53, -4.12], [-3.30, 0.68], [-5.67, -3.20]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.87,
                "init_y": 2.12,
                "init_a": 61.55,
                "velocity": 1.10,
                "goal_x": -0.35,
                "goal_y": -0.15,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.43,
                "init_y": 0.06,
                "init_a": 77.21,
                "velocity": 1.20,
                "goal_x": -0.80,
                "goal_y": -5.75,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_48_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.63,
                "init_y": 2.80,
                "init_a": -173.12,
                "velocity": 0.92,
                "goals": [[-5.16, -0.76], [3.50, -3.71], [-2.14, 4.52], [2.54, 2.39]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.66,
                "init_y": 1.80,
                "init_a": -173.12,
                "velocity": 0.92,
                "goal_x": -5.16,
                "goal_y": -0.76,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.65,
                "init_y": 3.64,
                "init_a": -7.87,
                "velocity": 1.15,
                "goals": [[4.57, 3.77], [0.79, 3.21], [0.04, 3.15], [-7.92, -0.87]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.82,
                "init_y": 3.09,
                "init_a": -7.87,
                "velocity": 1.15,
                "goal_x": 4.57,
                "goal_y": 3.77,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.24,
                "init_y": 1.32,
                "init_a": -165.93,
                "velocity": 0.95,
                "goal_x": -6.77,
                "goal_y": -1.10,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.20,
                "init_y": -0.75,
                "init_a": 166.29,
                "velocity": 1.15,
                "goal_x": -1.18,
                "goal_y": -2.13,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.21,
                "init_y": 0.14,
                "init_a": -27.55,
                "velocity": 1.20,
                "goal_x": 3.86,
                "goal_y": -5.84,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_49_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.73,
                "init_y": 1.50,
                "init_a": -43.41,
                "velocity": 1.10,
                "goal_x": 5.73,
                "goal_y": 1.50,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.51,
                "init_y": 2.13,
                "init_a": -43.41,
                "velocity": 1.10,
                "goals": [[5.73, 1.50], [2.33, -2.91], [-3.57, 5.86], [6.62, 4.54]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.23,
                "init_y": -5.14,
                "init_a": -57.70,
                "velocity": 0.87,
                "goal_x": 2.23,
                "goal_y": -5.14,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.44,
                "init_y": -4.16,
                "init_a": -57.70,
                "velocity": 0.87,
                "goals": [[2.23, -5.14], [2.53, -2.01], [4.54, 1.61], [1.16, 2.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.53,
                "init_y": 0.82,
                "init_a": -11.78,
                "velocity": 0.83,
                "goal_x": -1.01,
                "goal_y": -1.26,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.04,
                "init_y": -3.67,
                "init_a": -121.43,
                "velocity": 0.91,
                "goal_x": 6.62,
                "goal_y": 3.52,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.09,
                "init_y": 5.30,
                "init_a": 0.27,
                "velocity": 1.08,
                "goal_x": -5.69,
                "goal_y": 2.84,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_50_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.01,
                "init_y": -5.23,
                "init_a": -86.53,
                "velocity": 0.93,
                "goal_x": 5.01,
                "goal_y": -5.23,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.60,
                "init_y": -4.42,
                "init_a": -86.53,
                "velocity": 0.93,
                "goals": [[5.01, -5.23], [5.83, 5.06], [4.66, -4.39], [-6.36, 1.98]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.32,
                "init_y": 1.84,
                "init_a": 137.64,
                "velocity": 1.01,
                "goals": [[1.32, 1.84], [-1.85, 1.88], [-2.41, -1.91], [3.89, 1.90]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.39,
                "init_y": 2.22,
                "init_a": 137.64,
                "velocity": 1.01,
                "goal_x": 1.32,
                "goal_y": 1.84,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.83,
                "init_y": -4.83,
                "init_a": -0.31,
                "velocity": 1.05,
                "goal_x": 7.51,
                "goal_y": 1.54,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.20,
                "init_y": -5.91,
                "init_a": 50.73,
                "velocity": 1.00,
                "goal_x": -6.69,
                "goal_y": 1.33,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.43,
                "init_y": 5.77,
                "init_a": 165.08,
                "velocity": 1.14,
                "goal_x": -5.72,
                "goal_y": 3.98,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.20,
                "init_y": 0.75,
                "init_a": 128.89,
                "velocity": 1.06,
                "goal_x": -4.81,
                "goal_y": 1.98,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_51_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.08,
                "init_y": -2.95,
                "init_a": -32.01,
                "velocity": 0.85,
                "goal_x": -3.53,
                "goal_y": 3.45,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.30,
                "init_y": -2.34,
                "init_a": -32.01,
                "velocity": 0.85,
                "goals": [[-3.53, 3.45], [-0.48, -1.09], [-7.03, -1.10], [2.67, -5.43]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.61,
                "init_y": 0.06,
                "init_a": 61.58,
                "velocity": 1.12,
                "goal_x": 6.17,
                "goal_y": 2.16,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.67,
                "init_y": 1.06,
                "init_a": 61.58,
                "velocity": 1.12,
                "goal_x": 6.17,
                "goal_y": 2.16,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.83,
                "init_y": -5.77,
                "init_a": -84.96,
                "velocity": 1.12,
                "goals": [[-4.41, -4.56], [-7.55, -5.33], [-5.83, 4.00], [3.15, 0.68]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.35,
                "init_y": 1.25,
                "init_a": -72.79,
                "velocity": 0.80,
                "goal_x": -1.57,
                "goal_y": -3.14,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.33,
                "init_y": 4.33,
                "init_a": -174.49,
                "velocity": 1.19,
                "goal_x": -0.97,
                "goal_y": -2.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.12,
                "init_y": -0.91,
                "init_a": -39.51,
                "velocity": 0.88,
                "goal_x": -3.83,
                "goal_y": -3.31,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_52_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.11,
                "init_y": 1.59,
                "init_a": 54.55,
                "velocity": 1.03,
                "goal_x": -0.82,
                "goal_y": -4.15,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.93,
                "init_y": 2.16,
                "init_a": 54.55,
                "velocity": 1.03,
                "goal_x": -0.82,
                "goal_y": -4.15,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.50,
                "init_y": -3.94,
                "init_a": 21.73,
                "velocity": 1.06,
                "goals": [[1.47, 1.12], [0.76, -5.09], [-4.10, -3.53], [0.53, 4.43]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.49,
                "init_y": -4.09,
                "init_a": 21.73,
                "velocity": 1.06,
                "goal_x": 1.47,
                "goal_y": 1.12,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.23,
                "init_y": -1.20,
                "init_a": -163.11,
                "velocity": 1.12,
                "goals": [[0.64, -4.35], [1.08, 4.38], [-4.09, 0.50], [6.37, -0.37]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.99,
                "init_y": -1.32,
                "init_a": 12.62,
                "velocity": 0.87,
                "goal_x": 1.66,
                "goal_y": -3.46,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.92,
                "init_y": 1.14,
                "init_a": -162.21,
                "velocity": 1.06,
                "goal_x": 0.34,
                "goal_y": -4.14,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.88,
                "init_y": 2.64,
                "init_a": 89.13,
                "velocity": 1.10,
                "goal_x": -7.25,
                "goal_y": -4.53,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_53_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.70,
                "init_y": 4.83,
                "init_a": -28.91,
                "velocity": 0.93,
                "goals": [[-1.70, 4.83], [-0.86, -4.43], [-0.07, -2.11], [-7.63, -5.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.65,
                "init_y": 4.54,
                "init_a": -28.91,
                "velocity": 0.93,
                "goals": [[-1.70, 4.83], [6.67, -2.13], [-5.72, 4.45], [0.68, 1.95]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.75,
                "init_y": 0.98,
                "init_a": 22.73,
                "velocity": 0.92,
                "goal_x": 0.75,
                "goal_y": 0.98,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.62,
                "init_y": 1.47,
                "init_a": 22.73,
                "velocity": 0.92,
                "goal_x": 0.75,
                "goal_y": 0.98,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.37,
                "init_y": 4.04,
                "init_a": -131.76,
                "velocity": 1.05,
                "goal_x": -1.15,
                "goal_y": -5.12,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.62,
                "init_y": -0.05,
                "init_a": 16.07,
                "velocity": 1.13,
                "goal_x": -4.48,
                "goal_y": 4.09,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.81,
                "init_y": -2.64,
                "init_a": 94.23,
                "velocity": 1.02,
                "goal_x": 4.14,
                "goal_y": -2.26,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_54_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.63,
                "init_y": 0.82,
                "init_a": 101.38,
                "velocity": 0.98,
                "goal_x": 3.98,
                "goal_y": 2.75,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.64,
                "init_y": 0.94,
                "init_a": 101.38,
                "velocity": 0.98,
                "goals": [[3.98, 2.75], [4.89, -5.79], [-3.17, 4.97], [3.06, 1.57]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.96,
                "init_y": -2.39,
                "init_a": 12.74,
                "velocity": 1.04,
                "goal_x": 7.44,
                "goal_y": -4.34,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.97,
                "init_y": -2.46,
                "init_a": 12.74,
                "velocity": 1.04,
                "goal_x": 7.44,
                "goal_y": -4.34,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.07,
                "init_y": 3.46,
                "init_a": -56.42,
                "velocity": 0.95,
                "goal_x": -2.62,
                "goal_y": 4.71,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.06,
                "init_y": 5.78,
                "init_a": -14.62,
                "velocity": 0.84,
                "goal_x": -3.58,
                "goal_y": 3.91,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.07,
                "init_y": -1.98,
                "init_a": -73.07,
                "velocity": 0.99,
                "goals": [[1.21, -0.99], [3.12, -1.19], [7.76, 1.69], [5.74, -1.31]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.87,
                "init_y": -5.52,
                "init_a": 66.98,
                "velocity": 0.80,
                "goal_x": -2.75,
                "goal_y": -1.68,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.42,
                "init_y": 5.56,
                "init_a": -132.23,
                "velocity": 1.14,
                "goal_x": 4.25,
                "goal_y": -1.15,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_55_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.66,
                "init_y": -0.11,
                "init_a": -99.47,
                "velocity": 0.88,
                "goal_x": -3.66,
                "goal_y": -0.11,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.75,
                "init_y": 0.30,
                "init_a": -99.47,
                "velocity": 0.88,
                "goal_x": -3.66,
                "goal_y": -0.11,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.68,
                "init_y": -0.34,
                "init_a": -106.77,
                "velocity": 1.10,
                "goal_x": 3.68,
                "goal_y": -0.34,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.44,
                "init_y": 0.31,
                "init_a": -106.77,
                "velocity": 1.10,
                "goal_x": 3.68,
                "goal_y": -0.34,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.64,
                "init_y": 1.76,
                "init_a": 19.16,
                "velocity": 1.02,
                "goal_x": 7.19,
                "goal_y": -0.39,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.00,
                "init_y": -5.84,
                "init_a": 172.14,
                "velocity": 1.03,
                "goals": [[-2.93, -0.04], [-2.45, -1.03], [5.35, -4.46], [3.64, -4.27]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.85,
                "init_y": 4.65,
                "init_a": -41.29,
                "velocity": 1.15,
                "goal_x": 1.51,
                "goal_y": -1.57,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.93,
                "init_y": 5.75,
                "init_a": -147.49,
                "velocity": 1.13,
                "goals": [[-7.77, -3.47], [-4.53, 5.29], [-2.68, -0.95], [3.98, 3.03]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_56_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.82,
                "init_y": -4.08,
                "init_a": 128.85,
                "velocity": 1.01,
                "goal_x": -0.82,
                "goal_y": -4.08,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.17,
                "init_y": -4.23,
                "init_a": 128.85,
                "velocity": 1.01,
                "goal_x": -0.82,
                "goal_y": -4.08,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.34,
                "init_y": -1.27,
                "init_a": -88.45,
                "velocity": 0.90,
                "goal_x": 6.34,
                "goal_y": -1.27,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.62,
                "init_y": -0.31,
                "init_a": -88.45,
                "velocity": 0.90,
                "goal_x": 6.34,
                "goal_y": -1.27,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.78,
                "init_y": -1.42,
                "init_a": 134.78,
                "velocity": 1.04,
                "goals": [[-4.30, -3.27], [7.30, 5.06], [4.78, 3.46], [-7.62, -4.56]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.96,
                "init_y": 1.96,
                "init_a": -15.06,
                "velocity": 1.13,
                "goal_x": 1.87,
                "goal_y": 0.87,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.27,
                "init_y": -2.52,
                "init_a": 34.89,
                "velocity": 1.00,
                "goals": [[-0.33, 0.68], [-2.46, -5.27], [-6.45, 5.46], [-6.04, -3.40]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_57_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.13,
                "init_y": 2.94,
                "init_a": -121.11,
                "velocity": 0.97,
                "goal_x": 5.13,
                "goal_y": 2.94,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.47,
                "init_y": 2.00,
                "init_a": -121.11,
                "velocity": 0.97,
                "goals": [[5.13, 2.94], [4.55, 3.53], [5.70, 1.84], [1.40, 5.79]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.51,
                "init_y": -1.09,
                "init_a": 156.54,
                "velocity": 1.15,
                "goal_x": -1.51,
                "goal_y": -1.09,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.78,
                "init_y": -0.41,
                "init_a": 156.54,
                "velocity": 1.15,
                "goal_x": -1.51,
                "goal_y": -1.09,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.62,
                "init_y": 2.11,
                "init_a": 48.72,
                "velocity": 0.94,
                "goal_x": 0.75,
                "goal_y": -3.59,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.39,
                "init_y": -2.24,
                "init_a": 106.29,
                "velocity": 0.97,
                "goal_x": -1.12,
                "goal_y": 0.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.44,
                "init_y": -0.88,
                "init_a": -107.32,
                "velocity": 0.97,
                "goal_x": -3.48,
                "goal_y": 5.36,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.65,
                "init_y": -0.54,
                "init_a": -62.86,
                "velocity": 0.91,
                "goals": [[4.32, -0.49], [-6.14, 2.12], [4.41, -1.51], [3.34, 1.15]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.79,
                "init_y": -0.57,
                "init_a": -173.83,
                "velocity": 0.89,
                "goal_x": 6.57,
                "goal_y": -1.98,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_58_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.54,
                "init_y": 0.13,
                "init_a": -130.42,
                "velocity": 1.06,
                "goal_x": 7.54,
                "goal_y": 0.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.05,
                "init_y": 1.01,
                "init_a": -130.42,
                "velocity": 1.06,
                "goals": [[7.54, 0.13], [-0.09, -5.31], [-5.83, 3.89], [5.71, -5.84]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.86,
                "init_y": 0.29,
                "init_a": -173.85,
                "velocity": 1.18,
                "goal_x": -4.86,
                "goal_y": 0.29,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.04,
                "init_y": -0.28,
                "init_a": -173.85,
                "velocity": 1.18,
                "goals": [[-4.86, 0.29], [-1.63, 5.82], [4.73, 2.05], [-1.20, 5.48]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.81,
                "init_y": -5.35,
                "init_a": -95.29,
                "velocity": 0.91,
                "goal_x": -6.92,
                "goal_y": -1.36,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.64,
                "init_y": 3.14,
                "init_a": -152.49,
                "velocity": 0.81,
                "goal_x": -3.31,
                "goal_y": -0.79,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.38,
                "init_y": -4.34,
                "init_a": 45.85,
                "velocity": 0.92,
                "goal_x": 0.39,
                "goal_y": -0.54,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.19,
                "init_y": 5.40,
                "init_a": -23.82,
                "velocity": 1.19,
                "goal_x": -3.05,
                "goal_y": -5.56,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.50,
                "init_y": 0.29,
                "init_a": -156.39,
                "velocity": 1.11,
                "goal_x": 3.94,
                "goal_y": 2.05,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_59_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.57,
                "init_y": 1.57,
                "init_a": -32.70,
                "velocity": 1.10,
                "goal_x": -1.20,
                "goal_y": 1.27,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.28,
                "init_y": 1.05,
                "init_a": -32.70,
                "velocity": 1.10,
                "goals": [[-1.20, 1.27], [6.94, -2.52], [1.67, -1.33], [7.02, 1.74]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.73,
                "init_y": -0.15,
                "init_a": -96.60,
                "velocity": 0.92,
                "goal_x": 4.86,
                "goal_y": -0.21,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.87,
                "init_y": 0.37,
                "init_a": -96.60,
                "velocity": 0.92,
                "goal_x": 4.86,
                "goal_y": -0.21,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.89,
                "init_y": -2.66,
                "init_a": -125.40,
                "velocity": 1.14,
                "goals": [[-1.86, -2.58], [7.69, 2.82], [-3.79, 0.97], [0.98, 2.91]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.71,
                "init_y": 3.33,
                "init_a": 157.25,
                "velocity": 0.82,
                "goal_x": -6.72,
                "goal_y": 5.69,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.92,
                "init_y": 2.55,
                "init_a": 173.98,
                "velocity": 1.09,
                "goal_x": 0.63,
                "goal_y": 1.86,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.27,
                "init_y": 5.45,
                "init_a": 79.35,
                "velocity": 0.81,
                "goal_x": 2.70,
                "goal_y": 5.85,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_60_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.35,
                "init_y": 3.57,
                "init_a": -20.12,
                "velocity": 1.11,
                "goals": [[-3.35, 3.57], [-5.71, 5.76], [1.76, 1.31], [-3.22, 3.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.87,
                "init_y": 4.45,
                "init_a": -20.12,
                "velocity": 1.11,
                "goals": [[-3.35, 3.57], [6.27, -5.65], [-6.89, 4.01], [5.84, -1.87]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.28,
                "init_y": -1.34,
                "init_a": 37.99,
                "velocity": 0.81,
                "goal_x": 0.28,
                "goal_y": -1.34,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.16,
                "init_y": -0.35,
                "init_a": 37.99,
                "velocity": 0.81,
                "goal_x": 0.28,
                "goal_y": -1.34,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.98,
                "init_y": -5.17,
                "init_a": 34.84,
                "velocity": 1.18,
                "goal_x": 7.51,
                "goal_y": 3.84,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.02,
                "init_y": 2.79,
                "init_a": 140.78,
                "velocity": 0.84,
                "goal_x": -0.27,
                "goal_y": 5.15,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.91,
                "init_y": -2.06,
                "init_a": 52.26,
                "velocity": 1.19,
                "goal_x": 7.85,
                "goal_y": -1.87,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.36,
                "init_y": -2.75,
                "init_a": -143.58,
                "velocity": 1.18,
                "goal_x": 0.64,
                "goal_y": -5.88,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.37,
                "init_y": 5.20,
                "init_a": -29.40,
                "velocity": 1.19,
                "goal_x": -7.03,
                "goal_y": -4.57,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_61_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.99,
                "init_y": 2.09,
                "init_a": -141.43,
                "velocity": 0.95,
                "goals": [[0.99, 2.09], [3.04, 0.74], [-7.64, 4.00], [-5.78, -4.50]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.99,
                "init_y": 2.04,
                "init_a": -141.43,
                "velocity": 0.95,
                "goal_x": 0.99,
                "goal_y": 2.09,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.07,
                "init_y": -3.92,
                "init_a": -110.00,
                "velocity": 0.90,
                "goals": [[1.07, -3.92], [-2.81, -0.48], [0.06, -3.07], [0.74, -5.93]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.90,
                "init_y": -4.48,
                "init_a": -110.00,
                "velocity": 0.90,
                "goal_x": 1.07,
                "goal_y": -3.92,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.25,
                "init_y": 4.61,
                "init_a": -60.70,
                "velocity": 1.10,
                "goal_x": 5.56,
                "goal_y": 3.38,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.56,
                "init_y": -5.40,
                "init_a": -170.98,
                "velocity": 1.11,
                "goal_x": 4.71,
                "goal_y": -2.50,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.50,
                "init_y": 2.54,
                "init_a": -89.36,
                "velocity": 1.13,
                "goal_x": -6.74,
                "goal_y": -0.84,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_62_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.56,
                "init_y": -4.27,
                "init_a": 112.02,
                "velocity": 0.86,
                "goal_x": -7.59,
                "goal_y": 2.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.52,
                "init_y": -3.27,
                "init_a": 112.02,
                "velocity": 0.86,
                "goal_x": -7.59,
                "goal_y": 2.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.93,
                "init_y": 5.96,
                "init_a": -119.34,
                "velocity": 0.83,
                "goal_x": 2.85,
                "goal_y": -3.95,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.35,
                "init_y": 6.77,
                "init_a": -119.34,
                "velocity": 0.83,
                "goal_x": 2.85,
                "goal_y": -3.95,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.44,
                "init_y": -1.00,
                "init_a": 6.99,
                "velocity": 1.03,
                "goal_x": 6.05,
                "goal_y": -3.90,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.77,
                "init_y": 0.36,
                "init_a": 170.80,
                "velocity": 1.18,
                "goal_x": 6.05,
                "goal_y": -1.38,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.22,
                "init_y": 0.24,
                "init_a": -122.34,
                "velocity": 1.01,
                "goals": [[-2.34, -3.16], [-5.21, -2.33], [1.41, 3.18], [-0.05, 2.46]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.63,
                "init_y": 3.32,
                "init_a": -145.86,
                "velocity": 1.14,
                "goals": [[7.91, -1.49], [4.36, 4.96], [3.64, 3.20], [-6.80, 5.17]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.36,
                "init_y": 0.84,
                "init_a": 131.06,
                "velocity": 1.08,
                "goal_x": 5.61,
                "goal_y": 4.37,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_63_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.85,
                "init_y": 3.62,
                "init_a": -148.32,
                "velocity": 1.08,
                "goals": [[5.86, -3.97], [-0.60, 1.13], [6.02, -0.13], [0.17, -0.47]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.25,
                "init_y": 2.82,
                "init_a": -148.32,
                "velocity": 1.08,
                "goal_x": 5.86,
                "goal_y": -3.97,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.97,
                "init_y": -5.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goal_x": 2.18,
                "goal_y": -1.61,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.03,
                "init_y": -6.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goal_x": 2.18,
                "goal_y": -1.61,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.53,
                "init_y": 3.99,
                "init_a": -28.58,
                "velocity": 1.19,
                "goal_x": 4.54,
                "goal_y": 4.23,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.39,
                "init_y": 2.71,
                "init_a": -149.94,
                "velocity": 1.03,
                "goal_x": 5.92,
                "goal_y": -2.47,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.36,
                "init_y": 5.09,
                "init_a": -122.65,
                "velocity": 0.95,
                "goals": [[7.50, 1.97], [4.42, 5.46], [5.40, 2.59], [-1.29, -2.25]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_64_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.95,
                "init_y": 2.92,
                "init_a": -152.55,
                "velocity": 0.94,
                "goals": [[0.95, 2.92], [-2.92, 0.28], [-2.47, 5.59], [-2.96, -3.37]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.26,
                "init_y": 3.64,
                "init_a": -152.55,
                "velocity": 0.94,
                "goal_x": 0.95,
                "goal_y": 2.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.94,
                "init_y": 0.06,
                "init_a": -62.76,
                "velocity": 0.94,
                "goal_x": 5.94,
                "goal_y": 0.06,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.91,
                "init_y": 0.29,
                "init_a": -62.76,
                "velocity": 0.94,
                "goal_x": 5.94,
                "goal_y": 0.06,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.08,
                "init_y": -2.07,
                "init_a": 161.11,
                "velocity": 0.88,
                "goal_x": 4.02,
                "goal_y": 3.59,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.46,
                "init_y": 4.27,
                "init_a": -21.70,
                "velocity": 1.18,
                "goals": [[2.94, 5.50], [5.38, -2.07], [1.02, 5.08], [0.31, -0.48]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.12,
                "init_y": -5.78,
                "init_a": -142.86,
                "velocity": 1.10,
                "goal_x": -2.92,
                "goal_y": -2.55,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.75,
                "init_y": 0.55,
                "init_a": -109.54,
                "velocity": 1.03,
                "goal_x": -1.26,
                "goal_y": -1.95,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_65_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.61,
                "init_y": -4.95,
                "init_a": -6.29,
                "velocity": 0.99,
                "goal_x": 3.61,
                "goal_y": -4.95,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.64,
                "init_y": -5.16,
                "init_a": -6.29,
                "velocity": 0.99,
                "goal_x": 3.61,
                "goal_y": -4.95,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.85,
                "init_y": -1.73,
                "init_a": -0.49,
                "velocity": 0.82,
                "goal_x": -0.85,
                "goal_y": -1.73,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.32,
                "init_y": -2.61,
                "init_a": -0.49,
                "velocity": 0.82,
                "goals": [[-0.85, -1.73], [2.37, -0.84], [-6.32, 5.07], [4.04, -2.01]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.80,
                "init_y": 2.56,
                "init_a": -26.85,
                "velocity": 1.05,
                "goal_x": -1.73,
                "goal_y": -5.06,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.53,
                "init_y": -4.42,
                "init_a": 131.50,
                "velocity": 1.16,
                "goal_x": -5.68,
                "goal_y": -0.30,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.62,
                "init_y": 2.74,
                "init_a": 44.82,
                "velocity": 1.10,
                "goal_x": 0.76,
                "goal_y": 5.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.62,
                "init_y": -5.83,
                "init_a": -68.59,
                "velocity": 0.87,
                "goals": [[7.60, -5.23], [2.40, -5.22], [4.05, 3.52], [1.60, -5.70]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.89,
                "init_y": -0.85,
                "init_a": 37.13,
                "velocity": 1.05,
                "goal_x": -6.71,
                "goal_y": 3.74,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_66_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.94,
                "init_y": -5.04,
                "init_a": -57.89,
                "velocity": 0.85,
                "goal_x": 6.32,
                "goal_y": 2.26,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.21,
                "init_y": -5.72,
                "init_a": -57.89,
                "velocity": 0.85,
                "goal_x": 6.32,
                "goal_y": 2.26,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.72,
                "init_y": -5.03,
                "init_a": -61.58,
                "velocity": 0.95,
                "goal_x": -3.30,
                "goal_y": -0.14,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.97,
                "init_y": -5.70,
                "init_a": -61.58,
                "velocity": 0.95,
                "goal_x": -3.30,
                "goal_y": -0.14,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.51,
                "init_y": 3.82,
                "init_a": -71.78,
                "velocity": 0.84,
                "goals": [[5.15, 3.97], [-7.90, -3.42], [-3.82, 3.24], [-0.86, 2.23]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.72,
                "init_y": -1.73,
                "init_a": -102.43,
                "velocity": 1.06,
                "goals": [[-5.82, -2.46], [-7.18, 4.81], [-4.47, 4.20], [-3.90, 0.04]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.03,
                "init_y": 4.61,
                "init_a": 77.00,
                "velocity": 1.00,
                "goal_x": -6.64,
                "goal_y": -3.90,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.84,
                "init_y": 5.80,
                "init_a": -158.78,
                "velocity": 1.18,
                "goal_x": -2.54,
                "goal_y": -0.78,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_67_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.58,
                "init_y": -5.58,
                "init_a": 151.44,
                "velocity": 1.15,
                "goal_x": 3.58,
                "goal_y": -5.58,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.40,
                "init_y": -4.60,
                "init_a": 151.44,
                "velocity": 1.15,
                "goal_x": 3.58,
                "goal_y": -5.58,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.06,
                "init_y": -4.08,
                "init_a": -98.20,
                "velocity": 1.20,
                "goal_x": 6.06,
                "goal_y": -4.08,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.67,
                "init_y": -4.87,
                "init_a": -98.20,
                "velocity": 1.20,
                "goals": [[6.06, -4.08], [-7.94, -5.61], [-5.50, 5.81], [5.65, 1.27]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.14,
                "init_y": -0.24,
                "init_a": 16.58,
                "velocity": 1.18,
                "goals": [[-3.04, -2.77], [-2.09, -3.27], [1.03, 4.92], [-3.33, 3.34]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.91,
                "init_y": -0.27,
                "init_a": 165.91,
                "velocity": 1.16,
                "goal_x": -3.49,
                "goal_y": 5.11,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.94,
                "init_y": 2.24,
                "init_a": -9.44,
                "velocity": 0.88,
                "goal_x": -4.85,
                "goal_y": 2.66,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_68_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.42,
                "init_y": 2.94,
                "init_a": 22.54,
                "velocity": 0.99,
                "goals": [[-2.19, -1.06], [5.50, -1.49], [-5.41, -2.38], [-4.52, 0.05]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.33,
                "init_y": 3.60,
                "init_a": 22.54,
                "velocity": 0.99,
                "goal_x": -2.19,
                "goal_y": -1.06,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.62,
                "init_y": -3.74,
                "init_a": 132.75,
                "velocity": 1.04,
                "goal_x": 6.64,
                "goal_y": -5.39,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.49,
                "init_y": -3.25,
                "init_a": 132.75,
                "velocity": 1.04,
                "goals": [[6.64, -5.39], [-5.13, -0.11], [-7.44, -1.00], [-1.84, -1.44]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.64,
                "init_y": 0.83,
                "init_a": 101.87,
                "velocity": 0.85,
                "goal_x": -6.89,
                "goal_y": -2.20,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.34,
                "init_y": -4.02,
                "init_a": 52.76,
                "velocity": 1.18,
                "goal_x": -5.82,
                "goal_y": 2.39,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.16,
                "init_y": -2.92,
                "init_a": -171.59,
                "velocity": 1.06,
                "goal_x": -1.20,
                "goal_y": 0.52,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.15,
                "init_y": -4.80,
                "init_a": 177.34,
                "velocity": 0.96,
                "goal_x": 5.18,
                "goal_y": -0.20,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.40,
                "init_y": 0.32,
                "init_a": 128.88,
                "velocity": 0.83,
                "goal_x": -0.02,
                "goal_y": 0.98,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_69_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.38,
                "init_y": 0.82,
                "init_a": 56.35,
                "velocity": 0.89,
                "goals": [[-6.91, 1.91], [5.85, 3.72], [4.78, -5.27], [-4.33, -4.63]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.55,
                "init_y": 1.38,
                "init_a": 56.35,
                "velocity": 0.89,
                "goal_x": -6.91,
                "goal_y": 1.91,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.66,
                "init_y": 2.94,
                "init_a": 1.31,
                "velocity": 1.16,
                "goal_x": -4.05,
                "goal_y": -1.06,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.75,
                "init_y": 2.53,
                "init_a": 1.31,
                "velocity": 1.16,
                "goals": [[-4.05, -1.06], [-0.08, 1.64], [-4.41, 1.72], [1.74, 5.24]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.46,
                "init_y": 1.28,
                "init_a": 106.46,
                "velocity": 1.12,
                "goal_x": 4.34,
                "goal_y": 5.84,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.29,
                "init_y": -4.98,
                "init_a": -35.01,
                "velocity": 0.96,
                "goal_x": -3.06,
                "goal_y": 3.95,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.75,
                "init_y": -1.45,
                "init_a": -30.41,
                "velocity": 0.96,
                "goal_x": 5.81,
                "goal_y": -1.29,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_70_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.95,
                "init_y": 1.51,
                "init_a": -118.43,
                "velocity": 1.01,
                "goal_x": -1.43,
                "goal_y": 4.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.03,
                "init_y": 1.90,
                "init_a": -118.43,
                "velocity": 1.01,
                "goals": [[-1.43, 4.61], [-7.96, 2.81], [-6.66, -0.35], [4.69, -4.00]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.97,
                "init_y": -4.68,
                "init_a": -118.41,
                "velocity": 1.03,
                "goals": [[6.39, 4.01], [3.89, 1.80], [2.36, -0.09], [2.72, -0.22]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.86,
                "init_y": -5.12,
                "init_a": -118.41,
                "velocity": 1.03,
                "goal_x": 6.39,
                "goal_y": 4.01,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.40,
                "init_y": -3.58,
                "init_a": -6.10,
                "velocity": 1.07,
                "goal_x": -2.32,
                "goal_y": 2.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.67,
                "init_y": -1.32,
                "init_a": -96.86,
                "velocity": 1.13,
                "goal_x": 4.80,
                "goal_y": 1.90,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.54,
                "init_y": 3.23,
                "init_a": -129.37,
                "velocity": 1.11,
                "goal_x": 2.36,
                "goal_y": -4.09,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.13,
                "init_y": 1.65,
                "init_a": 152.00,
                "velocity": 0.86,
                "goal_x": 3.01,
                "goal_y": -1.64,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.45,
                "init_y": -0.87,
                "init_a": -140.80,
                "velocity": 0.83,
                "goal_x": 1.40,
                "goal_y": 1.41,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_71_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.42,
                "init_y": -2.08,
                "init_a": 113.86,
                "velocity": 1.15,
                "goals": [[-6.42, -2.08], [-6.16, -3.04], [6.47, -3.09], [-2.54, 4.85]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.56,
                "init_y": -1.09,
                "init_a": 113.86,
                "velocity": 1.15,
                "goal_x": -6.42,
                "goal_y": -2.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.10,
                "init_y": 3.40,
                "init_a": -32.45,
                "velocity": 0.85,
                "goal_x": 5.10,
                "goal_y": 3.40,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.51,
                "init_y": 4.31,
                "init_a": -32.45,
                "velocity": 0.85,
                "goals": [[5.10, 3.40], [-4.99, 3.70], [-0.71, 5.70], [0.75, 3.22]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.34,
                "init_y": -3.02,
                "init_a": 143.97,
                "velocity": 0.90,
                "goal_x": 5.00,
                "goal_y": 4.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.91,
                "init_y": 3.55,
                "init_a": 106.90,
                "velocity": 0.97,
                "goal_x": -1.46,
                "goal_y": -5.78,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.89,
                "init_y": 2.74,
                "init_a": -141.03,
                "velocity": 0.86,
                "goal_x": 4.39,
                "goal_y": 1.89,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.19,
                "init_y": 3.11,
                "init_a": 10.38,
                "velocity": 1.14,
                "goal_x": -5.77,
                "goal_y": -4.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.61,
                "init_y": -2.80,
                "init_a": 109.36,
                "velocity": 1.04,
                "goal_x": 3.58,
                "goal_y": -3.18,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_72_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.59,
                "init_y": -0.34,
                "init_a": -148.95,
                "velocity": 1.15,
                "goal_x": -7.07,
                "goal_y": -5.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.52,
                "init_y": -0.71,
                "init_a": -148.95,
                "velocity": 1.15,
                "goal_x": -7.07,
                "goal_y": -5.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.19,
                "init_y": -2.77,
                "init_a": 93.35,
                "velocity": 0.82,
                "goal_x": 1.89,
                "goal_y": -3.33,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.79,
                "init_y": -1.85,
                "init_a": 93.35,
                "velocity": 0.82,
                "goal_x": 1.89,
                "goal_y": -3.33,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.73,
                "init_y": 1.94,
                "init_a": 138.40,
                "velocity": 0.87,
                "goals": [[5.54, -2.21], [2.51, -1.63], [2.18, 2.54], [7.00, 3.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.08,
                "init_y": -2.17,
                "init_a": -34.84,
                "velocity": 1.12,
                "goals": [[2.74, 5.39], [1.21, 1.22], [0.05, 4.94], [-0.06, -1.75]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.56,
                "init_y": 3.81,
                "init_a": -153.11,
                "velocity": 0.91,
                "goal_x": -1.63,
                "goal_y": -2.04,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.76,
                "init_y": 2.48,
                "init_a": -108.38,
                "velocity": 0.85,
                "goal_x": 7.02,
                "goal_y": 1.04,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.74,
                "init_y": 4.17,
                "init_a": 160.26,
                "velocity": 1.18,
                "goal_x": -4.82,
                "goal_y": 1.87,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_73_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.40,
                "init_y": -1.35,
                "init_a": -138.70,
                "velocity": 0.97,
                "goal_x": -2.56,
                "goal_y": -1.93,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.52,
                "init_y": -2.34,
                "init_a": -138.70,
                "velocity": 0.97,
                "goal_x": -2.56,
                "goal_y": -1.93,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.23,
                "init_y": -5.25,
                "init_a": 15.15,
                "velocity": 1.16,
                "goals": [[-7.16, 0.16], [1.78, 5.55], [4.33, -0.88], [-4.57, -4.29]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.08,
                "init_y": -4.72,
                "init_a": 15.15,
                "velocity": 1.16,
                "goal_x": -7.16,
                "goal_y": 0.16,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.26,
                "init_y": 5.90,
                "init_a": -52.58,
                "velocity": 1.03,
                "goal_x": 1.08,
                "goal_y": 3.88,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.13,
                "init_y": -1.13,
                "init_a": -16.95,
                "velocity": 0.99,
                "goals": [[-6.72, -4.09], [1.87, -0.85], [-4.31, -4.31], [-5.21, 5.08]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.13,
                "init_y": -2.73,
                "init_a": -125.38,
                "velocity": 1.08,
                "goal_x": 6.91,
                "goal_y": 4.11,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_74_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.02,
                "init_y": 3.42,
                "init_a": 112.30,
                "velocity": 0.90,
                "goal_x": -6.21,
                "goal_y": 2.48,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.08,
                "init_y": 4.42,
                "init_a": 112.30,
                "velocity": 0.90,
                "goals": [[-6.21, 2.48], [0.44, 5.03], [-3.11, -4.04], [1.68, -1.90]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.70,
                "init_y": 5.00,
                "init_a": 59.38,
                "velocity": 0.83,
                "goal_x": 2.09,
                "goal_y": 3.91,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.03,
                "init_y": 4.06,
                "init_a": 59.38,
                "velocity": 0.83,
                "goal_x": 2.09,
                "goal_y": 3.91,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.72,
                "init_y": 3.96,
                "init_a": 60.85,
                "velocity": 1.01,
                "goal_x": 7.20,
                "goal_y": -2.26,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.94,
                "init_y": 4.46,
                "init_a": -80.30,
                "velocity": 0.92,
                "goal_x": 7.66,
                "goal_y": -1.08,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.72,
                "init_y": 1.94,
                "init_a": 73.06,
                "velocity": 0.81,
                "goal_x": 1.90,
                "goal_y": 2.82,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.84,
                "init_y": 2.69,
                "init_a": -165.69,
                "velocity": 0.89,
                "goals": [[4.55, 4.44], [-0.03, 0.44], [-0.61, -5.99], [-7.15, -5.26]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_75_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.56,
                "init_y": -2.16,
                "init_a": 168.66,
                "velocity": 0.84,
                "goal_x": 1.56,
                "goal_y": -2.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.87,
                "init_y": -1.43,
                "init_a": 168.66,
                "velocity": 0.84,
                "goal_x": 1.56,
                "goal_y": -2.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.66,
                "init_y": 1.49,
                "init_a": 20.31,
                "velocity": 1.08,
                "goals": [[-5.66, 1.49], [6.14, 5.31], [3.71, 3.05], [-4.06, -0.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.20,
                "init_y": 0.64,
                "init_a": 20.31,
                "velocity": 1.08,
                "goal_x": -5.66,
                "goal_y": 1.49,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.05,
                "init_y": 2.62,
                "init_a": 63.46,
                "velocity": 0.88,
                "goal_x": -4.47,
                "goal_y": -5.62,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.41,
                "init_y": 3.26,
                "init_a": -171.37,
                "velocity": 1.14,
                "goals": [[-6.55, 0.80], [5.82, 1.48], [0.07, -3.24], [0.43, -4.42]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.50,
                "init_y": -1.97,
                "init_a": -13.49,
                "velocity": 1.07,
                "goal_x": 3.56,
                "goal_y": -3.20,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.24,
                "init_y": 2.69,
                "init_a": 113.86,
                "velocity": 0.84,
                "goal_x": 3.58,
                "goal_y": 4.22,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.29,
                "init_y": -4.80,
                "init_a": -22.85,
                "velocity": 0.91,
                "goal_x": 0.64,
                "goal_y": 1.01,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_76_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.17,
                "init_y": 0.01,
                "init_a": -44.48,
                "velocity": 0.92,
                "goal_x": 3.16,
                "goal_y": -0.18,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.33,
                "init_y": -0.53,
                "init_a": -44.48,
                "velocity": 0.92,
                "goal_x": 3.16,
                "goal_y": -0.18,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.59,
                "init_y": 4.02,
                "init_a": -16.75,
                "velocity": 0.85,
                "goals": [[3.87, -1.18], [-3.22, 0.78], [0.64, -1.49], [-2.12, -2.44]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.45,
                "init_y": 3.03,
                "init_a": -16.75,
                "velocity": 0.85,
                "goals": [[3.87, -1.18], [0.76, 5.07], [0.04, -4.37], [-2.38, -2.01]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.46,
                "init_y": 3.18,
                "init_a": -21.39,
                "velocity": 1.03,
                "goal_x": 0.42,
                "goal_y": 1.82,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.56,
                "init_y": -1.81,
                "init_a": -45.64,
                "velocity": 1.05,
                "goal_x": 1.25,
                "goal_y": -5.61,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.31,
                "init_y": 4.43,
                "init_a": 179.07,
                "velocity": 1.10,
                "goal_x": -6.27,
                "goal_y": -3.68,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_77_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.53,
                "init_y": 3.14,
                "init_a": -1.21,
                "velocity": 0.87,
                "goal_x": 0.53,
                "goal_y": 3.14,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.49,
                "init_y": 4.14,
                "init_a": -1.21,
                "velocity": 0.87,
                "goals": [[0.53, 3.14], [-2.93, 0.61], [-4.96, 5.62], [-1.95, 2.91]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.35,
                "init_y": -0.77,
                "init_a": 26.77,
                "velocity": 0.94,
                "goal_x": 2.35,
                "goal_y": -0.77,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.92,
                "init_y": 0.05,
                "init_a": 26.77,
                "velocity": 0.94,
                "goal_x": 2.35,
                "goal_y": -0.77,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.25,
                "init_y": -3.23,
                "init_a": -165.99,
                "velocity": 0.82,
                "goals": [[3.45, -1.11], [-1.67, -5.34], [6.21, -3.05], [-4.47, -0.00]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.44,
                "init_y": 0.87,
                "init_a": -68.27,
                "velocity": 0.96,
                "goal_x": -1.49,
                "goal_y": 3.88,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.91,
                "init_y": -0.23,
                "init_a": 1.76,
                "velocity": 0.95,
                "goal_x": -3.93,
                "goal_y": -4.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.15,
                "init_y": 0.64,
                "init_a": -66.29,
                "velocity": 1.02,
                "goal_x": 4.76,
                "goal_y": -1.54,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.30,
                "init_y": 5.90,
                "init_a": 133.55,
                "velocity": 1.00,
                "goal_x": 1.78,
                "goal_y": 2.47,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_78_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.22,
                "init_y": -1.46,
                "init_a": 178.09,
                "velocity": 0.94,
                "goal_x": 0.22,
                "goal_y": -1.46,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.99,
                "init_y": -2.10,
                "init_a": 178.09,
                "velocity": 0.94,
                "goal_x": 0.22,
                "goal_y": -1.46,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.86,
                "init_y": -0.75,
                "init_a": -5.96,
                "velocity": 1.13,
                "goal_x": -2.86,
                "goal_y": -0.75,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.84,
                "init_y": -0.57,
                "init_a": -5.96,
                "velocity": 1.13,
                "goals": [[-2.86, -0.75], [0.17, -0.04], [7.94, -0.23], [7.01, 4.49]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.81,
                "init_y": 0.45,
                "init_a": 124.19,
                "velocity": 0.93,
                "goal_x": 2.35,
                "goal_y": 4.29,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.43,
                "init_y": -4.88,
                "init_a": -71.90,
                "velocity": 0.83,
                "goal_x": 5.33,
                "goal_y": -1.40,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.47,
                "init_y": -3.56,
                "init_a": -31.43,
                "velocity": 0.94,
                "goals": [[1.28, 2.97], [4.95, -2.28], [1.55, 4.84], [5.71, -0.95]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_79_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.38,
                "init_y": 0.75,
                "init_a": -22.80,
                "velocity": 1.08,
                "goal_x": -3.51,
                "goal_y": -2.41,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.41,
                "init_y": 0.98,
                "init_a": -22.80,
                "velocity": 1.08,
                "goal_x": -3.51,
                "goal_y": -2.41,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.61,
                "init_y": -4.98,
                "init_a": 87.89,
                "velocity": 0.89,
                "goal_x": -1.48,
                "goal_y": 0.97,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.54,
                "init_y": -4.61,
                "init_a": 87.89,
                "velocity": 0.89,
                "goal_x": -1.48,
                "goal_y": 0.97,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.75,
                "init_y": -0.80,
                "init_a": -46.44,
                "velocity": 1.18,
                "goal_x": -4.68,
                "goal_y": 5.18,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.07,
                "init_y": 0.80,
                "init_a": -131.31,
                "velocity": 0.94,
                "goals": [[4.55, 3.13], [-6.84, 0.73], [3.13, -4.96], [-6.87, 1.43]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.14,
                "init_y": -4.57,
                "init_a": -60.25,
                "velocity": 1.17,
                "goals": [[4.62, 1.01], [3.54, -3.07], [5.35, 1.96], [-2.34, -0.16]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.92,
                "init_y": 3.77,
                "init_a": 74.20,
                "velocity": 1.20,
                "goal_x": -2.72,
                "goal_y": -0.94,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.66,
                "init_y": -3.80,
                "init_a": 143.74,
                "velocity": 0.81,
                "goal_x": 3.93,
                "goal_y": 5.38,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_80_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.16,
                "init_y": 2.70,
                "init_a": 56.49,
                "velocity": 0.97,
                "goal_x": -6.75,
                "goal_y": -1.59,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.40,
                "init_y": 3.67,
                "init_a": 56.49,
                "velocity": 0.97,
                "goal_x": -6.75,
                "goal_y": -1.59,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.52,
                "init_y": -3.54,
                "init_a": 127.59,
                "velocity": 0.84,
                "goal_x": -7.11,
                "goal_y": -2.60,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.98,
                "init_y": -2.69,
                "init_a": 127.59,
                "velocity": 0.84,
                "goal_x": -7.11,
                "goal_y": -2.60,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.76,
                "init_y": -4.28,
                "init_a": 82.26,
                "velocity": 1.01,
                "goals": [[-2.97, -1.98], [6.52, 1.52], [-7.26, -1.23], [-5.41, -0.08]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.39,
                "init_y": -3.57,
                "init_a": -20.36,
                "velocity": 0.98,
                "goals": [[-3.33, 1.95], [4.32, -0.13], [7.78, 4.71], [-5.60, 2.35]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.13,
                "init_y": 1.02,
                "init_a": -72.97,
                "velocity": 1.03,
                "goal_x": -0.12,
                "goal_y": 1.97,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_81_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.01,
                "init_y": 0.38,
                "init_a": 132.14,
                "velocity": 0.82,
                "goals": [[-0.01, 0.38], [7.70, -1.79], [-7.02, -2.08], [-4.61, -1.42]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.30,
                "init_y": 1.33,
                "init_a": 132.14,
                "velocity": 0.82,
                "goal_x": -0.01,
                "goal_y": 0.38,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.90,
                "init_y": -2.01,
                "init_a": 32.91,
                "velocity": 1.03,
                "goal_x": 6.90,
                "goal_y": -2.01,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.39,
                "init_y": -2.87,
                "init_a": 32.91,
                "velocity": 1.03,
                "goal_x": 6.90,
                "goal_y": -2.01,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.94,
                "init_y": -0.32,
                "init_a": 79.63,
                "velocity": 1.15,
                "goals": [[2.01, -4.61], [6.33, -0.60], [0.97, 2.23], [0.32, -1.81]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.23,
                "init_y": -3.67,
                "init_a": -46.76,
                "velocity": 1.10,
                "goal_x": 3.27,
                "goal_y": 5.92,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.00,
                "init_y": -0.40,
                "init_a": -135.14,
                "velocity": 1.03,
                "goal_x": 1.92,
                "goal_y": -4.35,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_82_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.39,
                "init_y": 5.50,
                "init_a": 158.47,
                "velocity": 0.95,
                "goal_x": 3.52,
                "goal_y": -4.79,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.36,
                "init_y": 5.25,
                "init_a": 158.47,
                "velocity": 0.95,
                "goal_x": 3.52,
                "goal_y": -4.79,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.30,
                "init_y": -4.23,
                "init_a": 125.02,
                "velocity": 1.15,
                "goal_x": -6.63,
                "goal_y": -1.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.18,
                "init_y": -3.24,
                "init_a": 125.02,
                "velocity": 1.15,
                "goals": [[-6.63, -1.13], [7.94, 4.46], [-5.09, -5.30], [-0.96, -0.41]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.53,
                "init_y": 4.79,
                "init_a": 106.27,
                "velocity": 1.19,
                "goal_x": -7.03,
                "goal_y": -5.84,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.90,
                "init_y": 5.17,
                "init_a": 71.36,
                "velocity": 1.00,
                "goal_x": 5.11,
                "goal_y": -2.87,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.52,
                "init_y": 0.27,
                "init_a": -18.49,
                "velocity": 0.96,
                "goal_x": 7.43,
                "goal_y": -5.48,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.10,
                "init_y": 5.31,
                "init_a": -42.58,
                "velocity": 1.20,
                "goal_x": -2.11,
                "goal_y": 1.32,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.44,
                "init_y": -3.03,
                "init_a": -44.47,
                "velocity": 1.02,
                "goals": [[7.43, 2.56], [-3.80, 0.30], [-5.83, -1.39], [-3.39, 2.80]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_83_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.90,
                "init_y": 3.69,
                "init_a": 93.93,
                "velocity": 0.99,
                "goal_x": 0.90,
                "goal_y": 3.69,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.05,
                "init_y": 4.21,
                "init_a": 93.93,
                "velocity": 0.99,
                "goal_x": 0.90,
                "goal_y": 3.69,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.38,
                "init_y": 1.35,
                "init_a": 11.15,
                "velocity": 1.18,
                "goal_x": 7.38,
                "goal_y": 1.35,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 8.19,
                "init_y": 1.95,
                "init_a": 11.15,
                "velocity": 1.18,
                "goal_x": 7.38,
                "goal_y": 1.35,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.03,
                "init_y": -2.38,
                "init_a": -82.60,
                "velocity": 1.15,
                "goal_x": -2.84,
                "goal_y": 1.33,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.13,
                "init_y": -0.33,
                "init_a": 96.01,
                "velocity": 1.08,
                "goals": [[3.75, 4.82], [3.19, -5.96], [-2.08, 0.13], [3.77, 3.34]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.92,
                "init_y": -1.49,
                "init_a": 123.12,
                "velocity": 1.06,
                "goal_x": 7.39,
                "goal_y": -0.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.83,
                "init_y": 2.97,
                "init_a": 163.63,
                "velocity": 0.97,
                "goals": [[2.01, -2.50], [-5.20, -5.97], [-0.49, -3.95], [-2.83, 5.61]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_84_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.05,
                "init_y": -3.02,
                "init_a": 23.60,
                "velocity": 0.92,
                "goals": [[-5.76, -4.85], [3.82, -3.48], [0.51, 0.17], [0.16, -3.90]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.91,
                "init_y": -2.52,
                "init_a": 23.60,
                "velocity": 0.92,
                "goals": [[-5.76, -4.85], [-4.10, 1.10], [-3.05, -4.31], [-4.58, 1.54]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.70,
                "init_y": 0.75,
                "init_a": -40.36,
                "velocity": 0.84,
                "goal_x": -4.51,
                "goal_y": -5.16,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -8.04,
                "init_y": -0.19,
                "init_a": -40.36,
                "velocity": 0.84,
                "goal_x": -4.51,
                "goal_y": -5.16,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.87,
                "init_y": 3.19,
                "init_a": -53.55,
                "velocity": 1.04,
                "goal_x": -2.14,
                "goal_y": -0.38,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.43,
                "init_y": -1.02,
                "init_a": -141.47,
                "velocity": 0.87,
                "goal_x": 6.90,
                "goal_y": 1.19,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.49,
                "init_y": 1.54,
                "init_a": -117.03,
                "velocity": 1.07,
                "goal_x": 0.48,
                "goal_y": -0.81,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.38,
                "init_y": 3.85,
                "init_a": -135.10,
                "velocity": 0.88,
                "goal_x": -4.01,
                "goal_y": -3.23,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_85_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.65,
                "init_y": -2.16,
                "init_a": -27.93,
                "velocity": 1.01,
                "goal_x": -3.10,
                "goal_y": -5.21,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.64,
                "init_y": -1.16,
                "init_a": -27.93,
                "velocity": 1.01,
                "goal_x": -3.10,
                "goal_y": -5.21,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.45,
                "init_y": -3.63,
                "init_a": 63.92,
                "velocity": 0.81,
                "goals": [[-2.52, -2.69], [-1.57, 5.43], [3.36, 5.33], [-7.43, -4.49]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.67,
                "init_y": -3.01,
                "init_a": 63.92,
                "velocity": 0.81,
                "goal_x": -2.52,
                "goal_y": -2.69,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.09,
                "init_y": 0.42,
                "init_a": 171.14,
                "velocity": 0.93,
                "goals": [[4.55, -3.90], [0.25, -4.05], [-4.97, -4.01], [3.05, 4.15]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.64,
                "init_y": 5.97,
                "init_a": -69.34,
                "velocity": 0.95,
                "goal_x": -7.63,
                "goal_y": -0.95,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.73,
                "init_y": 2.84,
                "init_a": -155.75,
                "velocity": 0.86,
                "goal_x": -2.30,
                "goal_y": 2.64,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.77,
                "init_y": 2.27,
                "init_a": 168.69,
                "velocity": 1.04,
                "goal_x": -5.39,
                "goal_y": 5.53,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_86_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.08,
                "init_y": -3.00,
                "init_a": -165.80,
                "velocity": 1.03,
                "goal_x": 5.91,
                "goal_y": 3.44,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.12,
                "init_y": -3.29,
                "init_a": -165.80,
                "velocity": 1.03,
                "goal_x": 5.91,
                "goal_y": 3.44,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.69,
                "init_y": -4.53,
                "init_a": -24.70,
                "velocity": 0.82,
                "goals": [[-7.61, 4.56], [4.37, -5.89], [4.72, 2.31], [7.59, -1.95]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.02,
                "init_y": -3.79,
                "init_a": -24.70,
                "velocity": 0.82,
                "goal_x": -7.61,
                "goal_y": 4.56,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.11,
                "init_y": -3.70,
                "init_a": 69.15,
                "velocity": 0.83,
                "goals": [[-4.90, 1.72], [6.97, -4.69], [-0.68, -2.16], [-6.65, -0.51]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.31,
                "init_y": 4.99,
                "init_a": -140.64,
                "velocity": 0.80,
                "goal_x": 2.81,
                "goal_y": -0.67,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.49,
                "init_y": -3.22,
                "init_a": -15.36,
                "velocity": 0.98,
                "goal_x": 5.80,
                "goal_y": -4.98,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_87_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.80,
                "init_y": -5.85,
                "init_a": -131.38,
                "velocity": 0.94,
                "goal_x": -3.80,
                "goal_y": -5.85,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.68,
                "init_y": -5.37,
                "init_a": -131.38,
                "velocity": 0.94,
                "goals": [[-3.80, -5.85], [1.78, -4.26], [-5.07, 3.13], [5.75, 2.77]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.33,
                "init_y": 4.53,
                "init_a": 32.49,
                "velocity": 1.06,
                "goals": [[6.33, 4.53], [3.33, -4.42], [1.52, 5.18], [-4.19, -5.04]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.41,
                "init_y": 4.12,
                "init_a": 32.49,
                "velocity": 1.06,
                "goal_x": 6.33,
                "goal_y": 4.53,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.36,
                "init_y": 4.54,
                "init_a": 64.20,
                "velocity": 1.03,
                "goal_x": -5.60,
                "goal_y": -0.41,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.97,
                "init_y": -1.95,
                "init_a": 19.91,
                "velocity": 1.16,
                "goal_x": -1.07,
                "goal_y": -3.22,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.80,
                "init_y": 2.58,
                "init_a": -38.15,
                "velocity": 1.04,
                "goal_x": 3.74,
                "goal_y": 4.37,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_88_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.67,
                "init_y": -1.74,
                "init_a": -63.27,
                "velocity": 0.82,
                "goal_x": 1.67,
                "goal_y": -1.74,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.30,
                "init_y": -0.97,
                "init_a": -63.27,
                "velocity": 0.82,
                "goal_x": 1.67,
                "goal_y": -1.74,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.91,
                "init_y": -4.60,
                "init_a": 71.80,
                "velocity": 1.07,
                "goals": [[5.91, -4.60], [0.90, -4.46], [-4.51, -0.39], [1.63, -2.91]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.93,
                "init_y": -4.80,
                "init_a": 71.80,
                "velocity": 1.07,
                "goal_x": 5.91,
                "goal_y": -4.60,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.15,
                "init_y": 2.28,
                "init_a": 149.14,
                "velocity": 0.80,
                "goals": [[-3.71, 0.39], [1.23, -4.87], [7.67, 0.19], [0.08, 3.72]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.24,
                "init_y": 5.62,
                "init_a": -36.05,
                "velocity": 1.10,
                "goal_x": -7.02,
                "goal_y": -1.96,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.86,
                "init_y": 2.70,
                "init_a": -51.60,
                "velocity": 0.83,
                "goal_x": -7.69,
                "goal_y": -4.80,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_89_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.11,
                "init_y": 2.39,
                "init_a": 128.30,
                "velocity": 1.00,
                "goals": [[-1.11, 2.39], [-7.14, 2.77], [6.12, -2.47], [-0.32, 4.07]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.06,
                "init_y": 1.39,
                "init_a": 128.30,
                "velocity": 1.00,
                "goal_x": -1.11,
                "goal_y": 2.39,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.99,
                "init_y": -0.89,
                "init_a": -171.32,
                "velocity": 0.93,
                "goal_x": -4.99,
                "goal_y": -0.89,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.99,
                "init_y": -0.92,
                "init_a": -171.32,
                "velocity": 0.93,
                "goal_x": -4.99,
                "goal_y": -0.89,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.10,
                "init_y": 1.38,
                "init_a": -62.99,
                "velocity": 0.83,
                "goals": [[7.40, 5.93], [-4.78, -1.84], [-7.92, 1.96], [-4.67, 1.73]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.01,
                "init_y": -1.61,
                "init_a": -37.60,
                "velocity": 0.87,
                "goal_x": 3.92,
                "goal_y": 0.83,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.04,
                "init_y": 1.09,
                "init_a": 172.38,
                "velocity": 1.10,
                "goal_x": 2.50,
                "goal_y": 4.84,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_90_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.93,
                "init_y": 2.74,
                "init_a": 64.60,
                "velocity": 0.99,
                "goal_x": -0.93,
                "goal_y": 2.74,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.99,
                "init_y": 3.73,
                "init_a": 64.60,
                "velocity": 0.99,
                "goal_x": -0.93,
                "goal_y": 2.74,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.13,
                "init_y": -4.23,
                "init_a": 112.02,
                "velocity": 1.17,
                "goal_x": -3.13,
                "goal_y": -4.23,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.07,
                "init_y": -3.89,
                "init_a": 112.02,
                "velocity": 1.17,
                "goals": [[-3.13, -4.23], [-2.58, 4.22], [7.00, -2.63], [4.51, 5.17]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.05,
                "init_y": 5.39,
                "init_a": -4.71,
                "velocity": 1.16,
                "goal_x": 2.37,
                "goal_y": 2.14,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.51,
                "init_y": 2.65,
                "init_a": 171.66,
                "velocity": 1.19,
                "goals": [[-5.78, -4.44], [1.31, 3.42], [1.91, -2.62], [2.61, -1.08]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.54,
                "init_y": 5.12,
                "init_a": 27.87,
                "velocity": 1.12,
                "goal_x": -0.87,
                "goal_y": -4.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.08,
                "init_y": -1.20,
                "init_a": 108.61,
                "velocity": 0.96,
                "goal_x": 4.47,
                "goal_y": -1.06,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.10,
                "init_y": -1.78,
                "init_a": 109.96,
                "velocity": 0.99,
                "goal_x": -6.33,
                "goal_y": -5.26,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_91_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.69,
                "init_y": 4.52,
                "init_a": 178.13,
                "velocity": 0.82,
                "goal_x": -0.01,
                "goal_y": -0.15,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.72,
                "init_y": 4.76,
                "init_a": 178.13,
                "velocity": 0.82,
                "goal_x": -0.01,
                "goal_y": -0.15,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.61,
                "init_y": 4.42,
                "init_a": 133.09,
                "velocity": 1.18,
                "goal_x": 6.72,
                "goal_y": -5.01,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.56,
                "init_y": 4.74,
                "init_a": 133.09,
                "velocity": 1.18,
                "goal_x": 6.72,
                "goal_y": -5.01,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.31,
                "init_y": 5.46,
                "init_a": 16.99,
                "velocity": 1.16,
                "goal_x": 5.20,
                "goal_y": -5.13,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.41,
                "init_y": 2.20,
                "init_a": -87.37,
                "velocity": 1.00,
                "goals": [[4.01, -5.30], [-2.98, -3.56], [2.44, 4.85], [-4.68, 1.71]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.55,
                "init_y": 3.87,
                "init_a": -31.16,
                "velocity": 0.83,
                "goals": [[-3.96, -5.81], [1.31, -5.54], [-2.58, -0.55], [-0.58, -4.89]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_92_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.38,
                "init_y": 5.48,
                "init_a": 131.30,
                "velocity": 1.07,
                "goals": [[-3.22, -5.19], [-4.32, 2.85], [-0.47, 3.32], [2.57, -2.62]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.40,
                "init_y": 4.48,
                "init_a": 131.30,
                "velocity": 1.07,
                "goal_x": -3.22,
                "goal_y": -5.19,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.70,
                "init_y": -0.09,
                "init_a": -53.62,
                "velocity": 0.85,
                "goal_x": -3.26,
                "goal_y": -1.48,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.76,
                "init_y": 0.91,
                "init_a": -53.62,
                "velocity": 0.85,
                "goals": [[-3.26, -1.48], [-6.45, -1.23], [1.59, 0.73], [-5.52, 5.13]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.96,
                "init_y": -3.02,
                "init_a": -117.92,
                "velocity": 1.02,
                "goal_x": 4.20,
                "goal_y": -2.79,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.32,
                "init_y": -4.57,
                "init_a": 112.16,
                "velocity": 0.91,
                "goal_x": -2.21,
                "goal_y": 0.64,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.76,
                "init_y": 3.67,
                "init_a": 6.84,
                "velocity": 0.89,
                "goal_x": 0.04,
                "goal_y": -4.50,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.32,
                "init_y": -4.07,
                "init_a": 174.76,
                "velocity": 1.08,
                "goal_x": -2.29,
                "goal_y": 4.25,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_93_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.19,
                "init_y": 4.52,
                "init_a": 94.25,
                "velocity": 1.07,
                "goals": [[7.19, 4.52], [-0.71, 4.57], [-2.86, -4.24], [-0.87, -5.18]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.42,
                "init_y": 5.17,
                "init_a": 94.25,
                "velocity": 1.07,
                "goal_x": 7.19,
                "goal_y": 4.52,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.63,
                "init_y": -0.21,
                "init_a": -58.74,
                "velocity": 1.12,
                "goal_x": 3.63,
                "goal_y": -0.21,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.65,
                "init_y": -0.41,
                "init_a": -58.74,
                "velocity": 1.12,
                "goal_x": 3.63,
                "goal_y": -0.21,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.12,
                "init_y": 3.41,
                "init_a": -123.42,
                "velocity": 1.03,
                "goals": [[6.11, 2.16], [3.52, -0.63], [-3.02, 5.32], [-4.78, -2.09]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.17,
                "init_y": 0.19,
                "init_a": -115.45,
                "velocity": 1.12,
                "goal_x": 1.08,
                "goal_y": 5.96,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.25,
                "init_y": -5.08,
                "init_a": 45.43,
                "velocity": 0.88,
                "goal_x": 4.40,
                "goal_y": 5.35,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.59,
                "init_y": 0.53,
                "init_a": 16.78,
                "velocity": 1.02,
                "goal_x": 6.53,
                "goal_y": 0.29,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.22,
                "init_y": 0.97,
                "init_a": -165.06,
                "velocity": 1.01,
                "goal_x": 3.05,
                "goal_y": -1.26,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_94_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.89,
                "init_y": 5.31,
                "init_a": -3.78,
                "velocity": 0.94,
                "goals": [[2.89, 5.31], [5.25, -2.27], [-6.89, 1.85], [-7.02, 5.96]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.51,
                "init_y": 6.23,
                "init_a": -3.78,
                "velocity": 0.94,
                "goal_x": 2.89,
                "goal_y": 5.31,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.03,
                "init_y": -4.62,
                "init_a": -91.41,
                "velocity": 1.02,
                "goal_x": -1.03,
                "goal_y": -4.62,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.34,
                "init_y": -5.57,
                "init_a": -91.41,
                "velocity": 1.02,
                "goals": [[-1.03, -4.62], [-7.90, -0.27], [7.59, 3.71], [5.81, -2.42]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.90,
                "init_y": 5.02,
                "init_a": 75.35,
                "velocity": 1.19,
                "goal_x": -2.50,
                "goal_y": -0.70,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.65,
                "init_y": -1.35,
                "init_a": -47.70,
                "velocity": 0.97,
                "goal_x": -0.98,
                "goal_y": 0.12,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.03,
                "init_y": 0.65,
                "init_a": 128.01,
                "velocity": 0.83,
                "goal_x": 3.19,
                "goal_y": 3.93,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.12,
                "init_y": 3.61,
                "init_a": 116.14,
                "velocity": 1.14,
                "goal_x": -3.49,
                "goal_y": 2.80,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_95_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.58,
                "init_y": -1.86,
                "init_a": 157.73,
                "velocity": 0.85,
                "goal_x": 7.02,
                "goal_y": -3.76,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.12,
                "init_y": -2.74,
                "init_a": 157.73,
                "velocity": 0.85,
                "goals": [[7.02, -3.76], [-5.56, -3.95], [7.50, -5.74], [0.31, 5.71]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.41,
                "init_y": -3.67,
                "init_a": 137.27,
                "velocity": 1.08,
                "goal_x": -0.39,
                "goal_y": -4.50,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.42,
                "init_y": -3.51,
                "init_a": 137.27,
                "velocity": 1.08,
                "goal_x": -0.39,
                "goal_y": -4.50,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.22,
                "init_y": -0.54,
                "init_a": -83.15,
                "velocity": 1.08,
                "goal_x": 1.65,
                "goal_y": 4.25,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.95,
                "init_y": 0.18,
                "init_a": 89.82,
                "velocity": 1.20,
                "goal_x": 6.00,
                "goal_y": -3.35,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.69,
                "init_y": -3.58,
                "init_a": 55.47,
                "velocity": 0.89,
                "goals": [[-3.91, -5.08], [5.27, -5.28], [-6.37, -0.11], [-3.61, -2.47]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.19,
                "init_y": 4.61,
                "init_a": -169.14,
                "velocity": 1.19,
                "goal_x": 5.96,
                "goal_y": 0.82,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.03,
                "init_y": -0.89,
                "init_a": -137.35,
                "velocity": 1.00,
                "goal_x": 3.57,
                "goal_y": 5.44,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_96_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.04,
                "init_y": -3.74,
                "init_a": 30.13,
                "velocity": 0.89,
                "goal_x": -2.82,
                "goal_y": 3.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.07,
                "init_y": -3.47,
                "init_a": 30.13,
                "velocity": 0.89,
                "goal_x": -2.82,
                "goal_y": 3.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.37,
                "init_y": 1.84,
                "init_a": -21.00,
                "velocity": 0.82,
                "goal_x": 2.84,
                "goal_y": -0.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.42,
                "init_y": 2.15,
                "init_a": -21.00,
                "velocity": 0.82,
                "goals": [[2.84, -0.13], [-6.74, -5.04], [-0.26, 5.39], [-0.64, 2.29]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.67,
                "init_y": 3.32,
                "init_a": 113.88,
                "velocity": 0.89,
                "goal_x": 4.87,
                "goal_y": -2.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.25,
                "init_y": 4.21,
                "init_a": 26.08,
                "velocity": 1.09,
                "goals": [[-2.18, -1.43], [-0.31, 1.50], [-1.77, -3.84], [-4.95, -1.41]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.78,
                "init_y": -2.80,
                "init_a": -176.42,
                "velocity": 1.13,
                "goal_x": -7.31,
                "goal_y": -2.48,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.07,
                "init_y": -0.90,
                "init_a": 106.82,
                "velocity": 0.99,
                "goal_x": 7.27,
                "goal_y": 0.53,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.49,
                "init_y": 0.03,
                "init_a": 13.65,
                "velocity": 0.91,
                "goal_x": 4.14,
                "goal_y": -5.00,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_97_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.96,
                "init_y": -5.71,
                "init_a": 135.41,
                "velocity": 1.14,
                "goals": [[-3.96, -5.71], [3.99, -0.55], [0.18, 0.24], [-1.02, 1.96]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.92,
                "init_y": -5.98,
                "init_a": 135.41,
                "velocity": 1.14,
                "goal_x": -3.96,
                "goal_y": -5.71,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.00,
                "init_y": -3.76,
                "init_a": -8.34,
                "velocity": 0.81,
                "goal_x": -7.00,
                "goal_y": -3.76,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.64,
                "init_y": -4.53,
                "init_a": -8.34,
                "velocity": 0.81,
                "goal_x": -7.00,
                "goal_y": -3.76,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.13,
                "init_y": -3.01,
                "init_a": 60.90,
                "velocity": 0.86,
                "goal_x": -0.68,
                "goal_y": 2.36,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.89,
                "init_y": -3.42,
                "init_a": -142.97,
                "velocity": 0.81,
                "goal_x": 3.83,
                "goal_y": -0.08,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.71,
                "init_y": -1.43,
                "init_a": 100.92,
                "velocity": 1.16,
                "goal_x": -0.39,
                "goal_y": 3.59,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.54,
                "init_y": -4.39,
                "init_a": -84.48,
                "velocity": 1.05,
                "goals": [[-0.54, 0.07], [-6.72, -1.93], [2.26, -3.37], [5.52, 2.90]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_98_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 9
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.67,
                "init_y": 5.90,
                "init_a": 58.44,
                "velocity": 1.05,
                "goals": [[-5.49, -0.88], [-7.87, -0.94], [-5.98, -0.12], [-1.59, -4.26]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.62,
                "init_y": 6.23,
                "init_a": 58.44,
                "velocity": 1.05,
                "goal_x": -5.49,
                "goal_y": -0.88,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.00,
                "init_y": -4.89,
                "init_a": -51.17,
                "velocity": 0.92,
                "goal_x": -6.17,
                "goal_y": -2.00,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.68,
                "init_y": -5.83,
                "init_a": -51.17,
                "velocity": 0.92,
                "goal_x": -6.17,
                "goal_y": -2.00,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.96,
                "init_y": -2.65,
                "init_a": -69.77,
                "velocity": 0.81,
                "goal_x": -0.54,
                "goal_y": -2.42,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.94,
                "init_y": -3.58,
                "init_a": -179.63,
                "velocity": 1.17,
                "goal_x": -1.00,
                "goal_y": 2.93,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.94,
                "init_y": -2.81,
                "init_a": 128.03,
                "velocity": 0.82,
                "goals": [[-0.35, -3.38], [-5.40, -1.89], [-0.28, 4.99], [7.77, -0.04]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.62,
                "init_y": -0.98,
                "init_a": 0.26,
                "velocity": 0.93,
                "goal_x": -1.18,
                "goal_y": -4.67,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.63,
                "init_y": 3.92,
                "init_a": -113.88,
                "velocity": 1.07,
                "goal_x": 4.80,
                "goal_y": 4.32,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_99_stopped_medium(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 7
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.77,
                "init_y": 2.15,
                "init_a": -109.88,
                "velocity": 0.91,
                "goals": [[0.77, 2.15], [-0.10, 2.27], [-1.08, -0.35], [0.61, -5.83]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.52,
                "init_y": 1.48,
                "init_a": -109.88,
                "velocity": 0.91,
                "goal_x": 0.77,
                "goal_y": 2.15,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.24,
                "init_y": -5.77,
                "init_a": 94.01,
                "velocity": 0.99,
                "goal_x": 7.24,
                "goal_y": -5.77,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.65,
                "init_y": -6.58,
                "init_a": 94.01,
                "velocity": 0.99,
                "goal_x": 7.24,
                "goal_y": -5.77,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.35,
                "init_y": -5.99,
                "init_a": -43.20,
                "velocity": 0.94,
                "goal_x": -1.22,
                "goal_y": 2.65,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.32,
                "init_y": -0.58,
                "init_a": -131.71,
                "velocity": 1.19,
                "goal_x": 4.83,
                "goal_y": -4.33,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.35,
                "init_y": 0.48,
                "init_a": -56.40,
                "velocity": 1.13,
                "goals": [[1.13, 1.01], [7.10, 1.49], [7.87, -2.06], [-1.90, -3.85]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 7,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_100_walking_medium(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 8
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.88,
                "init_y": 2.40,
                "init_a": 105.02,
                "velocity": 0.87,
                "goal_x": 3.64,
                "goal_y": 3.27,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.46,
                "init_y": 3.21,
                "init_a": 105.02,
                "velocity": 0.87,
                "goal_x": 3.64,
                "goal_y": 3.27,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.06,
                "init_y": 2.90,
                "init_a": 132.79,
                "velocity": 0.81,
                "goal_x": -5.68,
                "goal_y": 3.83,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.96,
                "init_y": 2.47,
                "init_a": 132.79,
                "velocity": 0.81,
                "goal_x": -5.68,
                "goal_y": 3.83,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.81,
                "init_y": -1.41,
                "init_a": 58.29,
                "velocity": 1.05,
                "goals": [[-6.62, -3.01], [-2.79, 3.02], [5.16, 1.12], [0.89, 5.93]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.60,
                "init_y": 2.27,
                "init_a": -66.11,
                "velocity": 0.93,
                "goals": [[4.74, -4.53], [5.75, -2.47], [3.93, 3.99], [1.07, -0.98]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.12,
                "init_y": 1.83,
                "init_a": -5.97,
                "velocity": 1.19,
                "goal_x": 2.19,
                "goal_y": 3.78,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.08,
                "init_y": 2.40,
                "init_a": 42.33,
                "velocity": 0.98,
                "goal_x": -2.34,
                "goal_y": 4.03,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)
