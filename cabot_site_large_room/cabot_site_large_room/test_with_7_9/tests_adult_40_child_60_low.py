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

def tests_adult_40_child_60_test_case_01_walking_low(tester):
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
                "init_x": 6.39,
                "init_y": -4.27,
                "init_a": -80.12,
                "velocity": 0.85,
                "goal_x": 1.33,
                "goal_y": -4.88,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.39,
                "init_y": -4.30,
                "init_a": -80.12,
                "velocity": 0.85,
                "goal_x": 1.33,
                "goal_y": -4.88,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.16,
                "init_y": -3.10,
                "init_a": 178.69,
                "velocity": 0.82,
                "goals": [[-7.55, 0.68], [-2.69, -4.20], [-2.49, -1.65], [2.36, 0.51]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.85,
                "init_y": -2.37,
                "init_a": 178.69,
                "velocity": 0.82,
                "goals": [[-7.55, 0.68], [-6.87, 2.87], [-5.60, -1.20], [-5.88, -3.47]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.33,
                "init_y": -2.26,
                "init_a": 91.14,
                "velocity": 1.11,
                "goal_x": 2.39,
                "goal_y": -5.04,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.72,
                "init_y": 3.87,
                "init_a": 157.68,
                "velocity": 1.06,
                "goals": [[3.06, 1.47], [-0.28, -0.97], [-0.59, -2.27], [-6.37, 4.16]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.20,
                "init_y": -5.14,
                "init_a": -25.90,
                "velocity": 0.84,
                "goals": [[-6.90, -2.24], [7.90, -4.55], [-5.55, -5.69], [7.75, -5.04]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.94,
                "init_y": 5.53,
                "init_a": 35.17,
                "velocity": 0.83,
                "goals": [[-0.07, 0.32], [3.41, 5.59], [7.42, -4.70], [-1.56, -4.65]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.83,
                "init_y": -3.85,
                "init_a": -164.71,
                "velocity": 0.84,
                "goal_x": 4.23,
                "goal_y": -5.21,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_02_stopped_low(tester):
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
                "init_x": -2.20,
                "init_y": -5.37,
                "init_a": 54.41,
                "velocity": 1.18,
                "goal_x": -2.20,
                "goal_y": -5.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.03,
                "init_y": -5.92,
                "init_a": 54.41,
                "velocity": 1.18,
                "goal_x": -2.20,
                "goal_y": -5.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.10,
                "init_y": 2.95,
                "init_a": 45.97,
                "velocity": 1.05,
                "goal_x": 1.10,
                "goal_y": 2.95,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.08,
                "init_y": 3.95,
                "init_a": 45.97,
                "velocity": 1.05,
                "goals": [[1.10, 2.95], [-2.27, 5.11], [0.70, -2.86], [-2.48, 2.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.57,
                "init_y": 1.70,
                "init_a": -149.25,
                "velocity": 1.17,
                "goals": [[-1.03, 1.93], [3.77, -0.55], [4.06, -2.81], [-2.66, -1.19]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.35,
                "init_y": -1.49,
                "init_a": 152.80,
                "velocity": 0.99,
                "goals": [[-3.93, 2.51], [7.72, -5.18], [-7.06, 0.51], [-4.68, 2.00]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.62,
                "init_y": -3.01,
                "init_a": 43.75,
                "velocity": 0.85,
                "goals": [[4.69, 4.01], [1.97, -1.96], [-5.33, 2.23], [0.02, -4.11]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.39,
                "init_y": -3.38,
                "init_a": 158.02,
                "velocity": 0.91,
                "goals": [[4.82, 1.90], [6.87, 5.07], [-4.49, 3.11], [-5.29, 4.28]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.97,
                "init_y": 2.26,
                "init_a": -171.32,
                "velocity": 0.93,
                "goal_x": 5.16,
                "goal_y": 5.81,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_03_walking_low(tester):
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
                "init_x": -7.94,
                "init_y": 3.00,
                "init_a": 85.34,
                "velocity": 0.98,
                "goal_x": -7.21,
                "goal_y": -5.96,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -8.40,
                "init_y": 3.89,
                "init_a": 85.34,
                "velocity": 0.98,
                "goals": [[-7.21, -5.96], [7.82, 0.54], [-2.97, 1.60], [-3.96, -4.76]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.51,
                "init_y": 4.59,
                "init_a": -24.94,
                "velocity": 1.11,
                "goal_x": -1.61,
                "goal_y": 1.37,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.85,
                "init_y": 5.53,
                "init_a": -24.94,
                "velocity": 1.11,
                "goals": [[-1.61, 1.37], [1.20, 5.28], [7.74, 2.11], [1.85, -3.47]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.30,
                "init_y": -0.73,
                "init_a": -25.18,
                "velocity": 0.95,
                "goal_x": -2.94,
                "goal_y": 1.75,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.31,
                "init_y": -2.03,
                "init_a": 113.87,
                "velocity": 0.90,
                "goals": [[-1.00, 5.09], [-6.45, -0.06], [7.59, -4.64], [1.27, -2.90]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.71,
                "init_y": 5.82,
                "init_a": 28.16,
                "velocity": 0.87,
                "goals": [[-7.09, 2.88], [-0.14, -3.84], [0.55, 3.55], [-2.11, -5.91]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_04_stopped_low(tester):
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
                "goals": [[-1.18, -0.11], [7.87, 1.00], [-6.79, 3.05], [-1.26, -0.05]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.81,
                "init_y": -1.04,
                "init_a": -0.28,
                "velocity": 1.14,
                "goals": [[-1.18, -0.11], [-4.35, -3.93], [-7.56, -4.10], [4.60, -0.60]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.17,
                "init_y": 6.30,
                "init_a": 28.19,
                "velocity": 1.11,
                "goal_x": 4.13,
                "goal_y": 5.30,
                "n_actors": 8,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.78,
                "init_y": -4.93,
                "init_a": 37.63,
                "velocity": 1.18,
                "goals": [[2.10, -1.79], [5.97, -4.00], [4.28, 5.72], [-0.09, -1.83]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.10,
                "init_y": 3.43,
                "init_a": -42.69,
                "velocity": 0.94,
                "goals": [[5.73, 1.04], [-3.32, 2.14], [6.35, 2.37], [3.02, 3.63]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_05_walking_low(tester):
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
                "init_x": 5.89,
                "init_y": -5.54,
                "init_a": 94.19,
                "velocity": 1.09,
                "goal_x": -0.81,
                "goal_y": -2.77,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.97,
                "init_y": -5.94,
                "init_a": 94.19,
                "velocity": 1.09,
                "goals": [[-0.81, -2.77], [-5.05, -3.73], [-5.68, -3.80], [-6.69, -3.84]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.05,
                "init_y": -2.34,
                "init_a": -79.94,
                "velocity": 1.08,
                "goals": [[-2.10, 1.87], [0.09, -1.37], [1.65, -4.45], [-6.57, 1.47]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.32,
                "init_y": -3.03,
                "init_a": -79.94,
                "velocity": 1.08,
                "goal_x": -2.10,
                "goal_y": 1.87,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.24,
                "init_y": -2.50,
                "init_a": -148.03,
                "velocity": 1.16,
                "goals": [[2.12, 4.59], [1.78, -5.39], [7.74, -4.75], [7.71, -5.14]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.19,
                "init_y": -5.29,
                "init_a": -2.85,
                "velocity": 1.07,
                "goal_x": -2.98,
                "goal_y": 5.35,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.05,
                "init_y": -1.04,
                "init_a": 14.46,
                "velocity": 0.95,
                "goals": [[-0.44, 1.54], [-1.31, -1.69], [0.01, 5.90], [6.36, -1.50]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.66,
                "init_y": 4.36,
                "init_a": 118.49,
                "velocity": 1.02,
                "goal_x": 0.77,
                "goal_y": -2.60,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.02,
                "init_y": -5.00,
                "init_a": -71.24,
                "velocity": 1.11,
                "goals": [[-6.75, 5.18], [-6.96, -5.75], [6.44, 4.78], [2.14, 1.37]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_06_stopped_low(tester):
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
                "init_x": -7.86,
                "init_y": 4.64,
                "init_a": 9.91,
                "velocity": 0.98,
                "goal_x": -7.86,
                "goal_y": 4.64,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -8.51,
                "init_y": 3.87,
                "init_a": 9.91,
                "velocity": 0.98,
                "goals": [[-7.86, 4.64], [7.90, 0.15], [-7.55, -2.06], [-7.60, -5.55]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.29,
                "init_y": -1.92,
                "init_a": -101.85,
                "velocity": 1.02,
                "goal_x": 5.29,
                "goal_y": -1.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.57,
                "init_y": -2.61,
                "init_a": -101.85,
                "velocity": 1.02,
                "goal_x": 5.29,
                "goal_y": -1.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.30,
                "init_y": 0.50,
                "init_a": 130.30,
                "velocity": 0.85,
                "goals": [[-7.86, 0.86], [6.23, -1.59], [-0.74, 3.71], [-1.33, -2.61]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.82,
                "init_y": -1.05,
                "init_a": -12.64,
                "velocity": 0.98,
                "goals": [[3.97, 2.26], [-5.03, -4.10], [3.53, -1.81], [-1.25, 3.32]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.47,
                "init_y": -3.98,
                "init_a": -140.24,
                "velocity": 1.15,
                "goal_x": 1.27,
                "goal_y": -5.58,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.59,
                "init_y": 1.66,
                "init_a": -167.53,
                "velocity": 1.16,
                "goals": [[-3.61, 3.59], [-0.82, -4.93], [-3.29, -2.99], [-6.40, 2.39]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_07_stopped_low(tester):
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
                "init_x": 6.96,
                "init_y": 4.07,
                "init_a": 69.71,
                "velocity": 0.85,
                "goals": [[6.96, 4.07], [5.68, -5.62], [-0.96, -3.31], [1.15, 0.25]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.27,
                "init_y": 4.79,
                "init_a": 69.71,
                "velocity": 0.85,
                "goals": [[6.96, 4.07], [-4.44, 2.77], [4.98, -3.52], [1.84, -3.57]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.10,
                "init_y": -3.72,
                "init_a": -3.82,
                "velocity": 1.18,
                "goals": [[-6.10, -3.72], [-0.86, -2.00], [-2.31, -2.99], [0.43, -5.24]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.29,
                "init_y": -2.36,
                "init_a": -63.89,
                "velocity": 1.00,
                "goal_x": 2.70,
                "goal_y": -5.86,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.74,
                "init_y": -5.15,
                "init_a": 69.68,
                "velocity": 0.91,
                "goal_x": -7.54,
                "goal_y": 2.22,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.60,
                "init_y": 5.41,
                "init_a": -166.35,
                "velocity": 0.98,
                "goals": [[4.27, 0.76], [-0.88, 5.65], [-2.80, -5.86], [4.79, 0.91]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_08_stopped_low(tester):
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
                "init_x": -4.77,
                "init_y": 0.91,
                "init_a": 62.71,
                "velocity": 0.97,
                "goals": [[-4.77, 0.91], [7.75, -2.51], [-3.21, -4.11], [6.98, -4.07]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.72,
                "init_y": 1.23,
                "init_a": 62.71,
                "velocity": 0.97,
                "goals": [[-4.77, 0.91], [0.56, 0.10], [7.35, 5.91], [5.17, 4.38]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.52,
                "init_y": 3.30,
                "init_a": -75.07,
                "velocity": 0.82,
                "goal_x": -4.52,
                "goal_y": 3.30,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.53,
                "init_y": 2.30,
                "init_a": -75.07,
                "velocity": 0.82,
                "goals": [[-4.52, 3.30], [-4.24, 1.31], [4.05, 0.18], [6.42, 0.36]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.27,
                "init_y": 1.76,
                "init_a": -23.37,
                "velocity": 0.91,
                "goal_x": -7.91,
                "goal_y": 3.36,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.67,
                "init_y": -4.14,
                "init_a": 57.15,
                "velocity": 0.98,
                "goal_x": 7.70,
                "goal_y": 2.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.22,
                "init_y": -2.80,
                "init_a": 65.60,
                "velocity": 0.86,
                "goals": [[-1.73, -4.73], [-3.87, -1.89], [5.18, -3.23], [-1.02, -5.15]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.97,
                "init_y": 5.97,
                "init_a": -47.11,
                "velocity": 0.91,
                "goals": [[0.95, 1.70], [6.38, -2.83], [2.81, 0.73], [-1.15, 1.04]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.93,
                "init_y": 1.93,
                "init_a": 107.91,
                "velocity": 0.89,
                "goal_x": 7.18,
                "goal_y": -1.03,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_09_walking_low(tester):
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
                "init_x": 2.07,
                "init_y": -1.47,
                "init_a": 84.23,
                "velocity": 1.03,
                "goal_x": 7.80,
                "goal_y": -5.23,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.08,
                "init_y": -1.33,
                "init_a": 84.23,
                "velocity": 1.03,
                "goal_x": 7.80,
                "goal_y": -5.23,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.45,
                "init_y": 5.76,
                "init_a": 81.82,
                "velocity": 1.02,
                "goals": [[-6.60, -5.68], [-5.31, -3.56], [5.78, -4.39], [3.01, 3.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.45,
                "init_y": 5.76,
                "init_a": 81.82,
                "velocity": 1.02,
                "goals": [[-6.60, -5.68], [-4.92, 1.55], [-4.23, 1.65], [5.22, -0.69]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.40,
                "init_y": 1.55,
                "init_a": -34.54,
                "velocity": 0.94,
                "goal_x": -6.97,
                "goal_y": -0.60,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.42,
                "init_y": -1.54,
                "init_a": -53.11,
                "velocity": 1.18,
                "goals": [[-0.68, 0.10], [-3.95, 1.81], [-7.11, -5.75], [-7.17, -3.32]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.74,
                "init_y": 5.33,
                "init_a": -174.77,
                "velocity": 0.99,
                "goals": [[-0.67, 2.02], [-2.89, -2.07], [2.08, 1.29], [-7.69, 3.34]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.01,
                "init_y": -5.85,
                "init_a": -116.96,
                "velocity": 1.15,
                "goal_x": -4.53,
                "goal_y": -2.98,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_10_stopped_low(tester):
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
                "init_x": 5.20,
                "init_y": -4.57,
                "init_a": 29.17,
                "velocity": 1.06,
                "goals": [[5.20, -4.57], [-0.93, 2.33], [4.36, 3.20], [4.37, 1.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.26,
                "init_y": -4.91,
                "init_a": 29.17,
                "velocity": 1.06,
                "goal_x": 5.20,
                "goal_y": -4.57,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.86,
                "init_y": 5.62,
                "init_a": -172.39,
                "velocity": 0.92,
                "goal_x": -5.86,
                "goal_y": 5.62,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.79,
                "init_y": 6.61,
                "init_a": -172.39,
                "velocity": 0.92,
                "goals": [[-5.86, 5.62], [-0.41, 4.35], [-5.01, 3.39], [1.12, -5.15]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.33,
                "init_y": 4.37,
                "init_a": -44.53,
                "velocity": 0.98,
                "goal_x": 4.28,
                "goal_y": -1.59,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.09,
                "init_y": -5.54,
                "init_a": -4.08,
                "velocity": 1.19,
                "goals": [[2.28, 0.07], [-3.64, 1.69], [3.58, -1.80], [6.48, 2.61]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.45,
                "init_y": -1.80,
                "init_a": 42.77,
                "velocity": 1.07,
                "goals": [[-2.14, -4.66], [3.02, 4.73], [3.81, -1.10], [7.59, 3.08]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.05,
                "init_y": -2.21,
                "init_a": -157.14,
                "velocity": 0.82,
                "goal_x": -0.85,
                "goal_y": -2.97,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_11_stopped_low(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.13,
                "init_y": -0.65,
                "init_a": -46.10,
                "velocity": 1.02,
                "goals": [[-0.32, 0.33], [3.80, -0.09], [-5.48, 1.27], [-1.89, 1.37]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.78,
                "init_y": -2.39,
                "init_a": -175.17,
                "velocity": 1.16,
                "goals": [[-7.78, -2.39], [-4.93, -3.86], [-0.46, -1.64], [-6.28, -5.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.68,
                "init_y": -1.40,
                "init_a": -175.17,
                "velocity": 1.16,
                "goals": [[-7.78, -2.39], [-6.01, 3.33], [-6.40, -4.54], [-7.29, 0.97]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-7.65, -1.20], [4.95, -0.35], [-6.96, -0.26], [-5.25, -5.80]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_12_walking_low(tester):
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
                "init_x": -2.52,
                "init_y": -0.28,
                "init_a": 5.14,
                "velocity": 0.85,
                "goal_x": 2.00,
                "goal_y": 5.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.78,
                "init_y": 0.69,
                "init_a": 5.14,
                "velocity": 0.85,
                "goals": [[2.00, 5.30], [-5.75, 2.47], [6.33, 0.56], [3.29, 5.75]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.07,
                "init_y": 4.67,
                "init_a": -122.78,
                "velocity": 0.95,
                "goal_x": 0.82,
                "goal_y": 5.85,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.55,
                "init_y": 3.79,
                "init_a": -122.78,
                "velocity": 0.95,
                "goal_x": 0.82,
                "goal_y": 5.85,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.66,
                "init_y": 5.89,
                "init_a": 127.85,
                "velocity": 1.19,
                "goal_x": 7.95,
                "goal_y": 3.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.74,
                "init_y": 2.92,
                "init_a": 156.64,
                "velocity": 0.91,
                "goals": [[5.04, -0.76], [-2.26, -0.43], [-7.61, 0.87], [1.23, -0.77]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.35,
                "init_y": 5.06,
                "init_a": -95.34,
                "velocity": 0.86,
                "goals": [[4.74, 4.69], [-7.99, 4.45], [-7.42, 0.40], [-1.61, -2.30]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.68,
                "init_y": -4.72,
                "init_a": 162.11,
                "velocity": 0.83,
                "goals": [[4.32, 3.00], [3.48, -2.76], [-5.54, -5.58], [7.73, -4.26]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_13_stopped_low(tester):
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
                "init_x": -5.18,
                "init_y": -3.02,
                "init_a": -128.85,
                "velocity": 1.14,
                "goals": [[-5.18, -3.02], [7.10, -3.90], [7.15, 0.43], [0.22, 4.26]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.97,
                "init_y": -4.00,
                "init_a": -128.85,
                "velocity": 1.14,
                "goal_x": -5.18,
                "goal_y": -3.02,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.06,
                "init_y": 1.40,
                "init_a": 106.78,
                "velocity": 0.88,
                "goals": [[2.06, 1.40], [5.78, 0.92], [2.30, 1.26], [-6.82, 2.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.27,
                "init_y": 2.01,
                "init_a": 106.78,
                "velocity": 0.88,
                "goal_x": 2.06,
                "goal_y": 1.40,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.06,
                "init_y": 5.55,
                "init_a": -17.15,
                "velocity": 1.03,
                "goals": [[-5.29, -3.55], [-5.21, 2.90], [-0.23, -3.81], [-5.06, 3.93]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.92,
                "init_y": 1.84,
                "init_a": -51.62,
                "velocity": 1.18,
                "goals": [[6.06, 2.43], [-2.19, -3.16], [-3.08, 4.12], [-6.45, -1.73]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.44,
                "init_y": 0.03,
                "init_a": -80.05,
                "velocity": 0.87,
                "goal_x": -0.64,
                "goal_y": -2.50,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.01,
                "init_y": 3.68,
                "init_a": 126.76,
                "velocity": 1.20,
                "goals": [[4.85, 3.54], [5.98, 4.50], [7.43, 4.01], [4.85, 3.62]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.44,
                "init_y": 2.74,
                "init_a": -165.47,
                "velocity": 0.97,
                "goal_x": -6.41,
                "goal_y": -2.51,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_14_walking_low(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.65,
                "init_y": -4.35,
                "init_a": -29.65,
                "velocity": 0.86,
                "goals": [[-3.91, -4.23], [-0.86, 0.94], [5.58, 2.93], [3.77, 4.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.55,
                "init_y": 0.34,
                "init_a": 10.47,
                "velocity": 1.17,
                "goal_x": -0.96,
                "goal_y": 1.37,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.00,
                "init_y": -0.49,
                "init_a": 10.47,
                "velocity": 1.17,
                "goals": [[-0.96, 1.37], [-4.87, 5.10], [-5.92, 4.95], [-5.08, 3.45]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.40,
                "init_y": -0.17,
                "init_a": 103.05,
                "velocity": 0.96,
                "goals": [[-1.09, 4.27], [-3.10, 0.27], [7.87, -4.55], [2.06, 4.12]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.82,
                "init_y": -3.44,
                "init_a": -147.27,
                "velocity": 0.91,
                "goals": [[-3.70, 1.42], [2.91, 0.38], [-4.85, 4.96], [6.56, 4.65]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.41,
                "init_y": -0.93,
                "init_a": 25.66,
                "velocity": 0.88,
                "goal_x": 5.62,
                "goal_y": -4.80,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_15_stopped_low(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.15,
                "init_y": 3.21,
                "init_a": 127.26,
                "velocity": 1.07,
                "goals": [[4.61, 2.37], [-0.64, -5.10], [7.35, 1.86], [-3.26, 4.69]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.41,
                "init_y": -1.01,
                "init_a": 149.29,
                "velocity": 0.99,
                "goal_x": 1.41,
                "goal_y": -1.01,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.18,
                "init_y": -1.64,
                "init_a": 149.29,
                "velocity": 0.99,
                "goals": [[1.41, -1.01], [0.37, 0.79], [1.73, -1.75], [0.87, 3.31]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.99,
                "init_y": -5.30,
                "init_a": -52.63,
                "velocity": 0.97,
                "goals": [[-6.40, 4.68], [-6.67, 2.44], [4.18, -4.35], [-0.39, 5.60]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.79,
                "init_y": -0.02,
                "init_a": -127.88,
                "velocity": 1.13,
                "goals": [[-6.98, -0.12], [4.37, 5.51], [-1.11, 1.50], [3.30, -3.70]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.24,
                "init_y": 1.70,
                "init_a": -75.90,
                "velocity": 1.12,
                "goals": [[-0.63, -4.67], [5.29, -3.42], [-4.69, -3.71], [-4.87, -2.56]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.29,
                "init_y": -0.66,
                "init_a": -166.12,
                "velocity": 1.11,
                "goal_x": -0.38,
                "goal_y": 3.76,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_16_stopped_low(tester):
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
                "init_x": -6.55,
                "init_y": -5.05,
                "init_a": -118.17,
                "velocity": 1.03,
                "goal_x": -6.55,
                "goal_y": -5.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.79,
                "init_y": -5.71,
                "init_a": -118.17,
                "velocity": 1.03,
                "goals": [[-6.55, -5.05], [7.36, -3.70], [-2.00, -4.63], [-0.39, 2.99]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.69,
                "init_y": 0.82,
                "init_a": 171.03,
                "velocity": 1.16,
                "goal_x": 1.69,
                "goal_y": 0.82,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.14,
                "init_y": -0.02,
                "init_a": 171.03,
                "velocity": 1.16,
                "goals": [[1.69, 0.82], [4.81, -4.44], [7.92, -3.50], [1.34, 4.79]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.89,
                "init_y": -0.13,
                "init_a": -179.07,
                "velocity": 1.13,
                "goals": [[2.77, 5.66], [6.66, -5.22], [-0.14, -0.48], [3.63, -2.76]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.79,
                "init_y": 2.79,
                "init_a": -162.55,
                "velocity": 1.00,
                "goals": [[2.95, -1.66], [-0.89, -2.37], [-0.15, -5.55], [-7.47, -4.63]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.95,
                "init_y": -5.65,
                "init_a": 13.28,
                "velocity": 0.85,
                "goal_x": -5.43,
                "goal_y": 0.96,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.30,
                "init_y": 0.16,
                "init_a": -27.20,
                "velocity": 1.11,
                "goals": [[-1.50, -3.07], [7.32, 1.01], [6.97, -1.68], [-2.66, -3.53]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.14,
                "init_y": -5.54,
                "init_a": -177.69,
                "velocity": 1.04,
                "goal_x": 0.46,
                "goal_y": -4.98,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_17_walking_low(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.76,
                "init_y": 3.02,
                "init_a": -13.14,
                "velocity": 0.90,
                "goals": [[-2.68, -5.19], [5.03, -5.96], [-1.67, -0.39], [-6.97, -5.71]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.76,
                "init_y": 2.96,
                "init_a": -13.14,
                "velocity": 0.90,
                "goals": [[-2.68, -5.19], [-7.37, -3.96], [1.93, -1.24], [-2.57, 5.99]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.98,
                "init_y": 2.54,
                "init_a": -116.10,
                "velocity": 1.10,
                "goals": [[5.68, -0.28], [-2.75, 5.17], [2.72, 1.57], [-4.54, 4.46]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.31,
                "init_y": 1.19,
                "init_a": 104.55,
                "velocity": 0.97,
                "goals": [[3.73, 2.84], [4.38, -5.12], [-3.93, -3.95], [-0.25, -0.62]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_18_walking_low(tester):
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
                "init_x": -5.92,
                "init_y": -2.81,
                "init_a": -162.69,
                "velocity": 1.07,
                "goals": [[-2.54, -1.40], [4.35, -1.65], [-7.62, -4.34], [2.08, 5.80]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.92,
                "init_y": -2.81,
                "init_a": -162.69,
                "velocity": 1.07,
                "goals": [[-2.54, -1.40], [7.74, 4.51], [-1.16, -3.13], [1.52, 0.50]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.51,
                "init_y": 2.41,
                "init_a": -43.51,
                "velocity": 0.86,
                "goal_x": 4.80,
                "goal_y": -0.27,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.71,
                "init_y": 3.39,
                "init_a": -43.51,
                "velocity": 0.86,
                "goals": [[4.80, -0.27], [-2.49, 5.54], [6.49, 0.21], [4.11, -2.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.26,
                "init_y": -2.69,
                "init_a": 179.74,
                "velocity": 0.89,
                "goal_x": 7.37,
                "goal_y": -5.06,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.52,
                "init_y": 3.68,
                "init_a": 12.69,
                "velocity": 1.13,
                "goal_x": -3.45,
                "goal_y": -3.88,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.92,
                "init_y": 1.69,
                "init_a": 60.68,
                "velocity": 1.05,
                "goals": [[1.02, 1.77], [-1.23, -0.70], [-7.78, 4.87], [2.07, 3.01]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_19_walking_low(tester):
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
                "init_x": 0.31,
                "init_y": 0.91,
                "init_a": 139.11,
                "velocity": 0.99,
                "goal_x": 5.86,
                "goal_y": -5.17,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.58,
                "init_y": 0.45,
                "init_a": 139.11,
                "velocity": 0.99,
                "goals": [[5.86, -5.17], [-2.91, 3.34], [-5.94, -2.49], [-4.36, -2.87]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.55,
                "init_y": 1.76,
                "init_a": 115.59,
                "velocity": 0.98,
                "goals": [[1.77, 2.95], [3.37, 1.30], [-2.12, -5.43], [0.22, 1.92]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.65,
                "init_y": 0.76,
                "init_a": 115.59,
                "velocity": 0.98,
                "goal_x": 1.77,
                "goal_y": 2.95,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.74,
                "init_y": -0.52,
                "init_a": -140.26,
                "velocity": 1.00,
                "goals": [[-2.49, -3.58], [-7.54, -1.41], [-2.38, 2.32], [1.62, -4.49]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.27,
                "init_y": 2.25,
                "init_a": -73.45,
                "velocity": 1.03,
                "goals": [[-1.16, -3.00], [6.56, -1.29], [6.74, 5.63], [-7.04, -3.53]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.20,
                "init_y": 0.12,
                "init_a": -48.05,
                "velocity": 1.10,
                "goal_x": 7.29,
                "goal_y": 4.25,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_20_stopped_low(tester):
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
                "goals": [[-1.01, -0.02], [0.69, -0.97], [2.10, 5.89], [6.26, -5.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.23,
                "init_y": -1.65,
                "init_a": 125.73,
                "velocity": 1.08,
                "goal_x": 1.23,
                "goal_y": -1.65,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.15,
                "init_y": -0.65,
                "init_a": 125.73,
                "velocity": 1.08,
                "goals": [[1.23, -1.65], [-4.41, -4.63], [6.91, 0.89], [6.94, -4.18]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.53,
                "init_y": 3.64,
                "init_a": 54.42,
                "velocity": 0.88,
                "goals": [[4.04, 3.23], [-2.67, -3.13], [3.09, 3.66], [-5.09, -1.69]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.20,
                "init_y": -5.99,
                "init_a": -58.94,
                "velocity": 0.90,
                "goals": [[-4.91, 1.40], [7.97, 5.06], [5.96, -5.55], [0.55, -5.12]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_21_walking_low(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.23,
                "init_y": 1.87,
                "init_a": 74.64,
                "velocity": 0.84,
                "goals": [[5.12, -2.32], [-2.07, -1.53], [-4.52, -4.47], [2.39, -3.65]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.09,
                "init_y": 1.79,
                "init_a": 142.01,
                "velocity": 1.05,
                "goals": [[0.13, -4.10], [0.87, -5.15], [-2.08, -4.29], [-0.89, -5.88]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.90,
                "init_y": 5.63,
                "init_a": 173.36,
                "velocity": 0.88,
                "goal_x": -6.73,
                "goal_y": 4.83,
                "n_actors": 8,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.33,
                "init_y": 4.47,
                "init_a": 112.27,
                "velocity": 0.83,
                "goals": [[-2.19, -4.16], [-4.73, -1.31], [3.63, 0.74], [-6.86, 0.86]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.32,
                "init_y": -5.29,
                "init_a": 75.70,
                "velocity": 0.98,
                "goals": [[0.15, -3.75], [1.07, -5.33], [-6.28, -1.40], [3.78, -1.25]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_22_stopped_low(tester):
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
                "init_x": 7.30,
                "init_y": 4.45,
                "init_a": -56.98,
                "velocity": 1.01,
                "goal_x": 7.30,
                "goal_y": 4.45,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 8.16,
                "init_y": 4.96,
                "init_a": -56.98,
                "velocity": 1.01,
                "goal_x": 7.30,
                "goal_y": 4.45,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.23,
                "init_y": -4.96,
                "init_a": 127.67,
                "velocity": 1.20,
                "goal_x": 5.23,
                "goal_y": -4.96,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.55,
                "init_y": -4.23,
                "init_a": 127.67,
                "velocity": 1.20,
                "goals": [[5.23, -4.96], [0.25, -5.36], [5.17, -4.24], [2.84, -5.46]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.99,
                "init_y": 1.24,
                "init_a": -147.47,
                "velocity": 0.99,
                "goals": [[-3.62, 2.07], [-6.85, 4.21], [4.19, 2.65], [-7.13, 4.45]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.87,
                "init_y": 5.54,
                "init_a": -95.18,
                "velocity": 1.15,
                "goals": [[-5.73, -1.85], [5.22, 1.50], [5.12, 2.40], [5.92, -0.66]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.84,
                "init_y": 4.60,
                "init_a": -69.51,
                "velocity": 1.03,
                "goals": [[-5.16, -2.02], [0.72, 2.75], [3.53, 5.09], [-3.52, -1.30]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_23_walking_low(tester):
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
                "init_x": 6.27,
                "init_y": 4.46,
                "init_a": -165.93,
                "velocity": 0.86,
                "goals": [[-2.58, 4.91], [6.82, -2.61], [-1.20, -4.62], [5.84, -2.47]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.96,
                "init_y": 5.19,
                "init_a": -165.93,
                "velocity": 0.86,
                "goal_x": -2.58,
                "goal_y": 4.91,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.05,
                "init_y": 5.36,
                "init_a": -21.93,
                "velocity": 1.07,
                "goal_x": 4.46,
                "goal_y": 0.65,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.63,
                "init_y": 4.55,
                "init_a": -21.93,
                "velocity": 1.07,
                "goal_x": 4.46,
                "goal_y": 0.65,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.57,
                "init_y": 1.19,
                "init_a": 162.94,
                "velocity": 1.19,
                "goals": [[6.19, -3.25], [-1.70, -2.84], [0.14, -0.03], [-6.07, 1.73]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.09,
                "init_y": 3.93,
                "init_a": 95.03,
                "velocity": 1.03,
                "goals": [[-6.87, -3.96], [-0.03, -1.37], [6.45, -0.88], [-7.63, -2.06]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.07,
                "init_y": 2.94,
                "init_a": -137.50,
                "velocity": 0.92,
                "goals": [[1.23, -5.29], [-7.68, 2.83], [-3.30, 0.06], [0.19, -0.89]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_24_stopped_low(tester):
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
                "init_x": -7.24,
                "init_y": 1.09,
                "init_a": 144.32,
                "velocity": 1.15,
                "goal_x": -7.24,
                "goal_y": 1.09,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.92,
                "init_y": 2.04,
                "init_a": 144.32,
                "velocity": 1.15,
                "goals": [[-7.24, 1.09], [2.54, -1.21], [-3.60, -4.45], [2.60, 3.86]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.44,
                "init_y": -2.45,
                "init_a": 127.26,
                "velocity": 0.87,
                "goals": [[3.44, -2.45], [2.55, 5.65], [-4.08, 5.41], [-0.95, -1.40]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.44,
                "init_y": -2.37,
                "init_a": 127.26,
                "velocity": 0.87,
                "goals": [[3.44, -2.45], [1.12, 2.99], [-2.97, 1.59], [-7.50, -2.16]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.55,
                "init_y": -4.09,
                "init_a": -94.57,
                "velocity": 0.90,
                "goal_x": 1.02,
                "goal_y": 3.12,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.55,
                "init_y": 1.61,
                "init_a": 119.66,
                "velocity": 0.95,
                "goal_x": 1.18,
                "goal_y": 1.43,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.36,
                "init_y": -2.20,
                "init_a": 147.26,
                "velocity": 0.81,
                "goals": [[-1.24, -3.52], [2.36, -4.18], [4.36, 0.98], [-6.00, 5.41]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_25_walking_low(tester):
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
                "init_x": -5.95,
                "init_y": 5.13,
                "init_a": -121.34,
                "velocity": 1.13,
                "goals": [[3.72, -3.68], [3.24, 1.47], [-7.20, 2.54], [-4.45, -0.70]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.40,
                "init_y": 6.02,
                "init_a": -121.34,
                "velocity": 1.13,
                "goal_x": 3.72,
                "goal_y": -3.68,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.90,
                "init_y": -3.10,
                "init_a": 10.08,
                "velocity": 0.98,
                "goal_x": -1.16,
                "goal_y": -4.53,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.90,
                "init_y": -3.09,
                "init_a": 10.08,
                "velocity": 0.98,
                "goals": [[-1.16, -4.53], [-0.99, 4.60], [3.53, -0.12], [-4.23, 4.41]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.79,
                "init_y": -0.92,
                "init_a": -24.72,
                "velocity": 0.98,
                "goal_x": -3.78,
                "goal_y": -5.71,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.32,
                "init_y": -4.84,
                "init_a": -177.82,
                "velocity": 0.88,
                "goal_x": -7.07,
                "goal_y": -4.42,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.66,
                "init_y": 5.58,
                "init_a": -155.53,
                "velocity": 1.04,
                "goals": [[-1.05, 3.11], [-3.61, -2.77], [6.29, 2.02], [5.79, -2.20]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.72,
                "init_y": 4.41,
                "init_a": 109.03,
                "velocity": 0.82,
                "goals": [[3.67, 0.32], [-5.27, 2.38], [2.56, -2.96], [-0.67, 2.19]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_26_walking_low(tester):
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
                "goals": [[-0.24, 1.05], [5.36, -1.64], [-2.95, 1.65], [1.54, 1.79]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.22,
                "init_y": 1.88,
                "init_a": 22.56,
                "velocity": 0.92,
                "goals": [[-0.24, 1.05], [0.06, 3.27], [5.74, -2.27], [-6.74, -3.25]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.99,
                "init_y": 0.11,
                "init_a": -69.24,
                "velocity": 0.98,
                "goals": [[6.90, -0.14], [-5.11, 5.39], [-6.66, 1.43], [1.99, -0.09]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.17,
                "init_y": 3.30,
                "init_a": 109.81,
                "velocity": 1.09,
                "goals": [[4.50, -5.67], [2.59, -4.70], [-2.80, -0.80], [-5.31, -3.51]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.68,
                "init_y": -1.87,
                "init_a": -4.14,
                "velocity": 0.83,
                "goal_x": -5.08,
                "goal_y": 0.98,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.69,
                "init_y": 0.38,
                "init_a": -109.84,
                "velocity": 1.10,
                "goals": [[5.59, -2.72], [3.07, 0.91], [6.52, -3.00], [6.19, -2.01]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_27_stopped_low(tester):
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
                "init_x": 4.96,
                "init_y": -5.82,
                "init_a": -20.70,
                "velocity": 0.90,
                "goal_x": 4.96,
                "goal_y": -5.82,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.68,
                "init_y": -6.78,
                "init_a": -20.70,
                "velocity": 0.90,
                "goal_x": 4.96,
                "goal_y": -5.82,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.25,
                "init_y": -3.99,
                "init_a": -48.94,
                "velocity": 1.15,
                "goals": [[-2.25, -3.99], [-4.42, 2.94], [-7.81, 1.09], [-2.79, -4.11]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.25,
                "init_y": -3.99,
                "init_a": -48.94,
                "velocity": 1.15,
                "goals": [[-2.25, -3.99], [-5.59, -3.78], [6.13, 2.21], [4.86, 2.20]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.83,
                "init_y": 5.30,
                "init_a": 129.63,
                "velocity": 1.06,
                "goals": [[0.21, 0.12], [-7.77, -5.35], [-1.59, -4.15], [2.32, -4.26]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.79,
                "init_y": 2.76,
                "init_a": -47.16,
                "velocity": 1.02,
                "goals": [[-2.24, 0.89], [-0.54, 1.74], [-2.74, 4.09], [-5.08, -0.71]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.84,
                "init_y": -4.83,
                "init_a": -128.63,
                "velocity": 0.99,
                "goals": [[-3.35, 3.10], [-0.83, 2.13], [6.49, 5.76], [-5.43, 5.08]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.36,
                "init_y": -3.13,
                "init_a": 55.77,
                "velocity": 1.02,
                "goal_x": -0.03,
                "goal_y": -4.83,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.49,
                "init_y": 5.94,
                "init_a": 6.01,
                "velocity": 1.20,
                "goal_x": 0.24,
                "goal_y": 0.48,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_28_walking_low(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.31,
                "init_y": -3.52,
                "init_a": 0.96,
                "velocity": 0.84,
                "goals": [[7.13, 3.79], [3.13, -1.95], [-7.18, -4.91], [-3.27, -2.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.00,
                "init_y": 1.90,
                "init_a": 6.86,
                "velocity": 0.91,
                "goals": [[5.91, 0.30], [0.10, -0.53], [7.27, 1.40], [-4.92, -1.25]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.61,
                "init_y": 4.57,
                "init_a": -31.19,
                "velocity": 1.02,
                "goals": [[6.86, -1.43], [-4.98, 1.69], [-7.01, -3.68], [3.42, 3.50]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.95,
                "init_y": 1.66,
                "init_a": 52.24,
                "velocity": 1.02,
                "goals": [[-4.69, 2.62], [1.50, -5.08], [6.11, -4.98], [2.82, 1.38]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.43,
                "init_y": -0.13,
                "init_a": -37.55,
                "velocity": 0.94,
                "goal_x": 6.27,
                "goal_y": 5.21,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_29_stopped_low(tester):
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
                "init_x": 3.24,
                "init_y": 3.00,
                "init_a": 103.89,
                "velocity": 1.20,
                "goal_x": 3.24,
                "goal_y": 3.00,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.23,
                "init_y": 3.12,
                "init_a": 103.89,
                "velocity": 1.20,
                "goals": [[3.24, 3.00], [-4.53, -0.24], [1.82, -5.49], [2.07, -4.06]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.23,
                "init_y": 0.12,
                "init_a": 142.82,
                "velocity": 0.91,
                "goals": [[-7.23, 0.12], [6.19, -2.08], [1.62, -3.14], [5.03, 0.58]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.89,
                "init_y": -0.82,
                "init_a": 142.82,
                "velocity": 0.91,
                "goal_x": -7.23,
                "goal_y": 0.12,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.87,
                "init_y": 3.23,
                "init_a": 2.73,
                "velocity": 1.16,
                "goal_x": -1.94,
                "goal_y": -4.89,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.53,
                "init_y": -1.32,
                "init_a": -158.25,
                "velocity": 1.07,
                "goals": [[3.86, -1.31], [-2.01, 3.68], [-5.35, -5.54], [-0.84, 2.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.19,
                "init_y": -2.66,
                "init_a": 167.75,
                "velocity": 0.84,
                "goals": [[-2.81, -4.86], [-5.10, -1.56], [-2.04, -5.60], [0.90, -3.30]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.91,
                "init_y": -5.30,
                "init_a": -81.55,
                "velocity": 1.06,
                "goal_x": -1.04,
                "goal_y": -1.19,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_30_walking_low(tester):
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
                "init_x": -4.49,
                "init_y": -0.96,
                "init_a": -14.21,
                "velocity": 1.10,
                "goals": [[-5.84, -5.51], [2.23, -3.52], [2.49, 1.43], [-6.97, 2.87]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.15,
                "init_y": -1.90,
                "init_a": -14.21,
                "velocity": 1.10,
                "goals": [[-5.84, -5.51], [-6.04, 2.95], [0.58, -3.10], [-7.63, 2.42]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.81,
                "init_y": 5.17,
                "init_a": -109.88,
                "velocity": 0.83,
                "goal_x": -3.80,
                "goal_y": 0.36,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.03,
                "init_y": -2.63,
                "init_a": -62.33,
                "velocity": 1.10,
                "goals": [[-1.92, -2.70], [4.05, 1.41], [2.70, 1.68], [-0.31, -2.48]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[4.33, 1.13], [0.79, 5.40], [0.13, -3.05], [2.16, 4.36]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_31_stopped_low(tester):
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
                "goals": [[7.04, -2.91], [-0.83, -0.97], [-1.57, -4.21], [-3.40, -1.71]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.24,
                "init_y": -3.89,
                "init_a": 172.22,
                "velocity": 1.19,
                "goals": [[7.04, -2.91], [-7.64, -0.50], [-1.63, -5.72], [-6.63, 1.39]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.52,
                "init_y": -1.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goals": [[1.59, -2.71], [-1.23, -4.97], [5.38, 0.69], [5.91, -5.59]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.07,
                "init_y": -4.24,
                "init_a": -170.53,
                "velocity": 1.13,
                "goals": [[3.23, 2.89], [7.73, -2.11], [2.32, -5.41], [-4.13, -1.33]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.13,
                "init_y": -3.15,
                "init_a": 52.77,
                "velocity": 0.84,
                "goal_x": -6.95,
                "goal_y": 1.01,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_32_stopped_low(tester):
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
                "init_x": -7.93,
                "init_y": 5.89,
                "init_a": 65.90,
                "velocity": 0.94,
                "goals": [[-7.93, 5.89], [1.12, -5.66], [-4.23, -3.01], [-4.10, 1.77]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -8.69,
                "init_y": 6.54,
                "init_a": 65.90,
                "velocity": 0.94,
                "goal_x": -7.93,
                "goal_y": 5.89,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.53,
                "init_y": 4.18,
                "init_a": -95.26,
                "velocity": 0.95,
                "goal_x": 7.53,
                "goal_y": 4.18,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.63,
                "init_y": 3.18,
                "init_a": -95.26,
                "velocity": 0.95,
                "goals": [[7.53, 4.18], [4.30, 3.66], [4.06, -0.41], [2.96, 2.42]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.20,
                "init_y": 4.82,
                "init_a": -92.35,
                "velocity": 1.14,
                "goal_x": 2.00,
                "goal_y": 0.65,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.64,
                "init_y": 5.70,
                "init_a": 46.97,
                "velocity": 0.91,
                "goals": [[-1.01, -4.80], [-4.63, 3.35], [-1.07, 0.63], [-6.55, 4.12]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.42,
                "init_y": -5.74,
                "init_a": 120.10,
                "velocity": 0.97,
                "goals": [[-6.88, 3.92], [1.08, 0.51], [6.48, -0.01], [-5.63, 0.61]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_33_stopped_low(tester):
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
                "init_x": 6.00,
                "init_y": 5.47,
                "init_a": -77.91,
                "velocity": 1.04,
                "goal_x": 6.00,
                "goal_y": 5.47,
                "n_actors": 7,
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.78,
                "init_y": 4.94,
                "init_a": -171.83,
                "velocity": 0.87,
                "goals": [[5.78, 4.94], [-3.59, -4.49], [-6.17, -2.98], [0.21, -4.61]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[6.19, 4.16], [-6.08, 3.83], [-5.71, 2.38], [4.43, 1.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.94,
                "init_y": 0.43,
                "init_a": 164.76,
                "velocity": 0.89,
                "goals": [[5.30, -5.56], [-6.30, 0.77], [-2.46, -3.50], [1.58, 5.14]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.46,
                "init_y": 1.94,
                "init_a": -98.62,
                "velocity": 1.08,
                "goals": [[-5.18, 5.63], [1.27, 1.44], [-3.52, 3.85], [-7.29, -5.25]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_34_stopped_low(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 8.22,
                "init_y": 0.24,
                "init_a": 37.87,
                "velocity": 0.93,
                "goal_x": 7.29,
                "goal_y": 0.59,
                "n_actors": 8,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.62,
                "init_y": 0.87,
                "init_a": 51.97,
                "velocity": 0.85,
                "goals": [[-0.07, 0.15], [3.84, -4.43], [2.85, -0.82], [7.14, 4.44]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.16,
                "init_y": 1.66,
                "init_a": 16.28,
                "velocity": 0.92,
                "goal_x": 5.80,
                "goal_y": -1.24,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.24,
                "init_y": -5.96,
                "init_a": -148.42,
                "velocity": 0.99,
                "goals": [[4.55, -1.00], [0.86, -4.70], [-6.08, -5.85], [5.00, -4.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.68,
                "init_y": -3.74,
                "init_a": 27.48,
                "velocity": 1.14,
                "goals": [[6.72, -0.69], [-2.46, -3.09], [-3.81, 1.06], [0.35, -5.25]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.74,
                "init_y": -4.19,
                "init_a": 137.42,
                "velocity": 1.20,
                "goals": [[0.79, -2.01], [-0.05, 1.60], [5.05, 0.96], [-6.33, -4.78]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_35_walking_low(tester):
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
                "init_x": -4.66,
                "init_y": 3.34,
                "init_a": 56.70,
                "velocity": 1.00,
                "goal_x": 1.46,
                "goal_y": -2.66,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.56,
                "init_y": 3.78,
                "init_a": 56.70,
                "velocity": 1.00,
                "goal_x": 1.46,
                "goal_y": -2.66,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.95,
                "init_y": -0.73,
                "init_a": 3.93,
                "velocity": 1.01,
                "goals": [[5.96, -5.59], [-1.64, 2.33], [4.12, -1.77], [-5.95, -3.74]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.03,
                "init_y": -1.11,
                "init_a": 3.93,
                "velocity": 1.01,
                "goal_x": 5.96,
                "goal_y": -5.59,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.22,
                "init_y": -4.94,
                "init_a": 53.95,
                "velocity": 1.09,
                "goal_x": -3.83,
                "goal_y": 2.14,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.36,
                "init_y": -2.37,
                "init_a": -67.30,
                "velocity": 1.02,
                "goals": [[-1.21, 2.63], [-7.24, -3.55], [-5.32, 3.11], [-6.63, 0.87]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.07,
                "init_y": 0.76,
                "init_a": -94.02,
                "velocity": 0.91,
                "goals": [[-5.35, 1.00], [-7.20, 4.82], [-6.02, 5.32], [-0.39, 4.70]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.46,
                "init_y": -3.23,
                "init_a": 77.23,
                "velocity": 0.86,
                "goals": [[-6.34, 2.17], [6.88, 3.95], [-1.22, -1.07], [5.26, 1.49]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_36_walking_low(tester):
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
                "goals": [[0.54, -3.91], [-5.96, 2.44], [1.01, -0.28], [-3.14, 1.45]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-4.56, 4.52], [-0.62, 5.35], [-3.32, -1.00], [-3.52, 1.88]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.07,
                "init_y": -1.83,
                "init_a": 36.02,
                "velocity": 1.18,
                "goals": [[-4.56, 4.52], [-0.90, 3.55], [0.91, -3.15], [-6.89, -1.55]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.71,
                "init_y": 4.55,
                "init_a": -131.55,
                "velocity": 0.89,
                "goals": [[-4.73, 3.13], [-2.87, -2.34], [-7.71, -2.37], [-3.90, 5.78]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_37_walking_low(tester):
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
                "init_x": -5.42,
                "init_y": 1.11,
                "init_a": -40.68,
                "velocity": 0.94,
                "goals": [[2.74, -4.79], [-7.89, -5.89], [6.65, -0.01], [1.42, -2.40]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.53,
                "init_y": 0.65,
                "init_a": -40.68,
                "velocity": 0.94,
                "goal_x": 2.74,
                "goal_y": -4.79,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.81,
                "init_y": 4.28,
                "init_a": -45.33,
                "velocity": 1.19,
                "goals": [[6.20, 4.72], [-4.47, -5.98], [-5.25, -4.65], [-1.30, 4.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.83,
                "init_y": 4.10,
                "init_a": -45.33,
                "velocity": 1.19,
                "goals": [[6.20, 4.72], [-0.34, 5.18], [1.09, -5.07], [-3.94, 2.52]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.68,
                "init_y": -0.41,
                "init_a": 74.60,
                "velocity": 1.06,
                "goals": [[6.07, 1.61], [6.32, 3.45], [-1.29, -3.11], [3.96, -3.34]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.59,
                "init_y": 4.24,
                "init_a": -64.94,
                "velocity": 1.06,
                "goal_x": 7.08,
                "goal_y": -4.04,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.97,
                "init_y": 5.12,
                "init_a": -21.88,
                "velocity": 1.14,
                "goal_x": 5.89,
                "goal_y": 4.27,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.28,
                "init_y": -3.12,
                "init_a": -45.44,
                "velocity": 1.10,
                "goal_x": -6.29,
                "goal_y": 0.75,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_38_walking_low(tester):
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
                "init_x": -2.19,
                "init_y": -2.55,
                "init_a": 3.03,
                "velocity": 1.14,
                "goal_x": 1.07,
                "goal_y": 1.67,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.82,
                "init_y": -3.48,
                "init_a": 3.03,
                "velocity": 1.14,
                "goal_x": 1.07,
                "goal_y": 1.67,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.18,
                "init_y": -4.12,
                "init_a": -120.71,
                "velocity": 0.85,
                "goal_x": 6.44,
                "goal_y": -4.33,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.21,
                "init_y": -4.36,
                "init_a": -120.71,
                "velocity": 0.85,
                "goals": [[6.44, -4.33], [-3.54, 2.99], [7.25, 1.99], [-2.90, 2.91]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.64,
                "init_y": -0.80,
                "init_a": 171.31,
                "velocity": 0.89,
                "goals": [[7.38, 3.56], [-0.90, -5.14], [-5.32, 0.60], [-3.41, 2.71]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.45,
                "init_y": 2.72,
                "init_a": 68.59,
                "velocity": 1.11,
                "goals": [[3.60, -1.38], [4.96, 1.28], [4.96, 4.48], [-4.56, -1.07]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.69,
                "init_y": 4.38,
                "init_a": -80.68,
                "velocity": 1.11,
                "goals": [[2.81, -4.52], [6.70, 0.96], [3.95, 2.63], [0.28, -5.81]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_39_walking_low(tester):
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
                "init_x": -4.71,
                "init_y": -1.83,
                "init_a": -116.94,
                "velocity": 0.91,
                "goals": [[-1.97, 5.75], [-7.20, -3.18], [6.14, -0.13], [-2.95, 4.52]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.21,
                "init_y": -2.70,
                "init_a": -116.94,
                "velocity": 0.91,
                "goal_x": -1.97,
                "goal_y": 5.75,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.06,
                "init_y": -0.14,
                "init_a": -9.30,
                "velocity": 0.93,
                "goals": [[-0.69, 0.71], [4.18, 4.51], [6.16, -2.72], [-7.68, -0.81]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.93,
                "init_y": 0.36,
                "init_a": -9.30,
                "velocity": 0.93,
                "goals": [[-0.69, 0.71], [-1.00, 2.49], [-6.95, -5.01], [-3.01, -4.60]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.93,
                "init_y": 3.39,
                "init_a": 70.42,
                "velocity": 0.94,
                "goal_x": 5.51,
                "goal_y": 0.12,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.03,
                "init_y": -4.86,
                "init_a": 75.27,
                "velocity": 0.86,
                "goals": [[-1.84, 2.33], [-7.32, -0.56], [3.64, 4.61], [7.47, 3.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.65,
                "init_y": -4.06,
                "init_a": 17.86,
                "velocity": 1.04,
                "goal_x": -0.11,
                "goal_y": -0.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.38,
                "init_y": -2.24,
                "init_a": 120.23,
                "velocity": 1.04,
                "goal_x": -7.85,
                "goal_y": -5.38,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_40_walking_low(tester):
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
                "init_x": 1.01,
                "init_y": 1.22,
                "init_a": 150.93,
                "velocity": 1.08,
                "goals": [[4.02, -2.08], [7.08, 4.58], [-2.89, 2.24], [-3.00, 2.27]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.48,
                "init_y": 0.33,
                "init_a": 150.93,
                "velocity": 1.08,
                "goals": [[4.02, -2.08], [7.80, 3.34], [0.87, 1.16], [-0.90, 5.85]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.03,
                "init_y": 5.34,
                "init_a": -120.42,
                "velocity": 1.10,
                "goals": [[1.54, -0.24], [0.88, -3.05], [-6.53, -5.10], [-7.38, -3.66]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.97,
                "init_y": 5.40,
                "init_a": -120.42,
                "velocity": 1.10,
                "goal_x": 1.54,
                "goal_y": -0.24,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.35,
                "init_y": 3.90,
                "init_a": -44.30,
                "velocity": 0.86,
                "goals": [[-3.55, -0.33], [1.19, 0.07], [-4.86, 0.19], [6.13, -0.58]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.31,
                "init_y": 5.94,
                "init_a": -36.94,
                "velocity": 0.98,
                "goal_x": -0.57,
                "goal_y": -3.39,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.77,
                "init_y": 2.56,
                "init_a": 40.74,
                "velocity": 0.90,
                "goal_x": 1.33,
                "goal_y": -2.03,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.47,
                "init_y": -0.40,
                "init_a": -149.70,
                "velocity": 0.92,
                "goal_x": 6.65,
                "goal_y": 4.79,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_41_walking_low(tester):
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
                "init_x": 3.79,
                "init_y": -3.22,
                "init_a": -132.76,
                "velocity": 0.88,
                "goal_x": -7.88,
                "goal_y": 3.45,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.82,
                "init_y": -3.50,
                "init_a": -132.76,
                "velocity": 0.88,
                "goals": [[-7.88, 3.45], [4.01, 2.51], [-1.84, 1.30], [4.71, -5.53]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.47,
                "init_y": -0.21,
                "init_a": -76.59,
                "velocity": 1.03,
                "goals": [[5.38, -4.28], [7.64, 4.24], [-7.18, 0.15], [3.53, -1.56]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[5.49, -2.47], [-7.50, -1.69], [-7.49, 0.28], [1.39, 5.29]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.91,
                "init_y": 5.17,
                "init_a": -136.96,
                "velocity": 0.92,
                "goals": [[3.50, -1.06], [-2.84, 4.43], [6.22, 1.74], [-6.34, 0.01]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_42_walking_low(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.48,
                "init_y": -2.71,
                "init_a": -52.09,
                "velocity": 0.92,
                "goals": [[2.37, 5.22], [-2.41, 4.73], [6.74, 0.96], [-3.47, -5.08]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.76,
                "init_y": 1.80,
                "init_a": -27.05,
                "velocity": 1.08,
                "goals": [[7.65, 3.40], [6.41, -3.56], [6.65, 4.77], [-0.69, 3.43]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.38,
                "init_y": 2.73,
                "init_a": -27.05,
                "velocity": 1.08,
                "goal_x": 7.65,
                "goal_y": 3.40,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.24,
                "init_y": 1.73,
                "init_a": 137.96,
                "velocity": 0.97,
                "goal_x": -7.48,
                "goal_y": -1.91,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.73,
                "init_y": -5.77,
                "init_a": -153.13,
                "velocity": 1.06,
                "goals": [[-2.54, 4.18], [1.50, -0.04], [-6.35, -2.43], [2.88, 4.78]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.68,
                "init_y": -5.24,
                "init_a": 11.42,
                "velocity": 1.14,
                "goals": [[-5.03, 2.17], [-1.10, -2.11], [3.89, 0.44], [-2.12, -0.56]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.76,
                "init_y": 0.58,
                "init_a": 88.37,
                "velocity": 1.09,
                "goals": [[4.50, 4.39], [-6.40, 0.22], [-0.04, -4.76], [7.87, 2.88]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_43_stopped_low(tester):
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
                "init_x": 6.02,
                "init_y": 3.19,
                "init_a": -51.83,
                "velocity": 0.92,
                "goal_x": 6.02,
                "goal_y": 3.19,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.94,
                "init_y": 2.81,
                "init_a": -51.83,
                "velocity": 0.92,
                "goals": [[6.02, 3.19], [-7.95, -3.95], [4.04, 2.14], [-2.02, 1.43]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.74,
                "init_y": 5.47,
                "init_a": -5.71,
                "velocity": 0.88,
                "goal_x": 0.74,
                "goal_y": 5.47,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.20,
                "init_y": 6.36,
                "init_a": -5.71,
                "velocity": 0.88,
                "goals": [[0.74, 5.47], [-5.94, 1.07], [-0.49, 1.05], [4.69, -5.85]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.48,
                "init_y": -3.08,
                "init_a": -120.31,
                "velocity": 0.82,
                "goals": [[4.02, 1.16], [1.56, 4.76], [1.40, -4.71], [-4.25, -4.33]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.00,
                "init_y": -5.59,
                "init_a": -19.32,
                "velocity": 1.06,
                "goals": [[7.41, -0.38], [-4.01, -2.45], [-1.63, -1.93], [-0.40, 2.78]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.98,
                "init_y": 2.53,
                "init_a": -4.92,
                "velocity": 0.98,
                "goals": [[-7.31, -3.40], [7.02, 4.33], [4.49, 5.66], [5.09, 5.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_44_walking_low(tester):
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
                "init_x": -1.97,
                "init_y": 5.85,
                "init_a": -117.48,
                "velocity": 1.11,
                "goal_x": 0.38,
                "goal_y": -5.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.20,
                "init_y": 6.48,
                "init_a": -117.48,
                "velocity": 1.11,
                "goal_x": 0.38,
                "goal_y": -5.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.27,
                "init_y": -1.60,
                "init_a": -86.40,
                "velocity": 0.89,
                "goals": [[2.43, -0.14], [-0.02, -2.82], [3.46, 0.76], [4.79, 0.10]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.27,
                "init_y": -1.61,
                "init_a": -86.40,
                "velocity": 0.89,
                "goals": [[2.43, -0.14], [-6.56, -0.58], [2.58, 4.03], [-6.68, 4.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.62,
                "init_y": 4.54,
                "init_a": 159.95,
                "velocity": 0.95,
                "goals": [[6.97, -5.90], [5.54, 5.21], [2.38, -1.02], [3.79, -3.66]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.24,
                "init_y": -3.33,
                "init_a": -176.35,
                "velocity": 0.84,
                "goal_x": 5.89,
                "goal_y": -2.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.36,
                "init_y": 1.26,
                "init_a": -100.91,
                "velocity": 1.04,
                "goal_x": 2.90,
                "goal_y": -3.99,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.38,
                "init_y": 2.96,
                "init_a": -124.13,
                "velocity": 0.82,
                "goals": [[-0.79, -2.99], [-4.46, 2.25], [0.67, 1.82], [1.59, 0.71]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.23,
                "init_y": 4.31,
                "init_a": 108.14,
                "velocity": 0.96,
                "goals": [[5.88, 1.40], [-1.26, 3.31], [3.21, 1.71], [-4.82, 3.67]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_45_stopped_low(tester):
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
                "init_x": 3.96,
                "init_y": -5.80,
                "init_a": -169.11,
                "velocity": 1.17,
                "goal_x": 3.96,
                "goal_y": -5.80,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.59,
                "init_y": -6.57,
                "init_a": -169.11,
                "velocity": 1.17,
                "goal_x": 3.96,
                "goal_y": -5.80,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.60,
                "init_y": -3.44,
                "init_a": 101.70,
                "velocity": 1.03,
                "goals": [[3.60, -3.44], [7.26, 0.71], [7.14, 5.49], [4.77, -1.68]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.66,
                "init_y": -3.11,
                "init_a": 101.70,
                "velocity": 1.03,
                "goals": [[3.60, -3.44], [-4.02, -0.53], [2.40, -1.27], [2.80, -2.53]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.42,
                "init_y": -0.42,
                "init_a": -141.34,
                "velocity": 0.91,
                "goals": [[-2.67, -1.91], [2.13, -2.24], [-3.85, -0.90], [-5.39, -0.63]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_y": -1.38,
                "init_a": -167.79,
                "velocity": 0.93,
                "goal_x": 7.50,
                "goal_y": 3.94,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.51,
                "init_y": 4.39,
                "init_a": -91.71,
                "velocity": 1.11,
                "goals": [[-0.77, 3.00], [2.66, 3.26], [3.29, -2.87], [3.83, 2.79]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_46_stopped_low(tester):
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
                "goals": [[-1.98, -3.63], [7.85, 1.99], [-4.78, 4.70], [3.84, 0.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.24,
                "init_y": -3.16,
                "init_a": 38.68,
                "velocity": 0.97,
                "goals": [[-7.24, -3.16], [3.75, 2.82], [3.21, -0.04], [-3.97, 2.84]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.64,
                "init_y": -4.07,
                "init_a": 38.68,
                "velocity": 0.97,
                "goals": [[-7.24, -3.16], [1.91, 2.24], [-7.70, 1.69], [0.36, 1.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.95,
                "init_y": 1.95,
                "init_a": 19.32,
                "velocity": 0.95,
                "goal_x": -0.87,
                "goal_y": -3.60,
                "n_actors": 7,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.17,
                "init_y": -4.68,
                "init_a": -89.79,
                "velocity": 1.14,
                "goals": [[1.17, 3.21], [4.64, 4.33], [-7.39, 4.52], [6.15, -4.93]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_47_walking_low(tester):
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
                "init_x": 0.52,
                "init_y": -1.28,
                "init_a": -83.82,
                "velocity": 0.96,
                "goals": [[5.38, -4.67], [2.60, -4.17], [2.27, -5.95], [-3.20, -0.43]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_y": -1.39,
                "init_a": -83.82,
                "velocity": 0.96,
                "goal_x": 5.38,
                "goal_y": -4.67,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.28,
                "init_y": -3.32,
                "init_a": -46.08,
                "velocity": 1.09,
                "goals": [[-6.61, 4.32], [-5.57, -4.98], [-7.84, -4.67], [6.77, 5.17]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.14,
                "init_y": -2.33,
                "init_a": -46.08,
                "velocity": 1.09,
                "goal_x": -6.61,
                "goal_y": 4.32,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.60,
                "init_y": -4.23,
                "init_a": 35.99,
                "velocity": 1.04,
                "goal_x": 3.88,
                "goal_y": -5.23,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.75,
                "init_y": -3.94,
                "init_a": -10.22,
                "velocity": 0.89,
                "goals": [[2.55, -2.24], [6.28, 2.88], [-0.36, -5.47], [3.75, -3.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.25,
                "init_y": -3.42,
                "init_a": -149.79,
                "velocity": 0.95,
                "goals": [[-3.71, 2.67], [-1.24, 5.60], [0.05, -4.32], [-1.30, -4.50]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_48_walking_low(tester):
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
                "goals": [[-5.16, -0.76], [5.33, -5.75], [-3.33, 2.38], [-2.91, -1.77]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.66,
                "init_y": 1.80,
                "init_a": -173.12,
                "velocity": 0.92,
                "goals": [[-5.16, -0.76], [-1.60, -5.55], [0.88, 1.84], [-7.16, 0.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.65,
                "init_y": 3.64,
                "init_a": -7.87,
                "velocity": 1.15,
                "goal_x": 4.57,
                "goal_y": 3.77,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.82,
                "init_y": 3.09,
                "init_a": -7.87,
                "velocity": 1.15,
                "goals": [[4.57, 3.77], [6.92, -3.64], [-4.71, -5.27], [0.01, -4.66]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.21,
                "init_y": 0.14,
                "init_a": -27.55,
                "velocity": 1.20,
                "goals": [[3.86, -5.84], [-6.42, -5.46], [7.48, 2.36], [-7.29, -0.13]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_49_walking_low(tester):
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
                "init_x": 6.07,
                "init_y": -1.15,
                "init_a": -56.41,
                "velocity": 0.89,
                "goals": [[4.09, 3.57], [4.92, 1.61], [7.68, 1.21], [-5.32, -0.12]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.26,
                "init_y": -0.57,
                "init_a": -56.41,
                "velocity": 0.89,
                "goals": [[4.09, 3.57], [3.68, 0.00], [-1.75, 0.24], [-7.78, -0.07]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.19,
                "init_y": 4.24,
                "init_a": 34.19,
                "velocity": 1.07,
                "goals": [[4.49, 1.10], [-6.43, -5.30], [0.53, -2.47], [-6.22, -0.30]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.30,
                "init_y": 3.79,
                "init_a": 34.19,
                "velocity": 1.07,
                "goals": [[4.49, 1.10], [7.83, 3.45], [1.92, -2.75], [2.04, -5.84]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.47,
                "init_y": 5.92,
                "init_a": -29.06,
                "velocity": 1.05,
                "goal_x": -5.51,
                "goal_y": 4.39,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.71,
                "init_y": -3.31,
                "init_a": -104.12,
                "velocity": 0.80,
                "goal_x": -0.45,
                "goal_y": 3.79,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.94,
                "init_y": -1.27,
                "init_a": 27.10,
                "velocity": 1.09,
                "goal_x": -7.70,
                "goal_y": 3.21,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.13,
                "init_y": -0.55,
                "init_a": -22.34,
                "velocity": 0.88,
                "goal_x": -4.68,
                "goal_y": -4.74,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_50_stopped_low(tester):
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
                "goals": [[5.01, -5.23], [-7.35, -4.74], [-6.14, -0.30], [7.40, 4.07]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[1.32, 1.84], [-2.21, -5.08], [3.74, -0.27], [7.22, -1.58]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.20,
                "init_y": -5.91,
                "init_a": 50.73,
                "velocity": 1.00,
                "goals": [[-6.69, 1.33], [5.94, 0.01], [4.35, 4.22], [-4.94, -5.48]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.20,
                "init_y": 0.75,
                "init_a": 128.89,
                "velocity": 1.06,
                "goals": [[-4.81, 1.98], [5.74, -5.97], [0.27, 1.42], [2.63, 2.02]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_51_stopped_low(tester):
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
                "init_x": -2.15,
                "init_y": -3.09,
                "init_a": 39.50,
                "velocity": 0.99,
                "goal_x": -2.15,
                "goal_y": -3.09,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.15,
                "init_y": -2.09,
                "init_a": 39.50,
                "velocity": 0.99,
                "goals": [[-2.15, -3.09], [-5.57, 2.62], [5.47, -4.45], [0.98, 1.98]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.82,
                "init_y": -0.62,
                "init_a": 137.22,
                "velocity": 1.05,
                "goals": [[-4.82, -0.62], [-5.78, 3.88], [-4.78, 5.09], [5.36, -5.46]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.70,
                "init_y": 0.37,
                "init_a": 137.22,
                "velocity": 1.05,
                "goal_x": -4.82,
                "goal_y": -0.62,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.67,
                "init_y": 2.68,
                "init_a": -77.84,
                "velocity": 0.86,
                "goal_x": -0.59,
                "goal_y": 2.08,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.38,
                "init_y": -0.15,
                "init_a": 86.54,
                "velocity": 0.86,
                "goals": [[0.37, -5.05], [-1.57, 3.42], [3.82, 0.14], [3.02, -3.11]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.70,
                "init_y": -2.19,
                "init_a": 53.44,
                "velocity": 0.80,
                "goals": [[-5.93, -5.48], [2.20, 3.84], [-6.81, 2.77], [1.22, -0.45]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.08,
                "init_y": -2.20,
                "init_a": -133.94,
                "velocity": 0.93,
                "goal_x": -1.88,
                "goal_y": 1.21,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_52_stopped_low(tester):
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
                "init_x": 7.30,
                "init_y": -0.08,
                "init_a": 58.20,
                "velocity": 0.83,
                "goals": [[7.30, -0.08], [3.65, -2.59], [-2.52, -3.42], [-3.67, 4.13]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 8.28,
                "init_y": 0.13,
                "init_a": 58.20,
                "velocity": 0.83,
                "goal_x": 7.30,
                "goal_y": -0.08,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.56,
                "init_y": -4.03,
                "init_a": -132.32,
                "velocity": 0.97,
                "goals": [[-4.56, -4.03], [1.15, 1.23], [6.29, 4.94], [3.99, -2.83]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.87,
                "init_y": -3.30,
                "init_a": -132.32,
                "velocity": 0.97,
                "goal_x": -4.56,
                "goal_y": -4.03,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.72,
                "init_y": -1.52,
                "init_a": 85.48,
                "velocity": 0.86,
                "goal_x": -2.39,
                "goal_y": 0.71,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.94,
                "init_y": -1.14,
                "init_a": -29.67,
                "velocity": 1.15,
                "goal_x": 1.08,
                "goal_y": 5.17,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.87,
                "init_y": 2.75,
                "init_a": -89.74,
                "velocity": 1.11,
                "goals": [[-2.40, -5.47], [4.69, -5.08], [2.57, -4.52], [-6.21, -1.60]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.13,
                "init_y": -3.98,
                "init_a": 10.11,
                "velocity": 0.99,
                "goals": [[5.88, -4.46], [5.14, -3.98], [-4.53, -1.74], [4.32, -5.15]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_53_stopped_low(tester):
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
                "init_x": -1.70,
                "init_y": 4.83,
                "init_a": -28.91,
                "velocity": 0.93,
                "goal_x": -1.70,
                "goal_y": 4.83,
                "n_actors": 7,
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
                "goals": [[-1.70, 4.83], [1.63, 3.99], [7.67, -3.72], [3.87, 0.73]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.37,
                "init_y": 4.04,
                "init_a": -131.76,
                "velocity": 1.05,
                "goals": [[-1.15, -5.12], [-0.20, 2.49], [-7.98, 4.63], [-7.78, -5.00]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.62,
                "init_y": -0.05,
                "init_a": 16.07,
                "velocity": 1.13,
                "goals": [[-4.48, 4.09], [7.89, 3.03], [6.01, 2.60], [-5.99, 0.15]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.81,
                "init_y": -2.64,
                "init_a": 94.23,
                "velocity": 1.02,
                "goals": [[4.14, -2.26], [4.74, 4.57], [0.68, -5.13], [4.04, -5.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_54_walking_low(tester):
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
                "goals": [[3.98, 2.75], [1.56, -3.71], [-2.01, -1.18], [3.44, 4.33]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.96,
                "init_y": -2.39,
                "init_a": 12.74,
                "velocity": 1.04,
                "goals": [[7.44, -4.34], [4.54, -3.41], [7.10, -3.55], [2.66, -5.07]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.97,
                "init_y": -2.46,
                "init_a": 12.74,
                "velocity": 1.04,
                "goals": [[7.44, -4.34], [6.55, -2.30], [4.34, -3.50], [1.30, -2.79]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.06,
                "init_y": 5.78,
                "init_a": -14.62,
                "velocity": 0.84,
                "goals": [[-3.58, 3.91], [4.17, -1.09], [-2.21, 4.07], [2.29, 4.24]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.07,
                "init_y": -1.98,
                "init_a": -73.07,
                "velocity": 0.99,
                "goal_x": 1.21,
                "goal_y": -0.99,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.87,
                "init_y": -5.52,
                "init_a": 66.98,
                "velocity": 0.80,
                "goals": [[-2.75, -1.68], [-6.28, 1.54], [-2.02, 1.01], [-5.53, 5.65]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_55_walking_low(tester):
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
                "init_x": 7.73,
                "init_y": -3.32,
                "init_a": -120.24,
                "velocity": 1.05,
                "goal_x": -1.34,
                "goal_y": -4.55,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 8.59,
                "init_y": -3.82,
                "init_a": -120.24,
                "velocity": 1.05,
                "goals": [[-1.34, -4.55], [6.33, -0.77], [-1.30, -4.47], [5.97, -3.10]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.59,
                "init_y": -4.57,
                "init_a": 45.41,
                "velocity": 1.09,
                "goal_x": 1.47,
                "goal_y": -0.13,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.04,
                "init_y": -3.74,
                "init_a": 45.41,
                "velocity": 1.09,
                "goals": [[1.47, -0.13], [5.50, -2.21], [5.94, 1.82], [-2.80, -1.82]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.18,
                "init_y": 4.26,
                "init_a": 28.39,
                "velocity": 1.02,
                "goal_x": -6.21,
                "goal_y": 0.04,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.33,
                "init_y": -2.57,
                "init_a": -113.53,
                "velocity": 1.09,
                "goals": [[7.47, -0.66], [-5.85, 1.92], [-2.81, -1.89], [-4.44, 0.62]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.49,
                "init_y": -4.05,
                "init_a": 162.45,
                "velocity": 0.89,
                "goals": [[-5.99, -4.82], [-3.40, 4.15], [-6.04, -4.04], [-3.39, 2.05]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_56_walking_low(tester):
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
                "init_x": -6.46,
                "init_y": -0.59,
                "init_a": -140.34,
                "velocity": 1.20,
                "goals": [[-4.69, 0.44], [-6.34, -4.23], [-4.46, -5.01], [0.45, -3.75]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.10,
                "init_y": 0.35,
                "init_a": -140.34,
                "velocity": 1.20,
                "goal_x": -4.69,
                "goal_y": 0.44,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.17,
                "init_y": -1.24,
                "init_a": -143.67,
                "velocity": 0.83,
                "goals": [[2.94, -2.33], [-4.77, -5.87], [-6.25, 3.53], [4.42, -1.22]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.23,
                "init_y": -1.59,
                "init_a": -143.67,
                "velocity": 0.83,
                "goal_x": 2.94,
                "goal_y": -2.33,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.53,
                "init_y": 5.45,
                "init_a": 7.24,
                "velocity": 1.19,
                "goal_x": -4.13,
                "goal_y": -0.44,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.66,
                "init_y": -4.57,
                "init_a": 106.14,
                "velocity": 1.08,
                "goals": [[0.23, -2.98], [0.67, 1.53], [7.42, -1.05], [-2.73, -2.06]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.31,
                "init_y": 5.96,
                "init_a": -35.45,
                "velocity": 0.83,
                "goals": [[-6.44, -1.66], [1.46, -0.59], [-5.68, -4.46], [0.82, 1.94]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.21,
                "init_y": 5.34,
                "init_a": -49.97,
                "velocity": 0.81,
                "goal_x": -1.52,
                "goal_y": 1.95,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_57_stopped_low(tester):
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
                "init_x": 5.13,
                "init_y": 2.94,
                "init_a": -121.11,
                "velocity": 0.97,
                "goals": [[5.13, 2.94], [-5.02, 2.18], [6.33, 4.00], [2.60, -1.90]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.47,
                "init_y": 2.00,
                "init_a": -121.11,
                "velocity": 0.97,
                "goal_x": 5.13,
                "goal_y": 2.94,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.51,
                "init_y": -1.09,
                "init_a": 156.54,
                "velocity": 1.15,
                "goals": [[-1.51, -1.09], [-5.62, 1.43], [-0.00, -5.95], [-7.32, -5.18]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.62,
                "init_y": 2.11,
                "init_a": 48.72,
                "velocity": 0.94,
                "goals": [[0.75, -3.59], [-3.66, -3.16], [-3.70, 1.23], [6.08, -3.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.39,
                "init_y": -2.24,
                "init_a": 106.29,
                "velocity": 0.97,
                "goals": [[-1.12, 0.37], [-0.39, 0.09], [7.82, 0.63], [3.73, 2.67]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.65,
                "init_y": -0.54,
                "init_a": -62.86,
                "velocity": 0.91,
                "goal_x": 4.32,
                "goal_y": -0.49,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.79,
                "init_y": -0.57,
                "init_a": -173.83,
                "velocity": 0.89,
                "goals": [[6.57, -1.98], [5.69, 2.99], [1.82, -3.60], [2.97, 4.19]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_58_walking_low(tester):
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
                "init_x": -4.35,
                "init_y": -0.79,
                "init_a": -34.91,
                "velocity": 1.14,
                "goal_x": 7.59,
                "goal_y": -4.42,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.46,
                "init_y": -1.26,
                "init_a": -34.91,
                "velocity": 1.14,
                "goals": [[7.59, -4.42], [-7.59, -4.34], [3.21, 0.45], [2.95, -4.02]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.65,
                "init_y": -3.33,
                "init_a": 163.02,
                "velocity": 0.82,
                "goals": [[-3.20, -2.39], [-5.67, -4.26], [-3.21, -5.03], [-5.29, -4.36]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.75,
                "init_y": -3.77,
                "init_a": 163.02,
                "velocity": 0.82,
                "goal_x": -3.20,
                "goal_y": -2.39,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.81,
                "init_y": -1.27,
                "init_a": 168.39,
                "velocity": 0.91,
                "goal_x": 4.59,
                "goal_y": 5.70,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.76,
                "init_y": 3.08,
                "init_a": 0.03,
                "velocity": 0.82,
                "goals": [[2.48, -3.77], [-4.47, -2.09], [-0.05, -5.34], [5.55, 1.62]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.48,
                "init_y": -4.83,
                "init_a": -0.69,
                "velocity": 1.17,
                "goal_x": 7.10,
                "goal_y": 3.83,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.03,
                "init_y": -5.65,
                "init_a": -178.37,
                "velocity": 0.83,
                "goals": [[3.40, 5.09], [-1.45, -5.04], [-7.22, 3.91], [3.08, 1.58]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_59_walking_low(tester):
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
                "goals": [[-1.20, 1.27], [-5.89, -5.67], [-2.91, 3.40], [2.56, 0.37]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.73,
                "init_y": -0.15,
                "init_a": -96.60,
                "velocity": 0.92,
                "goals": [[4.86, -0.21], [-1.05, -3.15], [-4.33, -1.86], [-6.45, -2.07]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.87,
                "init_y": 0.37,
                "init_a": -96.60,
                "velocity": 0.92,
                "goals": [[4.86, -0.21], [6.66, 3.31], [-2.34, -1.63], [-6.94, -4.93]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
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
                "goals": [[-1.86, -2.58], [-7.32, -3.43], [-7.22, 4.93], [2.79, -3.20]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_60_walking_low(tester):
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
                "init_x": 0.64,
                "init_y": 1.29,
                "init_a": 54.76,
                "velocity": 1.11,
                "goal_x": 4.38,
                "goal_y": 5.93,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.28,
                "init_y": 2.23,
                "init_a": 54.76,
                "velocity": 1.11,
                "goal_x": 4.38,
                "goal_y": 5.93,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.61,
                "init_y": 5.59,
                "init_a": 47.70,
                "velocity": 1.07,
                "goal_x": -3.66,
                "goal_y": -3.74,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -8.60,
                "init_y": 5.71,
                "init_a": 47.70,
                "velocity": 1.07,
                "goals": [[-3.66, -3.74], [0.29, -4.92], [-5.97, 1.27], [-4.66, -2.44]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.55,
                "init_y": 3.34,
                "init_a": -140.22,
                "velocity": 0.94,
                "goals": [[-5.57, -4.46], [1.05, 2.57], [1.34, -0.39], [5.20, -0.66]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.07,
                "init_y": -5.15,
                "init_a": -178.76,
                "velocity": 0.98,
                "goals": [[6.53, 0.30], [-1.25, -2.98], [-3.82, 2.17], [-0.84, -4.67]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.62,
                "init_y": -4.18,
                "init_a": 89.81,
                "velocity": 0.86,
                "goals": [[3.93, -4.43], [5.36, 4.98], [-1.06, 1.56], [5.75, -3.10]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.73,
                "init_y": -1.18,
                "init_a": -77.11,
                "velocity": 0.82,
                "goal_x": 5.08,
                "goal_y": -1.60,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_61_stopped_low(tester):
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
                "goals": [[0.99, 2.09], [1.00, 3.50], [2.80, 1.01], [5.15, 2.14]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[1.07, -3.92], [-0.94, -5.50], [5.61, -0.80], [0.16, 1.85]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.90,
                "init_y": -4.48,
                "init_a": -110.00,
                "velocity": 0.90,
                "goals": [[1.07, -3.92], [4.14, -3.37], [6.67, 2.24], [7.13, -4.61]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.25,
                "init_y": 4.61,
                "init_a": -60.70,
                "velocity": 1.10,
                "goals": [[5.56, 3.38], [-0.79, 2.61], [-3.08, -3.44], [-3.75, -3.14]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_62_stopped_low(tester):
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
                "init_x": 5.98,
                "init_y": 4.30,
                "init_a": -172.71,
                "velocity": 0.80,
                "goals": [[5.98, 4.30], [-2.88, 2.27], [2.52, 3.54], [-1.12, -4.45]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.98,
                "init_y": 4.35,
                "init_a": -172.71,
                "velocity": 0.80,
                "goals": [[5.98, 4.30], [-0.79, 4.46], [-3.02, -4.73], [-3.91, 1.81]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.33,
                "init_y": 1.09,
                "init_a": -64.29,
                "velocity": 1.12,
                "goals": [[-1.33, 1.09], [-2.06, -2.11], [-5.41, -0.40], [1.37, -0.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.27,
                "init_y": 1.43,
                "init_a": -64.29,
                "velocity": 1.12,
                "goals": [[-1.33, 1.09], [2.30, 3.70], [5.23, 0.79], [-3.44, 5.76]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.50,
                "init_y": 0.96,
                "init_a": 90.77,
                "velocity": 1.13,
                "goals": [[-6.09, -0.56], [-4.55, 5.89], [-7.35, 0.68], [-1.15, -1.31]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.42,
                "init_y": -2.28,
                "init_a": -147.87,
                "velocity": 0.97,
                "goal_x": 1.69,
                "goal_y": -0.46,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.90,
                "init_y": -3.48,
                "init_a": -67.97,
                "velocity": 1.17,
                "goal_x": -2.12,
                "goal_y": 1.55,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.25,
                "init_y": -1.12,
                "init_a": 14.93,
                "velocity": 1.08,
                "goal_x": 3.27,
                "goal_y": 4.54,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.18,
                "init_y": 1.91,
                "init_a": -56.20,
                "velocity": 1.01,
                "goal_x": -3.15,
                "goal_y": 1.62,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_63_stopped_low(tester):
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
                "init_x": 7.87,
                "init_y": -2.53,
                "init_a": -157.72,
                "velocity": 1.19,
                "goals": [[7.87, -2.53], [4.45, 5.17], [2.83, -3.44], [2.54, 5.19]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 8.09,
                "init_y": -1.55,
                "init_a": -157.72,
                "velocity": 1.19,
                "goals": [[7.87, -2.53], [-2.46, 1.18], [-4.12, -1.61], [-7.59, 1.88]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.47,
                "init_y": -3.00,
                "init_a": 168.65,
                "velocity": 0.89,
                "goal_x": 1.47,
                "goal_y": -3.00,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.01,
                "init_y": -2.10,
                "init_a": 168.65,
                "velocity": 0.89,
                "goals": [[1.47, -3.00], [-1.50, -2.34], [3.02, 1.76], [6.75, 1.05]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.36,
                "init_y": -4.22,
                "init_a": 74.54,
                "velocity": 1.07,
                "goal_x": -6.73,
                "goal_y": 0.89,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.36,
                "init_y": 0.96,
                "init_a": -136.63,
                "velocity": 1.11,
                "goals": [[7.32, 0.24], [1.38, 0.49], [5.47, 1.72], [0.73, -4.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.90,
                "init_y": -1.93,
                "init_a": -37.05,
                "velocity": 1.14,
                "goal_x": -1.70,
                "goal_y": -3.80,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_64_walking_low(tester):
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
                "init_x": -1.11,
                "init_y": 4.39,
                "init_a": 100.23,
                "velocity": 0.82,
                "goals": [[6.25, -5.10], [6.57, 3.40], [6.89, -2.11], [3.57, -4.17]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.15,
                "init_y": 4.66,
                "init_a": 100.23,
                "velocity": 0.82,
                "goals": [[6.25, -5.10], [2.36, 5.80], [2.84, 2.08], [4.71, -5.16]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.02,
                "init_y": -4.35,
                "init_a": -116.00,
                "velocity": 0.93,
                "goal_x": -6.82,
                "goal_y": -1.31,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.73,
                "init_y": -3.64,
                "init_a": -116.00,
                "velocity": 0.93,
                "goal_x": -6.82,
                "goal_y": -1.31,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.48,
                "init_y": -3.61,
                "init_a": -147.54,
                "velocity": 1.16,
                "goal_x": 3.11,
                "goal_y": 5.94,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.78,
                "init_y": 4.18,
                "init_a": 74.75,
                "velocity": 0.92,
                "goal_x": -4.24,
                "goal_y": -0.40,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.12,
                "init_y": 3.43,
                "init_a": 12.27,
                "velocity": 1.14,
                "goals": [[1.58, -0.89], [-0.06, 0.52], [2.50, 3.92], [-1.94, -1.57]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.24,
                "init_y": -4.07,
                "init_a": 27.79,
                "velocity": 0.92,
                "goals": [[5.09, -3.04], [-7.77, -2.45], [-6.96, 0.88], [-6.55, -5.99]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_65_walking_low(tester):
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
                "init_x": 4.95,
                "init_y": 4.27,
                "init_a": -173.85,
                "velocity": 1.17,
                "goal_x": 6.29,
                "goal_y": 2.71,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.60,
                "init_y": 5.03,
                "init_a": -173.85,
                "velocity": 1.17,
                "goals": [[6.29, 2.71], [-5.31, 4.29], [4.66, 2.11], [-5.12, -5.85]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.28,
                "init_y": 2.28,
                "init_a": -134.52,
                "velocity": 1.01,
                "goal_x": -6.16,
                "goal_y": -4.07,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.47,
                "init_y": 2.87,
                "init_a": -134.52,
                "velocity": 1.01,
                "goals": [[-6.16, -4.07], [1.78, 5.37], [-3.26, -5.32], [7.40, 0.16]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.69,
                "init_y": 0.32,
                "init_a": -124.69,
                "velocity": 0.95,
                "goals": [[-1.30, -1.65], [2.72, -0.91], [5.82, -4.37], [-4.58, 0.73]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.80,
                "init_y": -1.29,
                "init_a": 110.40,
                "velocity": 0.83,
                "goal_x": 2.35,
                "goal_y": 5.77,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.10,
                "init_y": -0.52,
                "init_a": -15.25,
                "velocity": 1.19,
                "goals": [[-4.13, 2.82], [1.84, 1.47], [3.85, 0.97], [-3.04, -5.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.64,
                "init_y": -4.65,
                "init_a": 134.13,
                "velocity": 1.19,
                "goal_x": -7.18,
                "goal_y": -0.40,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_66_walking_low(tester):
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
                "init_x": 2.94,
                "init_y": -5.04,
                "init_a": -57.89,
                "velocity": 0.85,
                "goals": [[6.32, 2.26], [-7.94, 5.78], [3.35, -4.77], [-6.68, 2.48]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.97,
                "init_y": -5.70,
                "init_a": -61.58,
                "velocity": 0.95,
                "goals": [[-3.30, -0.14], [6.46, -0.38], [4.22, -2.21], [-1.92, -0.42]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
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
                "goals": [[5.15, 3.97], [1.90, 1.58], [-5.08, -3.39], [-5.26, -0.68]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.72,
                "init_y": -1.73,
                "init_a": -102.43,
                "velocity": 1.06,
                "goal_x": -5.82,
                "goal_y": -2.46,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.03,
                "init_y": 4.61,
                "init_a": 77.00,
                "velocity": 1.00,
                "goals": [[-6.64, -3.90], [0.89, -2.47], [0.92, -2.27], [1.53, -2.93]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_67_walking_low(tester):
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
                "init_x": -3.71,
                "init_y": 2.04,
                "init_a": -77.81,
                "velocity": 0.96,
                "goals": [[-6.86, -2.82], [-3.11, -1.20], [-2.25, -1.40], [6.34, -2.96]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.24,
                "init_y": 1.19,
                "init_a": -77.81,
                "velocity": 0.96,
                "goals": [[-6.86, -2.82], [1.42, 2.77], [-5.38, 0.59], [3.19, -1.41]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.46,
                "init_y": 3.84,
                "init_a": -0.43,
                "velocity": 0.97,
                "goals": [[-7.27, -4.33], [6.86, 0.80], [-7.83, 1.86], [3.54, -5.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.85,
                "init_y": 4.75,
                "init_a": -0.43,
                "velocity": 0.97,
                "goal_x": -7.27,
                "goal_y": -4.33,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.91,
                "init_y": 5.08,
                "init_a": 155.57,
                "velocity": 1.12,
                "goal_x": 0.46,
                "goal_y": -5.31,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.32,
                "init_y": -5.01,
                "init_a": -75.09,
                "velocity": 0.81,
                "goals": [[-0.75, -1.53], [0.88, 3.22], [-7.23, 5.09], [-5.35, 0.22]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.68,
                "init_y": 1.83,
                "init_a": 58.05,
                "velocity": 0.96,
                "goal_x": -7.17,
                "goal_y": -3.52,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.15,
                "init_y": 3.44,
                "init_a": 49.67,
                "velocity": 1.13,
                "goal_x": 6.27,
                "goal_y": -5.00,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_68_stopped_low(tester):
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
                "init_x": -1.49,
                "init_y": -0.10,
                "init_a": -39.32,
                "velocity": 1.01,
                "goal_x": -1.49,
                "goal_y": -0.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.79,
                "init_y": -0.82,
                "init_a": -39.32,
                "velocity": 1.01,
                "goal_x": -1.49,
                "goal_y": -0.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.26,
                "init_y": 2.08,
                "init_a": -104.95,
                "velocity": 0.82,
                "goals": [[-4.26, 2.08], [-0.76, 2.70], [-5.29, -3.41], [-0.06, 1.28]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.26,
                "init_y": 2.07,
                "init_a": -104.95,
                "velocity": 0.82,
                "goals": [[-4.26, 2.08], [-7.75, 3.98], [-6.38, 1.01], [0.70, -4.29]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.91,
                "init_y": -2.99,
                "init_a": -64.56,
                "velocity": 0.99,
                "goal_x": 7.74,
                "goal_y": -1.73,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.31,
                "init_y": -5.70,
                "init_a": 121.34,
                "velocity": 1.11,
                "goals": [[2.91, 4.38], [-1.21, 4.89], [-3.72, 4.29], [4.47, 1.12]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.17,
                "init_y": 4.17,
                "init_a": 4.82,
                "velocity": 1.16,
                "goal_x": -2.58,
                "goal_y": 4.79,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.55,
                "init_y": -1.39,
                "init_a": -159.26,
                "velocity": 1.05,
                "goals": [[-5.41, 2.67], [2.57, -5.79], [1.40, -1.22], [4.32, -2.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_69_walking_low(tester):
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
                "init_x": 6.38,
                "init_y": 0.82,
                "init_a": 56.35,
                "velocity": 0.89,
                "goal_x": -6.91,
                "goal_y": 1.91,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.55,
                "init_y": 1.38,
                "init_a": 56.35,
                "velocity": 0.89,
                "goals": [[-6.91, 1.91], [-3.12, 4.81], [-1.43, -0.61], [-7.57, 2.15]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-4.05, -1.06], [5.80, 5.63], [7.24, 2.15], [-2.54, 0.83]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.29,
                "init_y": -4.98,
                "init_a": -35.01,
                "velocity": 0.96,
                "goals": [[-3.06, 3.95], [0.79, 5.91], [0.83, -5.62], [-0.13, 3.83]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.75,
                "init_y": -1.45,
                "init_a": -30.41,
                "velocity": 0.96,
                "goals": [[5.81, -1.29], [-1.42, -5.42], [1.34, 5.07], [-7.02, 3.99]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_70_walking_low(tester):
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
                "goals": [[-1.43, 4.61], [5.35, -0.47], [3.49, -0.71], [2.71, -2.12]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[6.39, 4.01], [-5.05, -1.65], [6.28, -3.60], [-3.75, -2.94]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.40,
                "init_y": -3.58,
                "init_a": -6.10,
                "velocity": 1.07,
                "goals": [[-2.32, 2.16], [-3.39, 1.91], [3.94, 0.30], [7.12, -2.30]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.54,
                "init_y": 3.23,
                "init_a": -129.37,
                "velocity": 1.11,
                "goals": [[2.36, -4.09], [-1.29, 5.51], [-2.07, -3.91], [-2.04, 5.28]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.13,
                "init_y": 1.65,
                "init_a": 152.00,
                "velocity": 0.86,
                "goals": [[3.01, -1.64], [1.19, 4.05], [-4.37, -0.64], [-5.08, 0.01]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_71_walking_low(tester):
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
                "init_x": 6.33,
                "init_y": 3.33,
                "init_a": 82.77,
                "velocity": 0.84,
                "goal_x": 2.02,
                "goal_y": 5.48,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.48,
                "init_y": 3.86,
                "init_a": 82.77,
                "velocity": 0.84,
                "goals": [[2.02, 5.48], [5.46, 5.09], [5.36, 0.13], [3.33, 5.66]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.03,
                "init_y": 4.82,
                "init_a": -139.63,
                "velocity": 1.19,
                "goal_x": 7.75,
                "goal_y": -2.83,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.88,
                "init_y": 5.34,
                "init_a": -139.63,
                "velocity": 1.19,
                "goals": [[7.75, -2.83], [-5.25, -6.00], [0.50, 2.45], [-6.39, 2.89]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.06,
                "init_y": -3.79,
                "init_a": 29.01,
                "velocity": 0.91,
                "goal_x": -1.23,
                "goal_y": -1.95,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.28,
                "init_y": 3.82,
                "init_a": -168.74,
                "velocity": 0.92,
                "goals": [[-5.27, -4.64], [-7.82, 0.52], [6.85, 3.79], [-7.87, 3.78]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.00,
                "init_y": 0.57,
                "init_a": -179.50,
                "velocity": 1.15,
                "goals": [[-5.73, 1.57], [-3.99, -4.34], [-6.18, 5.23], [1.78, -5.63]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.34,
                "init_y": -3.29,
                "init_a": 10.53,
                "velocity": 0.98,
                "goal_x": 0.63,
                "goal_y": 3.95,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_72_stopped_low(tester):
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
                "init_x": -3.50,
                "init_y": -2.26,
                "init_a": 130.02,
                "velocity": 0.86,
                "goal_x": -3.50,
                "goal_y": -2.26,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.60,
                "init_y": -1.81,
                "init_a": 130.02,
                "velocity": 0.86,
                "goal_x": -3.50,
                "goal_y": -2.26,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.75,
                "init_y": -3.64,
                "init_a": 140.74,
                "velocity": 1.00,
                "goals": [[-6.75, -3.64], [-0.53, -3.82], [-2.81, 3.44], [-1.15, -4.91]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.75,
                "init_y": -3.70,
                "init_a": 140.74,
                "velocity": 1.00,
                "goals": [[-6.75, -3.64], [-6.00, -5.30], [-2.74, 3.76], [-1.69, 4.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.26,
                "init_y": 0.55,
                "init_a": -106.96,
                "velocity": 0.99,
                "goal_x": 1.67,
                "goal_y": -3.64,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.66,
                "init_y": -0.02,
                "init_a": 92.18,
                "velocity": 1.07,
                "goals": [[1.75, -0.05], [4.29, 4.77], [0.25, 5.04], [-0.90, -2.00]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.98,
                "init_y": 2.55,
                "init_a": -69.61,
                "velocity": 1.20,
                "goal_x": 6.73,
                "goal_y": 0.50,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.89,
                "init_y": 4.37,
                "init_a": 67.29,
                "velocity": 0.81,
                "goals": [[-6.96, 2.72], [-1.56, 1.43], [1.86, 2.60], [7.36, -2.63]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.50,
                "init_y": -4.21,
                "init_a": 26.27,
                "velocity": 0.97,
                "goals": [[1.53, -0.48], [3.33, -3.70], [-0.91, 4.03], [7.30, 4.33]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_73_walking_low(tester):
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
                "goals": [[-7.16, 0.16], [-3.35, -5.54], [-1.02, -4.12], [-0.58, -1.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.08,
                "init_y": -4.72,
                "init_a": 15.15,
                "velocity": 1.16,
                "goals": [[-7.16, 0.16], [2.33, -4.59], [2.41, -2.84], [2.30, 5.97]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.26,
                "init_y": 5.90,
                "init_a": -52.58,
                "velocity": 1.03,
                "goals": [[1.08, 3.88], [-4.76, -1.92], [-0.03, -0.09], [-4.41, 5.80]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.13,
                "init_y": -1.13,
                "init_a": -16.95,
                "velocity": 0.99,
                "goals": [[-6.72, -4.09], [0.80, 5.19], [-6.66, -1.91], [-1.59, -4.31]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_74_walking_low(tester):
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
                "goals": [[-6.21, 2.48], [1.78, 5.48], [5.51, 5.54], [2.37, 5.34]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.03,
                "init_y": 4.06,
                "init_a": 59.38,
                "velocity": 0.83,
                "goals": [[2.09, 3.91], [2.88, 0.40], [-5.94, 1.00], [-6.42, -5.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.72,
                "init_y": 3.96,
                "init_a": 60.85,
                "velocity": 1.01,
                "goals": [[7.20, -2.26], [-7.20, -4.52], [-7.81, 1.91], [7.72, 3.80]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.72,
                "init_y": 1.94,
                "init_a": 73.06,
                "velocity": 0.81,
                "goals": [[1.90, 2.82], [5.75, -2.60], [-4.71, -4.61], [-5.32, 1.26]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.84,
                "init_y": 2.69,
                "init_a": -165.69,
                "velocity": 0.89,
                "goal_x": 4.55,
                "goal_y": 4.44,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_75_walking_low(tester):
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
                "init_x": -6.37,
                "init_y": 3.43,
                "init_a": 72.15,
                "velocity": 0.99,
                "goal_x": 4.86,
                "goal_y": 1.57,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.00,
                "init_y": 2.66,
                "init_a": 72.15,
                "velocity": 0.99,
                "goal_x": 4.86,
                "goal_y": 1.57,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.80,
                "init_y": 2.01,
                "init_a": 65.93,
                "velocity": 1.04,
                "goal_x": -4.17,
                "goal_y": 2.05,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.04,
                "init_y": 2.98,
                "init_a": 65.93,
                "velocity": 1.04,
                "goals": [[-4.17, 2.05], [4.62, -2.43], [-4.53, 3.77], [-4.08, -4.92]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.49,
                "init_y": 0.45,
                "init_a": 25.33,
                "velocity": 1.13,
                "goals": [[0.54, -2.40], [-5.35, -1.75], [7.29, -0.81], [3.06, 2.12]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.87,
                "init_y": -3.46,
                "init_a": -44.09,
                "velocity": 0.91,
                "goals": [[-0.80, 0.52], [-0.41, -0.52], [6.69, 1.62], [2.40, -4.59]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.58,
                "init_y": 0.81,
                "init_a": -0.59,
                "velocity": 0.95,
                "goals": [[5.52, 3.58], [-6.20, 3.66], [0.91, -0.17], [7.43, -2.48]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_76_walking_low(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.33,
                "init_y": -0.53,
                "init_a": -44.48,
                "velocity": 0.92,
                "goals": [[3.16, -0.18], [-2.45, -5.18], [7.46, -2.70], [-3.71, 4.61]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.59,
                "init_y": 4.02,
                "init_a": -16.75,
                "velocity": 0.85,
                "goal_x": 3.87,
                "goal_y": -1.18,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.45,
                "init_y": 3.03,
                "init_a": -16.75,
                "velocity": 0.85,
                "goal_x": 3.87,
                "goal_y": -1.18,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.46,
                "init_y": 3.18,
                "init_a": -21.39,
                "velocity": 1.03,
                "goals": [[0.42, 1.82], [3.59, -3.00], [2.37, 1.41], [-6.78, 3.28]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.56,
                "init_y": -1.81,
                "init_a": -45.64,
                "velocity": 1.05,
                "goals": [[1.25, -5.61], [3.12, 1.04], [0.83, -3.46], [-7.78, -1.75]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.31,
                "init_y": 4.43,
                "init_a": 179.07,
                "velocity": 1.10,
                "goals": [[-6.27, -3.68], [-1.63, -2.95], [0.91, 5.86], [-7.42, -4.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_77_walking_low(tester):
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
                "init_x": -5.55,
                "init_y": -0.38,
                "init_a": 21.62,
                "velocity": 1.09,
                "goal_x": 3.54,
                "goal_y": 3.83,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.74,
                "init_y": -1.36,
                "init_a": 21.62,
                "velocity": 1.09,
                "goals": [[3.54, 3.83], [5.08, -2.31], [0.62, 3.40], [-4.15, -2.27]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.38,
                "init_y": 2.72,
                "init_a": 41.10,
                "velocity": 0.98,
                "goals": [[-0.41, -4.71], [-0.52, 5.14], [4.66, -1.17], [-0.27, 3.05]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.38,
                "init_y": 2.62,
                "init_a": 41.10,
                "velocity": 0.98,
                "goals": [[-0.41, -4.71], [-7.13, -2.88], [-6.13, 5.18], [2.97, 0.40]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.36,
                "init_y": 0.76,
                "init_a": 70.74,
                "velocity": 0.82,
                "goals": [[-7.83, -2.70], [5.04, 2.23], [6.60, 0.17], [-1.53, -4.03]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.64,
                "init_y": -1.38,
                "init_a": 131.93,
                "velocity": 1.09,
                "goal_x": 2.86,
                "goal_y": -3.32,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.62,
                "init_y": 5.33,
                "init_a": 11.41,
                "velocity": 0.95,
                "goal_x": 6.82,
                "goal_y": 2.03,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_78_walking_low(tester):
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
                "init_x": -2.05,
                "init_y": -2.12,
                "init_a": 49.28,
                "velocity": 1.15,
                "goal_x": 7.38,
                "goal_y": 4.41,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.61,
                "init_y": -3.02,
                "init_a": 49.28,
                "velocity": 1.15,
                "goal_x": 7.38,
                "goal_y": 4.41,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.05,
                "init_y": 0.86,
                "init_a": 72.42,
                "velocity": 0.91,
                "goals": [[-6.18, -1.31], [5.29, -5.20], [2.43, 1.65], [7.00, -2.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.49,
                "init_y": 1.76,
                "init_a": 72.42,
                "velocity": 0.91,
                "goals": [[-6.18, -1.31], [5.79, -2.58], [-5.72, 0.19], [2.16, -3.62]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.74,
                "init_y": -2.52,
                "init_a": -24.33,
                "velocity": 1.07,
                "goal_x": 4.90,
                "goal_y": 5.73,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.29,
                "init_y": 3.04,
                "init_a": 131.53,
                "velocity": 0.83,
                "goals": [[-5.27, -4.00], [6.19, -4.11], [6.04, 5.33], [-0.83, 2.06]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.26,
                "init_y": 1.94,
                "init_a": 101.52,
                "velocity": 1.17,
                "goal_x": -1.85,
                "goal_y": 3.97,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.53,
                "init_y": 3.38,
                "init_a": 4.71,
                "velocity": 0.84,
                "goals": [[-5.52, -3.85], [6.41, -1.97], [-0.61, -0.14], [-0.66, 5.46]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_79_stopped_low(tester):
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
                "init_x": -7.54,
                "init_y": -5.32,
                "init_a": -114.62,
                "velocity": 0.92,
                "goals": [[-7.54, -5.32], [-3.57, -2.69], [7.11, 1.89], [7.30, 1.83]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -8.44,
                "init_y": -4.88,
                "init_a": -114.62,
                "velocity": 0.92,
                "goal_x": -7.54,
                "goal_y": -5.32,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.87,
                "init_y": -3.35,
                "init_a": 109.27,
                "velocity": 0.89,
                "goal_x": 2.87,
                "goal_y": -3.35,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.87,
                "init_y": -3.25,
                "init_a": 109.27,
                "velocity": 0.89,
                "goals": [[2.87, -3.35], [-0.46, -5.58], [3.15, 5.94], [7.49, 5.83]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.31,
                "init_y": -1.97,
                "init_a": 123.27,
                "velocity": 1.05,
                "goals": [[-1.54, 5.84], [-4.23, 3.14], [1.10, -2.40], [6.97, 4.45]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.22,
                "init_y": -2.80,
                "init_a": -88.28,
                "velocity": 0.92,
                "goal_x": 4.17,
                "goal_y": -2.22,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.82,
                "init_y": 5.92,
                "init_a": 165.60,
                "velocity": 1.19,
                "goals": [[1.65, -5.65], [-2.45, -4.30], [-1.67, -3.13], [7.08, -2.09]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_80_walking_low(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.40,
                "init_y": 3.67,
                "init_a": 56.49,
                "velocity": 0.97,
                "goals": [[-6.75, -1.59], [2.36, 5.08], [-4.00, -1.94], [-6.01, 4.78]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.98,
                "init_y": -2.69,
                "init_a": 127.59,
                "velocity": 0.84,
                "goals": [[-7.11, -2.60], [1.38, 3.91], [3.45, -0.66], [6.84, -4.42]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.76,
                "init_y": -4.28,
                "init_a": 82.26,
                "velocity": 1.01,
                "goal_x": -2.97,
                "goal_y": -1.98,
                "n_actors": 7,
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
                "goals": [[-3.33, 1.95], [3.00, -2.73], [2.57, -4.41], [2.14, -4.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.13,
                "init_y": 1.02,
                "init_a": -72.97,
                "velocity": 1.03,
                "goals": [[-0.12, 1.97], [-7.03, 1.52], [4.89, 5.32], [7.78, 1.98]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_81_walking_low(tester):
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
                "init_x": 1.03,
                "init_y": 3.54,
                "init_a": 135.30,
                "velocity": 1.01,
                "goal_x": 0.92,
                "goal_y": -2.63,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.57,
                "init_y": 2.70,
                "init_a": 135.30,
                "velocity": 1.01,
                "goals": [[0.92, -2.63], [3.60, -4.85], [6.12, -0.33], [-7.31, -1.52]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.45,
                "init_y": -0.45,
                "init_a": -160.90,
                "velocity": 1.02,
                "goal_x": 1.65,
                "goal_y": 2.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.41,
                "init_y": 0.54,
                "init_a": -160.90,
                "velocity": 1.02,
                "goals": [[1.65, 2.37], [-6.62, -4.09], [-6.50, 4.20], [6.01, -0.57]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.63,
                "init_y": 1.46,
                "init_a": 108.75,
                "velocity": 1.01,
                "goals": [[0.57, -5.62], [3.13, 3.52], [-3.75, 3.84], [0.80, 2.40]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.32,
                "init_y": -5.25,
                "init_a": -11.24,
                "velocity": 1.00,
                "goals": [[1.11, 0.81], [-1.59, 0.01], [2.41, 5.26], [-3.21, -3.71]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.09,
                "init_y": -0.33,
                "init_a": 165.25,
                "velocity": 1.02,
                "goal_x": 3.04,
                "goal_y": 5.71,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.25,
                "init_y": 1.83,
                "init_a": -110.97,
                "velocity": 0.97,
                "goal_x": 0.27,
                "goal_y": 0.42,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.71,
                "init_y": 3.01,
                "init_a": -86.72,
                "velocity": 0.92,
                "goals": [[-3.66, 2.02], [6.02, -4.68], [-3.46, 5.78], [-4.87, -3.16]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_82_walking_low(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.30,
                "init_y": -4.23,
                "init_a": 125.02,
                "velocity": 1.15,
                "goals": [[-6.63, -1.13], [5.06, 2.93], [6.84, -5.08], [3.25, -3.82]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_y": -3.24,
                "init_a": 125.02,
                "velocity": 1.15,
                "goal_x": -6.63,
                "goal_y": -1.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.53,
                "init_y": 4.79,
                "init_a": 106.27,
                "velocity": 1.19,
                "goals": [[-7.03, -5.84], [3.18, 2.80], [1.38, 4.66], [-7.06, -0.06]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.90,
                "init_y": 5.17,
                "init_a": 71.36,
                "velocity": 1.00,
                "goals": [[5.11, -2.87], [3.57, -0.88], [6.93, 5.02], [1.96, 5.62]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.10,
                "init_y": 5.31,
                "init_a": -42.58,
                "velocity": 1.20,
                "goals": [[-2.11, 1.32], [2.24, -5.35], [7.54, -0.15], [-2.49, 4.89]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
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
                "goals": [[7.43, 2.56], [-0.29, -2.90], [2.90, 3.79], [4.32, -2.79]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_83_walking_low(tester):
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
                "init_x": 3.77,
                "init_y": -4.77,
                "init_a": 15.18,
                "velocity": 1.18,
                "goals": [[7.21, 2.86], [3.08, -4.31], [2.82, 5.76], [-0.67, 0.14]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.42,
                "init_y": -5.53,
                "init_a": 15.18,
                "velocity": 1.18,
                "goal_x": 7.21,
                "goal_y": 2.86,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.14,
                "init_y": -2.00,
                "init_a": 134.89,
                "velocity": 1.10,
                "goal_x": 3.62,
                "goal_y": -5.16,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.90,
                "init_y": -2.65,
                "init_a": 134.89,
                "velocity": 1.10,
                "goals": [[3.62, -5.16], [2.21, -5.73], [-0.45, 0.37], [-0.55, 5.42]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.37,
                "init_y": 2.13,
                "init_a": -155.57,
                "velocity": 0.88,
                "goal_x": 4.58,
                "goal_y": -4.29,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.52,
                "init_y": 0.26,
                "init_a": 163.92,
                "velocity": 1.11,
                "goals": [[-1.51, 4.88], [-1.85, -3.75], [-0.96, 0.70], [-1.70, -2.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.52,
                "init_y": 0.83,
                "init_a": -94.56,
                "velocity": 1.05,
                "goals": [[7.27, -3.39], [7.72, 3.77], [-4.66, 2.60], [-0.89, -5.77]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_84_stopped_low(tester):
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
                "init_x": -6.60,
                "init_y": -1.82,
                "init_a": -60.32,
                "velocity": 0.83,
                "goals": [[-6.60, -1.82], [2.93, -3.11], [3.88, 3.67], [-0.81, -1.70]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.49,
                "init_y": -2.27,
                "init_a": -60.32,
                "velocity": 0.83,
                "goals": [[-6.60, -1.82], [0.76, -3.70], [1.63, -5.04], [-0.24, -3.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.51,
                "init_y": -5.84,
                "init_a": 40.43,
                "velocity": 1.13,
                "goals": [[-1.51, -5.84], [-0.28, 5.93], [-4.94, -0.85], [1.24, -5.58]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.75,
                "init_y": -6.49,
                "init_a": 40.43,
                "velocity": 1.13,
                "goals": [[-1.51, -5.84], [-2.11, 5.84], [-5.53, -5.62], [-0.24, -2.77]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.47,
                "init_y": 0.85,
                "init_a": 158.04,
                "velocity": 1.09,
                "goal_x": 0.16,
                "goal_y": 2.64,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.11,
                "init_y": -5.78,
                "init_a": -174.61,
                "velocity": 1.13,
                "goal_x": 1.51,
                "goal_y": -0.47,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.93,
                "init_y": -3.53,
                "init_a": 41.06,
                "velocity": 0.90,
                "goal_x": -0.93,
                "goal_y": -3.28,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_85_stopped_low(tester):
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
                "init_x": -3.02,
                "init_y": 2.61,
                "init_a": -47.10,
                "velocity": 0.97,
                "goals": [[-3.02, 2.61], [7.93, -2.41], [0.45, 5.35], [-4.48, -5.40]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.70,
                "init_y": 3.34,
                "init_a": -47.10,
                "velocity": 0.97,
                "goals": [[-3.02, 2.61], [-5.32, 5.08], [-4.48, -3.94], [6.62, -0.89]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.32,
                "init_y": -2.63,
                "init_a": 167.90,
                "velocity": 0.87,
                "goal_x": -0.32,
                "goal_y": -2.63,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.16,
                "init_y": -2.08,
                "init_a": 167.90,
                "velocity": 0.87,
                "goal_x": -0.32,
                "goal_y": -2.63,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.11,
                "init_y": -2.19,
                "init_a": 110.59,
                "velocity": 1.06,
                "goal_x": -7.51,
                "goal_y": 4.66,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.30,
                "init_y": -4.00,
                "init_a": 157.21,
                "velocity": 1.19,
                "goal_x": -4.06,
                "goal_y": 5.03,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.48,
                "init_y": 2.99,
                "init_a": 107.28,
                "velocity": 0.93,
                "goals": [[2.41, -2.97], [7.01, 2.70], [3.47, -4.08], [-5.82, 1.98]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.81,
                "init_y": -3.25,
                "init_a": -14.28,
                "velocity": 0.93,
                "goals": [[6.59, 2.15], [5.95, -4.19], [-3.12, -2.24], [6.61, 5.08]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_86_walking_low(tester):
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
                "init_x": 3.08,
                "init_y": -3.00,
                "init_a": -165.80,
                "velocity": 1.03,
                "goals": [[5.91, 3.44], [-7.82, -5.15], [-0.10, 4.32], [7.11, -0.24]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.69,
                "init_y": -4.53,
                "init_a": -24.70,
                "velocity": 0.82,
                "goal_x": -7.61,
                "goal_y": 4.56,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.02,
                "init_y": -3.79,
                "init_a": -24.70,
                "velocity": 0.82,
                "goals": [[-7.61, 4.56], [-2.65, 5.40], [-0.98, -3.17], [7.00, -5.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.11,
                "init_y": -3.70,
                "init_a": 69.15,
                "velocity": 0.83,
                "goals": [[-4.90, 1.72], [-3.26, -1.94], [-2.52, 4.91], [1.19, -4.48]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.49,
                "init_y": -3.22,
                "init_a": -15.36,
                "velocity": 0.98,
                "goals": [[5.80, -4.98], [-7.53, -2.67], [2.37, 1.88], [2.70, 2.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_87_stopped_low(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.68,
                "init_y": -5.37,
                "init_a": -131.38,
                "velocity": 0.94,
                "goal_x": -3.80,
                "goal_y": -5.85,
                "n_actors": 7,
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
                "goals": [[6.33, 4.53], [-3.80, -3.86], [-4.76, 5.46], [-4.61, -2.62]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.41,
                "init_y": 4.12,
                "init_a": 32.49,
                "velocity": 1.06,
                "goals": [[6.33, 4.53], [6.09, -4.71], [-6.70, -1.10], [-5.24, -4.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.97,
                "init_y": -1.95,
                "init_a": 19.91,
                "velocity": 1.16,
                "goals": [[-1.07, -3.22], [-0.98, -1.29], [-5.87, -2.56], [-7.84, 3.26]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.80,
                "init_y": 2.58,
                "init_a": -38.15,
                "velocity": 1.04,
                "goals": [[3.74, 4.37], [2.99, 0.59], [2.53, 1.55], [3.77, 3.21]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_88_stopped_low(tester):
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
                "init_x": 1.67,
                "init_y": -1.74,
                "init_a": -63.27,
                "velocity": 0.82,
                "goals": [[1.67, -1.74], [5.98, 0.33], [-0.31, 4.53], [-7.37, -4.52]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[5.91, -4.60], [-6.97, -3.19], [7.49, -4.57], [0.95, -5.46]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.93,
                "init_y": -4.80,
                "init_a": 71.80,
                "velocity": 1.07,
                "goals": [[5.91, -4.60], [0.42, -1.49], [-2.06, 0.55], [-0.71, 4.23]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.15,
                "init_y": 2.28,
                "init_a": 149.14,
                "velocity": 0.80,
                "goal_x": -3.71,
                "goal_y": 0.39,
                "n_actors": 7,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.86,
                "init_y": 2.70,
                "init_a": -51.60,
                "velocity": 0.83,
                "goals": [[-7.69, -4.80], [2.07, -4.33], [5.79, -0.31], [-7.55, -1.00]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_89_walking_low(tester):
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
                "init_x": -2.76,
                "init_y": 0.94,
                "init_a": -82.96,
                "velocity": 0.89,
                "goals": [[0.14, -2.21], [-1.65, 5.47], [-0.25, 3.58], [4.96, -4.30]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -3.31,
                "init_y": 1.78,
                "init_a": -82.96,
                "velocity": 0.89,
                "goal_x": 0.14,
                "goal_y": -2.21,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.61,
                "init_y": 4.91,
                "init_a": 108.11,
                "velocity": 0.96,
                "goal_x": 4.90,
                "goal_y": 5.22,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.68,
                "init_y": 3.91,
                "init_a": 108.11,
                "velocity": 0.96,
                "goal_x": 4.90,
                "goal_y": 5.22,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.24,
                "init_y": 2.69,
                "init_a": -31.82,
                "velocity": 1.11,
                "goals": [[-5.11, -2.95], [7.15, -5.45], [-6.51, 5.01], [-0.84, 1.57]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.58,
                "init_y": -4.70,
                "init_a": 28.71,
                "velocity": 0.83,
                "goals": [[-0.55, -3.10], [-1.84, -0.90], [-2.61, -4.41], [7.87, -0.42]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.16,
                "init_y": 1.79,
                "init_a": 38.43,
                "velocity": 1.15,
                "goal_x": -5.70,
                "goal_y": -5.64,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.39,
                "init_y": -0.27,
                "init_a": -98.85,
                "velocity": 0.89,
                "goals": [[-2.89, -1.10], [7.32, 5.29], [-7.45, 5.78], [0.85, 1.96]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 9,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.10,
                "init_y": 5.93,
                "init_a": 97.14,
                "velocity": 1.12,
                "goals": [[-3.55, 2.27], [0.78, 0.67], [4.90, -3.79], [-3.13, -4.22]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_90_stopped_low(tester):
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
                "init_x": -0.93,
                "init_y": 2.74,
                "init_a": 64.60,
                "velocity": 0.99,
                "goals": [[-0.93, 2.74], [-3.14, -0.00], [4.22, 0.47], [-1.87, 5.79]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.99,
                "init_y": 3.73,
                "init_a": 64.60,
                "velocity": 0.99,
                "goals": [[-0.93, 2.74], [-4.68, -1.25], [1.01, 0.91], [-6.39, -3.87]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-3.13, -4.23], [-6.35, -0.90], [-1.38, 2.49], [1.04, 0.83]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-5.78, -4.44], [-2.64, -1.85], [1.06, -0.97], [3.04, 0.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.54,
                "init_y": 5.12,
                "init_a": 27.87,
                "velocity": 1.12,
                "goals": [[-0.87, -4.13], [-5.02, -3.02], [3.80, -3.05], [-1.24, -0.20]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_91_walking_low(tester):
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
                "init_x": -6.69,
                "init_y": 4.52,
                "init_a": 178.13,
                "velocity": 0.82,
                "goals": [[-0.01, -0.15], [-0.48, 2.31], [-7.43, -5.38], [-2.48, -5.06]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.61,
                "init_y": 4.42,
                "init_a": 133.09,
                "velocity": 1.18,
                "goals": [[6.72, -5.01], [-6.97, -5.11], [-4.19, 3.72], [-1.05, 5.57]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.31,
                "init_y": 5.46,
                "init_a": 16.99,
                "velocity": 1.16,
                "goals": [[5.20, -5.13], [2.82, -4.68], [-6.19, -1.74], [-3.58, -3.02]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.41,
                "init_y": 2.20,
                "init_a": -87.37,
                "velocity": 1.00,
                "goals": [[4.01, -5.30], [4.57, 2.06], [-5.42, 1.92], [1.51, 5.41]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.55,
                "init_y": 3.87,
                "init_a": -31.16,
                "velocity": 0.83,
                "goal_x": -3.96,
                "goal_y": -5.81,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_92_stopped_low(tester):
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
                "init_x": 5.47,
                "init_y": -3.71,
                "init_a": -69.69,
                "velocity": 0.98,
                "goal_x": 5.47,
                "goal_y": -3.71,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.71,
                "init_y": -4.37,
                "init_a": -69.69,
                "velocity": 0.98,
                "goals": [[5.47, -3.71], [-1.64, -2.50], [-0.17, -1.16], [1.97, 1.55]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.37,
                "init_y": 0.08,
                "init_a": 1.28,
                "velocity": 0.84,
                "goals": [[-0.37, 0.08], [4.81, -4.52], [6.43, -2.40], [-7.65, -0.56]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.35,
                "init_y": -0.14,
                "init_a": 1.28,
                "velocity": 0.84,
                "goal_x": -0.37,
                "goal_y": 0.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.59,
                "init_y": -0.39,
                "init_a": 90.21,
                "velocity": 1.16,
                "goals": [[-2.14, -4.77], [-3.40, 0.06], [5.93, -2.07], [5.08, -0.18]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.56,
                "init_y": 1.09,
                "init_a": 71.03,
                "velocity": 1.05,
                "goals": [[-4.70, -1.62], [0.80, 3.32], [5.67, -5.74], [7.38, -2.87]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.48,
                "init_y": 3.41,
                "init_a": 76.50,
                "velocity": 0.94,
                "goal_x": 2.90,
                "goal_y": -5.64,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.26,
                "init_y": 0.30,
                "init_a": 2.06,
                "velocity": 1.14,
                "goals": [[-7.29, 1.54], [-4.62, 0.88], [-3.02, 4.37], [3.31, -4.46]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -1.18,
                "init_y": -5.33,
                "init_a": 9.47,
                "velocity": 1.17,
                "goal_x": 3.51,
                "goal_y": 2.65,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_93_stopped_low(tester):
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
                "goals": [[7.19, 4.52], [-3.47, -4.29], [-4.24, -4.57], [6.55, -0.09]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 6.42,
                "init_y": 5.17,
                "init_a": 94.25,
                "velocity": 1.07,
                "goals": [[7.19, 4.52], [-1.29, -0.04], [7.16, 4.43], [-1.39, 4.15]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.63,
                "init_y": -0.21,
                "init_a": -58.74,
                "velocity": 1.12,
                "goals": [[3.63, -0.21], [2.98, -3.68], [-5.51, 2.53], [7.48, -4.00]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[6.11, 2.16], [2.90, 0.38], [4.37, -3.11], [1.67, -5.17]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.25,
                "init_y": -5.08,
                "init_a": 45.43,
                "velocity": 0.88,
                "goals": [[4.40, 5.35], [-4.43, 1.07], [-7.99, -2.70], [-2.21, 0.04]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_94_walking_low(tester):
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
                "init_x": 0.46,
                "init_y": 2.39,
                "init_a": -30.61,
                "velocity": 0.99,
                "goal_x": -1.64,
                "goal_y": 4.52,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.54,
                "init_y": 2.49,
                "init_a": -30.61,
                "velocity": 0.99,
                "goals": [[-1.64, 4.52], [-5.80, 5.28], [-5.13, -4.77], [-1.66, 4.82]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.74,
                "init_y": -1.97,
                "init_a": -98.02,
                "velocity": 1.07,
                "goals": [[4.68, -5.34], [-7.66, 5.65], [-6.53, 3.38], [5.48, -0.64]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 4.08,
                "init_y": -1.22,
                "init_a": -98.02,
                "velocity": 1.07,
                "goals": [[4.68, -5.34], [-5.07, 4.95], [-3.49, 5.15], [6.29, 3.84]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 5.24,
                "init_y": -3.02,
                "init_a": 26.55,
                "velocity": 0.95,
                "goal_x": 3.20,
                "goal_y": -3.41,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.76,
                "init_y": 1.51,
                "init_a": -97.85,
                "velocity": 1.18,
                "goal_x": 4.56,
                "goal_y": -3.14,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.11,
                "init_y": -0.67,
                "init_a": -155.03,
                "velocity": 1.15,
                "goals": [[-0.46, -0.01], [1.97, 5.34], [-3.45, -2.72], [3.44, -4.57]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_95_walking_low(tester):
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
                "init_x": -5.58,
                "init_y": -1.86,
                "init_a": 157.73,
                "velocity": 0.85,
                "goals": [[7.02, -3.76], [-1.65, 2.65], [-3.64, -1.03], [-1.27, -3.90]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.12,
                "init_y": -2.74,
                "init_a": 157.73,
                "velocity": 0.85,
                "goals": [[7.02, -3.76], [5.28, -1.55], [-7.45, -5.26], [-3.24, 5.08]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.41,
                "init_y": -3.67,
                "init_a": 137.27,
                "velocity": 1.08,
                "goals": [[-0.39, -4.50], [-5.21, -4.08], [-6.57, -5.13], [-1.77, -5.24]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.95,
                "init_y": 0.18,
                "init_a": 89.82,
                "velocity": 1.20,
                "goals": [[6.00, -3.35], [7.18, -4.79], [6.50, 2.95], [-6.45, -5.50]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.69,
                "init_y": -3.58,
                "init_a": 55.47,
                "velocity": 0.89,
                "goals": [[-3.91, -5.08], [-1.74, -5.53], [-6.61, 4.87], [6.61, 4.18]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_96_stopped_low(tester):
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
                "init_x": 1.16,
                "init_y": 3.19,
                "init_a": -13.09,
                "velocity": 1.08,
                "goal_x": 1.16,
                "goal_y": 3.19,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.16,
                "init_y": 3.15,
                "init_a": -13.09,
                "velocity": 1.08,
                "goals": [[1.16, 3.19], [0.08, 5.47], [2.85, 0.43], [-2.08, 2.18]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -5.08,
                "init_y": 0.38,
                "init_a": 39.15,
                "velocity": 1.09,
                "goals": [[-5.08, 0.38], [-1.33, -2.43], [6.91, -3.98], [-0.62, 0.10]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -4.53,
                "init_y": 1.21,
                "init_a": 39.15,
                "velocity": 1.09,
                "goal_x": -5.08,
                "goal_y": 0.38,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.64,
                "init_y": -3.32,
                "init_a": 151.53,
                "velocity": 1.05,
                "goals": [[-7.73, 3.23], [-7.47, -4.46], [6.83, 2.28], [-5.02, 5.29]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.24,
                "init_y": -2.78,
                "init_a": 87.39,
                "velocity": 1.01,
                "goal_x": -7.51,
                "goal_y": 2.53,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.61,
                "init_y": 1.70,
                "init_a": -156.35,
                "velocity": 0.98,
                "goals": [[-2.04, -1.36], [3.44, 1.97], [-0.52, 1.82], [-2.31, -2.89]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_y": -0.40,
                "init_a": -73.50,
                "velocity": 0.95,
                "goal_x": 6.30,
                "goal_y": 0.59,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.80,
                "init_y": 0.20,
                "init_a": 101.91,
                "velocity": 1.04,
                "goals": [[3.25, -5.64], [0.48, -2.47], [4.33, 0.38], [4.50, 3.20]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_97_walking_low(tester):
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
                "init_x": 6.53,
                "init_y": 0.99,
                "init_a": -160.74,
                "velocity": 0.94,
                "goals": [[2.60, -2.87], [-6.84, -3.01], [-2.14, -5.12], [0.72, 4.48]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 7.33,
                "init_y": 1.60,
                "init_a": -160.74,
                "velocity": 0.94,
                "goal_x": 2.60,
                "goal_y": -2.87,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.91,
                "init_y": -0.96,
                "init_a": 75.76,
                "velocity": 0.97,
                "goals": [[-2.32, -4.17], [2.32, 5.16], [-4.95, 2.61], [0.88, 5.30]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -2.15,
                "init_y": -0.30,
                "init_a": 75.76,
                "velocity": 0.97,
                "goals": [[-2.32, -4.17], [-1.03, -5.11], [1.41, -5.35], [-3.71, -4.46]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.39,
                "init_y": -0.57,
                "init_a": -133.27,
                "velocity": 0.99,
                "goal_x": -0.17,
                "goal_y": -4.25,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.82,
                "init_y": 1.14,
                "init_a": 82.00,
                "velocity": 0.89,
                "goal_x": -6.57,
                "goal_y": 5.79,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.71,
                "init_y": -0.66,
                "init_a": -21.13,
                "velocity": 0.87,
                "goals": [[4.26, -5.22], [0.25, 2.54], [0.19, 2.19], [-5.12, -1.89]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 0.09,
                "init_y": 3.57,
                "init_a": -103.90,
                "velocity": 0.86,
                "goal_x": -6.88,
                "goal_y": 3.07,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_98_stopped_low(tester):
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
                "init_x": -3.25,
                "init_y": -0.40,
                "init_a": 89.69,
                "velocity": 0.96,
                "goal_x": -3.25,
                "goal_y": -0.40,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.85,
                "init_y": -1.31,
                "init_a": 89.69,
                "velocity": 0.96,
                "goal_x": -3.25,
                "goal_y": -0.40,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.70,
                "init_y": -2.21,
                "init_a": -121.14,
                "velocity": 1.12,
                "goals": [[1.70, -2.21], [-3.50, -3.58], [-4.27, -5.36], [7.37, 0.91]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 2.19,
                "init_y": -1.34,
                "init_a": -121.14,
                "velocity": 1.12,
                "goals": [[1.70, -2.21], [1.40, -3.86], [-0.63, 4.14], [-2.18, 2.81]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 3.55,
                "init_y": -2.58,
                "init_a": -105.13,
                "velocity": 0.83,
                "goals": [[-1.70, 5.79], [7.01, 3.31], [6.76, 0.43], [7.94, -2.86]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.43,
                "init_y": -0.68,
                "init_a": -90.57,
                "velocity": 0.92,
                "goal_x": 2.65,
                "goal_y": 2.42,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.41,
                "init_y": -0.93,
                "init_a": -177.47,
                "velocity": 0.96,
                "goals": [[-7.36, 1.83], [1.09, -0.71], [-0.45, -0.75], [7.81, 2.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_40_child_60_test_case_99_walking_low(tester):
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
                "init_x": -0.34,
                "init_y": -2.38,
                "init_a": 91.23,
                "velocity": 0.81,
                "goals": [[-1.67, -5.92], [-1.65, 2.87], [2.61, 3.96], [-5.46, -3.65]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -0.95,
                "init_y": -3.18,
                "init_a": 91.23,
                "velocity": 0.81,
                "goal_x": -1.67,
                "goal_y": -5.92,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.13,
                "init_y": 2.66,
                "init_a": 56.12,
                "velocity": 0.87,
                "goal_x": -5.19,
                "goal_y": 0.28,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.14,
                "init_y": 2.79,
                "init_a": 56.12,
                "velocity": 0.87,
                "goal_x": -5.19,
                "goal_y": 0.28,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.26,
                "init_y": 0.24,
                "init_a": -98.86,
                "velocity": 1.02,
                "goals": [[-4.45, -2.68], [-4.12, 5.55], [-7.63, -1.45], [-7.76, 2.29]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": 1.33,
                "init_y": 1.47,
                "init_a": 171.72,
                "velocity": 0.87,
                "goals": [[-3.38, -1.67], [-3.20, 2.93], [4.35, 5.81], [2.55, -3.26]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 8,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.98,
                "init_y": 0.25,
                "init_a": -67.56,
                "velocity": 1.08,
                "goals": [[0.30, -5.34], [-2.47, -1.58], [0.87, 5.23], [3.51, 2.13]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -6.10,
                "init_y": 4.53,
                "init_a": -73.98,
                "velocity": 0.88,
                "goal_x": -0.99,
                "goal_y": -3.43,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_40_child_60_test_case_100_walking_low(tester):
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
                "init_x": 4.88,
                "init_y": 2.40,
                "init_a": 105.02,
                "velocity": 0.87,
                "goals": [[3.64, 3.27], [3.78, -3.17], [4.31, 0.37], [5.80, 1.32]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.06,
                "init_y": 2.90,
                "init_a": 132.79,
                "velocity": 0.81,
                "goals": [[-5.68, 3.83], [0.58, -2.95], [7.18, -5.33], [-4.27, -3.84]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.96,
                "init_y": 2.47,
                "init_a": 132.79,
                "velocity": 0.81,
                "goals": [[-5.68, 3.83], [1.76, 1.48], [7.18, 4.47], [-1.26, 2.87]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "init_x": -7.81,
                "init_y": -1.41,
                "init_a": 58.29,
                "velocity": 1.05,
                "goal_x": -6.62,
                "goal_y": -3.01,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.60,
                "init_y": 2.27,
                "init_a": -66.11,
                "velocity": 0.93,
                "goal_x": 4.74,
                "goal_y": -4.53,
                "n_actors": 8,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.08,
                "init_y": 2.40,
                "init_a": 42.33,
                "velocity": 0.98,
                "goals": [[-2.34, 4.03], [-4.96, -3.26], [-2.91, 2.34], [-3.68, 5.71]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
