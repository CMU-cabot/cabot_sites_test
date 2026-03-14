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

def tests_adult_50_child_50_test_case_01_stopped_high(tester):
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
                "init_x": 5.80,
                "init_y": -4.90,
                "init_a": 144.90,
                "velocity": 1.11,
                "goal_x": 5.80,
                "goal_y": -4.90,
                "n_actors": 7,
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.01,
                "init_y": 1.79,
                "init_a": -35.67,
                "velocity": 1.03,
                "goals": [[-2.01, 1.79], [7.86, 1.19], [4.14, 4.86], [-2.17, -0.31]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.79,
                "init_y": 1.16,
                "init_a": -35.67,
                "velocity": 1.03,
                "goal_x": -2.01,
                "goal_y": 1.79,
                "n_actors": 7,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.79,
                "init_y": -3.75,
                "init_a": 0.71,
                "velocity": 1.16,
                "goals": [[6.82, -5.77], [-2.31, -0.16], [4.69, -3.44], [-7.52, 4.98]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 6.04,
                "init_y": 1.64,
                "init_a": -33.61,
                "velocity": 0.85,
                "goals": [[5.80, -2.95], [6.34, 0.27], [-5.60, 0.57], [-6.68, -2.45]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_02_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.03,
                "init_y": -5.92,
                "init_a": 54.41,
                "velocity": 1.18,
                "goals": [[-2.20, -5.37], [-3.68, 1.35], [-0.34, -3.90], [-1.65, -5.98]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.08,
                "init_y": 3.95,
                "init_a": 45.97,
                "velocity": 1.05,
                "goal_x": 1.10,
                "goal_y": 2.95,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.57,
                "init_y": 1.70,
                "init_a": -149.25,
                "velocity": 1.17,
                "goal_x": -1.03,
                "goal_y": 1.93,
                "n_actors": 9,
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
                "goals": [[-3.93, 2.51], [7.10, 3.53], [-4.28, -0.90], [7.95, -1.92]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.62,
                "init_y": -3.01,
                "init_a": 43.75,
                "velocity": 0.85,
                "goal_x": 4.69,
                "goal_y": 4.01,
                "n_actors": 9,
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
                "goals": [[4.82, 1.90], [0.85, 5.71], [0.85, -3.96], [-2.15, -0.05]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.97,
                "init_y": 2.26,
                "init_a": -171.32,
                "velocity": 0.93,
                "goals": [[5.16, 5.81], [6.24, 4.18], [6.86, -1.09], [-1.33, 2.97]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_03_walking_high(tester):
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
                "goals": [[-7.21, -5.96], [7.72, 0.95], [-2.41, 2.18], [2.44, -0.50]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.51,
                "init_y": 4.59,
                "init_a": -24.94,
                "velocity": 1.11,
                "goals": [[-1.61, 1.37], [-6.11, -0.02], [3.29, -2.83], [6.44, 2.36]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.85,
                "init_y": 5.53,
                "init_a": -24.94,
                "velocity": 1.11,
                "goal_x": -1.61,
                "goal_y": 1.37,
                "n_actors": 7,
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
                "goals": [[-1.00, 5.09], [5.15, -3.03], [3.04, 5.87], [5.23, 3.31]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.71,
                "init_y": 5.82,
                "init_a": 28.16,
                "velocity": 0.87,
                "goal_x": -7.09,
                "goal_y": 2.88,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_04_walking_high(tester):
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
                "init_x": 6.10,
                "init_y": 3.69,
                "init_a": -170.80,
                "velocity": 1.10,
                "goals": [[3.93, -5.23], [6.87, 1.58], [2.97, -0.60], [6.73, 4.57]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.88,
                "init_y": 4.67,
                "init_a": -170.80,
                "velocity": 1.10,
                "goal_x": 3.93,
                "goal_y": -5.23,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.29,
                "init_y": -5.48,
                "init_a": 178.16,
                "velocity": 0.92,
                "goal_x": -3.28,
                "goal_y": -5.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.27,
                "init_y": -5.32,
                "init_a": 178.16,
                "velocity": 0.92,
                "goal_x": -3.28,
                "goal_y": -5.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.85,
                "init_y": 0.64,
                "init_a": -141.28,
                "velocity": 0.81,
                "goals": [[-6.62, -3.02], [2.04, -1.49], [-1.14, 3.83], [-5.89, 0.97]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.34,
                "init_y": 4.91,
                "init_a": 43.26,
                "velocity": 0.81,
                "goal_x": -1.59,
                "goal_y": 5.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.61,
                "init_y": 4.18,
                "init_a": 83.83,
                "velocity": 0.99,
                "goal_x": -3.91,
                "goal_y": 3.54,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.61,
                "init_y": 0.18,
                "init_a": -25.02,
                "velocity": 0.81,
                "goals": [[0.63, 1.83], [-1.15, 3.86], [3.94, 3.01], [-3.95, 0.71]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.40,
                "init_y": 5.53,
                "init_a": 113.44,
                "velocity": 0.95,
                "goals": [[5.78, 4.92], [6.00, 3.93], [4.84, 5.34], [-7.10, 0.47]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_05_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.75,
                "init_y": 3.58,
                "init_a": 23.79,
                "velocity": 0.85,
                "goals": [[2.05, 2.86], [3.23, 4.73], [2.50, -3.28], [3.02, -4.93]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.17,
                "init_y": 0.49,
                "init_a": -12.95,
                "velocity": 1.08,
                "goals": [[1.17, 0.49], [6.82, 4.96], [5.45, -0.12], [-1.86, 0.45]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[0.40, 0.79], [4.87, -3.67], [-6.01, 1.94], [2.25, 1.56]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.94,
                "init_y": -4.63,
                "init_a": 114.71,
                "velocity": 1.16,
                "goals": [[-4.39, 1.09], [6.11, 4.82], [0.34, -4.47], [1.10, 5.08]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_06_walking_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.23,
                "init_y": 6.03,
                "init_a": 115.33,
                "velocity": 1.06,
                "goal_x": -2.54,
                "goal_y": -5.99,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.75,
                "init_y": -1.75,
                "init_a": 85.54,
                "velocity": 1.06,
                "goals": [[4.82, 2.34], [-5.98, 2.63], [-1.91, -2.23], [7.20, 1.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.08,
                "init_y": 1.93,
                "init_a": 16.11,
                "velocity": 1.16,
                "goals": [[-3.12, -0.43], [-1.95, -2.97], [1.52, 0.39], [-2.02, 5.79]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.33,
                "init_y": -3.83,
                "init_a": 126.00,
                "velocity": 0.85,
                "goals": [[-5.04, -1.36], [7.13, 4.61], [3.95, -1.86], [-2.00, -4.53]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_07_walking_high(tester):
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
                "init_x": 6.33,
                "init_y": 2.19,
                "init_a": 59.94,
                "velocity": 0.92,
                "goal_x": 5.00,
                "goal_y": 4.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.14,
                "init_y": 3.18,
                "init_a": 59.94,
                "velocity": 0.92,
                "goals": [[5.00, 4.61], [5.16, 4.74], [-7.56, 4.27], [4.95, -3.90]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.07,
                "init_y": -2.22,
                "init_a": 70.22,
                "velocity": 1.00,
                "goal_x": 4.51,
                "goal_y": -1.38,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.77,
                "init_y": -1.51,
                "init_a": 70.22,
                "velocity": 1.00,
                "goals": [[4.51, -1.38], [-0.65, -0.20], [2.13, 1.68], [6.86, -2.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.92,
                "init_y": -4.91,
                "init_a": 87.02,
                "velocity": 0.97,
                "goals": [[-5.39, 1.51], [2.04, 2.38], [-5.71, -3.23], [4.01, -1.44]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.04,
                "init_y": 2.09,
                "init_a": 44.20,
                "velocity": 1.03,
                "goal_x": -2.75,
                "goal_y": 3.33,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.63,
                "init_y": -2.88,
                "init_a": -108.06,
                "velocity": 1.02,
                "goal_x": -4.87,
                "goal_y": -0.78,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.03,
                "init_y": -1.12,
                "init_a": 56.72,
                "velocity": 1.17,
                "goals": [[-6.89, 1.07], [5.56, 3.71], [2.03, -5.05], [2.40, 5.02]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.03,
                "init_y": -0.38,
                "init_a": 118.59,
                "velocity": 0.83,
                "goal_x": 1.77,
                "goal_y": -2.67,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_08_walking_high(tester):
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
                "init_x": -1.65,
                "init_y": -0.06,
                "init_a": 159.35,
                "velocity": 1.06,
                "goals": [[-4.33, 1.17], [-5.93, -3.07], [0.30, -0.88], [-5.50, 5.93]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-3.67, -5.49], [-0.63, 4.09], [-3.11, 1.18], [-5.49, -3.17]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.26,
                "init_y": 1.75,
                "init_a": 14.41,
                "velocity": 0.88,
                "goals": [[-2.16, -0.09], [7.80, 5.77], [3.91, 3.24], [-0.95, -5.88]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.42,
                "init_y": -1.79,
                "init_a": 64.83,
                "velocity": 1.20,
                "goals": [[4.49, -0.30], [2.38, -3.88], [-2.00, -5.01], [-1.34, -4.90]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.49,
                "init_y": 4.70,
                "init_a": -51.15,
                "velocity": 1.09,
                "goal_x": 7.74,
                "goal_y": 5.10,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_09_stopped_high(tester):
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
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.40,
                "init_y": 4.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goal_x": 3.40,
                "goal_y": 4.71,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.46,
                "init_y": 3.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goals": [[3.40, 4.71], [-0.13, -2.31], [6.27, -4.03], [6.63, 2.55]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.69,
                "init_y": -2.51,
                "init_a": 31.90,
                "velocity": 0.83,
                "goals": [[-0.89, 3.23], [-6.96, 2.84], [6.13, -4.80], [-7.42, -3.58]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[0.32, -3.93], [-2.96, -4.33], [-7.18, 4.18], [5.39, -2.70]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_10_walking_high(tester):
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
                "init_x": -6.29,
                "init_y": -5.58,
                "init_a": 154.03,
                "velocity": 0.81,
                "goal_x": 7.33,
                "goal_y": 2.68,
                "n_actors": 8,
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
                "goals": [[7.33, 2.68], [-0.12, 0.30], [3.20, -0.01], [-7.84, -2.05]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.34,
                "init_y": 1.29,
                "init_a": 78.72,
                "velocity": 0.94,
                "goals": [[6.95, 2.73], [0.92, 0.65], [-1.38, -1.32], [1.64, -3.07]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.47,
                "init_y": 0.69,
                "init_a": -70.82,
                "velocity": 1.15,
                "goals": [[-2.04, -4.32], [-5.62, -2.48], [4.48, 1.60], [-6.94, 2.98]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.95,
                "init_y": 5.78,
                "init_a": 74.95,
                "velocity": 0.82,
                "goals": [[4.23, -3.82], [2.19, 3.24], [3.27, 4.56], [-4.24, -5.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_11_walking_high(tester):
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
                "init_x": 3.71,
                "init_y": -2.60,
                "init_a": 150.95,
                "velocity": 1.20,
                "goal_x": 3.12,
                "goal_y": 4.98,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.53,
                "init_y": -3.17,
                "init_a": 150.95,
                "velocity": 1.20,
                "goal_x": 3.12,
                "goal_y": 4.98,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.23,
                "init_y": -1.79,
                "init_a": -56.37,
                "velocity": 0.91,
                "goal_x": 5.16,
                "goal_y": -3.74,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.72,
                "init_y": -2.65,
                "init_a": -56.37,
                "velocity": 0.91,
                "goals": [[5.16, -3.74], [0.16, -1.20], [-1.48, 1.38], [7.03, -3.29]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.62,
                "init_y": -0.48,
                "init_a": 41.59,
                "velocity": 0.87,
                "goal_x": 3.05,
                "goal_y": -0.44,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.19,
                "init_y": -4.13,
                "init_a": 45.66,
                "velocity": 1.12,
                "goals": [[-5.39, -3.91], [2.37, -2.30], [-3.62, 5.11], [6.98, 4.12]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.37,
                "init_y": -4.45,
                "init_a": 51.93,
                "velocity": 1.05,
                "goals": [[-4.40, -3.83], [-2.26, -3.13], [0.72, 1.31], [-4.82, 2.65]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.25,
                "init_y": -2.71,
                "init_a": -88.30,
                "velocity": 0.95,
                "goals": [[7.17, 4.38], [-5.72, 4.85], [-5.37, -0.51], [-7.40, 1.71]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_12_walking_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.78,
                "init_y": 0.69,
                "init_a": 5.14,
                "velocity": 0.85,
                "goal_x": 2.00,
                "goal_y": 5.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.07,
                "init_y": 4.67,
                "init_a": -122.78,
                "velocity": 0.95,
                "goals": [[0.82, 5.85], [-0.80, 5.15], [0.21, -2.99], [4.83, 2.93]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.66,
                "init_y": 5.89,
                "init_a": 127.85,
                "velocity": 1.19,
                "goals": [[7.95, 3.92], [7.00, -1.09], [3.50, 5.20], [2.51, -3.25]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.74,
                "init_y": 2.92,
                "init_a": 156.64,
                "velocity": 0.91,
                "goals": [[5.04, -0.76], [7.02, -4.41], [3.33, 3.80], [1.81, -3.43]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[4.74, 4.69], [-3.63, -1.48], [-0.33, 5.74], [-6.09, 3.40]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.68,
                "init_y": -4.72,
                "init_a": 162.11,
                "velocity": 0.83,
                "goal_x": 4.32,
                "goal_y": 3.00,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_13_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.67,
                "init_y": 4.81,
                "init_a": 164.71,
                "velocity": 0.96,
                "goals": [[5.82, -2.88], [-5.04, -5.39], [4.55, 2.47], [-2.30, -3.27]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 6.33,
                "init_y": -5.08,
                "init_a": -162.91,
                "velocity": 1.20,
                "goals": [[5.03, -2.07], [3.83, 0.68], [6.85, -4.21], [0.03, -0.65]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.18,
                "init_y": -5.60,
                "init_a": -162.91,
                "velocity": 1.20,
                "goal_x": 5.03,
                "goal_y": -2.07,
                "n_actors": 8,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.84,
                "init_y": -4.51,
                "init_a": -95.80,
                "velocity": 1.02,
                "goals": [[7.14, 1.64], [4.18, 3.59], [-2.70, 1.57], [4.29, -0.72]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.05,
                "init_y": -5.21,
                "init_a": 120.15,
                "velocity": 1.12,
                "goals": [[-2.46, -1.97], [0.80, 0.44], [7.67, 1.81], [1.41, 3.10]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_14_walking_high(tester):
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
                "init_x": 4.96,
                "init_y": -3.64,
                "init_a": -29.65,
                "velocity": 0.86,
                "goals": [[-3.91, -4.23], [7.33, 4.99], [1.00, 3.46], [-6.59, 0.50]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.65,
                "init_y": -4.35,
                "init_a": -29.65,
                "velocity": 0.86,
                "goals": [[-3.91, -4.23], [3.17, -2.04], [-3.55, -0.29], [-4.99, -5.38]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-0.96, 1.37], [5.49, 2.34], [3.00, 3.66], [4.66, 0.79]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_15_walking_high(tester):
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
                "init_x": 7.17,
                "init_y": 0.36,
                "init_a": 95.69,
                "velocity": 0.99,
                "goals": [[5.44, 1.84], [3.37, 0.41], [-1.73, 1.50], [1.44, -2.77]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 6.17,
                "init_y": 0.34,
                "init_a": 95.69,
                "velocity": 0.99,
                "goal_x": 5.44,
                "goal_y": 1.84,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.92,
                "init_y": 4.80,
                "init_a": 2.99,
                "velocity": 0.99,
                "goals": [[-4.50, -1.26], [-5.31, 2.31], [7.60, -1.30], [3.17, 2.94]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.06,
                "init_y": 4.29,
                "init_a": 2.99,
                "velocity": 0.99,
                "goal_x": -4.50,
                "goal_y": -1.26,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.58,
                "init_y": -4.36,
                "init_a": -55.50,
                "velocity": 0.91,
                "goal_x": 1.80,
                "goal_y": 1.34,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.78,
                "init_y": 3.56,
                "init_a": 174.91,
                "velocity": 1.09,
                "goal_x": -0.84,
                "goal_y": 0.85,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.30,
                "init_y": 1.15,
                "init_a": -105.55,
                "velocity": 1.01,
                "goals": [[-1.51, 1.04], [3.43, -0.13], [0.06, -3.18], [-1.93, 2.58]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -5.11,
                "init_y": -4.95,
                "init_a": -58.14,
                "velocity": 1.14,
                "goal_x": 3.89,
                "goal_y": 6.00,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.03,
                "init_y": 2.44,
                "init_a": -124.17,
                "velocity": 1.07,
                "goals": [[-5.48, -1.67], [-3.16, 4.37], [6.74, 4.29], [-0.96, -5.48]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_16_stopped_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.79,
                "init_y": -5.71,
                "init_a": -118.17,
                "velocity": 1.03,
                "goal_x": -6.55,
                "goal_y": -5.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.69,
                "init_y": 0.82,
                "init_a": 171.03,
                "velocity": 1.16,
                "goals": [[1.69, 0.82], [6.00, -4.41], [-7.48, -2.06], [3.45, -3.03]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.14,
                "init_y": -0.02,
                "init_a": 171.03,
                "velocity": 1.16,
                "goal_x": 1.69,
                "goal_y": 0.82,
                "n_actors": 9,
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
                "goals": [[2.77, 5.66], [0.34, 1.57], [-5.63, 5.59], [-5.09, -5.73]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[2.95, -1.66], [-1.77, 0.50], [2.51, 3.97], [-4.75, -0.57]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.95,
                "init_y": -5.65,
                "init_a": 13.28,
                "velocity": 0.85,
                "goals": [[-5.43, 0.96], [5.56, 3.46], [1.91, -0.60], [-3.24, -5.37]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.30,
                "init_y": 0.16,
                "init_a": -27.20,
                "velocity": 1.11,
                "goal_x": -1.50,
                "goal_y": -3.07,
                "n_actors": 9,
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

def tests_adult_50_child_50_test_case_17_walking_high(tester):
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
                "goals": [[-2.68, -5.19], [-4.31, -0.56], [5.99, -1.88], [4.72, -1.25]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.68, -5.19], [-5.80, 5.63], [-4.77, -4.92], [-2.36, -0.56]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[5.68, -0.28], [7.82, -0.99], [-6.66, 0.90], [-3.60, 3.73]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[3.73, 2.84], [-6.63, 0.28], [2.85, 5.79], [-2.43, -2.59]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_18_stopped_high(tester):
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
                "goals": [[0.31, -0.60], [-2.77, -5.67], [-4.88, -5.71], [-4.68, -0.36]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[3.35, 1.81], [-5.86, 3.85], [-6.47, 3.79], [0.82, 3.46]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.30,
                "init_y": 5.99,
                "init_a": 23.99,
                "velocity": 0.92,
                "goals": [[-3.81, -4.17], [5.84, -3.06], [7.26, 5.10], [-0.22, -4.40]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_19_walking_high(tester):
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
                "init_x": 0.31,
                "init_y": 0.91,
                "init_a": 139.11,
                "velocity": 0.99,
                "goals": [[5.86, -5.17], [1.29, 1.23], [3.99, -1.71], [-5.98, 0.20]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.58,
                "init_y": 0.45,
                "init_a": 139.11,
                "velocity": 0.99,
                "goal_x": 5.86,
                "goal_y": -5.17,
                "n_actors": 7,
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
                "goals": [[1.77, 2.95], [6.43, 4.39], [-7.38, 1.73], [1.31, -3.37]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.65,
                "init_y": 0.76,
                "init_a": 115.59,
                "velocity": 0.98,
                "goals": [[1.77, 2.95], [4.02, 4.80], [0.53, 0.14], [-2.14, 1.13]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.74,
                "init_y": -0.52,
                "init_a": -140.26,
                "velocity": 1.00,
                "goal_x": -2.49,
                "goal_y": -3.58,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.27,
                "init_y": 2.25,
                "init_a": -73.45,
                "velocity": 1.03,
                "goal_x": -1.16,
                "goal_y": -3.00,
                "n_actors": 7,
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

def tests_adult_50_child_50_test_case_20_stopped_high(tester):
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
                "goals": [[-1.01, -0.02], [3.71, 3.54], [1.32, -4.88], [1.41, -1.23]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.91,
                "init_y": -0.46,
                "init_a": 23.00,
                "velocity": 0.99,
                "goals": [[-1.01, -0.02], [2.35, 0.55], [4.60, -2.60], [-5.45, -4.42]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[1.23, -1.65], [1.84, -1.40], [1.19, -1.05], [-4.05, 0.29]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[4.04, 3.23], [0.44, 4.10], [0.50, -4.95], [-6.12, -1.10]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_21_walking_high(tester):
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
                "init_x": 0.51,
                "init_y": 0.91,
                "init_a": 74.64,
                "velocity": 0.84,
                "goals": [[5.12, -2.32], [2.19, 4.69], [-7.10, 2.57], [-5.83, 3.45]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.09,
                "init_y": 1.79,
                "init_a": 142.01,
                "velocity": 1.05,
                "goal_x": 0.13,
                "goal_y": -4.10,
                "n_actors": 8,
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
                "goals": [[-6.73, 4.83], [-6.31, 1.62], [4.91, 0.80], [5.37, -3.84]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.33,
                "init_y": 4.47,
                "init_a": 112.27,
                "velocity": 0.83,
                "goals": [[-2.19, -4.16], [2.09, -5.65], [-5.78, 4.77], [-3.67, -2.46]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[0.15, -3.75], [3.44, -5.52], [-0.25, 5.00], [1.91, 1.27]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_22_walking_high(tester):
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
                "init_x": 0.75,
                "init_y": 1.15,
                "init_a": -62.86,
                "velocity": 1.02,
                "goals": [[-0.12, -5.67], [1.87, 3.02], [-0.81, -2.41], [1.84, -2.77]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-1.87, 1.12], [-6.82, 5.53], [5.98, 5.85], [4.10, 2.17]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[6.26, -3.32], [-6.42, -4.44], [0.43, 2.45], [0.32, -5.50]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_23_walking_high(tester):
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
                "init_x": 6.27,
                "init_y": 4.46,
                "init_a": -165.93,
                "velocity": 0.86,
                "goal_x": -2.58,
                "goal_y": 4.91,
                "n_actors": 7,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.63,
                "init_y": 4.55,
                "init_a": -21.93,
                "velocity": 1.07,
                "goals": [[4.46, 0.65], [0.79, -5.41], [4.53, 4.98], [-4.95, -0.84]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.57,
                "init_y": 1.19,
                "init_a": 162.94,
                "velocity": 1.19,
                "goals": [[6.19, -3.25], [-5.83, 2.26], [-1.92, 0.84], [0.97, 0.35]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-6.87, -3.96], [1.88, -5.38], [-2.28, 3.96], [-2.23, -0.64]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.07,
                "init_y": 2.94,
                "init_a": -137.50,
                "velocity": 0.92,
                "goal_x": 1.23,
                "goal_y": -5.29,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_24_stopped_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.92,
                "init_y": 2.04,
                "init_a": 144.32,
                "velocity": 1.15,
                "goal_x": -7.24,
                "goal_y": 1.09,
                "n_actors": 7,
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
                "goals": [[3.44, -2.45], [-5.81, -2.75], [0.37, -4.33], [1.71, -1.16]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[3.44, -2.45], [7.65, 4.49], [4.54, -0.38], [-3.12, -0.42]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.55,
                "init_y": -4.09,
                "init_a": -94.57,
                "velocity": 0.90,
                "goals": [[1.02, 3.12], [-1.19, -5.20], [2.00, -1.04], [5.25, -4.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.36,
                "init_y": -2.20,
                "init_a": 147.26,
                "velocity": 0.81,
                "goal_x": -1.24,
                "goal_y": -3.52,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_25_stopped_high(tester):
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
                "goals": [[-0.45, -2.13], [6.58, -1.83], [1.28, 1.07], [-7.79, -2.91]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.83,
                "init_y": -3.75,
                "init_a": -121.25,
                "velocity": 1.04,
                "goals": [[-4.95, -2.61], [-1.27, 2.61], [4.08, 5.91], [-1.52, 4.82]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[1.13, -2.94], [2.11, 5.72], [-0.62, -2.69], [3.25, 0.48]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.39,
                "init_y": -5.56,
                "init_a": 103.82,
                "velocity": 1.15,
                "goals": [[-3.59, -5.29], [7.52, 0.02], [-3.35, 2.39], [7.17, -2.09]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_26_stopped_high(tester):
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
                "init_x": 3.65,
                "init_y": 0.78,
                "init_a": -127.80,
                "velocity": 1.12,
                "goals": [[3.65, 0.78], [6.96, -2.18], [1.13, 4.55], [1.00, 5.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.62,
                "init_y": -0.22,
                "init_a": -127.80,
                "velocity": 1.12,
                "goal_x": 3.65,
                "goal_y": 0.78,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.42,
                "init_y": 1.51,
                "init_a": 108.02,
                "velocity": 1.20,
                "goals": [[5.42, 1.51], [-2.89, 2.08], [-0.53, 3.99], [7.47, -0.57]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.65,
                "init_y": 0.87,
                "init_a": 108.02,
                "velocity": 1.20,
                "goal_x": 5.42,
                "goal_y": 1.51,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.38,
                "init_y": -4.17,
                "init_a": 108.82,
                "velocity": 1.01,
                "goal_x": 6.90,
                "goal_y": 1.37,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.02,
                "init_y": -0.17,
                "init_a": 43.03,
                "velocity": 0.95,
                "goal_x": 5.11,
                "goal_y": -3.62,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.54,
                "init_y": 4.74,
                "init_a": -41.76,
                "velocity": 0.82,
                "goals": [[0.69, -4.67], [5.47, 5.74], [-0.88, -3.43], [6.50, -4.91]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.31,
                "init_y": 1.46,
                "init_a": -126.73,
                "velocity": 0.81,
                "goals": [[0.40, 0.88], [5.99, -4.55], [-3.49, -3.86], [-1.44, 4.58]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.05,
                "init_y": 4.31,
                "init_a": 163.06,
                "velocity": 0.93,
                "goal_x": 5.92,
                "goal_y": 2.13,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_27_walking_high(tester):
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
                "goals": [[2.09, -0.92], [4.67, -3.63], [-6.22, 1.03], [7.20, 3.29]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.67,
                "init_y": 6.18,
                "init_a": -174.00,
                "velocity": 0.93,
                "goals": [[2.09, -0.92], [-1.14, 1.27], [-5.70, -0.58], [3.00, 1.14]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.09,
                "init_y": -4.75,
                "init_a": -166.89,
                "velocity": 0.80,
                "goals": [[-6.11, 2.08], [2.35, -0.37], [0.64, 2.50], [0.46, -0.46]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.54,
                "init_y": -2.62,
                "init_a": 121.59,
                "velocity": 0.97,
                "goal_x": -3.08,
                "goal_y": 0.68,
                "n_actors": 8,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.98,
                "init_y": 3.77,
                "init_a": -85.03,
                "velocity": 0.94,
                "goals": [[3.70, 0.17], [-2.87, 4.69], [2.51, 5.15], [7.07, -5.18]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_28_walking_high(tester):
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.23,
                "init_y": -2.68,
                "init_a": 0.96,
                "velocity": 0.84,
                "goals": [[7.13, 3.79], [3.05, 1.68], [-1.79, -1.65], [3.55, 5.33]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.00,
                "init_y": 1.90,
                "init_a": 6.86,
                "velocity": 0.91,
                "goals": [[5.91, 0.30], [2.38, -0.82], [-3.21, 2.33], [-7.09, 2.92]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.61,
                "init_y": 4.57,
                "init_a": -31.19,
                "velocity": 1.02,
                "goal_x": 6.86,
                "goal_y": -1.43,
                "n_actors": 8,
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
                "goals": [[-4.69, 2.62], [-5.96, 4.82], [-5.54, -4.64], [-3.72, 5.55]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 6.43,
                "init_y": -0.13,
                "init_a": -37.55,
                "velocity": 0.94,
                "goals": [[6.27, 5.21], [-6.92, -4.12], [0.23, 3.01], [5.02, 3.87]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_29_stopped_high(tester):
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
                "init_x": 3.24,
                "init_y": 3.00,
                "init_a": 103.89,
                "velocity": 1.20,
                "goals": [[3.24, 3.00], [6.36, -0.18], [-3.79, -3.98], [0.44, -4.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.23,
                "init_y": 3.12,
                "init_a": 103.89,
                "velocity": 1.20,
                "goal_x": 3.24,
                "goal_y": 3.00,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.23,
                "init_y": 0.12,
                "init_a": 142.82,
                "velocity": 0.91,
                "goal_x": -7.23,
                "goal_y": 0.12,
                "n_actors": 8,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.87,
                "init_y": 3.23,
                "init_a": 2.73,
                "velocity": 1.16,
                "goals": [[-1.94, -4.89], [6.77, 2.14], [7.61, -0.09], [-0.84, 5.70]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.53,
                "init_y": -1.32,
                "init_a": -158.25,
                "velocity": 1.07,
                "goals": [[3.86, -1.31], [4.50, -2.19], [0.71, -1.93], [-2.19, 2.51]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.81, -4.86], [-0.11, -4.18], [-6.37, -1.12], [-0.82, 0.69]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_30_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.15,
                "init_y": -1.90,
                "init_a": -14.21,
                "velocity": 1.10,
                "goals": [[-5.84, -5.51], [0.76, -4.26], [-3.88, -2.77], [-2.62, -5.09]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.03,
                "init_y": -2.47,
                "init_a": 32.18,
                "velocity": 0.98,
                "goals": [[2.42, 4.06], [-4.45, 1.99], [2.60, 4.12], [-7.68, 4.51]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.93,
                "init_y": -4.45,
                "init_a": 2.21,
                "velocity": 1.07,
                "goals": [[4.33, 1.13], [-4.37, -2.21], [-7.90, 3.02], [-0.19, 0.33]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_31_stopped_high(tester):
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
                "init_x": 7.04,
                "init_y": -2.91,
                "init_a": 172.22,
                "velocity": 1.19,
                "goal_x": 7.04,
                "goal_y": -2.91,
                "n_actors": 7,
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.59,
                "init_y": -2.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goals": [[1.59, -2.71], [-4.85, -1.14], [-0.12, -2.88], [-1.19, -5.27]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.07,
                "init_y": -4.24,
                "init_a": -170.53,
                "velocity": 1.13,
                "goals": [[3.23, 2.89], [2.38, -2.01], [0.59, 2.95], [-3.67, -2.24]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.46,
                "init_y": 4.54,
                "init_a": -162.87,
                "velocity": 1.13,
                "goals": [[7.42, -5.70], [-0.94, -0.45], [-3.41, 4.31], [-1.44, -3.90]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_32_walking_high(tester):
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
                "init_x": 2.51,
                "init_y": -1.87,
                "init_a": 76.89,
                "velocity": 1.18,
                "goals": [[-0.98, 0.29], [-5.33, 1.29], [-3.62, -5.06], [-4.69, 5.19]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.52,
                "init_y": -1.70,
                "init_a": 76.89,
                "velocity": 1.18,
                "goal_x": -0.98,
                "goal_y": 0.29,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.91,
                "init_y": -1.13,
                "init_a": 121.66,
                "velocity": 0.99,
                "goals": [[5.31, -0.65], [0.25, 3.37], [0.34, 5.12], [-2.17, 4.46]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.13,
                "init_y": -0.50,
                "init_a": 121.66,
                "velocity": 0.99,
                "goals": [[5.31, -0.65], [2.13, 4.18], [-3.02, -5.23], [3.27, 3.05]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.48,
                "init_y": -4.41,
                "init_a": 59.41,
                "velocity": 1.12,
                "goals": [[-5.68, -4.75], [-5.70, -1.36], [1.46, -3.12], [4.07, -5.06]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_33_walking_high(tester):
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
                "init_x": -4.13,
                "init_y": -1.15,
                "init_a": 76.09,
                "velocity": 1.11,
                "goal_x": -4.84,
                "goal_y": -0.47,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.65,
                "init_y": -2.03,
                "init_a": 76.09,
                "velocity": 1.11,
                "goal_x": -4.84,
                "goal_y": -0.47,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.51,
                "init_y": 5.10,
                "init_a": -59.71,
                "velocity": 0.91,
                "goal_x": -0.59,
                "goal_y": -2.55,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.48,
                "init_y": 4.88,
                "init_a": -59.71,
                "velocity": 0.91,
                "goals": [[-0.59, -2.55], [-7.05, -2.47], [-7.71, 3.23], [-7.68, -0.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -5.79,
                "init_y": 4.54,
                "init_a": 171.84,
                "velocity": 0.87,
                "goals": [[5.87, -3.53], [-0.75, -0.10], [4.96, 1.79], [6.92, -2.21]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.76,
                "init_y": 1.75,
                "init_a": -122.75,
                "velocity": 1.13,
                "goals": [[4.94, 1.43], [-3.73, 1.44], [3.79, -2.98], [6.88, -3.75]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.78,
                "init_y": -0.37,
                "init_a": -148.84,
                "velocity": 1.05,
                "goal_x": -0.41,
                "goal_y": 3.65,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.77,
                "init_y": 5.84,
                "init_a": -170.40,
                "velocity": 0.81,
                "goals": [[-4.07, 5.02], [-4.89, -1.61], [1.23, -2.72], [-1.37, -5.32]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_34_stopped_high(tester):
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
                "init_x": 7.29,
                "init_y": 0.59,
                "init_a": 37.87,
                "velocity": 0.93,
                "goals": [[7.29, 0.59], [-3.83, 2.57], [6.47, 4.19], [2.99, 0.08]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 8.22,
                "init_y": 0.24,
                "init_a": 37.87,
                "velocity": 0.93,
                "goals": [[7.29, 0.59], [-2.96, -3.19], [7.10, -0.43], [-3.51, 4.03]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.07,
                "init_y": 0.15,
                "init_a": 51.97,
                "velocity": 0.85,
                "goals": [[-0.07, 0.15], [1.88, 5.82], [6.54, -0.12], [-7.94, 3.28]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.62,
                "init_y": 0.87,
                "init_a": 51.97,
                "velocity": 0.85,
                "goals": [[-0.07, 0.15], [5.42, -0.55], [2.99, -3.60], [-2.04, -4.04]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_35_walking_high(tester):
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
                "init_x": -4.66,
                "init_y": 3.34,
                "init_a": 56.70,
                "velocity": 1.00,
                "goals": [[1.46, -2.66], [7.09, 3.62], [-5.56, 4.53], [-2.36, -2.86]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[5.96, -5.59], [-5.69, -1.60], [4.57, 2.57], [-6.07, -0.53]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-1.21, 2.63], [-3.32, -4.14], [-6.09, -5.68], [6.67, -5.17]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.07,
                "init_y": 0.76,
                "init_a": -94.02,
                "velocity": 0.91,
                "goal_x": -5.35,
                "goal_y": 1.00,
                "n_actors": 8,
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
                "goals": [[-6.34, 2.17], [4.87, -3.96], [-5.60, -5.53], [-2.82, -4.49]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_36_stopped_high(tester):
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
                "init_x": -3.70,
                "init_y": -3.49,
                "init_a": 63.37,
                "velocity": 1.05,
                "goals": [[-3.70, -3.49], [-5.19, 1.23], [-1.30, -0.63], [7.98, -1.97]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.23,
                "init_y": -2.61,
                "init_a": 63.37,
                "velocity": 1.05,
                "goals": [[-3.70, -3.49], [-4.52, -2.76], [7.84, -3.80], [-5.55, -5.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.13,
                "init_y": 5.07,
                "init_a": -14.49,
                "velocity": 0.86,
                "goal_x": -4.13,
                "goal_y": 5.07,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.71,
                "init_y": 5.98,
                "init_a": -14.49,
                "velocity": 0.86,
                "goal_x": -4.13,
                "goal_y": 5.07,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.07,
                "init_y": 3.47,
                "init_a": 142.87,
                "velocity": 1.03,
                "goal_x": 2.08,
                "goal_y": 4.91,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.22,
                "init_y": -5.94,
                "init_a": 133.05,
                "velocity": 1.16,
                "goals": [[3.43, -1.16], [0.80, -0.18], [5.51, 4.73], [7.10, -0.77]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.74,
                "init_y": -1.65,
                "init_a": -85.48,
                "velocity": 0.93,
                "goal_x": -0.60,
                "goal_y": 3.76,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_37_stopped_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.46,
                "init_y": 0.00,
                "init_a": 102.07,
                "velocity": 1.19,
                "goal_x": -5.02,
                "goal_y": 0.90,
                "n_actors": 7,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.35,
                "init_y": 2.58,
                "init_a": 59.29,
                "velocity": 0.97,
                "goals": [[-7.28, 2.94], [-4.82, -3.60], [0.17, -4.61], [-6.32, 2.90]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.91, 4.58], [5.03, -3.97], [-7.30, 3.17], [-7.55, -2.73]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.52,
                "init_y": 0.25,
                "init_a": 126.27,
                "velocity": 0.88,
                "goals": [[-7.76, -1.67], [-0.38, -0.18], [3.94, 3.71], [-3.08, 2.55]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_38_stopped_high(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.17,
                "init_y": 1.61,
                "init_a": 172.86,
                "velocity": 1.16,
                "goals": [[5.17, 1.61], [7.46, -4.67], [6.48, 4.96], [-3.84, 1.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 6.10,
                "init_y": 1.98,
                "init_a": 172.86,
                "velocity": 1.16,
                "goals": [[5.17, 1.61], [-7.29, 2.35], [1.51, -5.24], [-0.29, 5.24]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[0.17, 1.64], [-0.18, 3.26], [-1.95, 0.51], [-7.83, 5.94]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.18,
                "init_y": 2.18,
                "init_a": 59.07,
                "velocity": 0.99,
                "goals": [[2.19, -4.41], [3.12, -2.58], [-5.82, 2.95], [1.93, 0.19]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_39_walking_high(tester):
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
                "goals": [[-1.97, 5.75], [-7.30, 0.29], [0.75, 5.82], [5.77, 4.98]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.06,
                "init_y": -0.14,
                "init_a": -9.30,
                "velocity": 0.93,
                "goal_x": -0.69,
                "goal_y": 0.71,
                "n_actors": 8,
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
                "goals": [[-0.69, 0.71], [-1.14, -1.48], [3.94, -2.47], [3.47, 2.37]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.93,
                "init_y": 3.39,
                "init_a": 70.42,
                "velocity": 0.94,
                "goals": [[5.51, 0.12], [-4.20, -5.32], [7.38, -0.45], [-5.83, -3.27]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.03,
                "init_y": -4.86,
                "init_a": 75.27,
                "velocity": 0.86,
                "goal_x": -1.84,
                "goal_y": 2.33,
                "n_actors": 8,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.38,
                "init_y": -2.24,
                "init_a": 120.23,
                "velocity": 1.04,
                "goals": [[-7.85, -5.38], [-0.61, 5.75], [-5.22, -1.30], [7.05, 1.13]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_40_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.65,
                "init_y": 2.89,
                "init_a": -124.82,
                "velocity": 1.11,
                "goals": [[6.87, 3.87], [6.25, -1.73], [-0.47, 4.57], [6.13, 2.08]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-6.86, 1.23], [5.68, -2.60], [3.33, -3.33], [0.85, -4.55]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-4.59, 2.13], [-0.54, 4.91], [2.57, -2.17], [-3.28, -4.21]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_41_stopped_high(tester):
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
                "init_x": -1.93,
                "init_y": 2.39,
                "init_a": 59.77,
                "velocity": 0.91,
                "goal_x": -1.93,
                "goal_y": 2.39,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.72,
                "init_y": 1.41,
                "init_a": 59.77,
                "velocity": 0.91,
                "goals": [[-1.93, 2.39], [7.89, -2.06], [-4.46, 2.78], [-0.95, -5.92]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.94,
                "init_y": 5.24,
                "init_a": 70.36,
                "velocity": 0.90,
                "goal_x": 4.94,
                "goal_y": 5.24,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.91,
                "init_y": 5.49,
                "init_a": 70.36,
                "velocity": 0.90,
                "goals": [[4.94, 5.24], [-5.28, -3.25], [-1.21, -1.33], [-3.93, -1.63]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.66,
                "init_y": -3.23,
                "init_a": 128.60,
                "velocity": 1.03,
                "goal_x": 1.37,
                "goal_y": -0.70,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.43,
                "init_y": -3.08,
                "init_a": -131.12,
                "velocity": 1.13,
                "goal_x": 3.58,
                "goal_y": 2.53,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.41,
                "init_y": -0.24,
                "init_a": -65.68,
                "velocity": 1.18,
                "goals": [[-5.10, 3.83], [2.10, -5.07], [-1.42, -2.81], [-1.16, -3.89]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_42_stopped_high(tester):
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
                "init_x": -6.12,
                "init_y": 1.34,
                "init_a": 58.28,
                "velocity": 0.92,
                "goal_x": -6.12,
                "goal_y": 1.34,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.52,
                "init_y": 0.55,
                "init_a": 58.28,
                "velocity": 0.92,
                "goal_x": -6.12,
                "goal_y": 1.34,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.35,
                "init_y": 1.82,
                "init_a": -71.62,
                "velocity": 1.06,
                "goals": [[5.35, 1.82], [-1.90, 3.91], [-1.00, -2.99], [-0.07, 1.12]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.56,
                "init_y": 2.43,
                "init_a": -71.62,
                "velocity": 1.06,
                "goals": [[5.35, 1.82], [0.40, 2.12], [-2.03, 1.65], [-0.22, 1.69]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.62,
                "init_y": -4.47,
                "init_a": 38.86,
                "velocity": 0.83,
                "goal_x": 1.85,
                "goal_y": -0.85,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.18,
                "init_y": -2.71,
                "init_a": -90.97,
                "velocity": 0.81,
                "goal_x": -6.03,
                "goal_y": -2.63,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.70,
                "init_y": 1.95,
                "init_a": 41.99,
                "velocity": 0.86,
                "goals": [[-4.07, -3.95], [3.67, -5.35], [6.68, -4.01], [4.78, -5.00]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.77,
                "init_y": 1.42,
                "init_a": -74.87,
                "velocity": 1.07,
                "goal_x": 0.93,
                "goal_y": -2.93,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.78,
                "init_y": 1.87,
                "init_a": -25.37,
                "velocity": 0.99,
                "goals": [[-0.77, 2.38], [-2.51, -2.76], [5.75, -0.32], [-7.00, -2.38]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_43_stopped_high(tester):
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
                "goals": [[6.02, 3.19], [2.27, -3.76], [-4.67, -3.42], [7.61, -4.49]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 6.94,
                "init_y": 2.81,
                "init_a": -51.83,
                "velocity": 0.92,
                "goals": [[6.02, 3.19], [3.72, 1.64], [-1.06, 2.17], [-7.11, -1.69]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.74,
                "init_y": 5.47,
                "init_a": -5.71,
                "velocity": 0.88,
                "goals": [[0.74, 5.47], [-0.12, -5.13], [4.31, 5.51], [-3.51, 0.19]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.00,
                "init_y": -5.59,
                "init_a": -19.32,
                "velocity": 1.06,
                "goals": [[7.41, -0.38], [-0.21, 4.83], [2.11, -3.73], [-5.17, 3.87]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_44_stopped_high(tester):
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
                "init_x": -2.76,
                "init_y": -1.11,
                "init_a": 62.70,
                "velocity": 1.02,
                "goal_x": -2.76,
                "goal_y": -1.11,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.09,
                "init_y": -0.36,
                "init_a": 62.70,
                "velocity": 1.02,
                "goals": [[-2.76, -1.11], [1.24, -5.83], [6.60, -0.94], [7.29, 2.26]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.80,
                "init_y": 3.42,
                "init_a": 104.72,
                "velocity": 0.88,
                "goals": [[7.80, 3.42], [-3.58, -5.01], [-1.66, 2.68], [4.53, 5.15]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.27,
                "init_y": 4.27,
                "init_a": 104.72,
                "velocity": 0.88,
                "goals": [[7.80, 3.42], [-7.46, 2.32], [7.50, 2.24], [3.54, 2.75]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.63,
                "init_y": -1.61,
                "init_a": -32.34,
                "velocity": 1.05,
                "goals": [[4.79, 1.72], [-6.31, 1.59], [-0.69, 0.17], [6.45, -3.36]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_45_stopped_high(tester):
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
                "init_x": 3.96,
                "init_y": -5.80,
                "init_a": -169.11,
                "velocity": 1.17,
                "goals": [[3.96, -5.80], [-3.66, 4.25], [0.29, 0.09], [-3.54, 3.81]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[3.60, -3.44], [-1.00, 2.77], [-0.22, -0.09], [0.31, 2.83]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.66,
                "init_y": -3.11,
                "init_a": 101.70,
                "velocity": 1.03,
                "goal_x": 3.60,
                "goal_y": -3.44,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.42,
                "init_y": -0.42,
                "init_a": -141.34,
                "velocity": 0.91,
                "goal_x": -2.67,
                "goal_y": -1.91,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.31,
                "init_y": -1.38,
                "init_a": -167.79,
                "velocity": 0.93,
                "goals": [[7.50, 3.94], [-0.40, -4.50], [-3.34, -1.78], [-7.31, 3.78]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.51,
                "init_y": 4.39,
                "init_a": -91.71,
                "velocity": 1.11,
                "goal_x": -0.77,
                "goal_y": 3.00,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_46_stopped_high(tester):
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
                "init_x": -1.98,
                "init_y": -3.63,
                "init_a": 99.95,
                "velocity": 1.00,
                "goals": [[-1.98, -3.63], [-4.97, -2.66], [-1.06, -4.76], [6.84, 3.68]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.76,
                "init_y": -3.01,
                "init_a": 99.95,
                "velocity": 1.00,
                "goal_x": -1.98,
                "goal_y": -3.63,
                "n_actors": 7,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.64,
                "init_y": -4.07,
                "init_a": 38.68,
                "velocity": 0.97,
                "goals": [[-7.24, -3.16], [-1.49, -3.02], [-4.82, 5.67], [7.62, -1.01]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.95,
                "init_y": 1.95,
                "init_a": 19.32,
                "velocity": 0.95,
                "goals": [[-0.87, -3.60], [-5.71, -2.20], [-4.56, -5.49], [3.51, 1.85]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_47_walking_high(tester):
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
                "goals": [[5.38, -4.67], [-4.54, 1.53], [-2.46, -4.04], [-7.54, 1.41]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-6.61, 4.32], [7.61, -3.56], [4.96, 1.89], [-5.23, -0.23]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.60,
                "init_y": -4.23,
                "init_a": 35.99,
                "velocity": 1.04,
                "goals": [[3.88, -5.23], [-3.00, 1.83], [-3.44, -4.59], [5.64, -0.50]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.75,
                "init_y": -3.94,
                "init_a": -10.22,
                "velocity": 0.89,
                "goal_x": 2.55,
                "goal_y": -2.24,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.25,
                "init_y": -3.42,
                "init_a": -149.79,
                "velocity": 0.95,
                "goal_x": -3.71,
                "goal_y": 2.67,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_48_walking_high(tester):
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
                "goals": [[-5.16, -0.76], [-3.74, -2.93], [-3.91, -3.50], [6.77, -1.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.24,
                "init_y": 1.32,
                "init_a": -165.93,
                "velocity": 0.95,
                "goals": [[-6.77, -1.10], [4.19, -5.57], [-3.46, 2.61], [7.44, 1.86]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[3.86, -5.84], [3.72, -1.41], [-6.30, 5.12], [-3.62, 4.15]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_49_walking_high(tester):
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
                "goals": [[4.09, 3.57], [-2.65, 3.83], [7.97, -3.34], [2.62, 3.08]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[4.09, 3.57], [2.50, 1.28], [3.38, -3.33], [-0.42, 5.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.19,
                "init_y": 4.24,
                "init_a": 34.19,
                "velocity": 1.07,
                "goal_x": 4.49,
                "goal_y": 1.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.30,
                "init_y": 3.79,
                "init_a": 34.19,
                "velocity": 1.07,
                "goal_x": 4.49,
                "goal_y": 1.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.47,
                "init_y": 5.92,
                "init_a": -29.06,
                "velocity": 1.05,
                "goals": [[-5.51, 4.39], [-1.37, -1.97], [-1.10, -0.34], [-2.14, -0.71]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.13,
                "init_y": -0.55,
                "init_a": -22.34,
                "velocity": 0.88,
                "goals": [[-4.68, -4.74], [-6.27, -1.00], [0.92, 0.66], [-0.67, -1.74]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_50_walking_high(tester):
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
                "init_x": 0.83,
                "init_y": 3.18,
                "init_a": -24.16,
                "velocity": 1.13,
                "goals": [[0.27, -0.82], [0.44, -4.85], [-6.32, -0.48], [7.28, -0.74]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.46,
                "init_y": 2.40,
                "init_a": -24.16,
                "velocity": 1.13,
                "goal_x": 0.27,
                "goal_y": -0.82,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.74,
                "init_y": -1.10,
                "init_a": 12.39,
                "velocity": 0.85,
                "goal_x": 3.12,
                "goal_y": 1.80,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.81,
                "init_y": -0.74,
                "init_a": 12.39,
                "velocity": 0.85,
                "goals": [[3.12, 1.80], [5.34, 4.82], [3.00, -1.12], [-4.44, -2.31]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.02,
                "init_y": -4.37,
                "init_a": 126.79,
                "velocity": 0.89,
                "goal_x": -2.10,
                "goal_y": -4.06,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.36,
                "init_y": -5.69,
                "init_a": -61.18,
                "velocity": 1.13,
                "goal_x": 0.01,
                "goal_y": -1.76,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.89,
                "init_y": 0.78,
                "init_a": -133.53,
                "velocity": 1.20,
                "goals": [[-3.98, -4.85], [3.60, 1.13], [-5.74, -4.08], [3.52, 5.66]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_51_stopped_high(tester):
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
                "init_x": -2.15,
                "init_y": -3.09,
                "init_a": 39.50,
                "velocity": 0.99,
                "goals": [[-2.15, -3.09], [-0.83, 4.41], [-5.71, 3.64], [2.36, -3.88]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.15,
                "init_y": -2.09,
                "init_a": 39.50,
                "velocity": 0.99,
                "goal_x": -2.15,
                "goal_y": -3.09,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.82,
                "init_y": -0.62,
                "init_a": 137.22,
                "velocity": 1.05,
                "goal_x": -4.82,
                "goal_y": -0.62,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.70,
                "init_y": 0.37,
                "init_a": 137.22,
                "velocity": 1.05,
                "goals": [[-4.82, -0.62], [7.27, -2.07], [-6.76, -5.34], [0.10, -4.59]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.67,
                "init_y": 2.68,
                "init_a": -77.84,
                "velocity": 0.86,
                "goals": [[-0.59, 2.08], [3.91, 2.27], [-6.39, 5.53], [3.19, -0.35]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.38,
                "init_y": -0.15,
                "init_a": 86.54,
                "velocity": 0.86,
                "goals": [[0.37, -5.05], [5.01, 2.88], [-5.65, 5.32], [-0.05, 4.73]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.70,
                "init_y": -2.19,
                "init_a": 53.44,
                "velocity": 0.80,
                "goal_x": -5.93,
                "goal_y": -5.48,
                "n_actors": 8,
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

def tests_adult_50_child_50_test_case_52_stopped_high(tester):
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
                "init_x": 7.30,
                "init_y": -0.08,
                "init_a": 58.20,
                "velocity": 0.83,
                "goal_x": 7.30,
                "goal_y": -0.08,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 8.28,
                "init_y": 0.13,
                "init_a": 58.20,
                "velocity": 0.83,
                "goals": [[7.30, -0.08], [0.33, -2.58], [5.77, -0.93], [0.93, 1.12]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.56,
                "init_y": -4.03,
                "init_a": -132.32,
                "velocity": 0.97,
                "goals": [[-4.56, -4.03], [-3.07, 3.67], [-6.16, -0.96], [-5.47, -4.34]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.72,
                "init_y": -1.52,
                "init_a": 85.48,
                "velocity": 0.86,
                "goals": [[-2.39, 0.71], [1.43, -5.73], [2.73, -5.00], [-7.25, -3.41]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.94,
                "init_y": -1.14,
                "init_a": -29.67,
                "velocity": 1.15,
                "goals": [[1.08, 5.17], [-3.00, -6.00], [-1.99, 4.01], [7.37, 0.95]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.87,
                "init_y": 2.75,
                "init_a": -89.74,
                "velocity": 1.11,
                "goal_x": -2.40,
                "goal_y": -5.47,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.13,
                "init_y": -3.98,
                "init_a": 10.11,
                "velocity": 0.99,
                "goal_x": 5.88,
                "goal_y": -4.46,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_53_stopped_high(tester):
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
                "goals": [[-1.70, 4.83], [6.27, -5.00], [-3.80, -5.07], [-5.41, -4.59]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-1.70, 4.83], [-6.10, 1.95], [2.84, 0.26], [-6.80, 2.68]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.81,
                "init_y": -2.64,
                "init_a": 94.23,
                "velocity": 1.02,
                "goals": [[4.14, -2.26], [5.20, 4.96], [6.67, -0.95], [-4.23, -5.47]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_54_walking_high(tester):
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
                "goals": [[3.98, 2.75], [-1.56, -1.71], [-2.41, 3.62], [-7.33, -3.52]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.97,
                "init_y": -2.46,
                "init_a": 12.74,
                "velocity": 1.04,
                "goals": [[7.44, -4.34], [3.99, 5.45], [-3.50, -2.61], [-0.98, -4.58]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-3.58, 3.91], [-7.58, -1.69], [1.55, -2.87], [-3.43, 4.39]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.75, -1.68], [3.83, 4.16], [-0.78, 1.21], [-7.43, 2.64]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_55_stopped_high(tester):
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.44,
                "init_y": 0.31,
                "init_a": -106.77,
                "velocity": 1.10,
                "goals": [[3.68, -0.34], [4.10, -1.14], [-0.56, 5.75], [-6.91, -1.21]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.93, -0.04], [-1.33, -4.75], [-5.08, -5.22], [-1.38, -0.76]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.85,
                "init_y": 4.65,
                "init_a": -41.29,
                "velocity": 1.15,
                "goals": [[1.51, -1.57], [-2.97, -3.76], [-7.17, 0.32], [-6.56, -5.10]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.93,
                "init_y": 5.75,
                "init_a": -147.49,
                "velocity": 1.13,
                "goals": [[-7.77, -3.47], [-3.12, -0.08], [6.48, 1.78], [2.79, -1.58]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_56_stopped_high(tester):
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
                "init_x": -0.82,
                "init_y": -4.08,
                "init_a": 128.85,
                "velocity": 1.01,
                "goals": [[-0.82, -4.08], [-0.58, -0.02], [6.65, 4.12], [-6.17, 0.25]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.17,
                "init_y": -4.23,
                "init_a": 128.85,
                "velocity": 1.01,
                "goals": [[-0.82, -4.08], [-5.72, 0.95], [-5.00, -4.68], [-7.24, -1.06]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-4.30, -3.27], [-0.20, -3.93], [-2.11, 0.36], [-5.70, 5.14]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.27,
                "init_y": -2.52,
                "init_a": 34.89,
                "velocity": 1.00,
                "goal_x": -0.33,
                "goal_y": 0.68,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_57_walking_high(tester):
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
                "init_x": -3.37,
                "init_y": -0.99,
                "init_a": 105.65,
                "velocity": 1.18,
                "goals": [[-3.00, 1.72], [3.67, -1.82], [1.01, -4.99], [1.24, -5.57]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.77,
                "init_y": -1.91,
                "init_a": 105.65,
                "velocity": 1.18,
                "goals": [[-3.00, 1.72], [7.96, -3.73], [-3.30, -0.60], [-5.31, 1.65]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.14,
                "init_y": -2.97,
                "init_a": -81.96,
                "velocity": 1.11,
                "goals": [[2.34, 2.19], [3.81, 0.27], [-6.54, -1.00], [0.44, -2.28]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.95,
                "init_y": -2.38,
                "init_a": -81.96,
                "velocity": 1.11,
                "goal_x": 2.34,
                "goal_y": 2.19,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.91,
                "init_y": 1.21,
                "init_a": 92.39,
                "velocity": 1.11,
                "goals": [[-7.01, -1.92], [6.55, 3.35], [-3.34, -1.46], [-2.21, -1.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -5.93,
                "init_y": 4.20,
                "init_a": 144.29,
                "velocity": 1.07,
                "goal_x": -6.05,
                "goal_y": -3.53,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.33,
                "init_y": -2.44,
                "init_a": 64.95,
                "velocity": 1.04,
                "goal_x": 3.76,
                "goal_y": 4.39,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.67,
                "init_y": 2.07,
                "init_a": 167.06,
                "velocity": 1.19,
                "goal_x": 1.82,
                "goal_y": 2.68,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_58_stopped_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.05,
                "init_y": 1.01,
                "init_a": -130.42,
                "velocity": 1.06,
                "goal_x": 7.54,
                "goal_y": 0.13,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.86,
                "init_y": 0.29,
                "init_a": -173.85,
                "velocity": 1.18,
                "goals": [[-4.86, 0.29], [4.48, -1.52], [6.82, -0.20], [-7.77, -4.52]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.04,
                "init_y": -0.28,
                "init_a": -173.85,
                "velocity": 1.18,
                "goals": [[-4.86, 0.29], [-2.54, -5.23], [3.99, -2.23], [-1.05, -0.28]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.81,
                "init_y": -5.35,
                "init_a": -95.29,
                "velocity": 0.91,
                "goals": [[-6.92, -1.36], [-2.89, 1.75], [1.48, 0.34], [6.37, -4.92]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.50,
                "init_y": 0.29,
                "init_a": -156.39,
                "velocity": 1.11,
                "goals": [[3.94, 2.05], [-5.82, 4.79], [-4.62, 1.47], [6.35, 0.96]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_59_stopped_high(tester):
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
                "init_x": -2.03,
                "init_y": 2.97,
                "init_a": 120.09,
                "velocity": 1.04,
                "goal_x": -2.03,
                "goal_y": 2.97,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.50,
                "init_y": 2.09,
                "init_a": 120.09,
                "velocity": 1.04,
                "goals": [[-2.03, 2.97], [2.83, -2.88], [-6.74, 4.17], [-7.10, -2.83]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.88,
                "init_y": 2.24,
                "init_a": 29.74,
                "velocity": 0.95,
                "goal_x": -7.88,
                "goal_y": 2.24,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -8.30,
                "init_y": 1.33,
                "init_a": 29.74,
                "velocity": 0.95,
                "goal_x": -7.88,
                "goal_y": 2.24,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.93,
                "init_y": -1.30,
                "init_a": -85.66,
                "velocity": 0.84,
                "goals": [[2.91, 3.14], [-3.78, -0.36], [-1.30, -2.09], [0.99, -4.52]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_y": 1.52,
                "init_a": -115.22,
                "velocity": 1.08,
                "goals": [[-1.26, 5.49], [0.33, -0.85], [5.83, 5.42], [5.95, 3.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.43,
                "init_y": 3.73,
                "init_a": 2.90,
                "velocity": 1.12,
                "goal_x": -0.54,
                "goal_y": -0.27,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.56,
                "init_y": -3.81,
                "init_a": 148.06,
                "velocity": 0.86,
                "goal_x": -1.79,
                "goal_y": -3.44,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.29,
                "init_y": 5.16,
                "init_a": 104.51,
                "velocity": 1.16,
                "goals": [[2.05, -3.99], [1.05, 2.76], [-4.19, -2.55], [3.16, 1.74]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_60_walking_high(tester):
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
                "init_x": 0.64,
                "init_y": 1.29,
                "init_a": 54.76,
                "velocity": 1.11,
                "goals": [[4.38, 5.93], [-7.34, -5.74], [-0.15, 1.22], [6.31, 0.10]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.61,
                "init_y": 5.59,
                "init_a": 47.70,
                "velocity": 1.07,
                "goals": [[-3.66, -3.74], [0.05, 1.24], [1.41, 5.07], [4.70, 0.79]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -8.60,
                "init_y": 5.71,
                "init_a": 47.70,
                "velocity": 1.07,
                "goal_x": -3.66,
                "goal_y": -3.74,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.55,
                "init_y": 3.34,
                "init_a": -140.22,
                "velocity": 0.94,
                "goal_x": -5.57,
                "goal_y": -4.46,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.07,
                "init_y": -5.15,
                "init_a": -178.76,
                "velocity": 0.98,
                "goal_x": 6.53,
                "goal_y": 0.30,
                "n_actors": 8,
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
                "goals": [[3.93, -4.43], [5.36, -5.52], [5.00, 0.36], [-6.54, -1.34]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.73,
                "init_y": -1.18,
                "init_a": -77.11,
                "velocity": 0.82,
                "goals": [[5.08, -1.60], [0.78, 2.67], [4.17, -0.21], [-7.41, 5.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_61_stopped_high(tester):
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
                "goals": [[0.99, 2.09], [1.51, 2.94], [6.02, -4.86], [7.19, 0.81]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.07,
                "init_y": -3.92,
                "init_a": -110.00,
                "velocity": 0.90,
                "goal_x": 1.07,
                "goal_y": -3.92,
                "n_actors": 7,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.25,
                "init_y": 4.61,
                "init_a": -60.70,
                "velocity": 1.10,
                "goals": [[5.56, 3.38], [-4.28, 3.24], [2.08, -4.62], [-3.85, 0.79]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.56,
                "init_y": -5.40,
                "init_a": -170.98,
                "velocity": 1.11,
                "goals": [[4.71, -2.50], [-6.39, 3.48], [-6.18, -4.96], [3.15, -5.27]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_62_stopped_high(tester):
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
                "goals": [[5.98, 4.30], [-2.92, -2.31], [3.05, -1.08], [-1.84, -3.60]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 6.98,
                "init_y": 4.35,
                "init_a": -172.71,
                "velocity": 0.80,
                "goal_x": 5.98,
                "goal_y": 4.30,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.33,
                "init_y": 1.09,
                "init_a": -64.29,
                "velocity": 1.12,
                "goal_x": -1.33,
                "goal_y": 1.09,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.27,
                "init_y": 1.43,
                "init_a": -64.29,
                "velocity": 1.12,
                "goal_x": -1.33,
                "goal_y": 1.09,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.50,
                "init_y": 0.96,
                "init_a": 90.77,
                "velocity": 1.13,
                "goal_x": -6.09,
                "goal_y": -0.56,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.42,
                "init_y": -2.28,
                "init_a": -147.87,
                "velocity": 0.97,
                "goals": [[1.69, -0.46], [1.61, -2.78], [5.79, -3.00], [5.80, -5.48]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.25,
                "init_y": -1.12,
                "init_a": 14.93,
                "velocity": 1.08,
                "goals": [[3.27, 4.54], [-1.43, -5.06], [7.84, -4.70], [-5.34, 1.91]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.18,
                "init_y": 1.91,
                "init_a": -56.20,
                "velocity": 1.01,
                "goals": [[-3.15, 1.62], [5.17, -5.70], [-0.79, 2.42], [-6.56, -5.90]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_63_walking_high(tester):
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
                "init_x": -2.85,
                "init_y": 3.62,
                "init_a": -148.32,
                "velocity": 1.08,
                "goal_x": 5.86,
                "goal_y": -3.97,
                "n_actors": 7,
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.97,
                "init_y": -5.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goals": [[2.18, -1.61], [-1.87, 3.54], [-0.61, -4.43], [-5.65, 0.10]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.39,
                "init_y": 2.71,
                "init_a": -149.94,
                "velocity": 1.03,
                "goals": [[5.92, -2.47], [2.66, 1.75], [-5.55, -1.50], [-7.26, -3.74]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.36,
                "init_y": 5.09,
                "init_a": -122.65,
                "velocity": 0.95,
                "goals": [[7.50, 1.97], [-6.07, 2.00], [-6.57, -1.12], [-3.40, -2.84]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_64_walking_high(tester):
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
                "init_x": -1.11,
                "init_y": 4.39,
                "init_a": 100.23,
                "velocity": 0.82,
                "goal_x": 6.25,
                "goal_y": -5.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.15,
                "init_y": 4.66,
                "init_a": 100.23,
                "velocity": 0.82,
                "goal_x": 6.25,
                "goal_y": -5.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.02,
                "init_y": -4.35,
                "init_a": -116.00,
                "velocity": 0.93,
                "goals": [[-6.82, -1.31], [7.91, -2.53], [3.97, -0.58], [5.65, -4.60]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.73,
                "init_y": -3.64,
                "init_a": -116.00,
                "velocity": 0.93,
                "goals": [[-6.82, -1.31], [-2.49, 4.50], [-7.46, -1.87], [-2.63, -2.42]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.78,
                "init_y": 4.18,
                "init_a": 74.75,
                "velocity": 0.92,
                "goals": [[-4.24, -0.40], [-2.74, 1.82], [4.10, -4.89], [-7.78, -2.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.12,
                "init_y": 3.43,
                "init_a": 12.27,
                "velocity": 1.14,
                "goals": [[1.58, -0.89], [-3.26, 0.89], [4.91, -5.09], [-4.01, -2.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.24,
                "init_y": -4.07,
                "init_a": 27.79,
                "velocity": 0.92,
                "goal_x": 5.09,
                "goal_y": -3.04,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_65_walking_high(tester):
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
                "init_x": 4.95,
                "init_y": 4.27,
                "init_a": -173.85,
                "velocity": 1.17,
                "goals": [[6.29, 2.71], [3.33, 3.58], [0.67, 4.46], [7.98, 3.11]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.60,
                "init_y": 5.03,
                "init_a": -173.85,
                "velocity": 1.17,
                "goal_x": 6.29,
                "goal_y": 2.71,
                "n_actors": 8,
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
                "goals": [[-6.16, -4.07], [-6.31, -0.54], [2.07, 2.15], [-7.48, -0.06]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.69,
                "init_y": 0.32,
                "init_a": -124.69,
                "velocity": 0.95,
                "goal_x": -1.30,
                "goal_y": -1.65,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.80,
                "init_y": -1.29,
                "init_a": 110.40,
                "velocity": 0.83,
                "goals": [[2.35, 5.77], [-1.04, -3.54], [-3.85, 0.19], [4.83, 3.43]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.10,
                "init_y": -0.52,
                "init_a": -15.25,
                "velocity": 1.19,
                "goal_x": -4.13,
                "goal_y": 2.82,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.64,
                "init_y": -4.65,
                "init_a": 134.13,
                "velocity": 1.19,
                "goals": [[-7.18, -0.40], [7.99, -4.67], [-2.60, -4.92], [5.92, -3.05]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_66_stopped_high(tester):
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
                "init_x": -6.27,
                "init_y": -5.88,
                "init_a": -30.55,
                "velocity": 0.88,
                "goals": [[-6.27, -5.88], [-2.70, -0.63], [1.21, 0.99], [7.04, 2.96]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -5.82,
                "init_y": -4.99,
                "init_a": -30.55,
                "velocity": 0.88,
                "goal_x": -6.27,
                "goal_y": -5.88,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.27,
                "init_y": -0.65,
                "init_a": 52.54,
                "velocity": 1.02,
                "goals": [[-1.27, -0.65], [-3.54, 4.72], [-1.25, -1.87], [3.41, -1.10]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.28,
                "init_y": -0.50,
                "init_a": 52.54,
                "velocity": 1.02,
                "goal_x": -1.27,
                "goal_y": -0.65,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.42,
                "init_y": -4.77,
                "init_a": -108.03,
                "velocity": 0.98,
                "goal_x": -7.09,
                "goal_y": -5.41,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.24,
                "init_y": -2.94,
                "init_a": -71.74,
                "velocity": 0.98,
                "goal_x": 2.06,
                "goal_y": 4.29,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.13,
                "init_y": -4.25,
                "init_a": 128.99,
                "velocity": 0.83,
                "goals": [[-2.46, -4.50], [5.69, -5.10], [1.20, -2.65], [-7.34, 1.70]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_67_walking_high(tester):
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
                "goals": [[-6.86, -2.82], [3.65, 3.21], [0.58, -3.32], [3.97, -0.73]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.24,
                "init_y": 1.19,
                "init_a": -77.81,
                "velocity": 0.96,
                "goal_x": -6.86,
                "goal_y": -2.82,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.46,
                "init_y": 3.84,
                "init_a": -0.43,
                "velocity": 0.97,
                "goal_x": -7.27,
                "goal_y": -4.33,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.85,
                "init_y": 4.75,
                "init_a": -0.43,
                "velocity": 0.97,
                "goals": [[-7.27, -4.33], [3.61, -4.32], [7.60, 4.33], [1.01, -3.77]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.91,
                "init_y": 5.08,
                "init_a": 155.57,
                "velocity": 1.12,
                "goals": [[0.46, -5.31], [-7.02, -5.78], [-2.22, 1.90], [7.58, -5.20]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.32,
                "init_y": -5.01,
                "init_a": -75.09,
                "velocity": 0.81,
                "goal_x": -0.75,
                "goal_y": -1.53,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.68,
                "init_y": 1.83,
                "init_a": 58.05,
                "velocity": 0.96,
                "goals": [[-7.17, -3.52], [-5.31, 0.95], [-4.81, 5.91], [2.86, -1.36]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_68_walking_high(tester):
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
                "init_x": -0.42,
                "init_y": 2.94,
                "init_a": 22.54,
                "velocity": 0.99,
                "goal_x": -2.19,
                "goal_y": -1.06,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.33,
                "init_y": 3.60,
                "init_a": 22.54,
                "velocity": 0.99,
                "goals": [[-2.19, -1.06], [4.80, 0.06], [7.52, -2.28], [3.66, -2.03]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.62,
                "init_y": -3.74,
                "init_a": 132.75,
                "velocity": 1.04,
                "goals": [[6.64, -5.39], [-0.23, -0.89], [3.65, -4.23], [-5.71, 4.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -5.49,
                "init_y": -3.25,
                "init_a": 132.75,
                "velocity": 1.04,
                "goal_x": 6.64,
                "goal_y": -5.39,
                "n_actors": 9,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.34,
                "init_y": -4.02,
                "init_a": 52.76,
                "velocity": 1.18,
                "goals": [[-5.82, 2.39], [-0.73, 1.94], [6.34, -1.76], [-5.41, -0.07]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.40,
                "init_y": 0.32,
                "init_a": 128.88,
                "velocity": 0.83,
                "goals": [[-0.02, 0.98], [-1.78, -2.82], [4.85, -3.99], [-1.91, 2.70]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_69_stopped_high(tester):
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
                "init_x": 4.76,
                "init_y": -5.57,
                "init_a": -169.40,
                "velocity": 0.82,
                "goal_x": 4.76,
                "goal_y": -5.57,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.86,
                "init_y": -5.99,
                "init_a": -169.40,
                "velocity": 0.82,
                "goal_x": 4.76,
                "goal_y": -5.57,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.47,
                "init_y": 5.59,
                "init_a": 19.05,
                "velocity": 0.91,
                "goal_x": -3.47,
                "goal_y": 5.59,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.53,
                "init_y": 5.93,
                "init_a": 19.05,
                "velocity": 0.91,
                "goals": [[-3.47, 5.59], [7.61, -3.32], [3.63, 1.82], [-1.16, -5.00]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.94,
                "init_y": 4.92,
                "init_a": 29.78,
                "velocity": 0.87,
                "goals": [[-0.14, -4.01], [-6.61, -4.41], [3.44, -1.40], [-3.88, 3.83]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.35,
                "init_y": -2.53,
                "init_a": 154.49,
                "velocity": 0.96,
                "goal_x": 3.00,
                "goal_y": -2.26,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.31,
                "init_y": 1.90,
                "init_a": -11.53,
                "velocity": 1.20,
                "goals": [[-7.32, -3.19], [-0.60, -5.53], [-6.82, -0.43], [-4.68, -1.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_70_stopped_high(tester):
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
                "init_x": -5.48,
                "init_y": 3.20,
                "init_a": 159.87,
                "velocity": 0.95,
                "goals": [[-5.48, 3.20], [1.87, 3.54], [-0.00, -3.96], [1.16, 3.63]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.48,
                "init_y": 3.18,
                "init_a": 159.87,
                "velocity": 0.95,
                "goal_x": -5.48,
                "goal_y": 3.20,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.16,
                "init_y": -5.71,
                "init_a": -47.27,
                "velocity": 1.07,
                "goal_x": -5.16,
                "goal_y": -5.71,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.16,
                "init_y": -5.72,
                "init_a": -47.27,
                "velocity": 1.07,
                "goals": [[-5.16, -5.71], [-7.82, 1.47], [7.52, -5.06], [1.61, -2.70]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.54,
                "init_y": -0.16,
                "init_a": -104.65,
                "velocity": 0.81,
                "goal_x": 5.39,
                "goal_y": -0.02,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.89,
                "init_y": 3.95,
                "init_a": -29.62,
                "velocity": 1.12,
                "goal_x": 2.76,
                "goal_y": 5.18,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.19,
                "init_y": -2.26,
                "init_a": -62.34,
                "velocity": 1.10,
                "goals": [[-3.56, 5.83], [-6.41, -1.14], [2.68, 5.89], [1.89, -5.20]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_71_walking_high(tester):
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
                "init_x": 6.33,
                "init_y": 3.33,
                "init_a": 82.77,
                "velocity": 0.84,
                "goals": [[2.02, 5.48], [3.80, -1.17], [-0.72, -4.66], [-3.21, -2.78]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.48,
                "init_y": 3.86,
                "init_a": 82.77,
                "velocity": 0.84,
                "goals": [[2.02, 5.48], [3.93, -5.89], [-7.82, 4.31], [-5.48, -2.65]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[7.75, -2.83], [2.25, -2.71], [-2.38, -4.09], [-2.04, -2.43]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.06,
                "init_y": -3.79,
                "init_a": 29.01,
                "velocity": 0.91,
                "goals": [[-1.23, -1.95], [-2.96, 0.87], [7.62, 3.79], [5.25, -2.02]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.28,
                "init_y": 3.82,
                "init_a": -168.74,
                "velocity": 0.92,
                "goal_x": -5.27,
                "goal_y": -4.64,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.00,
                "init_y": 0.57,
                "init_a": -179.50,
                "velocity": 1.15,
                "goal_x": -5.73,
                "goal_y": 1.57,
                "n_actors": 8,
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

def tests_adult_50_child_50_test_case_72_stopped_high(tester):
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
                "init_x": -3.50,
                "init_y": -2.26,
                "init_a": 130.02,
                "velocity": 0.86,
                "goals": [[-3.50, -2.26], [2.53, 5.52], [-2.60, 1.50], [-6.99, 0.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.75,
                "init_y": -3.64,
                "init_a": 140.74,
                "velocity": 1.00,
                "goal_x": -6.75,
                "goal_y": -3.64,
                "n_actors": 9,
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
                "goals": [[-6.75, -3.64], [-7.05, 1.39], [5.62, -5.39], [-4.20, 1.59]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.26,
                "init_y": 0.55,
                "init_a": -106.96,
                "velocity": 0.99,
                "goals": [[1.67, -3.64], [5.08, -4.20], [5.18, -0.02], [2.59, -3.59]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.66,
                "init_y": -0.02,
                "init_a": 92.18,
                "velocity": 1.07,
                "goal_x": 1.75,
                "goal_y": -0.05,
                "n_actors": 9,
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
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.89,
                "init_y": 4.37,
                "init_a": 67.29,
                "velocity": 0.81,
                "goal_x": -6.96,
                "goal_y": 2.72,
                "n_actors": 9,
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
                "goals": [[1.53, -0.48], [-7.94, 2.70], [-7.37, 0.55], [4.67, -3.24]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_73_walking_high(tester):
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
                "goals": [[-7.16, 0.16], [1.64, -3.73], [0.45, -4.20], [-5.25, 3.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-7.16, 0.16], [5.63, -4.43], [-3.90, -3.45], [-6.15, -1.91]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-6.72, -4.09], [3.72, -2.06], [-5.80, -3.96], [-4.99, 3.91]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_74_walking_high(tester):
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
                "goals": [[-6.21, 2.48], [-7.96, -1.63], [-2.74, -3.85], [0.37, 5.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.70,
                "init_y": 5.00,
                "init_a": 59.38,
                "velocity": 0.83,
                "goals": [[2.09, 3.91], [1.79, -2.38], [6.19, -5.63], [-7.11, 5.38]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.03,
                "init_y": 4.06,
                "init_a": 59.38,
                "velocity": 0.83,
                "goals": [[2.09, 3.91], [3.04, -1.18], [0.38, 0.98], [4.52, 1.72]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[7.20, -2.26], [-4.14, 2.45], [-4.07, -0.94], [-2.67, 5.11]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_75_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.87,
                "init_y": -1.43,
                "init_a": 168.66,
                "velocity": 0.84,
                "goals": [[1.56, -2.16], [4.80, 3.20], [5.45, -0.36], [-6.45, 5.63]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -5.66,
                "init_y": 1.49,
                "init_a": 20.31,
                "velocity": 1.08,
                "goal_x": -5.66,
                "goal_y": 1.49,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.20,
                "init_y": 0.64,
                "init_a": 20.31,
                "velocity": 1.08,
                "goals": [[-5.66, 1.49], [-1.34, 0.47], [-0.54, 1.75], [7.17, -0.88]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.05,
                "init_y": 2.62,
                "init_a": 63.46,
                "velocity": 0.88,
                "goals": [[-4.47, -5.62], [3.43, -1.57], [6.20, -5.09], [4.99, -3.88]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.41,
                "init_y": 3.26,
                "init_a": -171.37,
                "velocity": 1.14,
                "goal_x": -6.55,
                "goal_y": 0.80,
                "n_actors": 9,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.24,
                "init_y": 2.69,
                "init_a": 113.86,
                "velocity": 0.84,
                "goals": [[3.58, 4.22], [-3.11, -4.74], [-1.62, 0.63], [3.09, 5.86]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_76_stopped_high(tester):
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
                "init_x": 5.08,
                "init_y": -3.31,
                "init_a": -107.43,
                "velocity": 1.03,
                "goal_x": 5.08,
                "goal_y": -3.31,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.78,
                "init_y": -2.36,
                "init_a": -107.43,
                "velocity": 1.03,
                "goals": [[5.08, -3.31], [5.75, -0.37], [-0.20, -4.87], [2.27, -1.42]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.12,
                "init_y": -2.17,
                "init_a": 49.74,
                "velocity": 1.12,
                "goal_x": 4.12,
                "goal_y": -2.17,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.25,
                "init_y": -3.16,
                "init_a": 49.74,
                "velocity": 1.12,
                "goals": [[4.12, -2.17], [-6.06, -3.99], [-0.70, 0.14], [-2.92, 1.18]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.91,
                "init_y": 5.47,
                "init_a": -147.17,
                "velocity": 0.99,
                "goals": [[1.40, -4.32], [3.67, 3.93], [-3.59, 0.59], [6.49, -4.89]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.94,
                "init_y": -1.27,
                "init_a": 143.26,
                "velocity": 1.07,
                "goal_x": -0.12,
                "goal_y": -5.71,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.35,
                "init_y": -2.49,
                "init_a": 117.15,
                "velocity": 0.81,
                "goal_x": -6.45,
                "goal_y": 1.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.61,
                "init_y": -2.48,
                "init_a": 70.00,
                "velocity": 1.04,
                "goal_x": -2.79,
                "goal_y": 5.21,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.42,
                "init_y": 1.96,
                "init_a": -85.06,
                "velocity": 1.10,
                "goals": [[-7.96, -4.40], [5.86, -4.80], [-4.69, 3.13], [-4.49, 2.85]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_77_walking_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.74,
                "init_y": -1.36,
                "init_a": 21.62,
                "velocity": 1.09,
                "goal_x": 3.54,
                "goal_y": 3.83,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.38,
                "init_y": 2.72,
                "init_a": 41.10,
                "velocity": 0.98,
                "goal_x": -0.41,
                "goal_y": -4.71,
                "n_actors": 7,
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
                "goals": [[-0.41, -4.71], [6.63, -4.22], [-0.80, 3.25], [5.41, -3.84]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-7.83, -2.70], [-5.59, 2.76], [2.56, 3.64], [0.96, -3.28]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.64,
                "init_y": -1.38,
                "init_a": 131.93,
                "velocity": 1.09,
                "goals": [[2.86, -3.32], [6.80, 2.93], [2.81, -4.52], [-1.49, -0.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_78_stopped_high(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.86,
                "init_y": -0.75,
                "init_a": -5.96,
                "velocity": 1.13,
                "goals": [[-2.86, -0.75], [4.88, 4.67], [2.38, 2.07], [-4.34, 3.51]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.84,
                "init_y": -0.57,
                "init_a": -5.96,
                "velocity": 1.13,
                "goals": [[-2.86, -0.75], [-3.27, -2.50], [-5.78, -1.79], [-2.62, -1.11]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[1.28, 2.97], [4.09, 5.00], [6.03, 5.43], [-3.04, -5.24]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_79_walking_high(tester):
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
                "init_x": -2.38,
                "init_y": 0.75,
                "init_a": -22.80,
                "velocity": 1.08,
                "goals": [[-3.51, -2.41], [2.58, 5.37], [-1.98, -5.07], [0.80, -2.09]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.41,
                "init_y": 0.98,
                "init_a": -22.80,
                "velocity": 1.08,
                "goals": [[-3.51, -2.41], [0.88, -1.45], [7.06, 3.30], [-1.25, -2.06]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.54,
                "init_y": -4.61,
                "init_a": 87.89,
                "velocity": 0.89,
                "goals": [[-1.48, 0.97], [-5.58, -4.53], [-0.14, 2.40], [-7.25, -4.36]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.07,
                "init_y": 0.80,
                "init_a": -131.31,
                "velocity": 0.94,
                "goal_x": 4.55,
                "goal_y": 3.13,
                "n_actors": 9,
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
                "goals": [[4.62, 1.01], [-2.08, -1.12], [-5.58, -3.57], [5.11, 2.31]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_80_stopped_high(tester):
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
                "init_x": -6.45,
                "init_y": -3.35,
                "init_a": -144.16,
                "velocity": 0.82,
                "goals": [[-6.45, -3.35], [4.08, -2.69], [0.90, 2.70], [1.67, 3.02]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.11,
                "init_y": -4.10,
                "init_a": -144.16,
                "velocity": 0.82,
                "goal_x": -6.45,
                "goal_y": -3.35,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.43,
                "init_y": 4.67,
                "init_a": -42.80,
                "velocity": 1.11,
                "goals": [[2.43, 4.67], [-5.59, 4.97], [-3.03, 5.95], [1.43, -4.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.28,
                "init_y": 5.20,
                "init_a": -42.80,
                "velocity": 1.11,
                "goals": [[2.43, 4.67], [4.81, 6.00], [-2.39, -0.21], [-0.62, -5.21]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.82,
                "init_y": 4.40,
                "init_a": -82.79,
                "velocity": 0.92,
                "goals": [[3.44, 5.40], [7.24, 1.95], [6.00, -3.32], [0.14, 3.79]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.96,
                "init_y": 2.58,
                "init_a": -26.58,
                "velocity": 0.87,
                "goal_x": 6.46,
                "goal_y": 2.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.70,
                "init_y": 1.83,
                "init_a": -81.47,
                "velocity": 1.15,
                "goal_x": -0.51,
                "goal_y": -1.76,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.16,
                "init_y": 1.19,
                "init_a": -37.19,
                "velocity": 0.86,
                "goal_x": -5.04,
                "goal_y": 3.70,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.89,
                "init_y": -5.73,
                "init_a": 35.40,
                "velocity": 1.16,
                "goal_x": -3.32,
                "goal_y": -2.47,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_81_stopped_high(tester):
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
                "init_x": -0.01,
                "init_y": 0.38,
                "init_a": 132.14,
                "velocity": 0.82,
                "goal_x": -0.01,
                "goal_y": 0.38,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.30,
                "init_y": 1.33,
                "init_a": 132.14,
                "velocity": 0.82,
                "goals": [[-0.01, 0.38], [-3.34, -4.28], [6.55, -4.67], [-2.42, -2.48]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.39,
                "init_y": -2.87,
                "init_a": 32.91,
                "velocity": 1.03,
                "goals": [[6.90, -2.01], [1.08, -3.66], [5.06, -3.97], [-5.81, -5.71]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.94,
                "init_y": -0.32,
                "init_a": 79.63,
                "velocity": 1.15,
                "goal_x": 2.01,
                "goal_y": -4.61,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.23,
                "init_y": -3.67,
                "init_a": -46.76,
                "velocity": 1.10,
                "goals": [[3.27, 5.92], [2.62, 4.04], [7.15, -5.74], [-7.18, -0.26]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_82_stopped_high(tester):
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
                "init_x": 7.16,
                "init_y": 0.99,
                "init_a": -126.75,
                "velocity": 1.12,
                "goal_x": 7.16,
                "goal_y": 0.99,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.43,
                "init_y": 1.95,
                "init_a": -126.75,
                "velocity": 1.12,
                "goals": [[7.16, 0.99], [1.14, 5.93], [6.97, 2.11], [4.28, -4.73]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.48,
                "init_y": -0.45,
                "init_a": -27.87,
                "velocity": 1.01,
                "goals": [[2.48, -0.45], [-6.70, 5.45], [6.58, 2.42], [5.14, -5.77]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.48,
                "init_y": -0.53,
                "init_a": -27.87,
                "velocity": 1.01,
                "goal_x": 2.48,
                "goal_y": -0.45,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.39,
                "init_y": 5.84,
                "init_a": -164.92,
                "velocity": 0.86,
                "goal_x": -2.83,
                "goal_y": 5.96,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.23,
                "init_y": -2.81,
                "init_a": -24.50,
                "velocity": 1.18,
                "goals": [[0.90, -0.63], [-1.50, 2.78], [2.91, 0.17], [-7.53, 5.45]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.30,
                "init_y": 2.35,
                "init_a": 40.39,
                "velocity": 0.81,
                "goal_x": -5.44,
                "goal_y": -0.74,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_83_stopped_high(tester):
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
                "init_x": 0.90,
                "init_y": 3.69,
                "init_a": 93.93,
                "velocity": 0.99,
                "goals": [[0.90, 3.69], [7.90, -2.24], [5.55, 5.33], [-6.73, -2.19]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.05,
                "init_y": 4.21,
                "init_a": 93.93,
                "velocity": 0.99,
                "goals": [[0.90, 3.69], [6.32, 1.26], [2.66, -2.17], [-5.76, -5.13]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.38,
                "init_y": 1.35,
                "init_a": 11.15,
                "velocity": 1.18,
                "goals": [[7.38, 1.35], [-5.57, 0.58], [6.27, -4.48], [-1.30, -4.72]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.13,
                "init_y": -0.33,
                "init_a": 96.01,
                "velocity": 1.08,
                "goal_x": 3.75,
                "goal_y": 4.82,
                "n_actors": 8,
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
                "goals": [[2.01, -2.50], [-1.57, -2.29], [-6.96, 3.29], [-0.88, -4.27]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_84_stopped_high(tester):
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
                "init_x": -6.60,
                "init_y": -1.82,
                "init_a": -60.32,
                "velocity": 0.83,
                "goal_x": -6.60,
                "goal_y": -1.82,
                "n_actors": 7,
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
                "goals": [[-6.60, -1.82], [-2.42, -1.95], [1.73, -2.70], [1.38, -3.68]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_y": -5.84,
                "init_a": 40.43,
                "velocity": 1.13,
                "goal_x": -1.51,
                "goal_y": -5.84,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.75,
                "init_y": -6.49,
                "init_a": 40.43,
                "velocity": 1.13,
                "goal_x": -1.51,
                "goal_y": -5.84,
                "n_actors": 7,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.11,
                "init_y": -5.78,
                "init_a": -174.61,
                "velocity": 1.13,
                "goals": [[1.51, -0.47], [-2.72, 0.53], [7.98, 0.82], [7.17, -3.76]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.93,
                "init_y": -3.53,
                "init_a": 41.06,
                "velocity": 0.90,
                "goals": [[-0.93, -3.28], [-0.24, -1.03], [-2.54, 1.70], [7.79, 5.87]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_85_walking_high(tester):
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
                "init_x": 0.65,
                "init_y": -2.16,
                "init_a": -27.93,
                "velocity": 1.01,
                "goals": [[-3.10, -5.21], [6.23, 5.94], [-4.40, 2.11], [-4.04, -1.36]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.45,
                "init_y": -3.63,
                "init_a": 63.92,
                "velocity": 0.81,
                "goal_x": -2.52,
                "goal_y": -2.69,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.67,
                "init_y": -3.01,
                "init_a": 63.92,
                "velocity": 0.81,
                "goals": [[-2.52, -2.69], [-6.71, -1.67], [2.72, 2.91], [5.22, 0.68]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.09,
                "init_y": 0.42,
                "init_a": 171.14,
                "velocity": 0.93,
                "goal_x": 4.55,
                "goal_y": -3.90,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.64,
                "init_y": 5.97,
                "init_a": -69.34,
                "velocity": 0.95,
                "goals": [[-7.63, -0.95], [-2.62, 0.44], [-6.69, -1.41], [2.36, 1.84]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.77,
                "init_y": 2.27,
                "init_a": 168.69,
                "velocity": 1.04,
                "goals": [[-5.39, 5.53], [3.71, 2.19], [3.61, 3.65], [-2.15, 3.60]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_86_stopped_high(tester):
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
                "init_x": 4.14,
                "init_y": -3.34,
                "init_a": -44.50,
                "velocity": 0.91,
                "goals": [[4.14, -3.34], [4.16, -3.98], [5.04, -4.22], [-2.97, -5.64]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.15,
                "init_y": -2.34,
                "init_a": -44.50,
                "velocity": 0.91,
                "goal_x": 4.14,
                "goal_y": -3.34,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.69,
                "init_y": -2.12,
                "init_a": -138.46,
                "velocity": 1.05,
                "goal_x": 0.69,
                "goal_y": -2.12,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.96,
                "init_y": -3.09,
                "init_a": -138.46,
                "velocity": 1.05,
                "goals": [[0.69, -2.12], [-0.93, 4.89], [-4.86, -0.86], [-3.74, -3.77]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -4.06,
                "init_y": -4.46,
                "init_a": -159.80,
                "velocity": 0.98,
                "goals": [[-0.77, -4.85], [-5.00, 2.50], [-3.30, 3.88], [5.51, 5.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.48,
                "init_y": 3.58,
                "init_a": 47.01,
                "velocity": 1.05,
                "goals": [[3.36, -5.42], [4.98, -1.35], [0.97, 2.34], [-5.02, 5.88]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.56,
                "init_y": 1.66,
                "init_a": -109.24,
                "velocity": 0.80,
                "goal_x": -2.36,
                "goal_y": 3.39,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.50,
                "init_y": 4.98,
                "init_a": 81.01,
                "velocity": 1.16,
                "goal_x": 0.39,
                "goal_y": -3.17,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_87_walking_high(tester):
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
                "init_x": -7.10,
                "init_y": -2.35,
                "init_a": -3.83,
                "velocity": 1.08,
                "goals": [[5.78, 0.95], [1.94, 1.22], [5.10, 5.28], [-7.14, 0.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -8.00,
                "init_y": -1.91,
                "init_a": -3.83,
                "velocity": 1.08,
                "goal_x": 5.78,
                "goal_y": 0.95,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.26,
                "init_y": 5.03,
                "init_a": 23.30,
                "velocity": 0.92,
                "goals": [[5.24, -3.04], [-1.55, 5.36], [-3.68, -0.92], [-6.80, 1.81]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.26,
                "init_y": 5.11,
                "init_a": 23.30,
                "velocity": 0.92,
                "goal_x": 5.24,
                "goal_y": -3.04,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.07,
                "init_y": 5.51,
                "init_a": 143.54,
                "velocity": 0.99,
                "goals": [[4.63, 5.51], [1.60, -3.62], [6.08, -2.87], [-5.19, -2.21]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.58,
                "init_y": 3.11,
                "init_a": -42.04,
                "velocity": 0.83,
                "goal_x": -4.66,
                "goal_y": -0.15,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.41,
                "init_y": -2.16,
                "init_a": -86.81,
                "velocity": 1.14,
                "goal_x": 1.29,
                "goal_y": 4.51,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.35,
                "init_y": -3.91,
                "init_a": 11.04,
                "velocity": 0.92,
                "goals": [[-4.62, 5.58], [-3.15, -2.84], [5.62, 1.12], [-6.97, -1.50]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_88_walking_high(tester):
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
                "init_x": 5.98,
                "init_y": 5.18,
                "init_a": -52.22,
                "velocity": 1.14,
                "goal_x": 2.02,
                "goal_y": -4.36,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.14,
                "init_y": 4.63,
                "init_a": -52.22,
                "velocity": 1.14,
                "goal_x": 2.02,
                "goal_y": -4.36,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.93,
                "init_y": 2.39,
                "init_a": -88.80,
                "velocity": 0.97,
                "goal_x": -6.64,
                "goal_y": -0.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.79,
                "init_y": 2.89,
                "init_a": -88.80,
                "velocity": 0.97,
                "goals": [[-6.64, -0.16], [-4.46, 2.68], [5.67, 3.70], [-4.06, 1.76]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.32,
                "init_y": -2.58,
                "init_a": -126.36,
                "velocity": 0.88,
                "goals": [[-0.34, -0.49], [-0.13, -3.95], [5.46, -5.46], [1.13, 2.08]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.59,
                "init_y": 0.46,
                "init_a": -80.05,
                "velocity": 1.08,
                "goals": [[-6.28, -0.19], [-2.90, 0.58], [6.63, 0.67], [6.91, 5.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.52,
                "init_y": -1.11,
                "init_a": 147.69,
                "velocity": 0.99,
                "goals": [[-1.04, 1.45], [7.90, -0.93], [1.04, 1.62], [-3.88, -2.66]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.08,
                "init_y": -0.56,
                "init_a": 135.74,
                "velocity": 1.04,
                "goal_x": -4.79,
                "goal_y": 5.44,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.71,
                "init_y": -4.46,
                "init_a": 159.18,
                "velocity": 1.03,
                "goal_x": -6.25,
                "goal_y": 5.48,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_89_stopped_high(tester):
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
                "goals": [[-1.11, 2.39], [5.09, -0.05], [-5.26, -0.91], [7.40, 4.66]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.99,
                "init_y": -0.92,
                "init_a": -171.32,
                "velocity": 0.93,
                "goals": [[-4.99, -0.89], [-5.16, -2.24], [-5.94, -2.05], [-4.82, 3.04]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.10,
                "init_y": 1.38,
                "init_a": -62.99,
                "velocity": 0.83,
                "goals": [[7.40, 5.93], [-4.52, 4.22], [-1.70, -0.09], [5.90, 0.44]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_90_walking_high(tester):
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
                "init_x": -0.06,
                "init_y": -4.18,
                "init_a": 49.74,
                "velocity": 0.81,
                "goal_x": 1.85,
                "goal_y": 5.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.48,
                "init_y": -3.27,
                "init_a": 49.74,
                "velocity": 0.81,
                "goal_x": 1.85,
                "goal_y": 5.05,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.53,
                "init_y": 2.85,
                "init_a": 27.25,
                "velocity": 1.05,
                "goal_x": -0.40,
                "goal_y": 4.33,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.86,
                "init_y": 1.91,
                "init_a": 27.25,
                "velocity": 1.05,
                "goals": [[-0.40, 4.33], [-1.83, 0.29], [-1.10, 2.13], [6.05, -4.21]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.42,
                "init_y": 4.08,
                "init_a": 70.96,
                "velocity": 1.19,
                "goal_x": 3.17,
                "goal_y": 3.62,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.31,
                "init_y": 4.82,
                "init_a": -44.84,
                "velocity": 1.19,
                "goals": [[-2.52, -3.74], [0.70, -3.80], [2.27, 1.14], [-3.71, 2.94]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.12,
                "init_y": -0.16,
                "init_a": 78.87,
                "velocity": 1.18,
                "goals": [[2.79, -2.65], [-3.66, 0.97], [5.97, -1.13], [1.37, 3.13]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -3.32,
                "init_y": -1.51,
                "init_a": 113.99,
                "velocity": 1.14,
                "goal_x": -1.04,
                "goal_y": 5.11,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.85,
                "init_y": -4.16,
                "init_a": -100.60,
                "velocity": 0.96,
                "goals": [[-3.15, 4.15], [-1.88, 1.29], [7.96, -5.96], [-3.41, 5.32]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_91_stopped_high(tester):
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
                "init_x": -6.96,
                "init_y": 4.39,
                "init_a": -13.42,
                "velocity": 0.93,
                "goal_x": -6.96,
                "goal_y": 4.39,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.96,
                "init_y": 4.50,
                "init_a": -13.42,
                "velocity": 0.93,
                "goals": [[-6.96, 4.39], [6.13, -5.00], [-6.32, 0.76], [-0.59, -0.97]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.51,
                "init_y": 4.82,
                "init_a": -48.16,
                "velocity": 1.02,
                "goal_x": -2.51,
                "goal_y": 4.82,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.51,
                "init_y": 4.69,
                "init_a": -48.16,
                "velocity": 1.02,
                "goal_x": -2.51,
                "goal_y": 4.82,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.75,
                "init_y": -2.91,
                "init_a": -108.30,
                "velocity": 1.20,
                "goals": [[-0.91, -0.02], [2.68, 5.31], [6.81, 1.32], [-7.29, -3.32]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -2.70,
                "init_y": -4.45,
                "init_a": 61.84,
                "velocity": 1.11,
                "goals": [[-6.31, 5.32], [-3.20, -3.97], [-5.53, 4.88], [5.80, 1.99]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.58,
                "init_y": -4.65,
                "init_a": 113.75,
                "velocity": 0.99,
                "goal_x": 4.20,
                "goal_y": 0.75,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_50_child_50_test_case_92_stopped_high(tester):
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
                "init_x": 5.47,
                "init_y": -3.71,
                "init_a": -69.69,
                "velocity": 0.98,
                "goals": [[5.47, -3.71], [6.86, 3.87], [-6.22, 2.36], [4.34, 5.46]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.71,
                "init_y": -4.37,
                "init_a": -69.69,
                "velocity": 0.98,
                "goal_x": 5.47,
                "goal_y": -3.71,
                "n_actors": 9,
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
                "goals": [[-0.37, 0.08], [-5.27, 1.52], [-4.77, 1.30], [6.18, -1.32]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.59,
                "init_y": -0.39,
                "init_a": 90.21,
                "velocity": 1.16,
                "goal_x": -2.14,
                "goal_y": -4.77,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.56,
                "init_y": 1.09,
                "init_a": 71.03,
                "velocity": 1.05,
                "goal_x": -4.70,
                "goal_y": -1.62,
                "n_actors": 9,
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
                "goals": [[-7.29, 1.54], [-6.56, -4.28], [-0.06, -4.29], [1.69, 5.45]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.18,
                "init_y": -5.33,
                "init_a": 9.47,
                "velocity": 1.17,
                "goals": [[3.51, 2.65], [7.20, -4.56], [3.71, -4.81], [7.25, -2.51]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_93_stopped_high(tester):
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
                "init_x": 7.19,
                "init_y": 4.52,
                "init_a": 94.25,
                "velocity": 1.07,
                "goal_x": 7.19,
                "goal_y": 4.52,
                "n_actors": 9,
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
                "goals": [[7.19, 4.52], [-2.54, 3.08], [-1.04, 0.04], [-2.79, -0.05]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.12,
                "init_y": 3.41,
                "init_a": -123.42,
                "velocity": 1.03,
                "goal_x": 6.11,
                "goal_y": 2.16,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.17,
                "init_y": 0.19,
                "init_a": -115.45,
                "velocity": 1.12,
                "goals": [[1.08, 5.96], [4.13, -6.00], [-5.39, -3.08], [6.50, 2.93]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.25,
                "init_y": -5.08,
                "init_a": 45.43,
                "velocity": 0.88,
                "goals": [[4.40, 5.35], [1.31, 0.67], [0.64, -1.63], [7.74, -0.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.59,
                "init_y": 0.53,
                "init_a": 16.78,
                "velocity": 1.02,
                "goals": [[6.53, 0.29], [2.78, 5.99], [2.78, 5.28], [-6.84, 5.71]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_94_walking_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.54,
                "init_y": 2.49,
                "init_a": -30.61,
                "velocity": 0.99,
                "goal_x": -1.64,
                "goal_y": 4.52,
                "n_actors": 7,
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
                "goals": [[4.68, -5.34], [-0.83, 0.81], [-5.40, 3.49], [0.30, -1.27]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.08,
                "init_y": -1.22,
                "init_a": -98.02,
                "velocity": 1.07,
                "goal_x": 4.68,
                "goal_y": -5.34,
                "n_actors": 7,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.76,
                "init_y": 1.51,
                "init_a": -97.85,
                "velocity": 1.18,
                "goals": [[4.56, -3.14], [-3.53, 4.73], [7.10, 3.44], [-4.44, -2.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.11,
                "init_y": -0.67,
                "init_a": -155.03,
                "velocity": 1.15,
                "goals": [[-0.46, -0.01], [-1.45, -3.63], [7.15, -5.31], [7.61, 3.43]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_95_stopped_high(tester):
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
                "init_x": -6.54,
                "init_y": -0.30,
                "init_a": 117.02,
                "velocity": 0.80,
                "goal_x": -6.54,
                "goal_y": -0.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.74,
                "init_y": -0.90,
                "init_a": 117.02,
                "velocity": 0.80,
                "goal_x": -6.54,
                "goal_y": -0.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.81,
                "init_y": -0.97,
                "init_a": 177.66,
                "velocity": 1.08,
                "goal_x": -6.81,
                "goal_y": -0.97,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.68,
                "init_y": -0.48,
                "init_a": 177.66,
                "velocity": 1.08,
                "goals": [[-6.81, -0.97], [7.75, -5.06], [4.06, 5.38], [-3.83, -0.97]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 7.57,
                "init_y": -4.10,
                "init_a": -113.57,
                "velocity": 1.03,
                "goals": [[0.55, -4.29], [2.47, -0.35], [-2.85, 4.19], [3.87, 1.67]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -0.46,
                "init_y": -2.46,
                "init_a": -1.37,
                "velocity": 0.86,
                "goal_x": 7.35,
                "goal_y": -2.17,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.18,
                "init_y": -5.24,
                "init_a": -30.90,
                "velocity": 1.11,
                "goals": [[4.81, -1.33], [-2.27, -3.98], [2.78, -1.95], [-7.85, -0.45]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 3.28,
                "init_y": 0.47,
                "init_a": -116.73,
                "velocity": 0.82,
                "goals": [[-5.06, -5.11], [0.63, -5.71], [-3.84, -5.14], [6.73, -2.75]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_96_stopped_high(tester):
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
                "init_x": 1.16,
                "init_y": 3.19,
                "init_a": -13.09,
                "velocity": 1.08,
                "goals": [[1.16, 3.19], [1.78, 1.33], [3.84, 0.57], [4.44, -0.55]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 0.16,
                "init_y": 3.15,
                "init_a": -13.09,
                "velocity": 1.08,
                "goal_x": 1.16,
                "goal_y": 3.19,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.08,
                "init_y": 0.38,
                "init_a": 39.15,
                "velocity": 1.09,
                "goal_x": -5.08,
                "goal_y": 0.38,
                "n_actors": 9,
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
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.64,
                "init_y": -3.32,
                "init_a": 151.53,
                "velocity": 1.05,
                "goal_x": -7.73,
                "goal_y": 3.23,
                "n_actors": 9,
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
                "goals": [[-2.04, -1.36], [-7.79, 3.42], [-6.81, -0.68], [-5.24, -3.50]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -6.06,
                "init_y": -0.40,
                "init_a": -73.50,
                "velocity": 0.95,
                "goals": [[6.30, 0.59], [-5.87, -4.18], [0.08, 0.32], [3.57, -2.53]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 1.80,
                "init_y": 0.20,
                "init_a": 101.91,
                "velocity": 1.04,
                "goals": [[3.25, -5.64], [6.92, -5.06], [2.19, 1.13], [-1.12, -1.53]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_97_walking_high(tester):
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
                "init_x": 6.53,
                "init_y": 0.99,
                "init_a": -160.74,
                "velocity": 0.94,
                "goal_x": 2.60,
                "goal_y": -2.87,
                "n_actors": 8,
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
                "goals": [[-2.32, -4.17], [-5.98, -3.92], [7.78, 4.09], [-3.54, -0.97]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.32, -4.17], [3.37, 5.04], [1.51, 5.87], [-0.19, 0.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.82,
                "init_y": 1.14,
                "init_a": 82.00,
                "velocity": 0.89,
                "goals": [[-6.57, 5.79], [-2.20, 2.91], [1.23, 2.56], [5.43, -3.83]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 4.71,
                "init_y": -0.66,
                "init_a": -21.13,
                "velocity": 0.87,
                "goals": [[4.26, -5.22], [-6.88, 2.00], [5.85, 2.19], [-7.99, -5.22]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_98_stopped_high(tester):
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
                "init_x": -3.25,
                "init_y": -0.40,
                "init_a": 89.69,
                "velocity": 0.96,
                "goals": [[-3.25, -0.40], [1.32, -1.77], [2.94, -4.55], [0.16, 2.94]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[1.70, -2.21], [-6.36, -4.17], [-4.68, 3.39], [-3.50, 2.72]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 2.19,
                "init_y": -1.34,
                "init_a": -121.14,
                "velocity": 1.12,
                "goal_x": 1.70,
                "goal_y": -2.21,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.55,
                "init_y": -2.58,
                "init_a": -105.13,
                "velocity": 0.83,
                "goal_x": -1.70,
                "goal_y": 5.79,
                "n_actors": 7,
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
                "goals": [[-7.36, 1.83], [6.68, 1.65], [4.35, -0.48], [-6.06, -4.14]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_99_stopped_high(tester):
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
                "init_x": 0.77,
                "init_y": 2.15,
                "init_a": -109.88,
                "velocity": 0.91,
                "goal_x": 0.77,
                "goal_y": 2.15,
                "n_actors": 7,
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.65,
                "init_y": -6.58,
                "init_a": 94.01,
                "velocity": 0.99,
                "goals": [[7.24, -5.77], [0.98, -5.52], [-0.56, -2.88], [0.84, 0.27]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.32,
                "init_y": -0.58,
                "init_a": -131.71,
                "velocity": 1.19,
                "goals": [[4.83, -4.33], [-4.92, 3.31], [-0.17, 0.86], [0.53, -0.64]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -1.35,
                "init_y": 0.48,
                "init_a": -56.40,
                "velocity": 1.13,
                "goals": [[1.13, 1.01], [-6.79, -3.80], [-4.42, -3.35], [0.85, 4.55]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_50_child_50_test_case_100_walking_high(tester):
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
                "goals": [[3.64, 3.27], [3.32, -5.64], [6.41, -1.81], [-5.55, 3.71]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": 5.46,
                "init_y": 3.21,
                "init_a": 105.02,
                "velocity": 0.87,
                "goals": [[3.64, 3.27], [4.22, 4.68], [1.17, 2.90], [6.89, -1.17]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "init_x": -7.06,
                "init_y": 2.90,
                "init_a": 132.79,
                "velocity": 0.81,
                "goals": [[-5.68, 3.83], [-1.53, 0.29], [3.20, 0.80], [-5.10, -5.32]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-5.68, 3.83], [7.08, 0.40], [-7.56, 5.50], [-0.22, 3.63]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
