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

def tests_adult_30_child_70_test_case_01_walking_high(tester):
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
                "init_x": 6.39,
                "init_y": -4.27,
                "init_a": -80.12,
                "velocity": 0.85,
                "goals": [[1.33, -4.88], [-1.46, 2.89], [5.37, -2.48], [4.45, -0.04]],
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
                "init_x": 7.39,
                "init_y": -4.30,
                "init_a": -80.12,
                "velocity": 0.85,
                "goals": [[1.33, -4.88], [-7.34, -1.69], [-4.32, 0.33], [0.00, 2.89]],
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
                "init_x": 0.16,
                "init_y": -3.10,
                "init_a": 178.69,
                "velocity": 0.82,
                "goal_x": -7.55,
                "goal_y": 0.68,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.85,
                "init_y": -2.37,
                "init_a": 178.69,
                "velocity": 0.82,
                "goal_x": -7.55,
                "goal_y": 0.68,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.33,
                "init_y": -2.26,
                "init_a": 91.14,
                "velocity": 1.11,
                "goals": [[2.39, -5.04], [-1.78, 3.94], [5.03, 5.29], [-1.15, 1.45]],
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
                "init_x": 5.72,
                "init_y": 3.87,
                "init_a": 157.68,
                "velocity": 1.06,
                "goals": [[3.06, 1.47], [-4.91, 3.49], [5.81, 4.08], [1.88, 2.22]],
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
                "init_x": 3.20,
                "init_y": -5.14,
                "init_a": -25.90,
                "velocity": 0.84,
                "goal_x": -6.90,
                "goal_y": -2.24,
                "n_actors": 9,
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
                "goals": [[-0.07, 0.32], [4.78, 3.00], [3.23, 0.70], [-7.15, -3.94]],
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
                "init_x": 4.83,
                "init_y": -3.85,
                "init_a": -164.71,
                "velocity": 0.84,
                "goals": [[4.23, -5.21], [4.99, -2.68], [1.04, -0.87], [-6.40, 0.51]],
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

def tests_adult_30_child_70_test_case_02_stopped_high(tester):
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
                "init_x": -2.20,
                "init_y": -5.37,
                "init_a": 54.41,
                "velocity": 1.18,
                "goals": [[-2.20, -5.37], [-5.29, 4.66], [-6.46, 1.90], [-0.27, -5.96]],
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
                "init_x": -3.03,
                "init_y": -5.92,
                "init_a": 54.41,
                "velocity": 1.18,
                "goals": [[-2.20, -5.37], [4.72, -0.24], [2.14, -4.33], [-2.11, 3.89]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.08,
                "init_y": 3.95,
                "init_a": 45.97,
                "velocity": 1.05,
                "goals": [[1.10, 2.95], [-4.41, -5.97], [2.49, -0.12], [-0.92, -5.19]],
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
                "init_x": -5.57,
                "init_y": 1.70,
                "init_a": -149.25,
                "velocity": 1.17,
                "goals": [[-1.03, 1.93], [-2.75, -0.93], [-6.21, -3.01], [5.68, -3.53]],
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
                "init_x": -2.35,
                "init_y": -1.49,
                "init_a": 152.80,
                "velocity": 0.99,
                "goals": [[-3.93, 2.51], [3.74, 2.49], [2.24, 5.59], [-1.29, 5.10]],
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
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.39,
                "init_y": -3.38,
                "init_a": 158.02,
                "velocity": 0.91,
                "goal_x": 4.82,
                "goal_y": 1.90,
                "n_actors": 9,
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
                "goals": [[5.16, 5.81], [-5.06, 5.64], [-6.76, -5.27], [6.67, -3.71]],
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

def tests_adult_30_child_70_test_case_03_stopped_high(tester):
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
                "init_x": -5.67,
                "init_y": 3.62,
                "init_a": 170.56,
                "velocity": 0.94,
                "goals": [[-5.67, 3.62], [2.94, 0.23], [-2.68, 3.03], [5.41, -2.58]],
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
                "init_x": -4.82,
                "init_y": 3.09,
                "init_a": 170.56,
                "velocity": 0.94,
                "goals": [[-5.67, 3.62], [-6.61, 0.46], [1.66, 5.04], [-3.55, -2.92]],
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
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.33,
                "init_y": -3.36,
                "init_a": -41.49,
                "velocity": 1.12,
                "goal_x": -2.68,
                "goal_y": -2.60,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.61,
                "init_y": -4.93,
                "init_a": 35.45,
                "velocity": 1.18,
                "goals": [[4.60, 0.13], [-0.24, -1.68], [-0.07, -4.39], [6.66, 3.70]],
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
                "init_x": -0.99,
                "init_y": 4.32,
                "init_a": 98.92,
                "velocity": 1.19,
                "goals": [[-3.83, 4.86], [-3.90, 2.60], [-6.61, -2.30], [2.36, 1.89]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.82,
                "init_y": -1.69,
                "init_a": 83.20,
                "velocity": 0.95,
                "goals": [[1.67, 2.40], [1.09, -2.68], [2.65, 4.30], [-5.68, 4.89]],
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

def tests_adult_30_child_70_test_case_04_walking_high(tester):
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
                "goals": [[3.93, -5.23], [1.64, -0.45], [1.13, 1.71], [6.96, -0.91]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.29,
                "init_y": -5.48,
                "init_a": 178.16,
                "velocity": 0.92,
                "goals": [[-3.28, -5.61], [-4.94, -3.28], [1.62, -2.89], [7.15, -2.26]],
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
                "init_x": -1.27,
                "init_y": -5.32,
                "init_a": 178.16,
                "velocity": 0.92,
                "goals": [[-3.28, -5.61], [-2.72, -2.46], [1.98, -4.46], [-5.29, 1.60]],
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
                "init_x": 4.85,
                "init_y": 0.64,
                "init_a": -141.28,
                "velocity": 0.81,
                "goals": [[-6.62, -3.02], [-3.29, 5.61], [6.73, 2.72], [-7.81, 0.23]],
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
                "init_x": 5.34,
                "init_y": 4.91,
                "init_a": 43.26,
                "velocity": 0.81,
                "goals": [[-1.59, 5.37], [0.28, -5.02], [0.30, 4.60], [-5.15, 3.77]],
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
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.61,
                "init_y": 0.18,
                "init_a": -25.02,
                "velocity": 0.81,
                "goal_x": 0.63,
                "goal_y": 1.83,
                "n_actors": 9,
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
                "goals": [[5.78, 4.92], [-2.72, -2.92], [-0.52, 1.56], [-4.87, -1.76]],
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

def tests_adult_30_child_70_test_case_05_walking_high(tester):
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
                "init_x": 5.89,
                "init_y": -5.54,
                "init_a": 94.19,
                "velocity": 1.09,
                "goals": [[-0.81, -2.77], [4.59, -3.96], [5.01, -5.13], [-2.18, -2.34]],
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
                "init_x": 4.97,
                "init_y": -5.94,
                "init_a": 94.19,
                "velocity": 1.09,
                "goal_x": -0.81,
                "goal_y": -2.77,
                "n_actors": 9,
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
                "goals": [[-2.10, 1.87], [7.60, 0.68], [1.69, 5.28], [-1.12, 1.93]],
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
                "goals": [[2.12, 4.59], [5.13, -5.17], [-1.83, -4.46], [-0.35, 1.62]],
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
                "init_x": -7.19,
                "init_y": -5.29,
                "init_a": -2.85,
                "velocity": 1.07,
                "goals": [[-2.98, 5.35], [-2.77, -5.26], [0.77, 2.27], [4.57, 3.93]],
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
                "init_x": -1.05,
                "init_y": -1.04,
                "init_a": 14.46,
                "velocity": 0.95,
                "goals": [[-0.44, 1.54], [-7.31, 1.84], [-6.90, 3.08], [-4.88, 1.88]],
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
                "init_x": -2.66,
                "init_y": 4.36,
                "init_a": 118.49,
                "velocity": 1.02,
                "goals": [[0.77, -2.60], [0.70, 3.14], [7.47, -1.08], [-7.93, 5.41]],
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
                "init_x": -5.02,
                "init_y": -5.00,
                "init_a": -71.24,
                "velocity": 1.11,
                "goal_x": -6.75,
                "goal_y": 5.18,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_06_stopped_high(tester):
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
                "init_x": -7.86,
                "init_y": 4.64,
                "init_a": 9.91,
                "velocity": 0.98,
                "goals": [[-7.86, 4.64], [-7.00, -5.20], [-4.16, -4.16], [-2.94, -0.20]],
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
                "init_x": -8.51,
                "init_y": 3.87,
                "init_a": 9.91,
                "velocity": 0.98,
                "goals": [[-7.86, 4.64], [-4.93, 4.18], [-6.17, 5.03], [-4.96, 5.99]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.57,
                "init_y": -2.61,
                "init_a": -101.85,
                "velocity": 1.02,
                "goals": [[5.29, -1.92], [-5.46, 3.87], [6.29, 3.07], [-5.80, 3.59]],
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
                "init_x": -0.30,
                "init_y": 0.50,
                "init_a": 130.30,
                "velocity": 0.85,
                "goal_x": -7.86,
                "goal_y": 0.86,
                "n_actors": 8,
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
                "goals": [[3.97, 2.26], [-6.89, 0.76], [-0.23, 1.76], [-0.51, 0.82]],
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
                "init_x": -7.47,
                "init_y": -3.98,
                "init_a": -140.24,
                "velocity": 1.15,
                "goals": [[1.27, -5.58], [1.08, 0.93], [-3.17, 1.38], [-7.87, -2.65]],
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
                "init_x": 1.59,
                "init_y": 1.66,
                "init_a": -167.53,
                "velocity": 1.16,
                "goal_x": -3.61,
                "goal_y": 3.59,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_07_walking_high(tester):
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
                "init_x": 6.33,
                "init_y": 2.19,
                "init_a": 59.94,
                "velocity": 0.92,
                "goals": [[5.00, 4.61], [5.28, -2.42], [5.06, 1.81], [5.41, -0.05]],
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
                "init_x": 6.14,
                "init_y": 3.18,
                "init_a": 59.94,
                "velocity": 0.92,
                "goal_x": 5.00,
                "goal_y": 4.61,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.07,
                "init_y": -2.22,
                "init_a": 70.22,
                "velocity": 1.00,
                "goals": [[4.51, -1.38], [-3.95, 3.29], [3.02, 5.14], [7.73, -3.16]],
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
                "init_x": -7.77,
                "init_y": -1.51,
                "init_a": 70.22,
                "velocity": 1.00,
                "goal_x": 4.51,
                "goal_y": -1.38,
                "n_actors": 9,
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
                "goals": [[-5.39, 1.51], [3.05, -2.68], [-3.06, -3.95], [-1.34, -0.48]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.63,
                "init_y": -2.88,
                "init_a": -108.06,
                "velocity": 1.02,
                "goals": [[-4.87, -0.78], [6.40, -3.80], [2.75, 2.10], [7.50, 1.24]],
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
                "init_x": 4.03,
                "init_y": -1.12,
                "init_a": 56.72,
                "velocity": 1.17,
                "goals": [[-6.89, 1.07], [5.39, -4.60], [5.32, 0.44], [0.34, -2.95]],
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
                "init_x": -1.03,
                "init_y": -0.38,
                "init_a": 118.59,
                "velocity": 0.83,
                "goals": [[1.77, -2.67], [-2.32, 1.59], [-3.64, -3.63], [3.50, -1.31]],
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

def tests_adult_30_child_70_test_case_08_walking_high(tester):
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
                "goals": [[-4.33, 1.17], [0.09, 5.85], [4.80, -1.89], [-3.02, -2.18]],
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
                "init_x": -0.80,
                "init_y": -0.57,
                "init_a": 159.35,
                "velocity": 1.06,
                "goals": [[-4.33, 1.17], [6.36, -0.44], [-7.97, 5.89], [7.85, -5.54]],
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
                "goals": [[-3.67, -5.49], [-6.46, 2.10], [2.83, -5.88], [-2.04, 1.97]],
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
                "goals": [[-2.16, -0.09], [7.19, 0.86], [4.64, -4.03], [6.48, -3.83]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.39,
                "init_y": -4.02,
                "init_a": 3.39,
                "velocity": 1.06,
                "goals": [[5.03, -0.86], [-4.61, -2.32], [6.33, -5.12], [-4.98, 3.07]],
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
                "goals": [[7.74, 5.10], [-2.51, -0.64], [-5.14, 3.86], [-5.87, -2.10]],
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

def tests_adult_30_child_70_test_case_09_walking_high(tester):
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
                "init_x": 2.07,
                "init_y": -1.47,
                "init_a": 84.23,
                "velocity": 1.03,
                "goals": [[7.80, -5.23], [4.85, -1.79], [-1.39, -3.88], [3.19, -2.95]],
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
                "init_x": 1.08,
                "init_y": -1.33,
                "init_a": 84.23,
                "velocity": 1.03,
                "goals": [[7.80, -5.23], [-2.32, 5.05], [-4.40, -4.44], [1.42, -3.84]],
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
                "init_x": 3.45,
                "init_y": 5.76,
                "init_a": 81.82,
                "velocity": 1.02,
                "goal_x": -6.60,
                "goal_y": -5.68,
                "n_actors": 8,
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
                "goals": [[-6.60, -5.68], [3.24, -1.70], [0.22, 4.24], [4.00, -2.35]],
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
                "goals": [[-0.68, 0.10], [5.41, 3.91], [3.34, -4.83], [1.38, -0.57]],
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
                "init_x": 7.74,
                "init_y": 5.33,
                "init_a": -174.77,
                "velocity": 0.99,
                "goals": [[-0.67, 2.02], [6.37, 5.47], [4.08, -1.80], [-5.49, 0.68]],
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

def tests_adult_30_child_70_test_case_10_stopped_high(tester):
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
                "goals": [[5.20, -4.57], [1.83, -2.15], [-5.34, 2.12], [5.24, 3.52]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.86,
                "init_y": 5.62,
                "init_a": -172.39,
                "velocity": 0.92,
                "goals": [[-5.86, 5.62], [-7.71, 2.62], [-5.17, 5.19], [3.95, -0.84]],
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
                "init_x": -5.79,
                "init_y": 6.61,
                "init_a": -172.39,
                "velocity": 0.92,
                "goals": [[-5.86, 5.62], [-2.66, -3.13], [5.62, 2.94], [7.21, 0.70]],
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
                "goals": [[2.28, 0.07], [-6.60, -5.72], [-7.51, 1.35], [-1.91, -2.44]],
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
                "init_x": 7.45,
                "init_y": -1.80,
                "init_a": 42.77,
                "velocity": 1.07,
                "goal_x": -2.14,
                "goal_y": -4.66,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.05,
                "init_y": -2.21,
                "init_a": -157.14,
                "velocity": 0.82,
                "goals": [[-0.85, -2.97], [-5.98, -5.91], [-3.04, 2.33], [3.78, -4.65]],
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

def tests_adult_30_child_70_test_case_11_walking_high(tester):
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
                "init_x": 3.71,
                "init_y": -2.60,
                "init_a": 150.95,
                "velocity": 1.20,
                "goals": [[3.12, 4.98], [2.13, -2.82], [7.46, 4.83], [-5.89, 3.30]],
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
                "init_x": 4.53,
                "init_y": -3.17,
                "init_a": 150.95,
                "velocity": 1.20,
                "goals": [[3.12, 4.98], [0.42, 2.73], [1.23, -5.48], [6.26, -2.11]],
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
                "goals": [[5.16, -3.74], [4.03, -1.09], [2.37, 0.44], [5.89, -0.48]],
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
                "init_x": -3.62,
                "init_y": -0.48,
                "init_a": 41.59,
                "velocity": 0.87,
                "goals": [[3.05, -0.44], [6.81, -4.69], [0.34, -2.36], [-6.62, 4.90]],
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
                "init_x": 6.19,
                "init_y": -4.13,
                "init_a": 45.66,
                "velocity": 1.12,
                "goal_x": -5.39,
                "goal_y": -3.91,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.37,
                "init_y": -4.45,
                "init_a": 51.93,
                "velocity": 1.05,
                "goal_x": -4.40,
                "goal_y": -3.83,
                "n_actors": 8,
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
                "goals": [[7.17, 4.38], [6.32, -5.19], [-1.33, 3.05], [1.35, 3.54]],
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

def tests_adult_30_child_70_test_case_12_walking_high(tester):
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
                "init_x": -2.52,
                "init_y": -0.28,
                "init_a": 5.14,
                "velocity": 0.85,
                "goals": [[2.00, 5.30], [6.96, -4.47], [-4.04, -0.18], [-1.30, 1.40]],
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
                "init_x": -2.78,
                "init_y": 0.69,
                "init_a": 5.14,
                "velocity": 0.85,
                "goals": [[2.00, 5.30], [-3.05, -1.41], [6.20, 2.89], [2.85, 5.79]],
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
                "init_x": -2.07,
                "init_y": 4.67,
                "init_a": -122.78,
                "velocity": 0.95,
                "goals": [[0.82, 5.85], [5.45, 3.33], [-4.24, 3.51], [-0.10, 3.53]],
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
                "goals": [[7.95, 3.92], [0.26, -4.64], [2.26, 4.64], [-2.41, -1.53]],
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
                "goals": [[5.04, -0.76], [5.71, 2.41], [-4.96, -4.46], [-7.62, -4.48]],
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
                "init_x": 4.35,
                "init_y": 5.06,
                "init_a": -95.34,
                "velocity": 0.86,
                "goal_x": 4.74,
                "goal_y": 4.69,
                "n_actors": 8,
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

def tests_adult_30_child_70_test_case_13_stopped_high(tester):
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
                "goals": [[-5.18, -3.02], [-1.99, 1.12], [5.55, -1.88], [6.37, 3.01]],
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
                "init_x": -4.97,
                "init_y": -4.00,
                "init_a": -128.85,
                "velocity": 1.14,
                "goals": [[-5.18, -3.02], [-4.59, -2.42], [-6.35, -0.78], [-7.52, -4.34]],
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
                "init_x": 2.06,
                "init_y": 1.40,
                "init_a": 106.78,
                "velocity": 0.88,
                "goals": [[2.06, 1.40], [1.97, 4.39], [-2.43, -3.92], [-3.51, 0.25]],
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
                "goals": [[-5.29, -3.55], [1.72, 3.15], [4.92, -3.04], [-2.25, 3.18]],
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
                "init_x": 6.92,
                "init_y": 1.84,
                "init_a": -51.62,
                "velocity": 1.18,
                "goals": [[6.06, 2.43], [2.54, -3.04], [5.10, 1.94], [1.14, 4.25]],
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
                "goals": [[4.85, 3.54], [0.69, 4.02], [-0.61, 1.68], [0.02, -5.18]],
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

def tests_adult_30_child_70_test_case_14_stopped_high(tester):
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
                "init_x": -3.73,
                "init_y": -3.35,
                "init_a": 136.60,
                "velocity": 1.16,
                "goals": [[-3.73, -3.35], [4.65, 3.22], [3.29, 5.26], [-0.68, 0.79]],
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
                "init_x": -3.90,
                "init_y": -2.36,
                "init_a": 136.60,
                "velocity": 1.16,
                "goal_x": -3.73,
                "goal_y": -3.35,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.95,
                "init_y": 1.56,
                "init_a": 137.97,
                "velocity": 0.85,
                "goals": [[-6.95, 1.56], [-7.25, -2.23], [7.70, 1.78], [-1.17, -4.76]],
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
                "init_x": -5.97,
                "init_y": 1.35,
                "init_a": 137.97,
                "velocity": 0.85,
                "goals": [[-6.95, 1.56], [-3.36, 5.50], [1.73, -2.16], [1.49, 2.22]],
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
                "init_x": 1.40,
                "init_y": -5.05,
                "init_a": -153.73,
                "velocity": 1.03,
                "goals": [[-0.21, -3.28], [-4.33, -1.97], [0.60, 0.74], [6.36, -0.87]],
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
                "init_x": -7.06,
                "init_y": -4.05,
                "init_a": 50.62,
                "velocity": 0.83,
                "goal_x": -0.68,
                "goal_y": -5.89,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.92,
                "init_y": 3.31,
                "init_a": 121.20,
                "velocity": 0.80,
                "goal_x": -4.43,
                "goal_y": -3.09,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.47,
                "init_y": 1.08,
                "init_a": 55.72,
                "velocity": 1.16,
                "goals": [[-1.23, -4.95], [4.36, -5.38], [-7.62, 4.61], [3.21, -3.79]],
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
                "init_x": -3.84,
                "init_y": 4.19,
                "init_a": -127.88,
                "velocity": 1.09,
                "goals": [[6.14, -4.00], [-0.52, -3.47], [1.96, -5.30], [-7.65, -5.43]],
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

def tests_adult_30_child_70_test_case_15_walking_high(tester):
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
                "goals": [[5.44, 1.84], [2.95, -0.82], [4.18, -3.63], [-5.48, -4.05]],
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
                "init_x": 6.17,
                "init_y": 0.34,
                "init_a": 95.69,
                "velocity": 0.99,
                "goals": [[5.44, 1.84], [-3.26, 2.84], [3.69, 2.96], [-5.20, -1.13]],
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
                "init_x": 5.92,
                "init_y": 4.80,
                "init_a": 2.99,
                "velocity": 0.99,
                "goals": [[-4.50, -1.26], [-4.18, 0.28], [-5.03, -3.42], [-3.02, -2.80]],
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
                "init_x": 5.06,
                "init_y": 4.29,
                "init_a": 2.99,
                "velocity": 0.99,
                "goals": [[-4.50, -1.26], [-4.65, 2.66], [0.24, 4.16], [2.77, 2.37]],
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
                "goals": [[-1.51, 1.04], [-1.06, 4.13], [4.65, -4.34], [-4.89, -4.34]],
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
                "init_x": -5.11,
                "init_y": -4.95,
                "init_a": -58.14,
                "velocity": 1.14,
                "goals": [[3.89, 6.00], [2.14, -3.41], [-4.68, -1.67], [-0.46, 1.34]],
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
                "init_x": -4.03,
                "init_y": 2.44,
                "init_a": -124.17,
                "velocity": 1.07,
                "goal_x": -5.48,
                "goal_y": -1.67,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_16_stopped_high(tester):
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
                "init_x": -6.55,
                "init_y": -5.05,
                "init_a": -118.17,
                "velocity": 1.03,
                "goals": [[-6.55, -5.05], [-5.42, 1.04], [-7.94, 4.78], [6.70, -0.75]],
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
                "init_x": -5.79,
                "init_y": -5.71,
                "init_a": -118.17,
                "velocity": 1.03,
                "goals": [[-6.55, -5.05], [-1.43, -1.51], [5.84, 3.01], [-2.77, -0.40]],
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
                "init_x": 1.69,
                "init_y": 0.82,
                "init_a": 171.03,
                "velocity": 1.16,
                "goals": [[1.69, 0.82], [4.21, -2.38], [-5.51, -1.51], [7.02, 3.49]],
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
                "init_x": 1.14,
                "init_y": -0.02,
                "init_a": 171.03,
                "velocity": 1.16,
                "goals": [[1.69, 0.82], [-2.17, -4.25], [-4.11, 1.71], [-6.59, -3.65]],
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
                "init_x": -5.89,
                "init_y": -0.13,
                "init_a": -179.07,
                "velocity": 1.13,
                "goal_x": 2.77,
                "goal_y": 5.66,
                "n_actors": 9,
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
                "goals": [[2.95, -1.66], [-5.69, 5.43], [-7.34, 3.71], [2.74, -2.80]],
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
                "goals": [[-5.43, 0.96], [5.64, 3.28], [4.32, 3.01], [-2.01, 1.61]],
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

def tests_adult_30_child_70_test_case_17_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.30,
                "init_y": -1.21,
                "init_a": -145.87,
                "velocity": 1.13,
                "goals": [[-6.97, -2.49], [-6.45, 1.32], [0.03, -0.59], [7.89, 3.66]],
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
                "init_x": -4.76,
                "init_y": 3.02,
                "init_a": -13.14,
                "velocity": 0.90,
                "goals": [[-2.68, -5.19], [2.42, -4.27], [-1.75, 1.36], [-1.54, 1.40]],
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
                "goals": [[-2.68, -5.19], [-0.86, -5.58], [-4.90, 5.91], [6.13, -0.00]],
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
                "goals": [[3.73, 2.84], [6.73, 0.35], [-6.36, -3.92], [-6.73, -1.09]],
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
                "init_x": -3.62,
                "init_y": -2.70,
                "init_a": 167.61,
                "velocity": 1.20,
                "goals": [[-0.72, 3.98], [-6.20, -0.41], [3.38, -1.54], [7.21, -3.51]],
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

def tests_adult_30_child_70_test_case_18_walking_high(tester):
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
                "goals": [[-2.54, -1.40], [5.21, -3.76], [7.54, -0.52], [-6.97, -3.48]],
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
                "init_x": -4.92,
                "init_y": -2.81,
                "init_a": -162.69,
                "velocity": 1.07,
                "goals": [[-2.54, -1.40], [7.64, 4.71], [-6.72, -5.74], [-3.81, 0.77]],
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
                "goals": [[4.80, -0.27], [0.10, 2.49], [5.01, 4.16], [2.81, 1.30]],
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
                "init_x": -1.26,
                "init_y": -2.69,
                "init_a": 179.74,
                "velocity": 0.89,
                "goals": [[7.37, -5.06], [-7.19, -3.15], [5.16, 3.75], [-2.10, -0.24]],
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
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.92,
                "init_y": 1.69,
                "init_a": 60.68,
                "velocity": 1.05,
                "goal_x": 1.02,
                "goal_y": 1.77,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_19_stopped_high(tester):
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
                "init_x": 3.29,
                "init_y": -3.00,
                "init_a": -0.90,
                "velocity": 1.01,
                "goals": [[3.29, -3.00], [-5.21, 5.19], [3.22, 5.41], [5.99, -6.00]],
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
                "init_x": 2.72,
                "init_y": -2.18,
                "init_a": -0.90,
                "velocity": 1.01,
                "goal_x": 3.29,
                "goal_y": -3.00,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.04,
                "init_y": -5.18,
                "init_a": 1.05,
                "velocity": 0.83,
                "goals": [[-1.04, -5.18], [7.27, -4.84], [-5.93, -2.12], [-5.80, -3.47]],
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
                "init_x": -1.94,
                "init_y": -5.60,
                "init_a": 1.05,
                "velocity": 0.83,
                "goals": [[-1.04, -5.18], [5.99, 0.65], [5.69, -2.33], [-6.33, -1.80]],
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
                "init_x": -3.39,
                "init_y": 5.43,
                "init_a": 0.93,
                "velocity": 1.18,
                "goals": [[6.62, 1.26], [-0.27, -1.88], [-4.72, 3.55], [1.68, -3.87]],
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

def tests_adult_30_child_70_test_case_20_walking_high(tester):
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
                "init_x": -6.51,
                "init_y": 1.92,
                "init_a": -15.04,
                "velocity": 0.87,
                "goal_x": -0.99,
                "goal_y": -4.21,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.76,
                "init_y": 2.89,
                "init_a": -15.04,
                "velocity": 0.87,
                "goal_x": -0.99,
                "goal_y": -4.21,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.76,
                "init_y": -0.08,
                "init_a": 20.01,
                "velocity": 1.19,
                "goals": [[-0.91, -2.09], [7.49, -0.55], [-1.11, 5.78], [-7.44, 2.31]],
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
                "init_x": -1.75,
                "init_y": 0.10,
                "init_a": 20.01,
                "velocity": 1.19,
                "goals": [[-0.91, -2.09], [-1.59, 3.17], [-5.46, -2.09], [-2.12, 0.97]],
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
                "init_x": 6.30,
                "init_y": -1.36,
                "init_a": 114.84,
                "velocity": 0.83,
                "goals": [[7.52, 4.59], [5.59, -2.31], [2.55, -0.43], [-6.60, 5.02]],
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
                "init_x": -4.49,
                "init_y": -0.92,
                "init_a": -149.47,
                "velocity": 0.98,
                "goal_x": -2.70,
                "goal_y": 5.94,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.88,
                "init_y": -3.15,
                "init_a": 66.56,
                "velocity": 1.02,
                "goals": [[3.08, -4.86], [0.13, 3.64], [1.88, -2.75], [-4.80, 4.78]],
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
                "init_x": 4.20,
                "init_y": 5.90,
                "init_a": 89.15,
                "velocity": 0.96,
                "goals": [[3.43, -2.21], [3.84, -1.78], [5.73, 5.96], [-5.13, 0.82]],
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

def tests_adult_30_child_70_test_case_21_stopped_high(tester):
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
                "init_x": 4.30,
                "init_y": 5.63,
                "init_a": 38.92,
                "velocity": 0.90,
                "goals": [[4.30, 5.63], [2.14, 3.57], [-0.65, -0.68], [-1.45, 2.14]],
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
                "init_x": 5.25,
                "init_y": 5.30,
                "init_a": 38.92,
                "velocity": 0.90,
                "goal_x": 4.30,
                "goal_y": 5.63,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.83,
                "init_y": -5.95,
                "init_a": 44.68,
                "velocity": 0.92,
                "goals": [[6.83, -5.95], [7.89, -5.57], [-4.70, 1.58], [2.81, -5.78]],
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
                "init_x": 7.19,
                "init_y": -5.01,
                "init_a": 44.68,
                "velocity": 0.92,
                "goal_x": 6.83,
                "goal_y": -5.95,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.64,
                "init_y": -2.02,
                "init_a": 37.98,
                "velocity": 1.08,
                "goal_x": -7.17,
                "goal_y": 3.33,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.96,
                "init_y": 1.86,
                "init_a": 1.30,
                "velocity": 1.15,
                "goals": [[-0.53, 1.73], [-4.45, -2.75], [-4.94, -2.09], [1.34, 0.60]],
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
                "init_x": 7.81,
                "init_y": 0.71,
                "init_a": -3.43,
                "velocity": 0.87,
                "goals": [[-5.95, 4.01], [0.42, 0.02], [-3.69, -1.41], [3.34, 5.93]],
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
                "init_x": 2.92,
                "init_y": -1.73,
                "init_a": 89.03,
                "velocity": 0.95,
                "goals": [[-3.26, -5.17], [0.77, -2.57], [7.52, -3.46], [-3.32, 1.70]],
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

def tests_adult_30_child_70_test_case_22_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.10,
                "init_y": 2.08,
                "init_a": -62.86,
                "velocity": 1.02,
                "goals": [[-0.12, -5.67], [5.35, -5.89], [7.02, 1.90], [-3.31, 4.51]],
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
                "init_x": -1.89,
                "init_y": -5.67,
                "init_a": 91.46,
                "velocity": 1.08,
                "goal_x": -1.87,
                "goal_y": 1.12,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.35,
                "init_y": -6.56,
                "init_a": 91.46,
                "velocity": 1.08,
                "goals": [[-1.87, 1.12], [-7.36, 5.96], [-2.16, -3.69], [6.43, 4.93]],
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
                "init_x": -1.17,
                "init_y": 0.70,
                "init_a": -150.22,
                "velocity": 1.11,
                "goals": [[3.01, -0.47], [-2.21, 5.28], [-1.14, 4.78], [6.99, 3.32]],
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
                "goals": [[6.26, -3.32], [3.50, 0.89], [7.32, -4.90], [1.60, 3.81]],
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

def tests_adult_30_child_70_test_case_23_walking_high(tester):
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
                "goals": [[-2.58, 4.91], [0.62, 4.81], [5.74, 0.33], [-0.74, 1.79]],
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
                "init_x": 6.96,
                "init_y": 5.19,
                "init_a": -165.93,
                "velocity": 0.86,
                "goals": [[-2.58, 4.91], [-6.73, 3.08], [-1.44, -1.75], [-6.71, 5.70]],
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
                "goals": [[4.46, 0.65], [3.05, 1.49], [0.41, 4.35], [-6.62, 4.71]],
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
                "init_x": 3.57,
                "init_y": 1.19,
                "init_a": 162.94,
                "velocity": 1.19,
                "goal_x": 6.19,
                "goal_y": -3.25,
                "n_actors": 7,
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
                "goals": [[-6.87, -3.96], [-4.41, 0.77], [-5.06, -3.00], [-3.92, 3.20]],
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

def tests_adult_30_child_70_test_case_24_walking_high(tester):
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
                "init_x": 0.98,
                "init_y": -1.49,
                "init_a": -157.41,
                "velocity": 0.85,
                "goals": [[3.12, 0.02], [0.49, -5.43], [6.16, 1.82], [-1.93, 0.42]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.15,
                "init_y": 5.85,
                "init_a": -93.65,
                "velocity": 0.93,
                "goals": [[-1.54, -1.08], [-3.11, -0.14], [-7.97, 4.06], [3.28, -4.46]],
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
                "goals": [[6.34, 4.38], [-7.47, 1.36], [7.94, -2.87], [5.36, -3.31]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.49,
                "init_y": -1.87,
                "init_a": -14.55,
                "velocity": 0.91,
                "goals": [[-1.22, -4.48], [-5.45, -5.82], [-4.23, 0.06], [6.88, -1.30]],
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
                "init_x": -2.91,
                "init_y": 3.99,
                "init_a": -60.64,
                "velocity": 0.98,
                "goals": [[0.34, 0.72], [-0.69, -5.24], [-5.19, -5.16], [-6.35, 2.61]],
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
                "init_x": 3.37,
                "init_y": -0.36,
                "init_a": 64.44,
                "velocity": 1.02,
                "goals": [[7.66, 1.79], [-3.62, -4.71], [-7.84, 0.23], [4.04, -0.50]],
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

def tests_adult_30_child_70_test_case_25_stopped_high(tester):
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
                "init_x": 1.98,
                "init_y": 0.61,
                "init_a": 119.93,
                "velocity": 0.87,
                "goals": [[1.98, 0.61], [-1.07, -0.97], [-4.24, -1.97], [2.47, 1.34]],
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
                "init_x": 1.50,
                "init_y": -0.27,
                "init_a": 119.93,
                "velocity": 0.87,
                "goals": [[1.98, 0.61], [7.26, 5.67], [6.80, -4.20], [1.06, -3.06]],
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
                "init_x": -0.45,
                "init_y": -2.13,
                "init_a": -100.62,
                "velocity": 0.84,
                "goal_x": -0.45,
                "goal_y": -2.13,
                "n_actors": 9,
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
                "goals": [[-4.95, -2.61], [-2.19, -2.09], [3.37, -4.11], [3.68, 3.95]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.23,
                "init_y": -1.59,
                "init_a": 46.51,
                "velocity": 0.89,
                "goals": [[5.63, -3.67], [-7.21, -0.19], [-4.40, 2.54], [-6.51, 4.72]],
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
                "init_x": -3.42,
                "init_y": 5.15,
                "init_a": -78.10,
                "velocity": 1.13,
                "goals": [[1.13, -2.94], [0.26, 2.53], [-5.41, 1.40], [-3.27, -4.69]],
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
                "goals": [[-3.59, -5.29], [5.91, 1.12], [-2.71, -0.36], [-1.12, 4.29]],
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

def tests_adult_30_child_70_test_case_26_stopped_high(tester):
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
                "goals": [[3.65, 0.78], [-3.30, -4.14], [3.56, -2.36], [5.44, -1.76]],
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
                "init_x": 3.62,
                "init_y": -0.22,
                "init_a": -127.80,
                "velocity": 1.12,
                "goals": [[3.65, 0.78], [6.32, 4.97], [6.80, -0.09], [1.42, 2.41]],
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
                "init_x": 5.42,
                "init_y": 1.51,
                "init_a": 108.02,
                "velocity": 1.20,
                "goal_x": 5.42,
                "goal_y": 1.51,
                "n_actors": 9,
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.38,
                "init_y": -4.17,
                "init_a": 108.82,
                "velocity": 1.01,
                "goals": [[6.90, 1.37], [1.57, -0.69], [2.41, 0.92], [-2.87, 5.71]],
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
                "goals": [[0.69, -4.67], [-0.44, 0.45], [-2.87, -1.60], [4.59, -1.90]],
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
                "goals": [[0.40, 0.88], [-4.68, -1.02], [-4.77, 3.96], [4.78, 4.80]],
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
                "init_x": -0.05,
                "init_y": 4.31,
                "init_a": 163.06,
                "velocity": 0.93,
                "goals": [[5.92, 2.13], [4.18, -4.82], [7.89, 3.72], [-2.16, -2.28]],
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

def tests_adult_30_child_70_test_case_27_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.68,
                "init_y": -6.78,
                "init_a": -20.70,
                "velocity": 0.90,
                "goals": [[4.96, -5.82], [7.53, 0.62], [7.26, 2.79], [-3.48, 2.56]],
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
                "init_x": -2.25,
                "init_y": -3.99,
                "init_a": -48.94,
                "velocity": 1.15,
                "goal_x": -2.25,
                "goal_y": -3.99,
                "n_actors": 9,
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
                "goals": [[-2.25, -3.99], [1.90, 0.09], [6.09, 5.31], [1.13, 4.49]],
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
                "init_x": 2.83,
                "init_y": 5.30,
                "init_a": 129.63,
                "velocity": 1.06,
                "goals": [[0.21, 0.12], [-2.90, 3.44], [-2.65, 0.42], [-3.61, 1.27]],
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
                "init_x": -6.79,
                "init_y": 2.76,
                "init_a": -47.16,
                "velocity": 1.02,
                "goal_x": -2.24,
                "goal_y": 0.89,
                "n_actors": 9,
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
                "goals": [[-3.35, 3.10], [2.25, 3.29], [-2.66, -4.05], [4.90, -3.43]],
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
                "init_x": -0.36,
                "init_y": -3.13,
                "init_a": 55.77,
                "velocity": 1.02,
                "goals": [[-0.03, -4.83], [5.23, 2.39], [2.60, 2.37], [5.12, 5.27]],
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
                "init_x": 0.49,
                "init_y": 5.94,
                "init_a": 6.01,
                "velocity": 1.20,
                "goals": [[0.24, 0.48], [6.71, 4.65], [5.70, 4.84], [7.81, -3.10]],
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

def tests_adult_30_child_70_test_case_28_stopped_high(tester):
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
                "init_x": -6.29,
                "init_y": -4.87,
                "init_a": 148.49,
                "velocity": 0.90,
                "goals": [[-6.29, -4.87], [-0.99, 4.67], [-3.04, 3.93], [-7.00, -4.59]],
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
                "init_x": -6.65,
                "init_y": -3.93,
                "init_a": 148.49,
                "velocity": 0.90,
                "goals": [[-6.29, -4.87], [-1.86, 5.25], [-7.25, -3.42], [0.09, 4.67]],
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
                "init_x": 5.85,
                "init_y": 0.21,
                "init_a": -17.76,
                "velocity": 1.06,
                "goals": [[5.85, 0.21], [7.07, 3.75], [-2.33, 1.23], [1.08, 0.52]],
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
                "init_x": 6.78,
                "init_y": 0.57,
                "init_a": -17.76,
                "velocity": 1.06,
                "goal_x": 5.85,
                "goal_y": 0.21,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.23,
                "init_y": -2.41,
                "init_a": -94.63,
                "velocity": 0.89,
                "goals": [[1.09, 5.72], [-5.94, -5.35], [-4.08, -4.78], [-3.41, -5.19]],
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
                "init_x": -2.78,
                "init_y": -4.79,
                "init_a": 118.31,
                "velocity": 0.83,
                "goal_x": 5.85,
                "goal_y": -2.80,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.09,
                "init_y": -2.91,
                "init_a": 64.66,
                "velocity": 1.18,
                "goals": [[4.21, 1.28], [3.31, 2.99], [-2.02, 2.46], [0.21, 2.93]],
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
                "init_x": 1.94,
                "init_y": -4.13,
                "init_a": -96.04,
                "velocity": 0.83,
                "goal_x": 1.19,
                "goal_y": -2.45,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.57,
                "init_y": -0.09,
                "init_a": -148.88,
                "velocity": 0.94,
                "goals": [[-0.23, 0.89], [6.53, 4.10], [1.94, -2.11], [-3.16, -0.15]],
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

def tests_adult_30_child_70_test_case_29_stopped_high(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.23,
                "init_y": 0.12,
                "init_a": 142.82,
                "velocity": 0.91,
                "goals": [[-7.23, 0.12], [-5.70, 0.71], [-4.00, 5.83], [4.44, 2.47]],
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
                "goals": [[-1.94, -4.89], [3.18, -4.81], [0.88, -3.15], [5.02, 4.79]],
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
                "goals": [[3.86, -1.31], [-7.92, 3.39], [5.41, 4.66], [-5.44, 4.24]],
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
                "goals": [[-2.81, -4.86], [-4.71, 5.27], [5.76, 5.05], [7.61, -3.09]],
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
                "init_x": -3.91,
                "init_y": -5.30,
                "init_a": -81.55,
                "velocity": 1.06,
                "goals": [[-1.04, -1.19], [-4.63, -0.86], [2.85, 5.09], [6.76, -2.68]],
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

def tests_adult_30_child_70_test_case_30_walking_high(tester):
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
                "goals": [[-5.84, -5.51], [7.62, 0.03], [-7.93, 0.67], [-2.22, 0.00]],
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
                "init_x": -4.15,
                "init_y": -1.90,
                "init_a": -14.21,
                "velocity": 1.10,
                "goals": [[-5.84, -5.51], [2.89, 5.40], [-0.42, 1.53], [-0.20, -1.87]],
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
                "init_x": -2.92,
                "init_y": 4.71,
                "init_a": -109.88,
                "velocity": 0.83,
                "goals": [[-3.80, 0.36], [-1.41, -3.73], [-0.30, 4.31], [-5.71, -2.81]],
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
                "goals": [[2.42, 4.06], [1.34, -0.62], [-5.30, -1.67], [-2.02, 2.33]],
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
                "init_x": -3.93,
                "init_y": -4.45,
                "init_a": 2.21,
                "velocity": 1.07,
                "goal_x": 4.33,
                "goal_y": 1.13,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_31_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.24,
                "init_y": -3.89,
                "init_a": 172.22,
                "velocity": 1.19,
                "goals": [[7.04, -2.91], [6.15, 2.71], [-5.85, -2.56], [-2.53, -2.76]],
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
                "init_x": 1.59,
                "init_y": -2.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goals": [[1.59, -2.71], [-7.69, 0.42], [7.49, 2.76], [5.54, 2.46]],
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
                "init_x": 1.52,
                "init_y": -1.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goals": [[1.59, -2.71], [4.54, 3.22], [7.18, -5.46], [-5.17, 1.91]],
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
                "init_x": 5.07,
                "init_y": -4.24,
                "init_a": -170.53,
                "velocity": 1.13,
                "goals": [[3.23, 2.89], [7.57, 5.44], [-7.79, -1.13], [7.68, -2.71]],
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

def tests_adult_30_child_70_test_case_32_walking_high(tester):
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
                "goals": [[-0.98, 0.29], [7.30, 4.57], [6.83, -1.77], [0.76, -5.88]],
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
                "init_x": 1.52,
                "init_y": -1.70,
                "init_a": 76.89,
                "velocity": 1.18,
                "goals": [[-0.98, 0.29], [-3.61, -1.03], [7.76, 4.53], [4.48, -5.51]],
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
                "goals": [[5.31, -0.65], [-2.18, -5.98], [-0.91, -1.45], [-3.03, 5.18]],
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
                "init_x": 2.20,
                "init_y": 4.35,
                "init_a": 128.58,
                "velocity": 0.83,
                "goals": [[-1.27, 5.80], [-5.64, 4.16], [4.84, 5.48], [0.91, 0.92]],
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
                "init_x": 1.48,
                "init_y": -4.41,
                "init_a": 59.41,
                "velocity": 1.12,
                "goals": [[-5.68, -4.75], [6.04, -3.95], [-5.64, 3.17], [2.14, 5.55]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.88,
                "init_y": -5.17,
                "init_a": -6.13,
                "velocity": 1.03,
                "goals": [[-5.75, 0.95], [2.65, 3.81], [5.14, -1.17], [4.05, -4.62]],
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

def tests_adult_30_child_70_test_case_33_stopped_high(tester):
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.41,
                "init_y": 4.02,
                "init_a": -171.83,
                "velocity": 0.87,
                "goals": [[5.78, 4.94], [-3.01, -4.22], [-6.39, 0.34], [7.72, -3.96]],
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
                "init_x": 2.38,
                "init_y": -3.94,
                "init_a": -33.94,
                "velocity": 1.07,
                "goals": [[6.19, 4.16], [1.66, 3.26], [-4.42, 2.57], [0.32, 3.35]],
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
                "init_x": 2.94,
                "init_y": 0.43,
                "init_a": 164.76,
                "velocity": 0.89,
                "goals": [[5.30, -5.56], [4.42, 1.71], [3.56, 2.14], [-1.90, 5.42]],
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
                "init_x": -5.46,
                "init_y": 1.94,
                "init_a": -98.62,
                "velocity": 1.08,
                "goals": [[-5.18, 5.63], [-2.56, -3.08], [0.11, -2.01], [2.99, -5.41]],
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

def tests_adult_30_child_70_test_case_34_stopped_high(tester):
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
                "goals": [[7.29, 0.59], [-4.09, 1.03], [-5.76, 0.38], [3.88, -4.10]],
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
                "goals": [[7.29, 0.59], [-2.41, 3.88], [-7.05, -0.26], [2.76, 1.33]],
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
                "goals": [[-0.07, 0.15], [2.75, -1.40], [-4.61, 3.29], [-6.90, -2.71]],
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
                "goals": [[5.80, -1.24], [6.24, 1.42], [-1.47, 3.43], [-4.60, 3.72]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.74,
                "init_y": -4.19,
                "init_a": 137.42,
                "velocity": 1.20,
                "goals": [[0.79, -2.01], [-2.77, -3.37], [-3.29, 3.48], [0.58, 0.03]],
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

def tests_adult_30_child_70_test_case_35_walking_high(tester):
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
                "goals": [[1.46, -2.66], [-7.79, -1.04], [7.48, -3.66], [2.66, 2.33]],
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
                "goals": [[5.96, -5.59], [-4.12, -4.78], [-7.12, 1.28], [-0.80, 2.47]],
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
                "init_x": -6.03,
                "init_y": -1.11,
                "init_a": 3.93,
                "velocity": 1.01,
                "goals": [[5.96, -5.59], [2.52, 1.05], [-6.01, 0.65], [-3.31, 2.35]],
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
                "goals": [[-1.21, 2.63], [-7.29, -3.33], [0.08, 2.00], [2.79, -1.28]],
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
                "goals": [[-6.34, 2.17], [-3.75, 3.48], [6.38, 5.67], [-3.55, -4.24]],
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

def tests_adult_30_child_70_test_case_36_stopped_high(tester):
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
                "init_x": -3.70,
                "init_y": -3.49,
                "init_a": 63.37,
                "velocity": 1.05,
                "goal_x": -3.70,
                "goal_y": -3.49,
                "n_actors": 7,
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
                "goals": [[-3.70, -3.49], [-2.75, -0.19], [6.36, 2.63], [-4.70, -1.03]],
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
                "init_x": -4.13,
                "init_y": 5.07,
                "init_a": -14.49,
                "velocity": 0.86,
                "goals": [[-4.13, 5.07], [6.24, -1.29], [-1.05, -4.87], [-2.85, -4.88]],
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
                "init_x": -3.71,
                "init_y": 5.98,
                "init_a": -14.49,
                "velocity": 0.86,
                "goals": [[-4.13, 5.07], [-6.29, -5.24], [-7.29, 2.68], [-0.98, 0.92]],
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
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.22,
                "init_y": -5.94,
                "init_a": 133.05,
                "velocity": 1.16,
                "goal_x": 3.43,
                "goal_y": -1.16,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.74,
                "init_y": -1.65,
                "init_a": -85.48,
                "velocity": 0.93,
                "goals": [[-0.60, 3.76], [0.98, 2.05], [-4.16, -1.05], [7.52, 5.36]],
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

def tests_adult_30_child_70_test_case_37_stopped_high(tester):
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
                "init_x": -5.02,
                "init_y": 0.90,
                "init_a": 102.07,
                "velocity": 1.19,
                "goals": [[-5.02, 0.90], [-5.38, 4.95], [5.94, -3.54], [4.28, -5.80]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.28,
                "init_y": 2.94,
                "init_a": 59.29,
                "velocity": 0.97,
                "goals": [[-7.28, 2.94], [-5.60, 4.84], [3.95, 1.61], [5.72, 1.84]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.76,
                "init_y": -2.95,
                "init_a": -30.08,
                "velocity": 1.03,
                "goals": [[-6.22, -1.79], [2.47, 2.07], [-3.63, 3.40], [2.07, -3.14]],
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
                "init_x": -5.77,
                "init_y": -1.62,
                "init_a": -60.49,
                "velocity": 0.85,
                "goals": [[-2.91, 4.58], [4.29, 0.92], [6.10, -3.41], [7.52, 1.17]],
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

def tests_adult_30_child_70_test_case_38_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.04,
                "init_y": 3.23,
                "init_a": -69.41,
                "velocity": 1.05,
                "goals": [[4.49, 2.40], [-4.73, -4.17], [3.78, -2.93], [7.26, 1.80]],
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
                "goals": [[5.17, 1.61], [2.78, 0.34], [-0.84, 3.29], [-7.18, -4.49]],
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
                "init_x": -1.78,
                "init_y": -0.10,
                "init_a": 97.54,
                "velocity": 0.96,
                "goal_x": 0.17,
                "goal_y": 1.64,
                "n_actors": 9,
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
                "goals": [[2.19, -4.41], [-7.95, -1.80], [2.53, 1.05], [3.70, -1.19]],
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
                "init_x": 4.68,
                "init_y": -2.56,
                "init_a": -100.52,
                "velocity": 0.92,
                "goals": [[1.65, 3.36], [7.90, 0.87], [6.30, 5.80], [5.36, -1.41]],
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
                "init_x": 7.46,
                "init_y": 2.45,
                "init_a": -39.44,
                "velocity": 0.87,
                "goals": [[-1.23, -5.42], [4.57, 4.06], [-1.74, 1.00], [5.96, -3.60]],
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
                "init_x": -7.07,
                "init_y": 0.59,
                "init_a": 47.54,
                "velocity": 0.92,
                "goals": [[6.65, -3.74], [-4.74, -2.35], [0.33, -0.39], [6.40, 1.19]],
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

def tests_adult_30_child_70_test_case_39_walking_high(tester):
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
                "goals": [[-1.97, 5.75], [-6.10, -3.17], [7.05, -0.20], [4.07, 3.74]],
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
                "init_x": -5.21,
                "init_y": -2.70,
                "init_a": -116.94,
                "velocity": 0.91,
                "goals": [[-1.97, 5.75], [5.23, -5.76], [4.31, -4.90], [5.45, 3.02]],
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
                "goals": [[-0.69, 0.71], [4.26, -1.98], [-0.78, -5.69], [5.15, -0.01]],
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
                "goals": [[5.51, 0.12], [-6.10, 3.05], [-6.77, 0.84], [-5.93, 0.47]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.65,
                "init_y": -4.06,
                "init_a": 17.86,
                "velocity": 1.04,
                "goals": [[-0.11, -0.10], [-1.10, -2.60], [-5.71, -3.04], [-6.35, -4.03]],
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

def tests_adult_30_child_70_test_case_40_walking_high(tester):
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
                "goals": [[4.02, -2.08], [-1.42, -0.90], [2.84, -0.04], [1.91, -5.24]],
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
                "init_x": 1.48,
                "init_y": 0.33,
                "init_a": 150.93,
                "velocity": 1.08,
                "goal_x": 4.02,
                "goal_y": -2.08,
                "n_actors": 8,
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
                "goals": [[1.54, -0.24], [-7.25, 5.80], [6.70, 0.36], [7.92, 5.58]],
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
                "goals": [[-3.55, -0.33], [-6.00, 6.00], [7.94, -4.75], [0.32, 1.61]],
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
                "init_x": -6.31,
                "init_y": 5.94,
                "init_a": -36.94,
                "velocity": 0.98,
                "goals": [[-0.57, -3.39], [6.37, 2.14], [0.13, 4.25], [-2.58, -1.84]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.47,
                "init_y": -0.40,
                "init_a": -149.70,
                "velocity": 0.92,
                "goals": [[6.65, 4.79], [-5.58, -2.40], [-7.25, 1.32], [4.96, 0.76]],
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

def tests_adult_30_child_70_test_case_41_stopped_high(tester):
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
                "init_x": -1.93,
                "init_y": 2.39,
                "init_a": 59.77,
                "velocity": 0.91,
                "goals": [[-1.93, 2.39], [-3.56, -0.90], [6.05, 0.85], [0.92, 5.18]],
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
                "init_x": -1.72,
                "init_y": 1.41,
                "init_a": 59.77,
                "velocity": 0.91,
                "goal_x": -1.93,
                "goal_y": 2.39,
                "n_actors": 7,
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
                "goals": [[4.94, 5.24], [5.29, -2.09], [-6.86, -3.65], [1.88, 5.29]],
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
                "init_x": 4.66,
                "init_y": -3.23,
                "init_a": 128.60,
                "velocity": 1.03,
                "goals": [[1.37, -0.70], [-2.67, 0.36], [7.30, -5.55], [6.98, 1.71]],
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
                "init_x": 7.43,
                "init_y": -3.08,
                "init_a": -131.12,
                "velocity": 1.13,
                "goals": [[3.58, 2.53], [-6.77, 4.19], [6.76, -3.04], [0.43, 4.89]],
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
                "init_x": -1.41,
                "init_y": -0.24,
                "init_a": -65.68,
                "velocity": 1.18,
                "goal_x": -5.10,
                "goal_y": 3.83,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_42_stopped_high(tester):
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
                "init_x": -6.12,
                "init_y": 1.34,
                "init_a": 58.28,
                "velocity": 0.92,
                "goals": [[-6.12, 1.34], [5.88, 1.01], [1.08, -5.73], [3.61, 0.31]],
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
                "goals": [[5.35, 1.82], [5.94, -2.64], [7.09, -3.99], [3.64, -1.12]],
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
                "init_x": 4.56,
                "init_y": 2.43,
                "init_a": -71.62,
                "velocity": 1.06,
                "goal_x": 5.35,
                "goal_y": 1.82,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.62,
                "init_y": -4.47,
                "init_a": 38.86,
                "velocity": 0.83,
                "goals": [[1.85, -0.85], [4.07, 2.62], [-0.99, 5.52], [-0.65, -4.90]],
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
                "init_x": 3.18,
                "init_y": -2.71,
                "init_a": -90.97,
                "velocity": 0.81,
                "goals": [[-6.03, -2.63], [-0.13, 5.36], [6.38, -4.19], [4.03, -1.17]],
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
                "init_x": -6.70,
                "init_y": 1.95,
                "init_a": 41.99,
                "velocity": 0.86,
                "goal_x": -4.07,
                "goal_y": -3.95,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.77,
                "init_y": 1.42,
                "init_a": -74.87,
                "velocity": 1.07,
                "goals": [[0.93, -2.93], [0.76, -4.18], [-7.43, -3.19], [7.74, -2.90]],
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
                "init_x": 7.78,
                "init_y": 1.87,
                "init_a": -25.37,
                "velocity": 0.99,
                "goals": [[-0.77, 2.38], [-2.74, -5.84], [6.28, 5.88], [3.44, 0.60]],
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

def tests_adult_30_child_70_test_case_43_stopped_high(tester):
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
                "goals": [[6.02, 3.19], [-3.15, 4.50], [-3.41, 0.82], [6.36, -2.09]],
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
                "goals": [[0.74, 5.47], [7.71, 3.64], [2.81, -0.07], [2.07, 2.31]],
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
                "goals": [[4.02, 1.16], [-3.55, 4.50], [2.83, 0.93], [-7.89, -4.20]],
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
                "init_x": -1.02,
                "init_y": 0.17,
                "init_a": 116.73,
                "velocity": 0.88,
                "goals": [[4.59, 0.20], [1.10, -4.38], [-1.13, 1.25], [-7.96, 2.40]],
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
                "init_x": -7.00,
                "init_y": -5.59,
                "init_a": -19.32,
                "velocity": 1.06,
                "goals": [[7.41, -0.38], [5.75, -3.49], [-2.93, -3.23], [-1.61, 4.39]],
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
                "init_x": 7.98,
                "init_y": 2.53,
                "init_a": -4.92,
                "velocity": 0.98,
                "goals": [[-7.31, -3.40], [-4.07, 3.40], [7.49, -4.57], [1.45, 3.66]],
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

def tests_adult_30_child_70_test_case_44_stopped_high(tester):
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
                "goals": [[-2.76, -1.11], [3.86, 1.12], [3.39, -2.50], [0.97, -0.57]],
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
                "goals": [[7.80, 3.42], [-4.67, 3.28], [0.92, -5.24], [5.95, -5.15]],
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
                "goals": [[7.80, 3.42], [-2.18, 5.83], [-6.31, 1.69], [3.67, -1.62]],
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
                "init_x": -1.36,
                "init_y": 0.85,
                "init_a": 88.60,
                "velocity": 1.04,
                "goals": [[-3.59, -0.02], [-2.34, 4.87], [6.79, -5.32], [2.51, 1.72]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.33,
                "init_y": 1.11,
                "init_a": -119.66,
                "velocity": 0.82,
                "goals": [[-7.51, 0.54], [0.06, 2.38], [2.59, -2.10], [7.99, 3.32]],
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

def tests_adult_30_child_70_test_case_45_walking_high(tester):
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
                "goals": [[6.75, 0.40], [-7.71, 1.83], [-6.29, -5.93], [0.73, 0.33]],
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
                "init_x": -3.58,
                "init_y": 0.63,
                "init_a": -133.51,
                "velocity": 1.17,
                "goals": [[6.75, 0.40], [2.79, 5.68], [2.35, -3.46], [-2.08, 1.63]],
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
                "init_x": 5.91,
                "init_y": 2.01,
                "init_a": -61.08,
                "velocity": 1.00,
                "goals": [[1.43, 1.81], [5.03, 1.88], [5.76, 0.84], [5.45, 4.66]],
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
                "init_x": 6.32,
                "init_y": 1.10,
                "init_a": -61.08,
                "velocity": 1.00,
                "goals": [[1.43, 1.81], [-3.27, -1.25], [4.86, -2.03], [-1.11, 1.43]],
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
                "init_x": 3.93,
                "init_y": 0.15,
                "init_a": 83.14,
                "velocity": 1.01,
                "goal_x": -1.29,
                "goal_y": 2.63,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.92,
                "init_y": -2.49,
                "init_a": -178.31,
                "velocity": 1.08,
                "goals": [[6.35, -5.18], [7.33, -1.12], [-1.01, -1.12], [-3.27, 2.30]],
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

def tests_adult_30_child_70_test_case_46_walking_high(tester):
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
                "init_x": 5.95,
                "init_y": 1.00,
                "init_a": 19.67,
                "velocity": 1.01,
                "goal_x": -5.50,
                "goal_y": 3.69,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.33,
                "init_y": 0.08,
                "init_a": 19.67,
                "velocity": 1.01,
                "goal_x": -5.50,
                "goal_y": 3.69,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.95,
                "init_y": -0.34,
                "init_a": 109.60,
                "velocity": 1.19,
                "goal_x": -3.89,
                "goal_y": -3.62,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.02,
                "init_y": -0.70,
                "init_a": 109.60,
                "velocity": 1.19,
                "goals": [[-3.89, -3.62], [5.24, -1.45], [-4.85, -0.14], [-5.43, -3.35]],
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
                "init_x": -7.47,
                "init_y": -4.48,
                "init_a": 9.62,
                "velocity": 0.84,
                "goals": [[7.28, -3.23], [7.36, -5.87], [-5.52, 1.68], [-7.07, 2.48]],
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
                "init_x": -0.96,
                "init_y": 0.98,
                "init_a": 29.90,
                "velocity": 1.00,
                "goals": [[7.74, 2.67], [2.90, 0.12], [-0.57, 3.35], [-0.31, 4.03]],
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
                "init_y": -1.12,
                "init_a": -92.14,
                "velocity": 1.01,
                "goals": [[-1.70, -4.43], [1.71, 4.41], [7.59, -0.73], [-1.01, -0.94]],
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
                "init_x": -1.52,
                "init_y": -3.75,
                "init_a": 93.07,
                "velocity": 0.90,
                "goals": [[4.64, -1.43], [6.53, -1.84], [3.04, 1.05], [7.46, -1.76]],
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

def tests_adult_30_child_70_test_case_47_stopped_high(tester):
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
                "goals": [[-0.67, 0.17], [-0.61, -4.10], [7.01, 4.54], [-2.82, -4.95]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.45,
                "init_y": 5.92,
                "init_a": -61.07,
                "velocity": 1.16,
                "goals": [[-7.45, 5.92], [-4.90, 1.52], [7.72, -3.63], [7.94, 4.92]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.42,
                "init_y": 3.37,
                "init_a": 38.20,
                "velocity": 1.03,
                "goals": [[7.93, 5.30], [-4.13, -2.61], [-4.26, 0.43], [-4.16, 5.23]],
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
                "init_x": 0.78,
                "init_y": 4.46,
                "init_a": -88.04,
                "velocity": 1.05,
                "goals": [[-4.74, 0.90], [6.08, -3.79], [6.42, 5.35], [-5.96, -2.15]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.43,
                "init_y": 0.06,
                "init_a": 77.21,
                "velocity": 1.20,
                "goals": [[-0.80, -5.75], [3.33, 3.46], [3.64, 3.56], [-4.38, -2.10]],
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

def tests_adult_30_child_70_test_case_48_stopped_high(tester):
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
                "init_x": 7.78,
                "init_y": 4.30,
                "init_a": -171.53,
                "velocity": 1.10,
                "goals": [[7.78, 4.30], [0.85, -1.19], [2.47, -4.54], [3.74, -5.26]],
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
                "init_x": 6.88,
                "init_y": 3.87,
                "init_a": -171.53,
                "velocity": 1.10,
                "goal_x": 7.78,
                "goal_y": 4.30,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.34,
                "init_y": 4.74,
                "init_a": 171.74,
                "velocity": 0.89,
                "goals": [[3.34, 4.74], [6.36, -1.67], [-1.37, 2.52], [5.31, -1.53]],
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
                "init_x": 3.87,
                "init_y": 3.89,
                "init_a": 171.74,
                "velocity": 0.89,
                "goal_x": 3.34,
                "goal_y": 4.74,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.70,
                "init_y": 5.04,
                "init_a": 93.06,
                "velocity": 0.87,
                "goal_x": -7.19,
                "goal_y": -5.23,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.47,
                "init_y": 0.04,
                "init_a": -59.45,
                "velocity": 1.09,
                "goals": [[-3.99, -5.37], [2.54, 1.71], [-3.71, -2.55], [3.71, 5.51]],
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
                "init_x": -4.36,
                "init_y": 3.63,
                "init_a": -52.24,
                "velocity": 1.09,
                "goals": [[7.42, -2.57], [4.26, 5.52], [-2.26, 0.78], [-6.23, 2.87]],
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
                "init_x": 5.19,
                "init_y": -0.21,
                "init_a": 93.32,
                "velocity": 0.88,
                "goals": [[-3.87, -5.18], [-6.01, 2.89], [2.32, 0.77], [5.57, -4.39]],
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

def tests_adult_30_child_70_test_case_49_stopped_high(tester):
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
                "init_x": 5.73,
                "init_y": 1.50,
                "init_a": -43.41,
                "velocity": 1.10,
                "goals": [[5.73, 1.50], [0.65, 0.42], [-4.19, 0.07], [4.32, -3.14]],
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
                "init_x": 6.51,
                "init_y": 2.13,
                "init_a": -43.41,
                "velocity": 1.10,
                "goal_x": 5.73,
                "goal_y": 1.50,
                "n_actors": 7,
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
                "goals": [[2.23, -5.14], [6.43, 4.06], [-7.59, 4.71], [-4.43, -1.42]],
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
                "init_x": -0.53,
                "init_y": 0.82,
                "init_a": -11.78,
                "velocity": 0.83,
                "goals": [[-1.01, -1.26], [-5.56, -3.47], [-3.85, 3.83], [5.69, 0.77]],
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
                "init_x": 0.04,
                "init_y": -3.67,
                "init_a": -121.43,
                "velocity": 0.91,
                "goals": [[6.62, 3.52], [-2.25, 3.74], [4.52, -3.58], [5.77, -3.15]],
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

def tests_adult_30_child_70_test_case_50_walking_high(tester):
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
                "goals": [[0.27, -0.82], [-0.20, 2.83], [-1.40, -2.36], [-4.04, 5.58]],
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
                "init_x": 1.46,
                "init_y": 2.40,
                "init_a": -24.16,
                "velocity": 1.13,
                "goals": [[0.27, -0.82], [-7.48, -1.17], [3.46, 4.72], [6.33, -2.97]],
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
                "init_x": 5.74,
                "init_y": -1.10,
                "init_a": 12.39,
                "velocity": 0.85,
                "goals": [[3.12, 1.80], [-7.85, -5.06], [7.66, -1.04], [-6.97, 5.29]],
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
                "init_x": 4.81,
                "init_y": -0.74,
                "init_a": 12.39,
                "velocity": 0.85,
                "goals": [[3.12, 1.80], [3.24, 3.70], [7.16, -2.51], [6.63, 3.03]],
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
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.89,
                "init_y": 0.78,
                "init_a": -133.53,
                "velocity": 1.20,
                "goal_x": -3.98,
                "goal_y": -4.85,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_51_stopped_high(tester):
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
                "goals": [[-2.15, -3.09], [-3.48, 1.08], [-5.81, 3.88], [4.08, 4.95]],
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
                "init_x": -2.15,
                "init_y": -2.09,
                "init_a": 39.50,
                "velocity": 0.99,
                "goals": [[-2.15, -3.09], [4.30, -4.16], [7.84, 1.16], [-0.38, -5.19]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.67,
                "init_y": 2.68,
                "init_a": -77.84,
                "velocity": 0.86,
                "goals": [[-0.59, 2.08], [0.31, 5.22], [-0.87, -4.52], [4.14, -4.95]],
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
                "goals": [[0.37, -5.05], [-5.33, -2.80], [0.76, 4.18], [-3.29, -0.76]],
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
                "init_x": -2.70,
                "init_y": -2.19,
                "init_a": 53.44,
                "velocity": 0.80,
                "goals": [[-5.93, -5.48], [4.89, 3.55], [-2.63, 1.98], [1.53, 0.58]],
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

def tests_adult_30_child_70_test_case_52_walking_high(tester):
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
                "goals": [[1.47, 1.12], [0.17, 4.05], [-3.70, 4.91], [4.51, 2.17]],
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
                "init_x": -3.49,
                "init_y": -4.09,
                "init_a": 21.73,
                "velocity": 1.06,
                "goals": [[1.47, 1.12], [-0.71, 5.89], [7.09, -5.84], [-7.12, 3.32]],
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
                "init_x": 7.23,
                "init_y": -1.20,
                "init_a": -163.11,
                "velocity": 1.12,
                "goals": [[0.64, -4.35], [-7.54, 4.31], [-3.54, -2.54], [7.79, -4.41]],
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
                "init_x": 0.99,
                "init_y": -1.32,
                "init_a": 12.62,
                "velocity": 0.87,
                "goals": [[1.66, -3.46], [-4.54, 1.95], [-7.64, 5.96], [2.85, 3.47]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.88,
                "init_y": 2.64,
                "init_a": 89.13,
                "velocity": 1.10,
                "goals": [[-7.25, -4.53], [2.59, 0.61], [-0.32, 3.93], [-2.43, -1.88]],
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

def tests_adult_30_child_70_test_case_53_walking_high(tester):
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
                "init_x": -0.84,
                "init_y": 4.08,
                "init_a": 0.95,
                "velocity": 1.15,
                "goal_x": -6.44,
                "goal_y": -1.76,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.15,
                "init_y": 4.22,
                "init_a": 0.95,
                "velocity": 1.15,
                "goal_x": -6.44,
                "goal_y": -1.76,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.45,
                "init_y": -2.72,
                "init_a": -170.03,
                "velocity": 0.96,
                "goals": [[-0.08, 3.28], [5.50, -2.54], [-0.50, 0.76], [-3.25, 5.42]],
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
                "init_x": 8.11,
                "init_y": -1.97,
                "init_a": -170.03,
                "velocity": 0.96,
                "goals": [[-0.08, 3.28], [0.14, 3.40], [-1.31, 5.30], [0.42, 1.18]],
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
                "init_x": -1.88,
                "init_y": -4.47,
                "init_a": 91.15,
                "velocity": 1.04,
                "goals": [[6.39, -3.55], [-6.09, 5.26], [-3.95, -4.27], [-6.77, -3.77]],
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
                "init_x": 1.36,
                "init_y": -1.89,
                "init_a": -164.74,
                "velocity": 1.10,
                "goal_x": 1.94,
                "goal_y": 1.91,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.34,
                "init_y": -5.71,
                "init_a": 59.28,
                "velocity": 1.04,
                "goals": [[-2.28, -1.42], [-2.83, -0.95], [1.18, 0.30], [-6.93, 5.28]],
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

def tests_adult_30_child_70_test_case_54_walking_high(tester):
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
                "goals": [[3.98, 2.75], [5.34, -1.51], [-1.00, -0.61], [1.84, 5.12]],
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
                "init_x": -4.96,
                "init_y": -2.39,
                "init_a": 12.74,
                "velocity": 1.04,
                "goals": [[7.44, -4.34], [6.54, 5.53], [-5.26, -1.79], [-0.42, 0.83]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.07,
                "init_y": 3.46,
                "init_a": -56.42,
                "velocity": 0.95,
                "goals": [[-2.62, 4.71], [7.07, -5.77], [-2.65, 5.90], [6.33, 0.97]],
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
                "init_x": 5.06,
                "init_y": 5.78,
                "init_a": -14.62,
                "velocity": 0.84,
                "goals": [[-3.58, 3.91], [7.08, 2.27], [6.55, -4.75], [3.31, 0.73]],
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
                "init_x": -0.07,
                "init_y": -1.98,
                "init_a": -73.07,
                "velocity": 0.99,
                "goals": [[1.21, -0.99], [0.19, 1.08], [-1.51, -2.60], [6.25, 1.57]],
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
                "init_x": 3.87,
                "init_y": -5.52,
                "init_a": 66.98,
                "velocity": 0.80,
                "goals": [[-2.75, -1.68], [3.54, -0.92], [7.13, 5.04], [-0.15, -0.00]],
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

def tests_adult_30_child_70_test_case_55_walking_high(tester):
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
                "init_x": 7.73,
                "init_y": -3.32,
                "init_a": -120.24,
                "velocity": 1.05,
                "goals": [[-1.34, -4.55], [2.30, 2.64], [7.52, -5.79], [1.33, 3.98]],
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
                "init_x": 8.59,
                "init_y": -3.82,
                "init_a": -120.24,
                "velocity": 1.05,
                "goals": [[-1.34, -4.55], [7.63, -1.28], [7.38, -0.60], [1.63, -2.69]],
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
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.04,
                "init_y": -3.74,
                "init_a": 45.41,
                "velocity": 1.09,
                "goal_x": 1.47,
                "goal_y": -0.13,
                "n_actors": 7,
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
                "goals": [[7.47, -0.66], [-4.22, 4.30], [-1.08, -1.85], [-5.85, 3.49]],
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
                "init_x": 0.49,
                "init_y": -4.05,
                "init_a": 162.45,
                "velocity": 0.89,
                "goals": [[-5.99, -4.82], [-5.54, -1.93], [-1.05, 0.87], [-0.23, -5.40]],
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

def tests_adult_30_child_70_test_case_56_stopped_high(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.34,
                "init_y": -1.27,
                "init_a": -88.45,
                "velocity": 0.90,
                "goals": [[6.34, -1.27], [-0.49, -0.25], [3.35, 5.88], [5.98, -4.84]],
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
                "init_x": 6.62,
                "init_y": -0.31,
                "init_a": -88.45,
                "velocity": 0.90,
                "goals": [[6.34, -1.27], [-0.46, -0.39], [0.94, 3.04], [6.88, -0.91]],
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
                "init_x": -4.78,
                "init_y": -1.42,
                "init_a": 134.78,
                "velocity": 1.04,
                "goals": [[-4.30, -3.27], [-6.59, 0.51], [5.86, 0.28], [-1.47, 3.77]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.27,
                "init_y": -2.52,
                "init_a": 34.89,
                "velocity": 1.00,
                "goals": [[-0.33, 0.68], [-7.49, 0.94], [-3.43, -1.78], [-2.11, -1.71]],
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

def tests_adult_30_child_70_test_case_57_stopped_high(tester):
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
                "goals": [[5.13, 2.94], [6.91, -3.27], [3.20, -3.28], [6.80, 5.05]],
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
                "init_x": 5.47,
                "init_y": 2.00,
                "init_a": -121.11,
                "velocity": 0.97,
                "goals": [[5.13, 2.94], [-1.71, 2.64], [0.39, 5.12], [2.63, 3.85]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.62,
                "init_y": 2.11,
                "init_a": 48.72,
                "velocity": 0.94,
                "goals": [[0.75, -3.59], [2.55, -1.40], [0.99, 5.79], [-4.09, -5.35]],
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
                "init_x": 6.39,
                "init_y": -2.24,
                "init_a": 106.29,
                "velocity": 0.97,
                "goals": [[-1.12, 0.37], [-6.70, -5.60], [-3.55, 5.85], [-3.29, -4.80]],
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
                "init_x": 6.44,
                "init_y": -0.88,
                "init_a": -107.32,
                "velocity": 0.97,
                "goals": [[-3.48, 5.36], [0.26, 1.41], [-6.36, 4.96], [-2.73, -2.37]],
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
                "init_x": 7.65,
                "init_y": -0.54,
                "init_a": -62.86,
                "velocity": 0.91,
                "goals": [[4.32, -0.49], [2.80, -5.03], [4.46, -2.35], [-2.16, -5.88]],
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

def tests_adult_30_child_70_test_case_58_walking_high(tester):
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
                "init_x": -4.35,
                "init_y": -0.79,
                "init_a": -34.91,
                "velocity": 1.14,
                "goals": [[7.59, -4.42], [1.10, 3.73], [7.08, -4.07], [-2.72, 5.67]],
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
                "init_x": -3.46,
                "init_y": -1.26,
                "init_a": -34.91,
                "velocity": 1.14,
                "goals": [[7.59, -4.42], [0.02, 2.51], [-6.17, 1.57], [-2.38, -4.14]],
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
                "init_x": 2.65,
                "init_y": -3.33,
                "init_a": 163.02,
                "velocity": 0.82,
                "goals": [[-3.20, -2.39], [7.56, -4.22], [5.69, -3.96], [-0.88, 2.03]],
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
                "init_x": 1.75,
                "init_y": -3.77,
                "init_a": 163.02,
                "velocity": 0.82,
                "goals": [[-3.20, -2.39], [0.34, -0.96], [6.63, -1.72], [-1.27, 5.09]],
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
                "goals": [[2.48, -3.77], [-7.89, -5.37], [6.49, -0.88], [-7.35, -4.39]],
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
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.03,
                "init_y": -5.65,
                "init_a": -178.37,
                "velocity": 0.83,
                "goal_x": 3.40,
                "goal_y": 5.09,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_59_stopped_high(tester):
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
                "init_x": -2.03,
                "init_y": 2.97,
                "init_a": 120.09,
                "velocity": 1.04,
                "goals": [[-2.03, 2.97], [-7.82, 3.80], [-2.96, -5.00], [-3.10, 5.11]],
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
                "init_x": -2.50,
                "init_y": 2.09,
                "init_a": 120.09,
                "velocity": 1.04,
                "goals": [[-2.03, 2.97], [6.62, -2.37], [4.64, -2.10], [-4.42, 4.31]],
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
                "init_x": -7.88,
                "init_y": 2.24,
                "init_a": 29.74,
                "velocity": 0.95,
                "goals": [[-7.88, 2.24], [7.64, -1.65], [6.48, -3.23], [-6.01, -0.47]],
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
                "init_x": -8.30,
                "init_y": 1.33,
                "init_a": 29.74,
                "velocity": 0.95,
                "goals": [[-7.88, 2.24], [-4.18, -2.13], [-1.89, -5.18], [1.79, -2.85]],
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
                "init_x": 7.93,
                "init_y": -1.30,
                "init_a": -85.66,
                "velocity": 0.84,
                "goal_x": 2.91,
                "goal_y": 3.14,
                "n_actors": 9,
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
                "goals": [[-1.26, 5.49], [3.89, 3.07], [-4.13, -3.07], [-7.98, 5.99]],
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
                "goals": [[2.05, -3.99], [-6.95, 5.10], [2.21, -1.03], [4.35, 2.62]],
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

def tests_adult_30_child_70_test_case_60_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.28,
                "init_y": 2.23,
                "init_a": 54.76,
                "velocity": 1.11,
                "goals": [[4.38, 5.93], [-3.87, -3.08], [0.97, 3.80], [1.88, -5.63]],
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
                "init_x": -7.61,
                "init_y": 5.59,
                "init_a": 47.70,
                "velocity": 1.07,
                "goals": [[-3.66, -3.74], [-4.41, -5.41], [-0.89, -5.87], [-5.25, -2.58]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.55,
                "init_y": 3.34,
                "init_a": -140.22,
                "velocity": 0.94,
                "goals": [[-5.57, -4.46], [3.07, -2.68], [6.03, -0.78], [-2.18, 2.67]],
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
                "init_x": 6.07,
                "init_y": -5.15,
                "init_a": -178.76,
                "velocity": 0.98,
                "goals": [[6.53, 0.30], [-2.37, -3.48], [2.08, 0.60], [-7.72, -5.20]],
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
                "init_x": 4.62,
                "init_y": -4.18,
                "init_a": 89.81,
                "velocity": 0.86,
                "goals": [[3.93, -4.43], [-1.68, 1.26], [-1.05, 5.37], [-3.18, -4.11]],
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

def tests_adult_30_child_70_test_case_61_stopped_high(tester):
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
                "goals": [[0.99, 2.09], [5.22, 4.88], [1.59, -4.45], [0.80, -2.42]],
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
                "init_x": 1.99,
                "init_y": 2.04,
                "init_a": -141.43,
                "velocity": 0.95,
                "goals": [[0.99, 2.09], [-3.25, 4.23], [-3.36, -0.14], [-5.07, -5.19]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.90,
                "init_y": -4.48,
                "init_a": -110.00,
                "velocity": 0.90,
                "goals": [[1.07, -3.92], [0.93, -4.54], [3.54, -2.56], [-7.26, -5.90]],
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
                "init_x": -6.25,
                "init_y": 4.61,
                "init_a": -60.70,
                "velocity": 1.10,
                "goals": [[5.56, 3.38], [-0.86, 2.04], [-6.60, 0.42], [-1.98, -2.61]],
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

def tests_adult_30_child_70_test_case_62_stopped_high(tester):
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
                "goals": [[5.98, 4.30], [2.19, 2.27], [-6.97, -5.57], [-2.30, 1.20]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.33,
                "init_y": 1.09,
                "init_a": -64.29,
                "velocity": 1.12,
                "goals": [[-1.33, 1.09], [2.11, -2.99], [1.87, 5.35], [1.41, 2.29]],
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
                "init_x": -2.27,
                "init_y": 1.43,
                "init_a": -64.29,
                "velocity": 1.12,
                "goals": [[-1.33, 1.09], [3.43, 2.98], [5.05, -1.12], [-7.30, 2.43]],
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
                "init_x": 5.50,
                "init_y": 0.96,
                "init_a": 90.77,
                "velocity": 1.13,
                "goals": [[-6.09, -0.56], [5.37, -2.89], [2.23, 4.59], [4.71, -0.38]],
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
                "init_x": 4.42,
                "init_y": -2.28,
                "init_a": -147.87,
                "velocity": 0.97,
                "goals": [[1.69, -0.46], [2.76, 2.85], [2.06, -5.39], [1.88, 1.41]],
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
                "goals": [[3.27, 4.54], [-2.39, -4.56], [-7.44, -4.36], [6.46, 5.60]],
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

def tests_adult_30_child_70_test_case_63_stopped_high(tester):
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
                "goals": [[7.87, -2.53], [-4.15, -1.83], [4.29, -2.54], [4.85, 3.32]],
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
                "init_x": 8.09,
                "init_y": -1.55,
                "init_a": -157.72,
                "velocity": 1.19,
                "goal_x": 7.87,
                "goal_y": -2.53,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.47,
                "init_y": -3.00,
                "init_a": 168.65,
                "velocity": 0.89,
                "goals": [[1.47, -3.00], [1.73, -1.25], [4.42, -3.72], [3.22, 4.15]],
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
                "init_x": 1.01,
                "init_y": -2.10,
                "init_a": 168.65,
                "velocity": 0.89,
                "goals": [[1.47, -3.00], [6.66, -3.13], [6.59, -5.46], [-5.90, 0.19]],
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
                "init_x": 5.36,
                "init_y": -4.22,
                "init_a": 74.54,
                "velocity": 1.07,
                "goals": [[-6.73, 0.89], [-7.77, 5.40], [7.10, 2.61], [0.79, 4.84]],
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
                "init_x": -4.36,
                "init_y": 0.96,
                "init_a": -136.63,
                "velocity": 1.11,
                "goal_x": 7.32,
                "goal_y": 0.24,
                "n_actors": 7,
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

def tests_adult_30_child_70_test_case_64_walking_high(tester):
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
                "goals": [[6.25, -5.10], [2.21, -2.18], [0.17, -3.20], [4.35, -3.53]],
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
                "goals": [[-6.82, -1.31], [5.26, -0.74], [-3.19, -1.85], [2.45, 5.09]],
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
                "goals": [[-6.82, -1.31], [1.21, 4.06], [3.90, 4.69], [5.80, -5.77]],
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
                "goals": [[-4.24, -0.40], [1.76, -5.60], [-4.82, -0.90], [-1.92, -5.40]],
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
                "init_x": -1.12,
                "init_y": 3.43,
                "init_a": 12.27,
                "velocity": 1.14,
                "goal_x": 1.58,
                "goal_y": -0.89,
                "n_actors": 8,
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
                "goals": [[5.09, -3.04], [6.95, -1.78], [6.39, 6.00], [0.53, 5.06]],
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

def tests_adult_30_child_70_test_case_65_stopped_high(tester):
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
                "init_x": 3.61,
                "init_y": -4.95,
                "init_a": -6.29,
                "velocity": 0.99,
                "goals": [[3.61, -4.95], [4.80, -1.26], [-5.24, -5.52], [5.64, 2.21]],
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
                "init_x": 2.64,
                "init_y": -5.16,
                "init_a": -6.29,
                "velocity": 0.99,
                "goals": [[3.61, -4.95], [-0.19, -3.38], [-1.05, 1.06], [-2.12, 4.06]],
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
                "init_x": -0.85,
                "init_y": -1.73,
                "init_a": -0.49,
                "velocity": 0.82,
                "goals": [[-0.85, -1.73], [-5.63, 2.10], [-4.55, -2.65], [-7.13, -0.14]],
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
                "init_x": -1.32,
                "init_y": -2.61,
                "init_a": -0.49,
                "velocity": 0.82,
                "goal_x": -0.85,
                "goal_y": -1.73,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.80,
                "init_y": 2.56,
                "init_a": -26.85,
                "velocity": 1.05,
                "goals": [[-1.73, -5.06], [-5.02, -0.53], [7.19, 3.78], [4.99, -4.91]],
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
                "init_x": 3.53,
                "init_y": -4.42,
                "init_a": 131.50,
                "velocity": 1.16,
                "goals": [[-5.68, -0.30], [2.55, 2.86], [0.50, 1.32], [-3.46, 4.98]],
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
                "init_x": 0.62,
                "init_y": 2.74,
                "init_a": 44.82,
                "velocity": 1.10,
                "goals": [[0.76, 5.05], [5.42, 0.10], [4.00, 2.98], [-8.00, 3.41]],
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
                "init_x": -3.62,
                "init_y": -5.83,
                "init_a": -68.59,
                "velocity": 0.87,
                "goal_x": 7.60,
                "goal_y": -5.23,
                "n_actors": 9,
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

def tests_adult_30_child_70_test_case_66_stopped_high(tester):
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
                "goals": [[-6.27, -5.88], [-3.41, 5.27], [0.44, 4.76], [-5.43, -0.60]],
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
                "goals": [[-1.27, -0.65], [-2.41, -0.85], [-0.31, -4.53], [5.47, -0.17]],
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
                "init_x": -0.28,
                "init_y": -0.50,
                "init_a": 52.54,
                "velocity": 1.02,
                "goals": [[-1.27, -0.65], [6.61, 0.40], [0.73, 2.96], [0.17, -5.30]],
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
                "init_x": 0.42,
                "init_y": -4.77,
                "init_a": -108.03,
                "velocity": 0.98,
                "goals": [[-7.09, -5.41], [2.40, 0.33], [-0.87, -4.99], [-1.09, -5.90]],
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
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.13,
                "init_y": -4.25,
                "init_a": 128.99,
                "velocity": 0.83,
                "goal_x": -2.46,
                "goal_y": -4.50,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_67_stopped_high(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.06,
                "init_y": -4.08,
                "init_a": -98.20,
                "velocity": 1.20,
                "goals": [[6.06, -4.08], [-1.17, -2.07], [-3.24, -5.24], [0.05, -4.77]],
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
                "init_x": 6.67,
                "init_y": -4.87,
                "init_a": -98.20,
                "velocity": 1.20,
                "goal_x": 6.06,
                "goal_y": -4.08,
                "n_actors": 7,
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
                "goals": [[-3.04, -2.77], [6.79, 0.80], [0.98, 0.77], [-7.75, 4.51]],
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
                "init_x": 6.91,
                "init_y": -0.27,
                "init_a": 165.91,
                "velocity": 1.16,
                "goals": [[-3.49, 5.11], [-2.88, -1.23], [-3.52, -0.01], [6.29, 4.08]],
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
                "init_x": 2.94,
                "init_y": 2.24,
                "init_a": -9.44,
                "velocity": 0.88,
                "goals": [[-4.85, 2.66], [-3.82, 4.70], [-7.25, -0.95], [-7.37, -3.11]],
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

def tests_adult_30_child_70_test_case_68_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.79,
                "init_y": -0.82,
                "init_a": -39.32,
                "velocity": 1.01,
                "goals": [[-1.49, -0.10], [4.41, 1.19], [-5.92, -3.95], [5.04, -5.99]],
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
                "init_x": -4.26,
                "init_y": 2.08,
                "init_a": -104.95,
                "velocity": 0.82,
                "goals": [[-4.26, 2.08], [-5.49, 0.47], [-2.64, -1.08], [-3.48, 4.68]],
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
                "init_x": -5.26,
                "init_y": 2.07,
                "init_a": -104.95,
                "velocity": 0.82,
                "goal_x": -4.26,
                "goal_y": 2.08,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.91,
                "init_y": -2.99,
                "init_a": -64.56,
                "velocity": 0.99,
                "goals": [[7.74, -1.73], [-4.52, -2.64], [-4.75, 5.77], [1.71, 4.21]],
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
                "init_x": -6.31,
                "init_y": -5.70,
                "init_a": 121.34,
                "velocity": 1.11,
                "goal_x": 2.91,
                "goal_y": 4.38,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.17,
                "init_y": 4.17,
                "init_a": 4.82,
                "velocity": 1.16,
                "goals": [[-2.58, 4.79], [-5.21, -2.44], [3.72, -4.56], [-3.74, 5.14]],
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
                "init_x": -1.55,
                "init_y": -1.39,
                "init_a": -159.26,
                "velocity": 1.05,
                "goals": [[-5.41, 2.67], [-4.79, -2.79], [0.20, -3.95], [-6.07, -4.95]],
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

def tests_adult_30_child_70_test_case_69_stopped_high(tester):
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
                "init_x": 4.76,
                "init_y": -5.57,
                "init_a": -169.40,
                "velocity": 0.82,
                "goals": [[4.76, -5.57], [3.75, -0.06], [-1.65, 3.36], [6.88, -4.56]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.47,
                "init_y": 5.59,
                "init_a": 19.05,
                "velocity": 0.91,
                "goals": [[-3.47, 5.59], [-3.17, 5.98], [-5.22, -0.81], [-6.99, 4.41]],
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
                "init_x": -2.53,
                "init_y": 5.93,
                "init_a": 19.05,
                "velocity": 0.91,
                "goal_x": -3.47,
                "goal_y": 5.59,
                "n_actors": 7,
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
                "goals": [[-0.14, -4.01], [-7.24, -1.77], [7.05, 2.54], [0.15, -0.68]],
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
                "goals": [[-7.32, -3.19], [-0.27, -2.70], [-0.42, -0.32], [7.32, 2.11]],
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

def tests_adult_30_child_70_test_case_70_stopped_high(tester):
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
                "goals": [[-5.48, 3.20], [3.75, -4.27], [-7.37, -4.54], [-5.56, -3.48]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.16,
                "init_y": -5.71,
                "init_a": -47.27,
                "velocity": 1.07,
                "goals": [[-5.16, -5.71], [-5.33, 2.49], [5.24, -5.43], [-1.50, -5.99]],
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
                "init_x": -6.16,
                "init_y": -5.72,
                "init_a": -47.27,
                "velocity": 1.07,
                "goals": [[-5.16, -5.71], [-1.34, 3.18], [-4.65, -3.12], [3.09, -5.26]],
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
                "goals": [[-3.56, 5.83], [-5.33, 2.82], [-3.00, 0.94], [-2.45, -0.22]],
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

def tests_adult_30_child_70_test_case_71_stopped_high(tester):
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
                "init_x": -6.42,
                "init_y": -2.08,
                "init_a": 113.86,
                "velocity": 1.15,
                "goal_x": -6.42,
                "goal_y": -2.08,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.56,
                "init_y": -1.09,
                "init_a": 113.86,
                "velocity": 1.15,
                "goals": [[-6.42, -2.08], [7.75, -4.03], [-2.75, -0.94], [6.03, -1.61]],
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
                "init_x": 5.10,
                "init_y": 3.40,
                "init_a": -32.45,
                "velocity": 0.85,
                "goals": [[5.10, 3.40], [-1.26, 5.97], [-0.52, 4.83], [-4.80, 1.03]],
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
                "init_x": 5.51,
                "init_y": 4.31,
                "init_a": -32.45,
                "velocity": 0.85,
                "goals": [[5.10, 3.40], [0.64, -2.11], [5.15, -0.62], [1.64, -5.37]],
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
                "init_x": -1.34,
                "init_y": -3.02,
                "init_a": 143.97,
                "velocity": 0.90,
                "goals": [[5.00, 4.08], [-1.17, -3.11], [2.65, 0.31], [-7.62, -2.42]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.19,
                "init_y": 3.11,
                "init_a": 10.38,
                "velocity": 1.14,
                "goals": [[-5.77, -4.61], [0.01, -3.95], [-2.41, 0.84], [-0.70, 1.77]],
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
                "init_x": -4.61,
                "init_y": -2.80,
                "init_a": 109.36,
                "velocity": 1.04,
                "goals": [[3.58, -3.18], [3.15, -0.66], [6.84, 1.99], [-7.00, 5.10]],
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

def tests_adult_30_child_70_test_case_72_walking_high(tester):
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
                "init_x": 5.59,
                "init_y": -0.34,
                "init_a": -148.95,
                "velocity": 1.15,
                "goals": [[-7.07, -5.05], [-1.51, -2.83], [-0.88, 3.90], [-4.29, 5.38]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.19,
                "init_y": -2.77,
                "init_a": 93.35,
                "velocity": 0.82,
                "goals": [[1.89, -3.33], [-1.21, 2.16], [4.63, 3.52], [-0.55, 2.30]],
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
                "goals": [[5.54, -2.21], [-7.06, -1.80], [6.66, -2.81], [-3.38, 1.49]],
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
                "init_x": -1.08,
                "init_y": -2.17,
                "init_a": -34.84,
                "velocity": 1.12,
                "goal_x": 2.74,
                "goal_y": 5.39,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.56,
                "init_y": 3.81,
                "init_a": -153.11,
                "velocity": 0.91,
                "goals": [[-1.63, -2.04], [4.08, 4.53], [4.52, 5.20], [6.25, 1.74]],
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
                "init_x": -2.76,
                "init_y": 2.48,
                "init_a": -108.38,
                "velocity": 0.85,
                "goals": [[7.02, 1.04], [-2.31, 2.35], [-5.92, -3.75], [-4.33, 1.63]],
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
                "init_x": -5.74,
                "init_y": 4.17,
                "init_a": 160.26,
                "velocity": 1.18,
                "goals": [[-4.82, 1.87], [-3.37, -4.14], [-4.46, 0.36], [-7.23, 3.96]],
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

def tests_adult_30_child_70_test_case_73_stopped_high(tester):
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
                "init_x": 5.90,
                "init_y": -2.77,
                "init_a": -116.60,
                "velocity": 0.82,
                "goal_x": 5.90,
                "goal_y": -2.77,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.90,
                "init_y": -2.68,
                "init_a": -116.60,
                "velocity": 0.82,
                "goals": [[5.90, -2.77], [3.84, 0.77], [7.19, 2.15], [5.88, -5.02]],
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
                "init_x": 7.95,
                "init_y": -3.46,
                "init_a": 157.32,
                "velocity": 0.82,
                "goals": [[7.95, -3.46], [1.02, 3.55], [7.21, 1.04], [0.75, 5.64]],
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
                "init_x": 8.16,
                "init_y": -4.44,
                "init_a": 157.32,
                "velocity": 0.82,
                "goals": [[7.95, -3.46], [7.30, 2.45], [2.04, 5.93], [-4.93, 0.68]],
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
                "init_x": -0.55,
                "init_y": 0.92,
                "init_a": -103.91,
                "velocity": 1.17,
                "goal_x": -5.19,
                "goal_y": 5.35,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.95,
                "init_y": 4.34,
                "init_a": -102.42,
                "velocity": 1.15,
                "goals": [[-2.42, 4.76], [4.06, 4.38], [6.57, 0.14], [-6.20, -5.02]],
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
                "init_x": -6.54,
                "init_y": 5.24,
                "init_a": -112.55,
                "velocity": 1.03,
                "goals": [[-1.60, -0.72], [-7.46, -0.50], [-0.09, -0.95], [2.81, 2.37]],
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
                "init_x": 1.71,
                "init_y": 2.52,
                "init_a": -136.41,
                "velocity": 0.82,
                "goal_x": 4.19,
                "goal_y": 1.06,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_74_walking_high(tester):
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
                "goals": [[-6.21, 2.48], [2.05, 3.86], [4.66, 4.16], [2.57, -1.76]],
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
                "goals": [[2.09, 3.91], [1.82, 1.53], [-6.40, -1.43], [4.00, -0.16]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.72,
                "init_y": 3.96,
                "init_a": 60.85,
                "velocity": 1.01,
                "goals": [[7.20, -2.26], [4.56, -5.83], [-6.32, -3.03], [-5.51, 3.36]],
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
                "init_x": -0.94,
                "init_y": 4.46,
                "init_a": -80.30,
                "velocity": 0.92,
                "goals": [[7.66, -1.08], [5.88, -5.06], [6.41, -1.11], [0.97, 2.57]],
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
                "init_x": 6.72,
                "init_y": 1.94,
                "init_a": 73.06,
                "velocity": 0.81,
                "goals": [[1.90, 2.82], [6.76, 0.90], [-5.49, -2.42], [0.42, 3.44]],
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

def tests_adult_30_child_70_test_case_75_walking_high(tester):
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
                "init_x": -6.37,
                "init_y": 3.43,
                "init_a": 72.15,
                "velocity": 0.99,
                "goals": [[4.86, 1.57], [-4.93, 1.98], [-2.94, 2.17], [-2.44, 0.10]],
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
                "goals": [[-4.17, 2.05], [-4.47, 4.90], [-6.76, 2.51], [3.17, 1.15]],
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
                "init_x": -2.49,
                "init_y": 0.45,
                "init_a": 25.33,
                "velocity": 1.13,
                "goal_x": 0.54,
                "goal_y": -2.40,
                "n_actors": 7,
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
                "goals": [[-0.80, 0.52], [4.24, 3.49], [4.57, 3.53], [0.08, -1.58]],
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
                "init_x": -4.58,
                "init_y": 0.81,
                "init_a": -0.59,
                "velocity": 0.95,
                "goals": [[5.52, 3.58], [3.07, 3.62], [1.50, -0.18], [-3.23, 5.54]],
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

def tests_adult_30_child_70_test_case_76_walking_high(tester):
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
                "goals": [[3.87, -1.18], [-4.23, 5.91], [-7.86, 1.88], [4.84, -5.94]],
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
                "init_x": 5.45,
                "init_y": 3.03,
                "init_a": -16.75,
                "velocity": 0.85,
                "goals": [[3.87, -1.18], [-0.86, 1.16], [-7.98, -1.37], [-1.46, 5.52]],
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.56,
                "init_y": -1.81,
                "init_a": -45.64,
                "velocity": 1.05,
                "goals": [[1.25, -5.61], [-4.11, -3.07], [5.29, -5.21], [-0.28, 0.21]],
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
                "init_x": -0.31,
                "init_y": 4.43,
                "init_a": 179.07,
                "velocity": 1.10,
                "goals": [[-6.27, -3.68], [-0.41, -2.59], [4.97, -4.02], [-4.67, -4.55]],
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

def tests_adult_30_child_70_test_case_77_stopped_high(tester):
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
                "init_x": 0.53,
                "init_y": 3.14,
                "init_a": -1.21,
                "velocity": 0.87,
                "goals": [[0.53, 3.14], [-5.27, -5.97], [-6.36, 1.71], [2.20, -2.18]],
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
                "init_x": 0.49,
                "init_y": 4.14,
                "init_a": -1.21,
                "velocity": 0.87,
                "goals": [[0.53, 3.14], [-5.34, -2.14], [-0.21, 2.58], [5.13, -5.95]],
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
                "init_x": 2.35,
                "init_y": -0.77,
                "init_a": 26.77,
                "velocity": 0.94,
                "goals": [[2.35, -0.77], [0.61, -0.60], [3.81, -2.27], [-4.90, -1.46]],
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
                "init_x": 2.92,
                "init_y": 0.05,
                "init_a": 26.77,
                "velocity": 0.94,
                "goals": [[2.35, -0.77], [0.19, -3.02], [3.22, -2.44], [7.63, 2.42]],
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
                "init_x": 5.25,
                "init_y": -3.23,
                "init_a": -165.99,
                "velocity": 0.82,
                "goal_x": 3.45,
                "goal_y": -1.11,
                "n_actors": 9,
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.91,
                "init_y": -0.23,
                "init_a": 1.76,
                "velocity": 0.95,
                "goals": [[-3.93, -4.08], [5.22, -3.25], [-5.50, -3.92], [4.93, -0.13]],
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
                "init_x": -4.15,
                "init_y": 0.64,
                "init_a": -66.29,
                "velocity": 1.02,
                "goals": [[4.76, -1.54], [-6.26, 1.38], [5.95, 1.20], [-4.99, 2.64]],
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

def tests_adult_30_child_70_test_case_78_stopped_high(tester):
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
                "init_x": 0.22,
                "init_y": -1.46,
                "init_a": 178.09,
                "velocity": 0.94,
                "goals": [[0.22, -1.46], [-4.94, 4.55], [1.90, 2.98], [-3.18, -4.27]],
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
                "goals": [[-2.86, -0.75], [-6.08, -3.30], [-5.00, 1.68], [1.63, -1.60]],
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
                "init_x": -3.84,
                "init_y": -0.57,
                "init_a": -5.96,
                "velocity": 1.13,
                "goal_x": -2.86,
                "goal_y": -0.75,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.81,
                "init_y": 0.45,
                "init_a": 124.19,
                "velocity": 0.93,
                "goals": [[2.35, 4.29], [-6.08, 4.83], [4.86, -5.77], [-1.32, -4.79]],
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
                "init_x": -1.43,
                "init_y": -4.88,
                "init_a": -71.90,
                "velocity": 0.83,
                "goals": [[5.33, -1.40], [-4.80, -3.12], [3.09, -2.02], [5.51, 4.73]],
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
                "init_x": 6.47,
                "init_y": -3.56,
                "init_a": -31.43,
                "velocity": 0.94,
                "goal_x": 1.28,
                "goal_y": 2.97,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_79_walking_high(tester):
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
                "goals": [[-3.51, -2.41], [7.75, 3.15], [3.74, -3.78], [-0.05, -5.09]],
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
                "goals": [[-3.51, -2.41], [0.33, 0.04], [2.67, 0.04], [6.06, 0.94]],
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
                "init_x": -2.61,
                "init_y": -4.98,
                "init_a": 87.89,
                "velocity": 0.89,
                "goals": [[-1.48, 0.97], [1.67, -5.50], [3.18, 3.48], [1.18, -1.50]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.75,
                "init_y": -0.80,
                "init_a": -46.44,
                "velocity": 1.18,
                "goals": [[-4.68, 5.18], [3.63, -5.14], [0.60, -2.33], [-0.17, 0.24]],
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
                "init_x": -6.07,
                "init_y": 0.80,
                "init_a": -131.31,
                "velocity": 0.94,
                "goals": [[4.55, 3.13], [-1.98, -3.75], [1.91, -4.18], [2.16, -1.21]],
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
                "init_x": 5.14,
                "init_y": -4.57,
                "init_a": -60.25,
                "velocity": 1.17,
                "goals": [[4.62, 1.01], [-7.97, 2.93], [6.75, -0.27], [-5.38, -1.19]],
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

def tests_adult_30_child_70_test_case_80_stopped_high(tester):
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
                "init_x": -6.45,
                "init_y": -3.35,
                "init_a": -144.16,
                "velocity": 0.82,
                "goal_x": -6.45,
                "goal_y": -3.35,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.11,
                "init_y": -4.10,
                "init_a": -144.16,
                "velocity": 0.82,
                "goals": [[-6.45, -3.35], [-3.95, 4.56], [3.58, -0.16], [-3.30, 2.09]],
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
                "init_x": 2.43,
                "init_y": 4.67,
                "init_a": -42.80,
                "velocity": 1.11,
                "goals": [[2.43, 4.67], [-5.86, -5.44], [0.03, -4.99], [-1.62, -0.96]],
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
                "goals": [[2.43, 4.67], [-4.31, -0.18], [0.09, -4.33], [2.64, -5.72]],
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
                "init_x": 0.82,
                "init_y": 4.40,
                "init_a": -82.79,
                "velocity": 0.92,
                "goal_x": 3.44,
                "goal_y": 5.40,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.96,
                "init_y": 2.58,
                "init_a": -26.58,
                "velocity": 0.87,
                "goals": [[6.46, 2.05], [-5.97, -1.07], [0.50, 0.06], [1.81, 3.80]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.16,
                "init_y": 1.19,
                "init_a": -37.19,
                "velocity": 0.86,
                "goals": [[-5.04, 3.70], [1.59, -1.74], [5.29, 4.98], [-2.36, -5.43]],
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
                "init_x": -6.89,
                "init_y": -5.73,
                "init_a": 35.40,
                "velocity": 1.16,
                "goals": [[-3.32, -2.47], [0.54, -1.95], [3.72, 0.56], [7.14, 1.24]],
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

def tests_adult_30_child_70_test_case_81_stopped_high(tester):
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
                "goals": [[-0.01, 0.38], [-4.57, -5.06], [-4.04, -3.91], [2.37, -2.63]],
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
                "init_x": 6.90,
                "init_y": -2.01,
                "init_a": 32.91,
                "velocity": 1.03,
                "goals": [[6.90, -2.01], [-3.58, -3.69], [-0.81, 5.34], [-6.80, 3.59]],
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
                "init_x": 6.39,
                "init_y": -2.87,
                "init_a": 32.91,
                "velocity": 1.03,
                "goals": [[6.90, -2.01], [-3.83, 2.37], [6.16, 5.98], [6.40, -0.05]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.00,
                "init_y": -0.40,
                "init_a": -135.14,
                "velocity": 1.03,
                "goals": [[1.92, -4.35], [1.85, 0.83], [-1.29, -0.35], [-5.29, 1.10]],
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

def tests_adult_30_child_70_test_case_82_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.36,
                "init_y": 5.25,
                "init_a": 158.47,
                "velocity": 0.95,
                "goals": [[3.52, -4.79], [-0.77, 2.09], [-2.03, -4.43], [0.06, 4.12]],
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
                "init_x": 2.30,
                "init_y": -4.23,
                "init_a": 125.02,
                "velocity": 1.15,
                "goals": [[-6.63, -1.13], [0.40, 3.91], [6.50, 3.35], [-2.27, 3.44]],
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
                "init_x": 2.18,
                "init_y": -3.24,
                "init_a": 125.02,
                "velocity": 1.15,
                "goals": [[-6.63, -1.13], [-1.34, 5.29], [6.27, 1.09], [4.70, 0.94]],
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
                "init_x": -0.53,
                "init_y": 4.79,
                "init_a": 106.27,
                "velocity": 1.19,
                "goals": [[-7.03, -5.84], [7.62, -1.67], [-0.70, 1.65], [-0.64, -3.02]],
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
                "init_x": 1.90,
                "init_y": 5.17,
                "init_a": 71.36,
                "velocity": 1.00,
                "goals": [[5.11, -2.87], [7.18, 1.68], [-5.74, 1.28], [-3.26, 5.58]],
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
                "goals": [[-2.11, 1.32], [-3.83, -2.18], [2.58, -5.61], [0.94, 0.40]],
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
                "init_x": 5.44,
                "init_y": -3.03,
                "init_a": -44.47,
                "velocity": 1.02,
                "goal_x": 7.43,
                "goal_y": 2.56,
                "n_actors": 9,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_83_walking_high(tester):
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
                "goals": [[7.21, 2.86], [2.14, -2.03], [-4.59, 5.57], [-7.15, 1.88]],
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
                "goals": [[3.62, -5.16], [0.10, -2.76], [-4.37, 1.50], [-5.21, -0.66]],
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
                "init_x": -5.37,
                "init_y": 2.13,
                "init_a": -155.57,
                "velocity": 0.88,
                "goals": [[4.58, -4.29], [-7.46, 2.05], [-2.52, 0.38], [0.78, 3.88]],
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
                "init_x": 5.52,
                "init_y": 0.26,
                "init_a": 163.92,
                "velocity": 1.11,
                "goals": [[-1.51, 4.88], [-4.78, 3.39], [-2.55, -2.13], [-4.14, -0.00]],
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
                "init_x": -6.52,
                "init_y": 0.83,
                "init_a": -94.56,
                "velocity": 1.05,
                "goal_x": 7.27,
                "goal_y": -3.39,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_84_walking_high(tester):
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
                "goals": [[-5.76, -4.85], [-3.36, -0.09], [-7.99, -4.40], [4.34, 4.19]],
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
                "init_x": 7.91,
                "init_y": -2.52,
                "init_a": 23.60,
                "velocity": 0.92,
                "goals": [[-5.76, -4.85], [5.78, -0.47], [2.54, -0.78], [-4.71, -3.55]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.87,
                "init_y": 3.19,
                "init_a": -53.55,
                "velocity": 1.04,
                "goals": [[-2.14, -0.38], [2.27, 4.57], [0.95, 0.66], [2.60, 5.35]],
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
                "init_x": 5.43,
                "init_y": -1.02,
                "init_a": -141.47,
                "velocity": 0.87,
                "goals": [[6.90, 1.19], [-7.58, -2.45], [-6.68, -4.33], [2.92, -3.24]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.38,
                "init_y": 3.85,
                "init_a": -135.10,
                "velocity": 0.88,
                "goals": [[-4.01, -3.23], [2.82, 5.40], [1.00, -3.57], [-2.82, 3.09]],
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

def tests_adult_30_child_70_test_case_85_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.64,
                "init_y": -1.16,
                "init_a": -27.93,
                "velocity": 1.01,
                "goals": [[-3.10, -5.21], [-7.56, 2.01], [6.52, 0.84], [-4.30, 1.01]],
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
                "init_x": -4.45,
                "init_y": -3.63,
                "init_a": 63.92,
                "velocity": 0.81,
                "goals": [[-2.52, -2.69], [0.10, -0.46], [-7.15, -4.90], [3.86, 1.02]],
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
                "goals": [[4.55, -3.90], [5.40, 0.96], [3.45, -2.78], [1.19, 2.63]],
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
                "init_x": -6.64,
                "init_y": 5.97,
                "init_a": -69.34,
                "velocity": 0.95,
                "goals": [[-7.63, -0.95], [5.47, 3.80], [6.97, -0.40], [-3.76, 3.23]],
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
                "init_x": 3.73,
                "init_y": 2.84,
                "init_a": -155.75,
                "velocity": 0.86,
                "goals": [[-2.30, 2.64], [2.23, -0.19], [7.23, 0.46], [-6.65, -3.91]],
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

def tests_adult_30_child_70_test_case_86_walking_high(tester):
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
                "goals": [[5.91, 3.44], [-3.34, -4.97], [-7.80, -2.70], [-3.69, -1.51]],
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
                "init_x": 2.12,
                "init_y": -3.29,
                "init_a": -165.80,
                "velocity": 1.03,
                "goals": [[5.91, 3.44], [5.40, -4.48], [-7.72, 2.34], [-7.74, -5.88]],
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
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.11,
                "init_y": -3.70,
                "init_a": 69.15,
                "velocity": 0.83,
                "goal_x": -4.90,
                "goal_y": 1.72,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.31,
                "init_y": 4.99,
                "init_a": -140.64,
                "velocity": 0.80,
                "goals": [[2.81, -0.67], [-4.94, -5.25], [4.78, -1.02], [3.16, 3.81]],
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
                "init_x": 7.49,
                "init_y": -3.22,
                "init_a": -15.36,
                "velocity": 0.98,
                "goals": [[5.80, -4.98], [-2.35, -5.91], [7.92, 5.34], [-4.84, 1.57]],
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

def tests_adult_30_child_70_test_case_87_walking_high(tester):
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
                "goals": [[5.78, 0.95], [0.44, -0.64], [-0.22, -3.30], [6.75, -5.45]],
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
                "init_x": -8.00,
                "init_y": -1.91,
                "init_a": -3.83,
                "velocity": 1.08,
                "goals": [[5.78, 0.95], [-7.59, 2.82], [-3.39, -2.15], [-5.04, 1.62]],
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
                "init_x": 2.26,
                "init_y": 5.03,
                "init_a": 23.30,
                "velocity": 0.92,
                "goal_x": 5.24,
                "goal_y": -3.04,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.26,
                "init_y": 5.11,
                "init_a": 23.30,
                "velocity": 0.92,
                "goals": [[5.24, -3.04], [6.20, -5.83], [4.95, -1.95], [-1.98, 4.48]],
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
                "init_x": -0.07,
                "init_y": 5.51,
                "init_a": 143.54,
                "velocity": 0.99,
                "goals": [[4.63, 5.51], [-3.45, -3.80], [-1.17, 2.47], [-4.07, 5.35]],
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
                "init_x": -6.58,
                "init_y": 3.11,
                "init_a": -42.04,
                "velocity": 0.83,
                "goals": [[-4.66, -0.15], [5.31, -2.06], [-3.76, -3.23], [-5.37, 0.38]],
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
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.35,
                "init_y": -3.91,
                "init_a": 11.04,
                "velocity": 0.92,
                "goal_x": -4.62,
                "goal_y": 5.58,
                "n_actors": 8,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_88_walking_high(tester):
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
                "init_x": 5.98,
                "init_y": 5.18,
                "init_a": -52.22,
                "velocity": 1.14,
                "goals": [[2.02, -4.36], [5.65, -0.73], [0.47, -2.98], [2.97, 5.68]],
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
                "init_x": 5.14,
                "init_y": 4.63,
                "init_a": -52.22,
                "velocity": 1.14,
                "goals": [[2.02, -4.36], [-4.51, 5.99], [-3.18, 3.43], [-5.79, 3.86]],
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
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.79,
                "init_y": 2.89,
                "init_a": -88.80,
                "velocity": 0.97,
                "goal_x": -6.64,
                "goal_y": -0.16,
                "n_actors": 9,
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
                "goals": [[-0.34, -0.49], [2.49, -5.45], [4.24, 4.08], [-7.20, -2.94]],
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
                "goals": [[-6.28, -0.19], [-0.77, 2.72], [-6.87, 1.77], [4.19, 4.89]],
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
                "init_x": 1.52,
                "init_y": -1.11,
                "init_a": 147.69,
                "velocity": 0.99,
                "goal_x": -1.04,
                "goal_y": 1.45,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.08,
                "init_y": -0.56,
                "init_a": 135.74,
                "velocity": 1.04,
                "goals": [[-4.79, 5.44], [-3.55, -3.40], [5.14, -4.95], [-5.63, -3.36]],
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
                "init_x": 3.71,
                "init_y": -4.46,
                "init_a": 159.18,
                "velocity": 1.03,
                "goals": [[-6.25, 5.48], [5.11, -4.98], [-6.00, 4.49], [0.31, -2.90]],
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

def tests_adult_30_child_70_test_case_89_stopped_high(tester):
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
                "goals": [[-1.11, 2.39], [7.30, -3.66], [4.50, 5.27], [-3.59, -2.93]],
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
                "goals": [[-4.99, -0.89], [-5.94, 0.26], [-6.14, -1.11], [1.54, 4.62]],
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
                "init_x": 3.10,
                "init_y": 1.38,
                "init_a": -62.99,
                "velocity": 0.83,
                "goal_x": 7.40,
                "goal_y": 5.93,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.01,
                "init_y": -1.61,
                "init_a": -37.60,
                "velocity": 0.87,
                "goals": [[3.92, 0.83], [-0.87, 5.39], [-0.15, -4.07], [4.56, -3.94]],
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
                "init_x": -5.04,
                "init_y": 1.09,
                "init_a": 172.38,
                "velocity": 1.10,
                "goals": [[2.50, 4.84], [-7.46, -2.76], [-4.27, -5.72], [-5.74, 0.58]],
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

def tests_adult_30_child_70_test_case_90_stopped_high(tester):
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
                "goals": [[-0.93, 2.74], [1.20, -1.10], [-7.57, 4.74], [2.37, -2.26]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.13,
                "init_y": -4.23,
                "init_a": 112.02,
                "velocity": 1.17,
                "goals": [[-3.13, -4.23], [2.31, 1.91], [-4.57, 0.16], [3.22, -2.09]],
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
                "init_x": -4.07,
                "init_y": -3.89,
                "init_a": 112.02,
                "velocity": 1.17,
                "goals": [[-3.13, -4.23], [-5.93, 0.03], [4.71, 3.92], [4.09, -4.58]],
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
                "init_x": -3.05,
                "init_y": 5.39,
                "init_a": -4.71,
                "velocity": 1.16,
                "goals": [[2.37, 2.14], [-6.76, 2.48], [0.43, -4.42], [-5.90, -4.84]],
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
                "init_x": 1.51,
                "init_y": 2.65,
                "init_a": 171.66,
                "velocity": 1.19,
                "goals": [[-5.78, -4.44], [7.94, 5.33], [2.70, -4.67], [3.72, -2.53]],
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
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.10,
                "init_y": -1.78,
                "init_a": 109.96,
                "velocity": 0.99,
                "goals": [[-6.33, -5.26], [0.96, 2.08], [1.23, 3.40], [-2.85, 2.80]],
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

def tests_adult_30_child_70_test_case_91_stopped_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.96,
                "init_y": 4.50,
                "init_a": -13.42,
                "velocity": 0.93,
                "goal_x": -6.96,
                "goal_y": 4.39,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.51,
                "init_y": 4.82,
                "init_a": -48.16,
                "velocity": 1.02,
                "goals": [[-2.51, 4.82], [7.13, -3.45], [-0.76, 3.36], [-5.53, 4.81]],
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
                "init_x": -3.51,
                "init_y": 4.69,
                "init_a": -48.16,
                "velocity": 1.02,
                "goals": [[-2.51, 4.82], [2.47, -0.93], [-7.11, 2.34], [0.01, -0.17]],
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
                "init_x": 1.75,
                "init_y": -2.91,
                "init_a": -108.30,
                "velocity": 1.20,
                "goal_x": -0.91,
                "goal_y": -0.02,
                "n_actors": 7,
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
                "goals": [[-6.31, 5.32], [-1.97, -4.22], [-6.43, -4.33], [1.95, -0.92]],
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
                "init_x": 2.58,
                "init_y": -4.65,
                "init_a": 113.75,
                "velocity": 0.99,
                "goals": [[4.20, 0.75], [6.66, -2.74], [-2.96, 0.15], [-5.17, -2.47]],
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

def tests_adult_30_child_70_test_case_92_stopped_high(tester):
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
                "goals": [[-0.37, 0.08], [4.05, -5.92], [-3.60, 5.55], [2.10, 3.14]],
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
                "init_x": -1.35,
                "init_y": -0.14,
                "init_a": 1.28,
                "velocity": 0.84,
                "goals": [[-0.37, 0.08], [-0.63, -3.09], [1.10, 5.87], [-0.15, -3.37]],
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
                "init_x": -4.59,
                "init_y": -0.39,
                "init_a": 90.21,
                "velocity": 1.16,
                "goals": [[-2.14, -4.77], [-1.03, -3.08], [2.19, 4.15], [3.43, 2.43]],
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
                "init_x": -4.56,
                "init_y": 1.09,
                "init_a": 71.03,
                "velocity": 1.05,
                "goals": [[-4.70, -1.62], [2.17, -0.91], [-6.55, -2.73], [-5.81, 5.32]],
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
                "init_x": 0.48,
                "init_y": 3.41,
                "init_a": 76.50,
                "velocity": 0.94,
                "goals": [[2.90, -5.64], [3.99, 1.01], [-7.15, -0.40], [-4.95, 0.47]],
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
                "init_x": -4.26,
                "init_y": 0.30,
                "init_a": 2.06,
                "velocity": 1.14,
                "goal_x": -7.29,
                "goal_y": 1.54,
                "n_actors": 9,
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
                "goals": [[3.51, 2.65], [2.61, 4.41], [3.27, 1.51], [3.87, -5.14]],
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

def tests_adult_30_child_70_test_case_93_walking_high(tester):
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
                "init_x": 5.83,
                "init_y": 4.03,
                "init_a": 26.10,
                "velocity": 0.99,
                "goals": [[-1.77, 3.82], [-4.39, -3.31], [0.27, -3.43], [7.32, -0.62]],
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
                "init_x": 5.51,
                "init_y": 4.97,
                "init_a": 26.10,
                "velocity": 0.99,
                "goal_x": -1.77,
                "goal_y": 3.82,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.97,
                "init_y": -3.55,
                "init_a": -139.66,
                "velocity": 0.89,
                "goals": [[-4.21, -5.96], [-3.03, 3.61], [-0.66, -4.80], [-3.17, -2.60]],
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
                "init_x": -2.97,
                "init_y": -3.59,
                "init_a": -139.66,
                "velocity": 0.89,
                "goal_x": -4.21,
                "goal_y": -5.96,
                "n_actors": 7,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.75,
                "init_y": 5.55,
                "init_a": 14.50,
                "velocity": 1.05,
                "goals": [[6.38, -4.82], [7.67, 5.39], [7.15, -1.17], [1.15, -2.40]],
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
                "init_x": -3.41,
                "init_y": 2.45,
                "init_a": -71.39,
                "velocity": 1.02,
                "goals": [[1.99, -4.98], [-6.82, -4.47], [-4.44, -5.86], [6.37, 2.23]],
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
                "init_x": -3.46,
                "init_y": 4.62,
                "init_a": 112.00,
                "velocity": 1.13,
                "goal_x": -0.18,
                "goal_y": -4.11,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_94_stopped_high(tester):
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
                "goals": [[2.89, 5.31], [0.45, -4.01], [-2.94, 2.30], [-5.94, 0.80]],
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
                "init_x": 2.51,
                "init_y": 6.23,
                "init_a": -3.78,
                "velocity": 0.94,
                "goals": [[2.89, 5.31], [3.76, -4.84], [-1.27, 2.86], [-1.92, 4.47]],
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
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.34,
                "init_y": -5.57,
                "init_a": -91.41,
                "velocity": 1.02,
                "goal_x": -1.03,
                "goal_y": -4.62,
                "n_actors": 8,
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.65,
                "init_y": -1.35,
                "init_a": -47.70,
                "velocity": 0.97,
                "goals": [[-0.98, 0.12], [4.11, 5.37], [2.97, -2.31], [-7.65, -0.50]],
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
                "init_x": 7.03,
                "init_y": 0.65,
                "init_a": 128.01,
                "velocity": 0.83,
                "goals": [[3.19, 3.93], [7.91, 4.97], [7.11, -3.96], [-1.38, -2.14]],
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
                "init_x": -4.12,
                "init_y": 3.61,
                "init_a": 116.14,
                "velocity": 1.14,
                "goals": [[-3.49, 2.80], [0.72, -1.72], [6.94, 1.75], [-4.21, -4.52]],
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

def tests_adult_30_child_70_test_case_95_stopped_high(tester):
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.81,
                "init_y": -0.97,
                "init_a": 177.66,
                "velocity": 1.08,
                "goals": [[-6.81, -0.97], [1.34, -2.88], [-3.49, -0.02], [-5.63, -2.39]],
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
                "init_x": -7.68,
                "init_y": -0.48,
                "init_a": 177.66,
                "velocity": 1.08,
                "goals": [[-6.81, -0.97], [2.81, -1.10], [-4.38, -5.90], [4.69, 2.09]],
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
                "goals": [[0.55, -4.29], [4.33, -5.93], [5.02, -1.84], [-4.11, 0.23]],
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
                "init_x": -0.46,
                "init_y": -2.46,
                "init_a": -1.37,
                "velocity": 0.86,
                "goals": [[7.35, -2.17], [6.48, 0.05], [1.64, 0.32], [-2.85, 1.29]],
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
                "init_x": 2.18,
                "init_y": -5.24,
                "init_a": -30.90,
                "velocity": 1.11,
                "goal_x": 4.81,
                "goal_y": -1.33,
                "n_actors": 8,
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
                "goals": [[-5.06, -5.11], [3.27, -2.03], [-7.11, -2.62], [4.05, 2.04]],
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

def tests_adult_30_child_70_test_case_96_stopped_high(tester):
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
                "goals": [[1.16, 3.19], [2.23, 0.40], [-1.65, 5.30], [-3.71, -3.98]],
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
                "init_x": 0.16,
                "init_y": 3.15,
                "init_a": -13.09,
                "velocity": 1.08,
                "goals": [[1.16, 3.19], [-6.50, 0.22], [7.18, -4.87], [-1.66, -5.81]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.64,
                "init_y": -3.32,
                "init_a": 151.53,
                "velocity": 1.05,
                "goals": [[-7.73, 3.23], [-0.88, -4.34], [-2.26, -0.99], [-4.03, 5.89]],
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
                "init_x": 1.24,
                "init_y": -2.78,
                "init_a": 87.39,
                "velocity": 1.01,
                "goals": [[-7.51, 2.53], [-5.89, 4.37], [-7.85, -3.00], [1.13, 0.91]],
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
                "init_x": 2.61,
                "init_y": 1.70,
                "init_a": -156.35,
                "velocity": 0.98,
                "goal_x": -2.04,
                "goal_y": -1.36,
                "n_actors": 9,
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
                "goals": [[6.30, 0.59], [6.46, -2.74], [3.30, -0.06], [7.60, 5.94]],
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
                "goals": [[3.25, -5.64], [0.76, -3.08], [-0.57, -5.54], [-3.88, -4.28]],
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

def tests_adult_30_child_70_test_case_97_walking_high(tester):
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
                "goals": [[2.60, -2.87], [-4.23, -3.48], [3.75, -2.55], [-4.78, 0.10]],
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
                "init_x": 7.33,
                "init_y": 1.60,
                "init_a": -160.74,
                "velocity": 0.94,
                "goals": [[2.60, -2.87], [4.68, 4.08], [3.41, -5.84], [4.38, 3.20]],
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
                "init_x": -2.91,
                "init_y": -0.96,
                "init_a": 75.76,
                "velocity": 0.97,
                "goals": [[-2.32, -4.17], [5.24, 5.66], [-1.63, -0.93], [1.01, -3.95]],
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
                "goals": [[-2.32, -4.17], [1.41, -4.92], [-1.09, 2.06], [7.77, -5.32]],
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
                "init_x": -0.39,
                "init_y": -0.57,
                "init_a": -133.27,
                "velocity": 0.99,
                "goals": [[-0.17, -4.25], [-1.98, 0.68], [-5.84, 4.18], [-4.29, -2.51]],
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
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.71,
                "init_y": -0.66,
                "init_a": -21.13,
                "velocity": 0.87,
                "goal_x": 4.26,
                "goal_y": -5.22,
                "n_actors": 8,
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

def tests_adult_30_child_70_test_case_98_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.85,
                "init_y": -1.31,
                "init_a": 89.69,
                "velocity": 0.96,
                "goals": [[-3.25, -0.40], [-3.43, -0.40], [1.56, 5.70], [-4.17, -1.65]],
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
                "init_x": 1.70,
                "init_y": -2.21,
                "init_a": -121.14,
                "velocity": 1.12,
                "goals": [[1.70, -2.21], [-4.07, 4.21], [-7.92, -5.08], [-3.25, 3.82]],
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
                "init_x": 2.19,
                "init_y": -1.34,
                "init_a": -121.14,
                "velocity": 1.12,
                "goals": [[1.70, -2.21], [5.24, 1.26], [2.12, -0.21], [6.27, 4.66]],
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
                "init_x": 3.55,
                "init_y": -2.58,
                "init_a": -105.13,
                "velocity": 0.83,
                "goals": [[-1.70, 5.79], [3.92, 3.23], [5.71, -5.49], [-1.13, 4.15]],
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
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.41,
                "init_y": -0.93,
                "init_a": -177.47,
                "velocity": 0.96,
                "goal_x": -7.36,
                "goal_y": 1.83,
                "n_actors": 7,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_30_child_70_test_case_99_walking_high(tester):
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
                "goals": [[-1.67, -5.92], [-6.95, -0.93], [-1.45, -0.63], [1.08, -4.22]],
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
                "init_x": -0.95,
                "init_y": -3.18,
                "init_a": 91.23,
                "velocity": 0.81,
                "goals": [[-1.67, -5.92], [-6.27, 5.03], [7.09, 5.25], [5.79, -0.62]],
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
                "goals": [[-4.45, -2.68], [-6.41, -2.69], [-0.73, 4.11], [2.18, 1.01]],
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
                "init_x": 1.33,
                "init_y": 1.47,
                "init_a": 171.72,
                "velocity": 0.87,
                "goals": [[-3.38, -1.67], [-4.64, 1.59], [6.25, -3.75], [3.59, -1.02]],
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
                "init_x": 6.98,
                "init_y": 0.25,
                "init_a": -67.56,
                "velocity": 1.08,
                "goal_x": 0.30,
                "goal_y": -5.34,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.10,
                "init_y": 4.53,
                "init_a": -73.98,
                "velocity": 0.88,
                "goals": [[-0.99, -3.43], [-2.96, 4.65], [4.46, 4.38], [3.52, -2.60]],
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

def tests_adult_30_child_70_test_case_100_stopped_high(tester):
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
                "init_x": -0.41,
                "init_y": -3.70,
                "init_a": -13.51,
                "velocity": 1.16,
                "goal_x": -0.41,
                "goal_y": -3.70,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.04,
                "init_y": -2.93,
                "init_a": -13.51,
                "velocity": 1.16,
                "goals": [[-0.41, -3.70], [5.64, -4.36], [-0.87, -2.01], [7.05, 4.12]],
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
                "init_x": 5.70,
                "init_y": 0.93,
                "init_a": -126.29,
                "velocity": 0.96,
                "goals": [[5.70, 0.93], [5.10, 4.39], [6.44, -0.32], [-3.50, 5.34]],
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
                "init_x": 5.22,
                "init_y": 0.05,
                "init_a": -126.29,
                "velocity": 0.96,
                "goal_x": 5.70,
                "goal_y": 0.93,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.19,
                "init_y": -5.62,
                "init_a": 116.97,
                "velocity": 0.92,
                "goals": [[6.21, 4.51], [5.77, -0.22], [3.99, 5.89], [7.60, 5.76]],
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
                "init_x": 3.14,
                "init_y": 3.78,
                "init_a": 161.95,
                "velocity": 1.07,
                "goal_x": 7.95,
                "goal_y": -5.10,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.09,
                "init_y": 2.93,
                "init_a": 84.73,
                "velocity": 1.13,
                "goals": [[-5.01, 1.58], [-5.36, -4.82], [-2.59, -3.37], [4.37, 5.86]],
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
                "init_x": -6.23,
                "init_y": -4.61,
                "init_a": 140.29,
                "velocity": 1.12,
                "goals": [[0.60, -4.75], [-6.73, -3.68], [-5.26, -0.28], [7.78, -1.38]],
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
