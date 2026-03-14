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

def tests_adult_0_child_100_test_case_01_stopped_low(tester):
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
                "goals": [[5.80, -4.90], [-0.44, 4.23], [5.52, -1.99], [6.31, -5.35]],
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
                "init_x": 4.97,
                "init_y": -4.35,
                "init_a": 144.90,
                "velocity": 1.11,
                "goals": [[5.80, -4.90], [4.91, -5.66], [-7.02, -2.43], [3.03, -1.98]],
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
                "init_x": -2.01,
                "init_y": 1.79,
                "init_a": -35.67,
                "velocity": 1.03,
                "goals": [[-2.01, 1.79], [-3.71, 5.56], [3.02, 2.95], [-5.05, 3.25]],
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
                "init_x": -2.79,
                "init_y": 1.16,
                "init_a": -35.67,
                "velocity": 1.03,
                "goals": [[-2.01, 1.79], [-7.88, 0.32], [-7.12, -1.50], [5.83, -1.85]],
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
                "init_x": -7.19,
                "init_y": 2.85,
                "init_a": 75.06,
                "velocity": 1.17,
                "goals": [[3.44, 1.87], [4.43, -2.78], [-3.25, -4.73], [-4.97, -2.78]],
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
                "init_x": 3.79,
                "init_y": -3.75,
                "init_a": 0.71,
                "velocity": 1.16,
                "goals": [[6.82, -5.77], [4.20, -4.76], [3.60, 5.11], [3.29, 4.43]],
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
                "init_x": 6.04,
                "init_y": 1.64,
                "init_a": -33.61,
                "velocity": 0.85,
                "goals": [[5.80, -2.95], [-5.63, 2.12], [7.60, -4.25], [0.54, -1.58]],
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

def tests_adult_0_child_100_test_case_02_walking_low(tester):
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
                "init_x": 0.00,
                "init_y": 3.33,
                "init_a": 23.06,
                "velocity": 1.15,
                "goals": [[6.16, 1.18], [6.22, -0.30], [2.21, 3.73], [-7.39, 4.58]],
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
                "init_x": 1.00,
                "init_y": 3.34,
                "init_a": 23.06,
                "velocity": 1.15,
                "goals": [[6.16, 1.18], [3.65, 0.79], [1.75, 5.01], [6.28, 3.38]],
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
                "init_x": -3.72,
                "init_y": 0.15,
                "init_a": 177.69,
                "velocity": 0.82,
                "goals": [[4.38, 4.53], [-3.65, -4.31], [4.42, -2.53], [4.73, 5.74]],
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
                "init_x": -2.74,
                "init_y": 0.32,
                "init_a": 177.69,
                "velocity": 0.82,
                "goals": [[4.38, 4.53], [0.52, 5.00], [0.42, -4.35], [5.15, -4.86]],
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
                "init_x": -7.76,
                "init_y": -3.90,
                "init_a": -37.15,
                "velocity": 0.95,
                "goals": [[0.24, 0.89], [-0.19, -3.27], [7.73, -1.69], [7.21, -3.36]],
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
                "init_x": -1.64,
                "init_y": -5.50,
                "init_a": 14.01,
                "velocity": 0.81,
                "goals": [[-2.39, 5.83], [5.57, 3.10], [3.79, 3.63], [2.60, -2.68]],
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
                "init_x": 5.56,
                "init_y": 5.06,
                "init_a": 34.64,
                "velocity": 1.20,
                "goals": [[4.60, -2.08], [-4.13, 0.96], [-1.43, 3.09], [6.38, 0.78]],
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
                "init_x": -3.81,
                "init_y": -1.39,
                "init_a": -21.06,
                "velocity": 0.82,
                "goals": [[-3.37, -1.33], [-5.80, 1.62], [-4.35, 2.73], [0.83, 2.42]],
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
                "init_x": -0.38,
                "init_y": 3.74,
                "init_a": -150.00,
                "velocity": 0.85,
                "goals": [[-5.25, -5.43], [5.84, -1.31], [7.55, -2.51], [-5.68, -3.20]],
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

def tests_adult_0_child_100_test_case_03_walking_low(tester):
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
                "init_x": -7.94,
                "init_y": 3.00,
                "init_a": 85.34,
                "velocity": 0.98,
                "goals": [[-7.21, -5.96], [-5.62, 5.64], [-6.40, 2.42], [-2.91, 3.70]],
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
                "init_x": -8.40,
                "init_y": 3.89,
                "init_a": 85.34,
                "velocity": 0.98,
                "goals": [[-7.21, -5.96], [1.65, -5.79], [-3.79, -2.23], [0.88, -2.83]],
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
                "init_x": 1.51,
                "init_y": 4.59,
                "init_a": -24.94,
                "velocity": 1.11,
                "goals": [[-1.61, 1.37], [3.46, 1.28], [-6.65, 1.82], [0.03, -0.35]],
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
                "init_x": 1.85,
                "init_y": 5.53,
                "init_a": -24.94,
                "velocity": 1.11,
                "goals": [[-1.61, 1.37], [-4.62, -5.19], [-6.27, 2.66], [0.22, 5.04]],
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
                "init_x": -6.30,
                "init_y": -0.73,
                "init_a": -25.18,
                "velocity": 0.95,
                "goals": [[-2.94, 1.75], [-1.09, 3.95], [-2.47, -3.81], [6.79, 3.15]],
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
                "init_x": -7.31,
                "init_y": -2.03,
                "init_a": 113.87,
                "velocity": 0.90,
                "goals": [[-1.00, 5.09], [1.96, -1.74], [3.79, -2.21], [-7.12, 2.76]],
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
                "goals": [[-7.09, 2.88], [1.06, -3.39], [3.30, -0.15], [4.55, 5.04]],
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

def tests_adult_0_child_100_test_case_04_stopped_low(tester):
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
                "goals": [[-1.18, -0.11], [-5.52, -5.09], [5.09, -2.25], [-5.58, -1.39]],
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
                "goals": [[-1.18, -0.11], [2.15, 1.06], [-7.34, 2.51], [-7.26, -2.41]],
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
                "init_x": 4.13,
                "init_y": 5.30,
                "init_a": 28.19,
                "velocity": 1.11,
                "goals": [[4.13, 5.30], [-6.80, -4.65], [-6.40, -2.56], [-1.60, 1.29]],
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
                "init_x": 4.17,
                "init_y": 6.30,
                "init_a": 28.19,
                "velocity": 1.11,
                "goals": [[4.13, 5.30], [0.74, 0.63], [0.25, 4.17], [-1.52, 4.78]],
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
                "init_x": -2.74,
                "init_y": -0.64,
                "init_a": 62.90,
                "velocity": 1.11,
                "goals": [[-5.83, 2.41], [6.35, 4.12], [-5.29, -4.77], [-2.56, -3.73]],
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
                "init_x": 6.78,
                "init_y": -4.93,
                "init_a": 37.63,
                "velocity": 1.18,
                "goals": [[2.10, -1.79], [-1.12, -2.96], [5.77, 1.53], [-7.86, -2.68]],
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
                "init_x": -5.68,
                "init_y": 0.57,
                "init_a": 106.34,
                "velocity": 1.15,
                "goals": [[-3.02, -1.83], [7.85, -4.07], [-5.10, -5.00], [-4.57, 0.77]],
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
                "init_x": 6.10,
                "init_y": 3.43,
                "init_a": -42.69,
                "velocity": 0.94,
                "goals": [[5.73, 1.04], [-1.58, 1.64], [-2.49, 4.78], [4.33, 4.17]],
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

def tests_adult_0_child_100_test_case_05_walking_low(tester):
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
                "goals": [[-0.81, -2.77], [4.01, -4.18], [5.26, -0.47], [-1.86, 5.81]],
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
                "init_x": 4.97,
                "init_y": -5.94,
                "init_a": 94.19,
                "velocity": 1.09,
                "goals": [[-0.81, -2.77], [-2.54, -5.31], [7.15, 0.80], [-5.02, -5.49]],
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
                "goals": [[-2.10, 1.87], [-4.03, -1.43], [7.71, -3.17], [6.16, -3.90]],
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
                "init_x": -2.32,
                "init_y": -3.03,
                "init_a": -79.94,
                "velocity": 1.08,
                "goals": [[-2.10, 1.87], [0.43, -4.06], [7.70, 1.38], [-6.78, 3.22]],
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
                "init_x": 1.24,
                "init_y": -2.50,
                "init_a": -148.03,
                "velocity": 1.16,
                "goals": [[2.12, 4.59], [2.42, 1.86], [3.85, 0.34], [-2.05, -1.46]],
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
                "init_x": -7.19,
                "init_y": -5.29,
                "init_a": -2.85,
                "velocity": 1.07,
                "goals": [[-2.98, 5.35], [6.48, 3.45], [-1.99, 5.77], [-0.71, -0.77]],
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
                "init_x": -1.05,
                "init_y": -1.04,
                "init_a": 14.46,
                "velocity": 0.95,
                "goals": [[-0.44, 1.54], [0.74, -5.98], [-7.22, -5.27], [6.58, 2.24]],
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
                "init_x": -2.66,
                "init_y": 4.36,
                "init_a": 118.49,
                "velocity": 1.02,
                "goals": [[0.77, -2.60], [7.71, 5.87], [-3.72, 5.06], [0.03, -0.61]],
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
                "init_x": -5.02,
                "init_y": -5.00,
                "init_a": -71.24,
                "velocity": 1.11,
                "goals": [[-6.75, 5.18], [-1.68, -5.14], [-3.63, 5.02], [-4.09, 3.34]],
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

def tests_adult_0_child_100_test_case_06_walking_low(tester):
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
                "init_x": 0.28,
                "init_y": 5.18,
                "init_a": 115.33,
                "velocity": 1.06,
                "goals": [[-2.54, -5.99], [1.53, 0.61], [-1.72, -4.27], [-7.28, 5.51]],
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
                "init_x": -0.23,
                "init_y": 6.03,
                "init_a": 115.33,
                "velocity": 1.06,
                "goals": [[-2.54, -5.99], [5.63, 0.80], [-7.23, -5.90], [1.64, -2.27]],
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
                "init_x": 5.75,
                "init_y": -1.75,
                "init_a": 85.54,
                "velocity": 1.06,
                "goals": [[4.82, 2.34], [0.29, 4.59], [-2.26, -0.77], [5.00, 0.17]],
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
                "init_x": 6.33,
                "init_y": -0.94,
                "init_a": 85.54,
                "velocity": 1.06,
                "goals": [[4.82, 2.34], [-1.48, 0.49], [2.06, -4.63], [0.58, 2.84]],
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
                "init_x": -0.08,
                "init_y": 1.93,
                "init_a": 16.11,
                "velocity": 1.16,
                "goals": [[-3.12, -0.43], [-0.04, 1.12], [3.46, 1.83], [7.33, 1.17]],
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
                "init_x": -6.33,
                "init_y": -3.83,
                "init_a": 126.00,
                "velocity": 0.85,
                "goals": [[-5.04, -1.36], [4.62, -0.78], [-1.97, 2.19], [7.65, -5.86]],
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
                "init_x": -2.67,
                "init_y": 5.84,
                "init_a": 11.02,
                "velocity": 0.83,
                "goals": [[-5.00, 3.06], [5.23, 3.52], [2.09, -2.41], [4.61, -1.07]],
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

def tests_adult_0_child_100_test_case_07_walking_low(tester):
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
                "goals": [[5.00, 4.61], [-6.56, -4.09], [-0.92, 3.60], [-2.38, -1.95]],
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
                "init_x": 6.14,
                "init_y": 3.18,
                "init_a": 59.94,
                "velocity": 0.92,
                "goals": [[5.00, 4.61], [1.76, -1.28], [-0.45, 4.08], [-4.96, 0.62]],
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
                "init_x": -7.07,
                "init_y": -2.22,
                "init_a": 70.22,
                "velocity": 1.00,
                "goals": [[4.51, -1.38], [-1.39, -5.15], [4.26, -2.40], [-1.76, -4.74]],
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
                "init_x": -7.77,
                "init_y": -1.51,
                "init_a": 70.22,
                "velocity": 1.00,
                "goals": [[4.51, -1.38], [-1.28, -2.50], [-5.07, 3.94], [-7.53, -0.15]],
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
                "init_x": -1.92,
                "init_y": -4.91,
                "init_a": 87.02,
                "velocity": 0.97,
                "goals": [[-5.39, 1.51], [-5.50, -0.39], [7.17, 2.95], [5.42, -0.84]],
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
                "init_x": -1.04,
                "init_y": 2.09,
                "init_a": 44.20,
                "velocity": 1.03,
                "goals": [[-2.75, 3.33], [-4.51, -4.72], [0.44, 0.95], [-6.68, -4.08]],
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
                "init_x": 0.63,
                "init_y": -2.88,
                "init_a": -108.06,
                "velocity": 1.02,
                "goals": [[-4.87, -0.78], [5.02, -4.49], [1.77, 1.60], [2.64, -3.79]],
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
                "init_x": 4.03,
                "init_y": -1.12,
                "init_a": 56.72,
                "velocity": 1.17,
                "goals": [[-6.89, 1.07], [5.70, 3.30], [4.56, -5.50], [1.63, 3.61]],
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
                "init_x": -1.03,
                "init_y": -0.38,
                "init_a": 118.59,
                "velocity": 0.83,
                "goals": [[1.77, -2.67], [6.78, 4.12], [-5.53, 4.61], [-1.98, -1.38]],
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

def tests_adult_0_child_100_test_case_08_stopped_low(tester):
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
                "goals": [[-4.77, 0.91], [5.74, 2.30], [-2.66, -1.02], [-0.76, 4.75]],
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
                "goals": [[-4.77, 0.91], [7.81, -2.58], [-1.55, 2.56], [3.78, 5.00]],
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
                "init_x": -4.52,
                "init_y": 3.30,
                "init_a": -75.07,
                "velocity": 0.82,
                "goals": [[-4.52, 3.30], [-7.01, 2.89], [3.14, -0.28], [7.07, -4.20]],
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
                "init_x": -4.53,
                "init_y": 2.30,
                "init_a": -75.07,
                "velocity": 0.82,
                "goals": [[-4.52, 3.30], [-1.26, -3.15], [-4.71, -4.08], [-3.27, 2.49]],
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
                "init_x": -7.27,
                "init_y": 1.76,
                "init_a": -23.37,
                "velocity": 0.91,
                "goals": [[-7.91, 3.36], [-3.44, -1.41], [5.27, 3.85], [0.75, -3.61]],
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
                "init_x": 5.67,
                "init_y": -4.14,
                "init_a": 57.15,
                "velocity": 0.98,
                "goals": [[7.70, 2.37], [7.70, 1.26], [6.89, -3.78], [2.71, 0.47]],
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
                "init_x": 5.22,
                "init_y": -2.80,
                "init_a": 65.60,
                "velocity": 0.86,
                "goals": [[-1.73, -4.73], [2.48, -3.83], [1.00, -1.99], [-1.49, 4.59]],
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
                "goals": [[0.95, 1.70], [3.47, -1.47], [-1.70, -3.55], [-2.54, 0.96]],
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
                "init_x": -4.93,
                "init_y": 1.93,
                "init_a": 107.91,
                "velocity": 0.89,
                "goals": [[7.18, -1.03], [-7.40, 3.46], [5.70, -2.45], [-5.34, -0.17]],
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

def tests_adult_0_child_100_test_case_09_stopped_low(tester):
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
                "init_x": -3.95,
                "init_y": 0.77,
                "init_a": 24.94,
                "velocity": 1.04,
                "goals": [[-3.95, 0.77], [0.31, -2.68], [-0.56, 2.44], [-2.26, 0.23]],
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
                "init_x": -4.95,
                "init_y": 0.69,
                "init_a": 24.94,
                "velocity": 1.04,
                "goals": [[-3.95, 0.77], [-7.12, 2.31], [5.11, -3.77], [-1.88, 0.82]],
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
                "init_x": 3.40,
                "init_y": 4.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goals": [[3.40, 4.71], [1.50, -3.69], [0.53, 5.15], [-5.43, 2.72]],
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
                "init_x": 3.46,
                "init_y": 3.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goals": [[3.40, 4.71], [-5.37, -1.46], [-0.72, 3.54], [7.03, -3.64]],
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
                "init_x": 7.69,
                "init_y": -2.51,
                "init_a": 31.90,
                "velocity": 0.83,
                "goals": [[-0.89, 3.23], [-6.38, -5.23], [6.38, 1.53], [-3.85, 0.46]],
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
                "init_x": 1.74,
                "init_y": 2.87,
                "init_a": -38.87,
                "velocity": 1.12,
                "goals": [[-4.04, 4.58], [-5.32, -3.00], [7.12, -1.83], [6.18, 1.03]],
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
                "init_x": -3.11,
                "init_y": -4.04,
                "init_a": 48.60,
                "velocity": 1.01,
                "goals": [[0.32, -3.93], [-6.05, 1.94], [1.40, 2.10], [7.44, 3.17]],
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

def tests_adult_0_child_100_test_case_10_stopped_low(tester):
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
                "goals": [[5.20, -4.57], [5.65, 5.12], [2.26, -4.49], [5.63, 4.89]],
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
                "init_x": 4.26,
                "init_y": -4.91,
                "init_a": 29.17,
                "velocity": 1.06,
                "goals": [[5.20, -4.57], [6.87, -3.82], [5.28, 2.87], [-0.99, 1.01]],
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
                "init_x": -5.86,
                "init_y": 5.62,
                "init_a": -172.39,
                "velocity": 0.92,
                "goals": [[-5.86, 5.62], [3.47, -4.93], [0.22, -5.56], [-0.25, 5.26]],
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
                "init_x": -5.79,
                "init_y": 6.61,
                "init_a": -172.39,
                "velocity": 0.92,
                "goals": [[-5.86, 5.62], [-3.41, -0.71], [-3.98, -2.05], [7.73, 3.68]],
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
                "init_x": 1.33,
                "init_y": 4.37,
                "init_a": -44.53,
                "velocity": 0.98,
                "goals": [[4.28, -1.59], [0.24, -2.34], [-1.57, 3.73], [1.87, 0.21]],
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
                "init_x": -3.09,
                "init_y": -5.54,
                "init_a": -4.08,
                "velocity": 1.19,
                "goals": [[2.28, 0.07], [3.12, 1.62], [6.21, 2.69], [1.61, 1.18]],
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
                "goals": [[-2.14, -4.66], [-1.37, 0.82], [7.32, 1.17], [2.11, -5.97]],
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
                "init_x": 3.05,
                "init_y": -2.21,
                "init_a": -157.14,
                "velocity": 0.82,
                "goals": [[-0.85, -2.97], [2.42, 1.52], [-7.64, -4.86], [4.19, 2.88]],
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

def tests_adult_0_child_100_test_case_11_walking_low(tester):
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
                "goals": [[3.12, 4.98], [4.20, -3.08], [4.64, -3.96], [1.13, -5.98]],
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
                "init_x": 4.53,
                "init_y": -3.17,
                "init_a": 150.95,
                "velocity": 1.20,
                "goals": [[3.12, 4.98], [-4.78, 0.32], [-6.60, -2.63], [3.39, 0.33]],
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
                "init_x": 2.23,
                "init_y": -1.79,
                "init_a": -56.37,
                "velocity": 0.91,
                "goals": [[5.16, -3.74], [5.96, -0.51], [-7.02, -0.35], [7.65, 1.96]],
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
                "init_x": 1.72,
                "init_y": -2.65,
                "init_a": -56.37,
                "velocity": 0.91,
                "goals": [[5.16, -3.74], [-0.87, 2.95], [6.55, -3.91], [0.34, -2.19]],
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
                "init_x": -3.62,
                "init_y": -0.48,
                "init_a": 41.59,
                "velocity": 0.87,
                "goals": [[3.05, -0.44], [-6.48, 3.49], [7.02, -3.57], [3.52, -4.74]],
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
                "init_x": 6.19,
                "init_y": -4.13,
                "init_a": 45.66,
                "velocity": 1.12,
                "goals": [[-5.39, -3.91], [3.90, 3.41], [3.17, -5.30], [-0.27, -2.99]],
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
                "init_x": 7.37,
                "init_y": -4.45,
                "init_a": 51.93,
                "velocity": 1.05,
                "goals": [[-4.40, -3.83], [6.30, -4.69], [-3.60, 4.66], [-1.00, -2.69]],
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
                "init_x": -7.25,
                "init_y": -2.71,
                "init_a": -88.30,
                "velocity": 0.95,
                "goals": [[7.17, 4.38], [-2.71, -5.94], [-7.38, 4.65], [-5.64, -0.05]],
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

def tests_adult_0_child_100_test_case_12_walking_low(tester):
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
                "goals": [[2.00, 5.30], [-0.90, 2.14], [-7.74, -0.39], [3.10, -5.74]],
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
                "init_x": -2.78,
                "init_y": 0.69,
                "init_a": 5.14,
                "velocity": 0.85,
                "goals": [[2.00, 5.30], [-5.91, 3.48], [7.57, 5.23], [-7.21, 3.17]],
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
                "init_x": -2.07,
                "init_y": 4.67,
                "init_a": -122.78,
                "velocity": 0.95,
                "goals": [[0.82, 5.85], [7.33, 5.76], [-6.97, 4.40], [3.41, 4.68]],
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
                "init_x": -2.55,
                "init_y": 3.79,
                "init_a": -122.78,
                "velocity": 0.95,
                "goals": [[0.82, 5.85], [6.68, -4.68], [-3.07, 3.61], [2.37, -2.11]],
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
                "init_x": -0.66,
                "init_y": 5.89,
                "init_a": 127.85,
                "velocity": 1.19,
                "goals": [[7.95, 3.92], [-7.77, -2.99], [0.98, 1.67], [-7.64, -0.72]],
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
                "init_x": 5.74,
                "init_y": 2.92,
                "init_a": 156.64,
                "velocity": 0.91,
                "goals": [[5.04, -0.76], [7.00, -0.09], [3.38, -0.96], [-4.85, 0.06]],
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
                "goals": [[4.74, 4.69], [-2.04, 1.54], [-7.18, 5.55], [2.22, 4.70]],
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
                "goals": [[4.32, 3.00], [3.35, 0.82], [-4.26, 2.27], [5.35, -2.07]],
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

def tests_adult_0_child_100_test_case_13_stopped_low(tester):
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
                "goals": [[-5.18, -3.02], [2.49, -2.48], [-1.68, 4.32], [-1.93, -0.33]],
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
                "init_x": -4.97,
                "init_y": -4.00,
                "init_a": -128.85,
                "velocity": 1.14,
                "goals": [[-5.18, -3.02], [-1.62, 3.61], [4.91, -5.21], [-0.01, 0.19]],
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
                "init_x": 2.06,
                "init_y": 1.40,
                "init_a": 106.78,
                "velocity": 0.88,
                "goals": [[2.06, 1.40], [0.36, -2.69], [-3.97, -2.96], [-1.08, -4.51]],
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
                "init_x": 1.27,
                "init_y": 2.01,
                "init_a": 106.78,
                "velocity": 0.88,
                "goals": [[2.06, 1.40], [-5.69, 2.60], [0.89, 0.09], [-3.42, -1.31]],
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
                "init_x": 6.06,
                "init_y": 5.55,
                "init_a": -17.15,
                "velocity": 1.03,
                "goals": [[-5.29, -3.55], [-3.92, 0.83], [0.53, 0.81], [1.95, -4.37]],
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
                "goals": [[6.06, 2.43], [7.11, -4.01], [-2.77, -5.80], [-3.08, -0.66]],
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
                "init_x": -5.44,
                "init_y": 0.03,
                "init_a": -80.05,
                "velocity": 0.87,
                "goals": [[-0.64, -2.50], [-0.97, 3.37], [3.92, 2.73], [1.43, -1.78]],
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
                "init_x": -5.01,
                "init_y": 3.68,
                "init_a": 126.76,
                "velocity": 1.20,
                "goals": [[4.85, 3.54], [-0.00, 4.42], [6.27, 4.91], [-2.91, 5.95]],
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
                "init_x": 2.44,
                "init_y": 2.74,
                "init_a": -165.47,
                "velocity": 0.97,
                "goals": [[-6.41, -2.51], [-7.78, -0.36], [4.15, -5.24], [1.93, 3.90]],
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

def tests_adult_0_child_100_test_case_14_stopped_low(tester):
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
                "goals": [[-3.73, -3.35], [-4.56, 4.61], [-4.07, 2.25], [-3.57, -4.20]],
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
                "init_x": -3.90,
                "init_y": -2.36,
                "init_a": 136.60,
                "velocity": 1.16,
                "goals": [[-3.73, -3.35], [3.73, 2.25], [-0.26, -5.44], [-7.77, 2.66]],
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
                "init_x": -6.95,
                "init_y": 1.56,
                "init_a": 137.97,
                "velocity": 0.85,
                "goals": [[-6.95, 1.56], [5.73, -5.95], [7.67, 4.61], [0.85, -1.88]],
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
                "init_x": -5.97,
                "init_y": 1.35,
                "init_a": 137.97,
                "velocity": 0.85,
                "goals": [[-6.95, 1.56], [2.85, 5.02], [1.31, -5.88], [2.86, -2.16]],
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
                "init_x": 1.40,
                "init_y": -5.05,
                "init_a": -153.73,
                "velocity": 1.03,
                "goals": [[-0.21, -3.28], [6.28, 1.27], [-4.51, 5.48], [7.04, -3.13]],
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
                "init_x": -7.06,
                "init_y": -4.05,
                "init_a": 50.62,
                "velocity": 0.83,
                "goals": [[-0.68, -5.89], [-3.91, 3.10], [4.28, 3.68], [6.17, -1.90]],
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
                "init_x": -5.92,
                "init_y": 3.31,
                "init_a": 121.20,
                "velocity": 0.80,
                "goals": [[-4.43, -3.09], [-2.58, 5.31], [1.28, -1.04], [-2.39, -4.73]],
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
                "init_x": 3.47,
                "init_y": 1.08,
                "init_a": 55.72,
                "velocity": 1.16,
                "goals": [[-1.23, -4.95], [1.75, 0.70], [3.27, -5.85], [-0.87, 5.37]],
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
                "init_x": -3.84,
                "init_y": 4.19,
                "init_a": -127.88,
                "velocity": 1.09,
                "goals": [[6.14, -4.00], [-1.92, -4.52], [-1.19, 5.04], [-4.91, -2.23]],
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

def tests_adult_0_child_100_test_case_15_walking_low(tester):
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
                "goals": [[5.44, 1.84], [-3.74, 1.38], [4.82, -4.77], [3.52, 5.83]],
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
                "init_x": 6.17,
                "init_y": 0.34,
                "init_a": 95.69,
                "velocity": 0.99,
                "goals": [[5.44, 1.84], [7.93, 2.72], [7.81, 1.22], [-2.07, -3.95]],
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
                "init_x": 5.92,
                "init_y": 4.80,
                "init_a": 2.99,
                "velocity": 0.99,
                "goals": [[-4.50, -1.26], [6.76, 2.70], [-6.97, 3.93], [4.93, 5.57]],
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
                "init_x": 5.06,
                "init_y": 4.29,
                "init_a": 2.99,
                "velocity": 0.99,
                "goals": [[-4.50, -1.26], [-0.93, -0.84], [2.92, 1.42], [-7.90, -2.26]],
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
                "init_x": -7.58,
                "init_y": -4.36,
                "init_a": -55.50,
                "velocity": 0.91,
                "goals": [[1.80, 1.34], [2.90, -2.34], [1.26, 2.15], [-7.99, 2.55]],
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
                "init_x": 0.78,
                "init_y": 3.56,
                "init_a": 174.91,
                "velocity": 1.09,
                "goals": [[-0.84, 0.85], [-1.30, 4.64], [3.89, -0.09], [-3.53, 5.87]],
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
                "init_x": 2.30,
                "init_y": 1.15,
                "init_a": -105.55,
                "velocity": 1.01,
                "goals": [[-1.51, 1.04], [-4.73, 3.70], [-7.64, 0.68], [2.86, -2.43]],
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
                "init_x": -5.11,
                "init_y": -4.95,
                "init_a": -58.14,
                "velocity": 1.14,
                "goals": [[3.89, 6.00], [-3.07, -5.80], [-2.16, -3.89], [6.17, 2.65]],
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
                "init_x": -4.03,
                "init_y": 2.44,
                "init_a": -124.17,
                "velocity": 1.07,
                "goals": [[-5.48, -1.67], [2.50, 0.89], [2.37, -3.35], [-3.63, -3.76]],
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

def tests_adult_0_child_100_test_case_16_stopped_low(tester):
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
                "goals": [[-6.55, -5.05], [3.92, -1.85], [-7.27, -5.45], [-2.29, -4.16]],
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
                "init_x": -5.79,
                "init_y": -5.71,
                "init_a": -118.17,
                "velocity": 1.03,
                "goals": [[-6.55, -5.05], [3.99, 4.48], [-4.86, -2.44], [-5.70, -3.68]],
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
                "init_x": 1.69,
                "init_y": 0.82,
                "init_a": 171.03,
                "velocity": 1.16,
                "goals": [[1.69, 0.82], [0.22, -3.24], [-1.83, -1.31], [0.63, 1.13]],
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
                "init_x": 1.14,
                "init_y": -0.02,
                "init_a": 171.03,
                "velocity": 1.16,
                "goals": [[1.69, 0.82], [-6.67, -5.12], [3.01, -4.53], [-5.32, 1.76]],
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
                "goals": [[2.77, 5.66], [3.00, -2.52], [1.51, 2.06], [-7.41, 2.36]],
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
                "goals": [[2.95, -1.66], [3.27, -3.10], [-5.04, -0.64], [-3.37, 1.52]],
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
                "init_x": -2.95,
                "init_y": -5.65,
                "init_a": 13.28,
                "velocity": 0.85,
                "goals": [[-5.43, 0.96], [-5.81, 0.95], [3.86, 1.61], [-2.80, 4.50]],
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
                "init_x": -3.30,
                "init_y": 0.16,
                "init_a": -27.20,
                "velocity": 1.11,
                "goals": [[-1.50, -3.07], [-7.53, -0.12], [1.74, -0.77], [-7.13, 2.97]],
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
                "init_x": 7.14,
                "init_y": -5.54,
                "init_a": -177.69,
                "velocity": 1.04,
                "goals": [[0.46, -4.98], [-1.83, 0.65], [-0.99, 2.53], [-4.59, -1.80]],
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

def tests_adult_0_child_100_test_case_17_walking_low(tester):
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
                "init_x": -2.40,
                "init_y": -0.79,
                "init_a": -145.87,
                "velocity": 1.13,
                "goals": [[-6.97, -2.49], [3.57, -2.00], [3.12, 0.20], [-1.57, 0.90]],
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
                "init_x": -3.30,
                "init_y": -1.21,
                "init_a": -145.87,
                "velocity": 1.13,
                "goals": [[-6.97, -2.49], [-1.31, 5.82], [6.62, -3.76], [-7.98, 2.73]],
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
                "init_x": -4.76,
                "init_y": 3.02,
                "init_a": -13.14,
                "velocity": 0.90,
                "goals": [[-2.68, -5.19], [-5.20, -5.56], [1.35, 5.49], [-1.93, -3.60]],
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
                "goals": [[-2.68, -5.19], [-3.33, -3.97], [-1.06, -3.39], [-5.76, 5.23]],
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
                "goals": [[5.68, -0.28], [0.79, -4.16], [2.52, -1.62], [-4.38, 5.11]],
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
                "goals": [[3.73, 2.84], [0.91, -2.55], [2.73, -2.51], [-0.30, -0.95]],
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
                "init_x": -3.62,
                "init_y": -2.70,
                "init_a": 167.61,
                "velocity": 1.20,
                "goals": [[-0.72, 3.98], [-0.99, 0.07], [4.52, -5.11], [-0.32, -0.63]],
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
                "init_x": 7.20,
                "init_y": 2.57,
                "init_a": -114.47,
                "velocity": 0.97,
                "goals": [[-4.25, -3.98], [-0.13, -3.30], [7.32, -1.84], [-5.29, 0.85]],
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

def tests_adult_0_child_100_test_case_18_stopped_low(tester):
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
                "init_x": 0.31,
                "init_y": -0.60,
                "init_a": -84.34,
                "velocity": 1.01,
                "goals": [[0.31, -0.60], [5.59, 2.51], [-3.08, 5.25], [3.50, -3.48]],
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
                "init_x": 0.95,
                "init_y": 0.17,
                "init_a": -84.34,
                "velocity": 1.01,
                "goals": [[0.31, -0.60], [-3.43, 3.56], [0.07, -0.03], [-3.44, -0.91]],
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
                "init_x": 3.35,
                "init_y": 1.81,
                "init_a": -69.12,
                "velocity": 0.81,
                "goals": [[3.35, 1.81], [1.59, 1.09], [1.22, -3.73], [-6.25, 1.12]],
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
                "init_x": 2.36,
                "init_y": 1.96,
                "init_a": -69.12,
                "velocity": 0.81,
                "goals": [[3.35, 1.81], [-3.52, 0.96], [4.27, 4.16], [5.60, 2.02]],
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
                "init_x": -6.30,
                "init_y": 5.99,
                "init_a": 23.99,
                "velocity": 0.92,
                "goals": [[-3.81, -4.17], [-4.65, -5.53], [-6.88, -1.26], [-1.61, -4.63]],
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
                "init_x": 1.39,
                "init_y": 4.85,
                "init_a": 70.57,
                "velocity": 0.82,
                "goals": [[-5.02, 0.17], [-4.75, 0.20], [-0.93, 2.15], [-2.32, -0.24]],
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
                "init_x": 5.93,
                "init_y": 1.89,
                "init_a": 133.26,
                "velocity": 0.91,
                "goals": [[2.02, 4.88], [1.88, -4.15], [-1.47, -2.56], [7.82, 5.90]],
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

def tests_adult_0_child_100_test_case_19_stopped_low(tester):
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
                "goals": [[3.29, -3.00], [-7.26, 2.23], [-1.88, 3.84], [4.54, 1.44]],
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
                "init_x": 2.72,
                "init_y": -2.18,
                "init_a": -0.90,
                "velocity": 1.01,
                "goals": [[3.29, -3.00], [-5.19, 4.28], [1.13, 5.07], [-5.01, 3.93]],
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
                "init_x": -1.04,
                "init_y": -5.18,
                "init_a": 1.05,
                "velocity": 0.83,
                "goals": [[-1.04, -5.18], [-3.36, 1.61], [-4.40, -5.12], [-3.95, -2.95]],
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
                "init_x": -1.94,
                "init_y": -5.60,
                "init_a": 1.05,
                "velocity": 0.83,
                "goals": [[-1.04, -5.18], [-2.30, -0.46], [1.25, 2.88], [0.18, 2.31]],
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
                "init_x": -3.39,
                "init_y": 5.43,
                "init_a": 0.93,
                "velocity": 1.18,
                "goals": [[6.62, 1.26], [-6.81, 1.95], [2.44, 3.86], [-0.39, -4.49]],
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
                "init_x": 3.01,
                "init_y": -2.28,
                "init_a": 87.85,
                "velocity": 1.08,
                "goals": [[-1.40, 0.87], [-5.60, -5.51], [-1.56, -1.72], [-5.43, 1.72]],
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
                "init_x": 4.67,
                "init_y": 3.01,
                "init_a": 111.63,
                "velocity": 0.99,
                "goals": [[6.14, -5.92], [5.78, 4.13], [7.53, 3.99], [-7.50, -0.89]],
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

def tests_adult_0_child_100_test_case_20_stopped_low(tester):
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
                "goals": [[-1.01, -0.02], [1.27, 0.54], [5.06, -0.84], [-4.70, 0.09]],
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
                "init_x": -1.91,
                "init_y": -0.46,
                "init_a": 23.00,
                "velocity": 0.99,
                "goals": [[-1.01, -0.02], [5.80, -1.40], [-6.39, 4.56], [2.59, -2.11]],
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
                "init_x": 1.23,
                "init_y": -1.65,
                "init_a": 125.73,
                "velocity": 1.08,
                "goals": [[1.23, -1.65], [-6.95, 5.12], [1.25, 3.18], [4.92, -1.02]],
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
                "init_x": 1.15,
                "init_y": -0.65,
                "init_a": 125.73,
                "velocity": 1.08,
                "goals": [[1.23, -1.65], [-7.14, 2.69], [-1.90, -3.18], [5.91, -4.62]],
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
                "init_x": 7.56,
                "init_y": 1.95,
                "init_a": 133.43,
                "velocity": 0.98,
                "goals": [[5.90, -1.13], [7.18, 5.90], [6.10, -3.55], [1.65, 5.61]],
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
                "init_x": -3.53,
                "init_y": 3.64,
                "init_a": 54.42,
                "velocity": 0.88,
                "goals": [[4.04, 3.23], [-4.51, 1.05], [5.27, -3.21], [4.36, -0.65]],
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
                "init_x": 7.96,
                "init_y": 2.02,
                "init_a": -169.07,
                "velocity": 1.16,
                "goals": [[0.74, 4.24], [2.73, 4.22], [-5.56, 5.71], [-0.97, -3.43]],
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
                "init_x": -2.20,
                "init_y": -5.99,
                "init_a": -58.94,
                "velocity": 0.90,
                "goals": [[-4.91, 1.40], [-4.70, -3.75], [4.35, -5.73], [-0.37, 1.76]],
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

def tests_adult_0_child_100_test_case_21_stopped_low(tester):
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
                "goals": [[4.30, 5.63], [-7.69, 2.59], [-0.18, 5.57], [0.51, 5.25]],
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
                "init_x": 5.25,
                "init_y": 5.30,
                "init_a": 38.92,
                "velocity": 0.90,
                "goals": [[4.30, 5.63], [-2.25, 4.76], [1.92, 3.55], [-2.24, 4.40]],
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
                "init_x": 6.83,
                "init_y": -5.95,
                "init_a": 44.68,
                "velocity": 0.92,
                "goals": [[6.83, -5.95], [7.18, 3.33], [0.10, 5.77], [0.41, 4.18]],
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
                "init_x": 7.19,
                "init_y": -5.01,
                "init_a": 44.68,
                "velocity": 0.92,
                "goals": [[6.83, -5.95], [2.63, 1.82], [6.01, -5.62], [7.81, -4.33]],
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
                "init_x": 1.64,
                "init_y": -2.02,
                "init_a": 37.98,
                "velocity": 1.08,
                "goals": [[-7.17, 3.33], [7.48, -2.38], [-0.12, 0.60], [-5.22, 5.73]],
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
                "init_x": -4.96,
                "init_y": 1.86,
                "init_a": 1.30,
                "velocity": 1.15,
                "goals": [[-0.53, 1.73], [-1.99, -2.66], [-3.51, -5.91], [-7.94, 5.09]],
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
                "init_x": 7.81,
                "init_y": 0.71,
                "init_a": -3.43,
                "velocity": 0.87,
                "goals": [[-5.95, 4.01], [0.37, -1.64], [-7.39, -5.62], [-1.20, 0.82]],
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
                "init_x": 2.92,
                "init_y": -1.73,
                "init_a": 89.03,
                "velocity": 0.95,
                "goals": [[-3.26, -5.17], [0.37, 0.29], [-2.69, -0.98], [-5.93, 5.32]],
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

def tests_adult_0_child_100_test_case_22_walking_low(tester):
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
                "goals": [[-0.12, -5.67], [-6.02, -2.39], [6.06, -4.13], [-6.95, 5.59]],
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
                "init_x": 1.10,
                "init_y": 2.08,
                "init_a": -62.86,
                "velocity": 1.02,
                "goals": [[-0.12, -5.67], [0.25, 4.12], [5.41, -3.80], [4.60, -5.51]],
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
                "init_x": -1.89,
                "init_y": -5.67,
                "init_a": 91.46,
                "velocity": 1.08,
                "goals": [[-1.87, 1.12], [-6.81, -0.36], [6.49, -5.61], [3.48, -2.58]],
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
                "init_x": -2.35,
                "init_y": -6.56,
                "init_a": 91.46,
                "velocity": 1.08,
                "goals": [[-1.87, 1.12], [-4.34, 4.43], [-2.94, -1.89], [-4.77, 3.20]],
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
                "init_x": -1.17,
                "init_y": 0.70,
                "init_a": -150.22,
                "velocity": 1.11,
                "goals": [[3.01, -0.47], [-0.05, 4.10], [-4.21, 1.56], [5.82, -4.17]],
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
                "init_x": -3.78,
                "init_y": -1.27,
                "init_a": -128.07,
                "velocity": 0.88,
                "goals": [[3.60, 1.88], [-6.44, -5.71], [-6.63, -4.72], [-6.86, 0.33]],
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
                "init_x": -0.28,
                "init_y": 3.41,
                "init_a": -123.21,
                "velocity": 1.03,
                "goals": [[6.26, -3.32], [5.83, 5.67], [2.87, 4.96], [6.16, 2.13]],
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

def tests_adult_0_child_100_test_case_23_walking_low(tester):
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
                "goals": [[-2.58, 4.91], [-2.42, -1.00], [-4.75, -0.24], [-0.54, -0.60]],
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
                "init_x": 6.96,
                "init_y": 5.19,
                "init_a": -165.93,
                "velocity": 0.86,
                "goals": [[-2.58, 4.91], [6.21, -0.85], [-5.60, -1.15], [-5.69, -0.54]],
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
                "init_x": 0.05,
                "init_y": 5.36,
                "init_a": -21.93,
                "velocity": 1.07,
                "goals": [[4.46, 0.65], [-6.01, 3.51], [-3.38, -4.90], [-0.64, 2.91]],
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
                "init_x": 0.63,
                "init_y": 4.55,
                "init_a": -21.93,
                "velocity": 1.07,
                "goals": [[4.46, 0.65], [1.73, 5.67], [7.17, 5.10], [-6.14, -5.68]],
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
                "init_x": 3.57,
                "init_y": 1.19,
                "init_a": 162.94,
                "velocity": 1.19,
                "goals": [[6.19, -3.25], [7.36, 3.73], [2.47, -1.76], [6.05, 0.10]],
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
                "goals": [[-6.87, -3.96], [5.35, -3.20], [6.75, 2.28], [-4.36, 4.26]],
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
                "goals": [[1.23, -5.29], [5.52, 4.44], [-5.57, 3.33], [1.96, 5.81]],
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

def tests_adult_0_child_100_test_case_24_walking_low(tester):
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
                "goals": [[3.12, 0.02], [-2.00, -4.17], [-2.70, 4.70], [2.54, -3.48]],
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
                "init_x": 1.98,
                "init_y": -1.50,
                "init_a": -157.41,
                "velocity": 0.85,
                "goals": [[3.12, 0.02], [-3.48, -2.31], [6.94, 5.62], [4.51, -1.62]],
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
                "init_x": 2.15,
                "init_y": 5.85,
                "init_a": -93.65,
                "velocity": 0.93,
                "goals": [[-1.54, -1.08], [5.77, 0.06], [0.84, -5.11], [-6.41, -5.85]],
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
                "init_x": 2.32,
                "init_y": 4.87,
                "init_a": -93.65,
                "velocity": 0.93,
                "goals": [[-1.54, -1.08], [-7.99, 0.43], [-0.38, -4.83], [-0.85, -2.43]],
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
                "init_x": -0.01,
                "init_y": 5.22,
                "init_a": 28.78,
                "velocity": 1.05,
                "goals": [[6.34, 4.38], [7.52, -3.33], [1.70, 3.75], [-7.28, -3.50]],
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
                "init_x": 0.89,
                "init_y": -0.98,
                "init_a": 47.75,
                "velocity": 1.15,
                "goals": [[-7.70, 1.71], [2.44, 5.72], [-7.70, -5.32], [-5.88, 1.66]],
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
                "init_x": 5.49,
                "init_y": -1.87,
                "init_a": -14.55,
                "velocity": 0.91,
                "goals": [[-1.22, -4.48], [4.18, -2.96], [0.73, 4.72], [3.70, -1.15]],
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
                "init_x": -2.91,
                "init_y": 3.99,
                "init_a": -60.64,
                "velocity": 0.98,
                "goals": [[0.34, 0.72], [2.43, -4.27], [-5.64, 2.61], [-1.17, -3.41]],
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
                "init_x": 3.37,
                "init_y": -0.36,
                "init_a": 64.44,
                "velocity": 1.02,
                "goals": [[7.66, 1.79], [-7.86, -5.43], [4.77, 0.51], [-2.82, -4.52]],
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

def tests_adult_0_child_100_test_case_25_stopped_low(tester):
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
                "goals": [[1.98, 0.61], [-4.32, -1.41], [7.41, 4.53], [-4.06, 5.94]],
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
                "init_x": 1.50,
                "init_y": -0.27,
                "init_a": 119.93,
                "velocity": 0.87,
                "goals": [[1.98, 0.61], [-0.86, -1.84], [-4.72, 3.94], [3.25, 0.89]],
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
                "init_x": -0.45,
                "init_y": -2.13,
                "init_a": -100.62,
                "velocity": 0.84,
                "goals": [[-0.45, -2.13], [6.82, -2.95], [3.49, -0.67], [6.18, 5.66]],
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
                "init_x": -0.96,
                "init_y": -2.99,
                "init_a": -100.62,
                "velocity": 0.84,
                "goals": [[-0.45, -2.13], [7.28, -2.61], [4.75, 1.50], [-3.77, 5.66]],
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
                "init_x": -3.83,
                "init_y": -3.75,
                "init_a": -121.25,
                "velocity": 1.04,
                "goals": [[-4.95, -2.61], [5.96, -0.26], [-1.15, -2.96], [-3.17, 0.95]],
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
                "init_x": -4.26,
                "init_y": -5.25,
                "init_a": 99.04,
                "velocity": 0.84,
                "goals": [[-7.89, -4.06], [-6.72, 4.43], [-6.06, 2.44], [5.61, -5.41]],
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
                "init_x": 7.23,
                "init_y": -1.59,
                "init_a": 46.51,
                "velocity": 0.89,
                "goals": [[5.63, -3.67], [-7.04, 5.09], [6.56, -2.01], [-2.30, -5.75]],
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
                "init_x": -3.42,
                "init_y": 5.15,
                "init_a": -78.10,
                "velocity": 1.13,
                "goals": [[1.13, -2.94], [-2.99, 2.44], [7.28, -4.09], [-7.31, 4.43]],
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
                "init_x": 7.39,
                "init_y": -5.56,
                "init_a": 103.82,
                "velocity": 1.15,
                "goals": [[-3.59, -5.29], [-7.63, 3.03], [-5.03, -1.11], [-6.46, 4.48]],
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

def tests_adult_0_child_100_test_case_26_walking_low(tester):
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
                "goals": [[-0.24, 1.05], [-6.54, -5.73], [7.52, 0.41], [3.44, 4.16]],
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
                "goals": [[-0.24, 1.05], [-2.98, 4.88], [-2.01, -1.27], [-7.94, 4.88]],
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
                "init_x": 6.47,
                "init_y": -0.85,
                "init_a": -53.33,
                "velocity": 1.09,
                "goals": [[-0.05, 0.07], [0.36, 1.86], [-3.06, -3.10], [-3.56, -2.60]],
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
                "init_x": 7.37,
                "init_y": -1.29,
                "init_a": -53.33,
                "velocity": 1.09,
                "goals": [[-0.05, 0.07], [6.81, -1.25], [1.56, 3.14], [1.14, 1.18]],
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
                "init_x": 4.99,
                "init_y": 0.11,
                "init_a": -69.24,
                "velocity": 0.98,
                "goals": [[6.90, -0.14], [4.37, -3.55], [-7.54, 0.43], [2.58, -5.30]],
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
                "goals": [[4.50, -5.67], [3.29, -4.25], [-0.26, 2.88], [3.64, 2.16]],
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
                "init_x": -2.68,
                "init_y": -1.87,
                "init_a": -4.14,
                "velocity": 0.83,
                "goals": [[-5.08, 0.98], [3.85, 1.57], [6.59, 2.07], [-6.06, -2.96]],
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
                "init_x": -4.69,
                "init_y": 0.38,
                "init_a": -109.84,
                "velocity": 1.10,
                "goals": [[5.59, -2.72], [-3.30, 2.99], [1.25, 3.77], [-4.22, 1.83]],
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
                "init_x": 1.13,
                "init_y": -5.11,
                "init_a": 57.07,
                "velocity": 0.85,
                "goals": [[-1.79, 0.87], [-5.92, -4.48], [-6.99, -0.59], [-1.10, -2.79]],
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

def tests_adult_0_child_100_test_case_27_walking_low(tester):
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
                "goals": [[2.09, -0.92], [-2.99, -3.84], [-0.05, 5.23], [4.67, -5.33]],
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
                "init_x": -3.67,
                "init_y": 6.18,
                "init_a": -174.00,
                "velocity": 0.93,
                "goals": [[2.09, -0.92], [-1.92, -3.68], [-1.10, 3.04], [-6.82, -4.74]],
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
                "init_x": -1.36,
                "init_y": 5.49,
                "init_a": 64.23,
                "velocity": 1.17,
                "goals": [[-2.03, -4.57], [-5.74, -3.72], [4.90, -1.77], [5.18, 0.21]],
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
                "init_x": -1.62,
                "init_y": 4.53,
                "init_a": 64.23,
                "velocity": 1.17,
                "goals": [[-2.03, -4.57], [0.54, 0.71], [1.31, 4.64], [0.55, -1.18]],
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
                "init_x": -3.09,
                "init_y": -4.75,
                "init_a": -166.89,
                "velocity": 0.80,
                "goals": [[-6.11, 2.08], [-1.75, -4.16], [-0.86, -4.19], [-6.24, 0.22]],
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
                "init_x": -2.54,
                "init_y": -2.62,
                "init_a": 121.59,
                "velocity": 0.97,
                "goals": [[-3.08, 0.68], [-6.56, 0.42], [5.13, -3.32], [5.55, -5.63]],
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
                "init_x": 4.86,
                "init_y": 1.46,
                "init_a": -61.77,
                "velocity": 0.96,
                "goals": [[-4.06, 3.71], [5.17, 3.51], [6.91, -4.88], [-6.20, 2.95]],
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
                "init_x": -3.98,
                "init_y": 3.77,
                "init_a": -85.03,
                "velocity": 0.94,
                "goals": [[3.70, 0.17], [-7.34, -5.37], [-0.46, 2.44], [-6.07, 2.34]],
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

def tests_adult_0_child_100_test_case_28_walking_low(tester):
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
                "init_x": 4.13,
                "init_y": 2.22,
                "init_a": -131.64,
                "velocity": 1.17,
                "goals": [[7.97, -3.35], [2.78, 1.94], [-1.67, 4.57], [-6.15, -4.81]],
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
                "init_x": 4.93,
                "init_y": 2.82,
                "init_a": -131.64,
                "velocity": 1.17,
                "goals": [[7.97, -3.35], [-1.00, 1.41], [-2.31, -5.27], [5.79, 2.84]],
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
                "init_x": -0.31,
                "init_y": -3.52,
                "init_a": 0.96,
                "velocity": 0.84,
                "goals": [[7.13, 3.79], [4.32, -0.29], [2.67, 1.15], [3.52, -4.00]],
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
                "init_x": 0.23,
                "init_y": -2.68,
                "init_a": 0.96,
                "velocity": 0.84,
                "goals": [[7.13, 3.79], [-0.58, 3.33], [-4.78, -4.09], [-4.43, -1.25]],
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
                "init_x": -3.00,
                "init_y": 1.90,
                "init_a": 6.86,
                "velocity": 0.91,
                "goals": [[5.91, 0.30], [7.10, 2.01], [6.10, -1.86], [-5.93, -3.79]],
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
                "goals": [[6.86, -1.43], [5.21, -1.39], [-3.86, -4.54], [3.59, -5.67]],
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
                "goals": [[-4.69, 2.62], [2.61, 1.04], [2.56, -3.41], [0.91, 5.25]],
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
                "init_x": 6.43,
                "init_y": -0.13,
                "init_a": -37.55,
                "velocity": 0.94,
                "goals": [[6.27, 5.21], [-5.68, 3.37], [-0.40, 3.00], [0.42, -4.68]],
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

def tests_adult_0_child_100_test_case_29_walking_low(tester):
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
                "init_x": -0.78,
                "init_y": -4.87,
                "init_a": 107.23,
                "velocity": 0.97,
                "goals": [[-4.89, 0.56], [-3.22, 0.55], [-3.42, -5.28], [2.42, 2.74]],
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
                "init_x": -0.79,
                "init_y": -3.87,
                "init_a": 107.23,
                "velocity": 0.97,
                "goals": [[-4.89, 0.56], [-5.01, -2.78], [-7.72, -4.61], [1.15, 0.84]],
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
                "init_x": -7.30,
                "init_y": -0.17,
                "init_a": -176.10,
                "velocity": 0.99,
                "goals": [[2.76, 2.61], [-7.33, 3.17], [6.44, -5.31], [-6.91, 1.05]],
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
                "init_x": -6.45,
                "init_y": 0.35,
                "init_a": -176.10,
                "velocity": 0.99,
                "goals": [[2.76, 2.61], [-1.01, 2.34], [0.37, -1.08], [-4.46, -5.98]],
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
                "init_x": 3.36,
                "init_y": 5.71,
                "init_a": 121.57,
                "velocity": 0.99,
                "goals": [[-4.09, 2.16], [-4.51, 1.33], [1.61, 0.53], [0.92, 5.12]],
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
                "init_x": -7.25,
                "init_y": -3.25,
                "init_a": -91.75,
                "velocity": 0.82,
                "goals": [[-1.68, -1.92], [0.05, -1.92], [-2.99, 0.66], [-4.04, 2.22]],
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
                "init_x": 0.55,
                "init_y": -5.72,
                "init_a": 39.05,
                "velocity": 0.93,
                "goals": [[-5.65, -0.63], [5.51, -3.98], [6.75, 3.75], [1.56, 4.56]],
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
                "init_x": 0.17,
                "init_y": -1.93,
                "init_a": -136.42,
                "velocity": 0.99,
                "goals": [[-0.85, 3.41], [3.32, 5.91], [-2.51, 0.77], [2.99, 2.46]],
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

def tests_adult_0_child_100_test_case_30_stopped_low(tester):
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
                "init_x": -2.44,
                "init_y": -0.34,
                "init_a": 56.83,
                "velocity": 1.19,
                "goals": [[-2.44, -0.34], [1.34, 3.64], [-4.70, 3.78], [4.50, 1.20]],
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
                "init_x": -2.67,
                "init_y": -1.31,
                "init_a": 56.83,
                "velocity": 1.19,
                "goals": [[-2.44, -0.34], [4.90, -1.12], [-7.43, 0.09], [-6.50, 3.97]],
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
                "init_x": 0.25,
                "init_y": -2.83,
                "init_a": 43.47,
                "velocity": 1.10,
                "goals": [[0.25, -2.83], [2.70, -1.14], [-4.89, 3.61], [1.57, -2.08]],
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
                "init_x": 1.20,
                "init_y": -2.53,
                "init_a": 43.47,
                "velocity": 1.10,
                "goals": [[0.25, -2.83], [-4.10, 1.36], [-7.43, 0.08], [-7.99, 1.53]],
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
                "init_x": -6.36,
                "init_y": 4.99,
                "init_a": 154.58,
                "velocity": 1.03,
                "goals": [[-7.14, 5.87], [4.72, -0.95], [7.32, -0.63], [0.57, 0.15]],
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
                "init_x": -1.63,
                "init_y": 5.94,
                "init_a": -85.45,
                "velocity": 1.09,
                "goals": [[0.18, -3.26], [2.40, -0.02], [3.49, 0.93], [7.96, -1.36]],
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
                "init_x": -0.44,
                "init_y": -5.91,
                "init_a": -77.71,
                "velocity": 1.05,
                "goals": [[-0.62, 2.05], [5.98, 5.92], [3.89, 3.35], [-7.97, -0.52]],
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
                "init_x": -2.52,
                "init_y": 2.61,
                "init_a": 135.16,
                "velocity": 0.83,
                "goals": [[6.23, 1.80], [0.56, -0.37], [6.21, -5.80], [4.66, -0.79]],
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
                "init_x": 5.70,
                "init_y": -2.66,
                "init_a": 117.10,
                "velocity": 1.02,
                "goals": [[6.39, -0.36], [7.27, -1.58], [-1.81, -0.83], [-6.91, -4.76]],
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

def tests_adult_0_child_100_test_case_31_stopped_low(tester):
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
                "goals": [[7.04, -2.91], [-7.37, 1.97], [3.17, -2.93], [-6.81, -3.86]],
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
                "goals": [[7.04, -2.91], [-3.08, 0.83], [7.34, 1.25], [1.00, -4.88]],
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
                "init_x": 1.59,
                "init_y": -2.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goals": [[1.59, -2.71], [4.12, 5.48], [2.54, -0.56], [5.72, 5.15]],
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
                "init_x": 1.52,
                "init_y": -1.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goals": [[1.59, -2.71], [-6.76, -0.62], [4.20, -5.34], [4.37, -2.51]],
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
                "goals": [[3.23, 2.89], [-7.33, -0.89], [6.87, -0.39], [6.66, 5.96]],
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
                "init_x": 0.46,
                "init_y": 4.54,
                "init_a": -162.87,
                "velocity": 1.13,
                "goals": [[7.42, -5.70], [1.16, -1.45], [-5.47, -2.48], [-4.94, -4.22]],
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
                "init_x": -7.13,
                "init_y": -3.15,
                "init_a": 52.77,
                "velocity": 0.84,
                "goals": [[-6.95, 1.01], [-1.56, 4.90], [-1.28, -4.18], [7.35, -2.48]],
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

def tests_adult_0_child_100_test_case_32_stopped_low(tester):
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
                "goals": [[-7.93, 5.89], [-3.33, -0.32], [-7.78, -0.59], [2.04, 2.83]],
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
                "init_x": -8.69,
                "init_y": 6.54,
                "init_a": 65.90,
                "velocity": 0.94,
                "goals": [[-7.93, 5.89], [-2.00, 1.59], [-4.38, -0.73], [2.55, -2.17]],
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
                "init_x": 7.53,
                "init_y": 4.18,
                "init_a": -95.26,
                "velocity": 0.95,
                "goals": [[7.53, 4.18], [-1.50, -2.85], [3.21, -3.08], [4.72, 0.70]],
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
                "init_x": 7.63,
                "init_y": 3.18,
                "init_a": -95.26,
                "velocity": 0.95,
                "goals": [[7.53, 4.18], [-5.03, -4.01], [6.65, 1.18], [-0.24, 2.51]],
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
                "init_x": -4.20,
                "init_y": 4.82,
                "init_a": -92.35,
                "velocity": 1.14,
                "goals": [[2.00, 0.65], [-7.76, -4.62], [3.23, 1.01], [-2.66, 2.26]],
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
                "init_x": 4.64,
                "init_y": 5.70,
                "init_a": 46.97,
                "velocity": 0.91,
                "goals": [[-1.01, -4.80], [-3.81, -3.66], [6.01, 0.18], [1.47, 2.67]],
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
                "goals": [[-6.88, 3.92], [1.95, 4.51], [-2.51, 1.15], [5.35, 0.07]],
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

def tests_adult_0_child_100_test_case_33_stopped_low(tester):
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
                "goals": [[6.00, 5.47], [-4.96, -0.42], [-5.41, -0.75], [-7.49, 4.26]],
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
                "init_x": 6.38,
                "init_y": 6.39,
                "init_a": -77.91,
                "velocity": 1.04,
                "goals": [[6.00, 5.47], [2.03, 0.88], [-4.94, 5.25], [6.77, 1.16]],
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
                "init_x": 5.78,
                "init_y": 4.94,
                "init_a": -171.83,
                "velocity": 0.87,
                "goals": [[5.78, 4.94], [4.01, 3.84], [6.48, -4.05], [0.06, -0.89]],
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
                "init_y": 4.02,
                "init_a": -171.83,
                "velocity": 0.87,
                "goals": [[5.78, 4.94], [6.83, -4.09], [5.72, 3.20], [-4.58, 1.57]],
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
                "init_x": 2.38,
                "init_y": -3.94,
                "init_a": -33.94,
                "velocity": 1.07,
                "goals": [[6.19, 4.16], [4.27, -5.68], [5.58, 5.02], [5.90, 3.55]],
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
                "goals": [[5.30, -5.56], [-0.29, 4.69], [7.62, 3.38], [1.92, -5.61]],
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
                "goals": [[-5.18, 5.63], [5.46, -5.86], [5.11, 3.48], [-2.52, -1.64]],
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

def tests_adult_0_child_100_test_case_34_walking_low(tester):
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
                "init_x": -3.82,
                "init_y": -0.12,
                "init_a": -177.86,
                "velocity": 0.96,
                "goals": [[-5.18, -5.55], [1.15, -1.40], [6.19, 4.11], [7.73, 1.61]],
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
                "init_x": -4.20,
                "init_y": 0.80,
                "init_a": -177.86,
                "velocity": 0.96,
                "goals": [[-5.18, -5.55], [-0.63, 0.75], [-5.72, 2.09], [5.81, 4.04]],
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
                "init_x": 2.93,
                "init_y": 0.83,
                "init_a": 172.32,
                "velocity": 0.81,
                "goals": [[-4.45, 4.16], [-2.95, -1.39], [4.40, 5.26], [2.93, 3.50]],
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
                "init_x": 3.50,
                "init_y": 1.65,
                "init_a": 172.32,
                "velocity": 0.81,
                "goals": [[-4.45, 4.16], [0.64, 1.10], [2.13, -5.85], [-2.67, -0.69]],
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
                "init_x": -6.75,
                "init_y": 1.31,
                "init_a": 14.10,
                "velocity": 0.98,
                "goals": [[-7.44, 5.58], [-6.30, -2.01], [3.33, -2.89], [-1.76, -3.20]],
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
                "init_x": -0.58,
                "init_y": -2.74,
                "init_a": -95.95,
                "velocity": 1.05,
                "goals": [[6.34, 4.21], [3.80, 5.54], [-1.47, -0.37], [-5.16, 5.61]],
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
                "init_x": 2.99,
                "init_y": -5.51,
                "init_a": 147.11,
                "velocity": 1.08,
                "goals": [[-0.71, -1.26], [-7.77, -4.78], [-1.75, -4.00], [7.68, -4.81]],
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
                "init_x": 1.41,
                "init_y": -1.12,
                "init_a": 24.71,
                "velocity": 0.98,
                "goals": [[6.40, -4.51], [4.97, -0.65], [-6.01, 3.21], [-0.15, 3.07]],
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
                "init_x": -7.43,
                "init_y": 3.14,
                "init_a": -174.29,
                "velocity": 0.91,
                "goals": [[-1.91, -1.61], [-4.72, -0.29], [-5.27, 3.89], [-3.63, 4.46]],
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

def tests_adult_0_child_100_test_case_35_stopped_low(tester):
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
                "init_x": 4.61,
                "init_y": 0.41,
                "init_a": 131.72,
                "velocity": 1.13,
                "goals": [[4.61, 0.41], [-1.96, -3.77], [2.63, -4.89], [-3.62, -5.52]],
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
                "init_x": 4.91,
                "init_y": -0.54,
                "init_a": 131.72,
                "velocity": 1.13,
                "goals": [[4.61, 0.41], [-2.45, -5.01], [-2.05, -0.18], [6.33, -2.95]],
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
                "init_x": -5.93,
                "init_y": 3.07,
                "init_a": 83.88,
                "velocity": 0.93,
                "goals": [[-5.93, 3.07], [5.48, -3.91], [-5.23, -0.39], [-5.27, 5.17]],
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
                "init_x": -6.86,
                "init_y": 3.44,
                "init_a": 83.88,
                "velocity": 0.93,
                "goals": [[-5.93, 3.07], [-7.43, -3.13], [2.82, -3.22], [7.73, -2.91]],
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
                "init_x": 4.96,
                "init_y": -5.06,
                "init_a": 104.22,
                "velocity": 1.04,
                "goals": [[1.45, -4.68], [-5.31, 1.87], [-4.20, 2.08], [-2.39, 4.23]],
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
                "init_x": 6.86,
                "init_y": 2.78,
                "init_a": 151.85,
                "velocity": 0.90,
                "goals": [[0.18, -5.87], [-4.87, 3.63], [1.96, 4.45], [0.21, -4.16]],
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
                "init_x": -3.53,
                "init_y": 0.65,
                "init_a": -46.43,
                "velocity": 1.12,
                "goals": [[2.38, -3.10], [-4.74, 2.38], [-7.46, -2.44], [4.28, 4.87]],
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
                "init_x": -6.06,
                "init_y": -5.40,
                "init_a": -123.89,
                "velocity": 1.13,
                "goals": [[-4.51, -1.89], [-0.44, -3.59], [0.15, 3.42], [-7.65, 4.31]],
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
                "init_x": -4.92,
                "init_y": 5.72,
                "init_a": -117.69,
                "velocity": 0.80,
                "goals": [[-7.93, 4.58], [4.22, -5.41], [-5.06, -3.40], [6.77, 4.24]],
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

def tests_adult_0_child_100_test_case_36_stopped_low(tester):
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
                "goals": [[-3.70, -3.49], [1.35, 1.70], [4.76, -2.46], [0.26, -1.37]],
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
                "init_x": -3.23,
                "init_y": -2.61,
                "init_a": 63.37,
                "velocity": 1.05,
                "goals": [[-3.70, -3.49], [2.69, 0.81], [7.02, 5.45], [-5.16, 3.72]],
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
                "init_x": -4.13,
                "init_y": 5.07,
                "init_a": -14.49,
                "velocity": 0.86,
                "goals": [[-4.13, 5.07], [7.55, 2.16], [5.95, 1.24], [3.89, -3.50]],
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
                "init_x": -3.71,
                "init_y": 5.98,
                "init_a": -14.49,
                "velocity": 0.86,
                "goals": [[-4.13, 5.07], [1.73, 3.40], [1.84, -1.82], [-5.59, 4.85]],
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
                "init_x": -3.07,
                "init_y": 3.47,
                "init_a": 142.87,
                "velocity": 1.03,
                "goals": [[2.08, 4.91], [-2.83, -0.19], [-0.88, -1.54], [-2.98, -5.57]],
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
                "init_x": -3.22,
                "init_y": -5.94,
                "init_a": 133.05,
                "velocity": 1.16,
                "goals": [[3.43, -1.16], [3.75, 1.12], [-3.07, 5.52], [-4.67, 3.72]],
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
                "init_x": 1.74,
                "init_y": -1.65,
                "init_a": -85.48,
                "velocity": 0.93,
                "goals": [[-0.60, 3.76], [0.35, 3.35], [1.29, 1.87], [3.47, -2.39]],
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

def tests_adult_0_child_100_test_case_37_stopped_low(tester):
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
                "goals": [[-5.02, 0.90], [-6.29, 3.63], [-7.63, 4.19], [-7.27, -1.42]],
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
                "init_x": -5.46,
                "init_y": 0.00,
                "init_a": 102.07,
                "velocity": 1.19,
                "goals": [[-5.02, 0.90], [7.10, 2.15], [7.36, 4.45], [6.11, 4.56]],
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
                "init_x": -7.28,
                "init_y": 2.94,
                "init_a": 59.29,
                "velocity": 0.97,
                "goals": [[-7.28, 2.94], [5.16, 5.96], [3.82, -4.10], [-1.89, -0.40]],
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
                "init_x": -6.35,
                "init_y": 2.58,
                "init_a": 59.29,
                "velocity": 0.97,
                "goals": [[-7.28, 2.94], [-4.25, -1.71], [2.02, -1.44], [-2.61, 1.23]],
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
                "init_x": 7.76,
                "init_y": -2.95,
                "init_a": -30.08,
                "velocity": 1.03,
                "goals": [[-6.22, -1.79], [-6.39, -2.90], [-4.34, -2.19], [5.12, -3.86]],
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
                "init_x": -5.77,
                "init_y": -1.62,
                "init_a": -60.49,
                "velocity": 0.85,
                "goals": [[-2.91, 4.58], [3.88, -4.61], [1.98, -0.94], [-4.07, 5.75]],
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
                "init_x": 0.52,
                "init_y": 0.25,
                "init_a": 126.27,
                "velocity": 0.88,
                "goals": [[-7.76, -1.67], [0.01, 0.50], [6.94, -0.40], [0.45, 5.85]],
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

def tests_adult_0_child_100_test_case_38_stopped_low(tester):
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
                "init_x": 4.49,
                "init_y": 2.40,
                "init_a": -69.41,
                "velocity": 1.05,
                "goals": [[4.49, 2.40], [5.46, -5.65], [5.27, 4.31], [0.11, 2.87]],
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
                "init_x": 5.04,
                "init_y": 3.23,
                "init_a": -69.41,
                "velocity": 1.05,
                "goals": [[4.49, 2.40], [2.49, -1.01], [6.19, -5.94], [-5.35, 0.64]],
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
                "init_x": 5.17,
                "init_y": 1.61,
                "init_a": 172.86,
                "velocity": 1.16,
                "goals": [[5.17, 1.61], [1.03, 1.52], [4.21, -4.96], [-1.80, -4.91]],
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
                "init_x": 6.10,
                "init_y": 1.98,
                "init_a": 172.86,
                "velocity": 1.16,
                "goals": [[5.17, 1.61], [-1.48, -3.64], [6.92, -3.34], [-1.28, 0.47]],
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
                "init_x": -1.78,
                "init_y": -0.10,
                "init_a": 97.54,
                "velocity": 0.96,
                "goals": [[0.17, 1.64], [-5.11, -5.39], [-2.27, -4.41], [-1.44, 0.69]],
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
                "init_x": -1.18,
                "init_y": 2.18,
                "init_a": 59.07,
                "velocity": 0.99,
                "goals": [[2.19, -4.41], [-3.54, 4.40], [-4.29, -4.15], [-7.76, 2.93]],
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
                "init_x": 4.68,
                "init_y": -2.56,
                "init_a": -100.52,
                "velocity": 0.92,
                "goals": [[1.65, 3.36], [5.23, -0.81], [-5.97, -3.49], [-4.22, -3.11]],
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
                "init_x": 7.46,
                "init_y": 2.45,
                "init_a": -39.44,
                "velocity": 0.87,
                "goals": [[-1.23, -5.42], [-3.68, 5.11], [3.04, 4.04], [5.95, -5.32]],
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
                "init_x": -7.07,
                "init_y": 0.59,
                "init_a": 47.54,
                "velocity": 0.92,
                "goals": [[6.65, -3.74], [3.18, -0.97], [-6.13, 4.15], [7.95, 2.30]],
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

def tests_adult_0_child_100_test_case_39_stopped_low(tester):
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
                "goals": [[1.38, -2.33], [6.05, -4.07], [-6.87, 4.41], [-7.08, -5.19]],
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
                "init_x": 1.15,
                "init_y": -1.36,
                "init_a": 0.15,
                "velocity": 1.07,
                "goals": [[1.38, -2.33], [1.91, -1.31], [7.71, 0.89], [-3.47, 3.90]],
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
                "init_x": 5.80,
                "init_y": -4.06,
                "init_a": -161.85,
                "velocity": 1.12,
                "goals": [[5.80, -4.06], [-4.90, 0.11], [-7.53, 3.18], [1.41, -2.99]],
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
                "init_x": 4.82,
                "init_y": -3.83,
                "init_a": -161.85,
                "velocity": 1.12,
                "goals": [[5.80, -4.06], [0.33, 2.02], [-7.90, 5.36], [-5.08, 4.43]],
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
                "init_x": -5.39,
                "init_y": -1.09,
                "init_a": 97.81,
                "velocity": 1.15,
                "goals": [[-4.19, -4.10], [-4.92, 5.55], [5.76, -3.15], [-3.50, 0.58]],
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
                "init_x": 2.84,
                "init_y": 1.30,
                "init_a": 153.54,
                "velocity": 0.90,
                "goals": [[0.87, 1.56], [7.84, 2.38], [4.02, -5.48], [3.77, -0.09]],
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
                "init_x": 6.97,
                "init_y": 3.81,
                "init_a": 49.81,
                "velocity": 1.20,
                "goals": [[-5.73, -1.01], [-1.05, 3.76], [-3.38, 4.75], [0.88, -3.67]],
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

def tests_adult_0_child_100_test_case_40_stopped_low(tester):
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
                "init_x": 6.87,
                "init_y": 3.87,
                "init_a": -124.82,
                "velocity": 1.11,
                "goals": [[6.87, 3.87], [-1.66, 5.16], [-1.88, -3.75], [6.80, 0.72]],
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
                "init_x": 6.65,
                "init_y": 2.89,
                "init_a": -124.82,
                "velocity": 1.11,
                "goals": [[6.87, 3.87], [-0.13, -1.27], [-1.80, 2.69], [-7.15, -0.43]],
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
                "init_x": -6.86,
                "init_y": 1.23,
                "init_a": 59.64,
                "velocity": 1.16,
                "goals": [[-6.86, 1.23], [5.59, 3.47], [7.70, 3.90], [0.39, 1.43]],
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
                "init_x": -6.40,
                "init_y": 2.12,
                "init_a": 59.64,
                "velocity": 1.16,
                "goals": [[-6.86, 1.23], [-3.88, 5.26], [-4.64, 4.28], [-0.01, -2.85]],
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
                "init_x": 5.32,
                "init_y": 1.57,
                "init_a": -165.67,
                "velocity": 1.03,
                "goals": [[-6.97, -5.11], [4.81, 3.71], [3.16, -4.89], [-3.93, -0.90]],
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
                "init_x": 6.78,
                "init_y": 5.11,
                "init_a": -139.55,
                "velocity": 0.82,
                "goals": [[-4.59, 2.13], [5.95, 4.31], [-0.04, 3.39], [-5.10, 3.82]],
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
                "init_x": -0.61,
                "init_y": -3.01,
                "init_a": 29.82,
                "velocity": 0.89,
                "goals": [[-6.62, -4.67], [6.14, -5.71], [-1.22, -2.16], [-4.32, 3.42]],
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

def tests_adult_0_child_100_test_case_41_walking_low(tester):
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
                "goals": [[-7.88, 3.45], [-5.16, -0.63], [0.65, -5.15], [-7.50, -2.76]],
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
                "init_x": 2.82,
                "init_y": -3.50,
                "init_a": -132.76,
                "velocity": 0.88,
                "goals": [[-7.88, 3.45], [7.34, -2.39], [7.14, 5.90], [-0.56, -5.66]],
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
                "init_x": 5.60,
                "init_y": 0.49,
                "init_a": -8.20,
                "velocity": 0.96,
                "goals": [[-2.82, -2.38], [-1.52, -0.70], [-7.48, 3.98], [4.59, 4.11]],
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
                "init_x": 4.73,
                "init_y": -0.01,
                "init_a": -8.20,
                "velocity": 0.96,
                "goals": [[-2.82, -2.38], [-6.37, 3.59], [6.71, 2.89], [7.52, 2.22]],
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
                "init_x": 5.47,
                "init_y": -0.21,
                "init_a": -76.59,
                "velocity": 1.03,
                "goals": [[5.38, -4.28], [6.63, 0.38], [-7.31, -5.43], [-3.18, 1.07]],
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
                "init_x": -5.49,
                "init_y": -5.63,
                "init_a": 28.58,
                "velocity": 1.16,
                "goals": [[5.00, -2.36], [1.71, 3.47], [3.87, -5.79], [-6.21, -1.22]],
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
                "init_x": -0.03,
                "init_y": 3.17,
                "init_a": -26.91,
                "velocity": 0.84,
                "goals": [[5.49, -2.47], [1.71, -2.34], [-5.02, 4.11], [-7.91, -4.25]],
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
                "goals": [[3.50, -1.06], [7.13, -1.97], [-4.76, 5.43], [-7.53, 2.10]],
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

def tests_adult_0_child_100_test_case_42_walking_low(tester):
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
                "init_x": -4.48,
                "init_y": -2.73,
                "init_a": -52.09,
                "velocity": 0.92,
                "goals": [[2.37, 5.22], [6.52, 2.93], [5.18, -0.38], [5.72, -1.71]],
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
                "init_x": -3.48,
                "init_y": -2.71,
                "init_a": -52.09,
                "velocity": 0.92,
                "goals": [[2.37, 5.22], [3.40, 1.95], [-2.19, 4.75], [4.52, 3.06]],
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
                "goals": [[7.65, 3.40], [-5.92, -1.47], [1.99, 1.26], [6.95, -5.02]],
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
                "init_x": -6.38,
                "init_y": 2.73,
                "init_a": -27.05,
                "velocity": 1.08,
                "goals": [[7.65, 3.40], [2.73, -3.24], [2.13, 0.12], [0.04, 5.14]],
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
                "init_x": 5.24,
                "init_y": 1.73,
                "init_a": 137.96,
                "velocity": 0.97,
                "goals": [[-7.48, -1.91], [-0.60, 1.71], [-2.39, 0.22], [7.78, 3.73]],
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
                "init_x": 5.73,
                "init_y": -5.77,
                "init_a": -153.13,
                "velocity": 1.06,
                "goals": [[-2.54, 4.18], [7.93, 0.57], [-3.26, -0.15], [7.36, 4.09]],
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
                "goals": [[-5.03, 2.17], [0.86, 0.61], [-5.36, -2.33], [-2.15, 5.78]],
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
                "goals": [[4.50, 4.39], [-6.66, 4.58], [6.27, -3.88], [-6.02, -1.81]],
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
                "init_x": 5.57,
                "init_y": -0.62,
                "init_a": -134.04,
                "velocity": 1.05,
                "goals": [[-4.23, 5.54], [0.86, 0.40], [5.03, 3.81], [5.88, 5.67]],
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

def tests_adult_0_child_100_test_case_43_stopped_low(tester):
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
                "goals": [[6.02, 3.19], [3.75, 4.82], [-4.37, 5.75], [-6.97, -0.39]],
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
                "init_x": 6.94,
                "init_y": 2.81,
                "init_a": -51.83,
                "velocity": 0.92,
                "goals": [[6.02, 3.19], [-4.85, 0.24], [5.21, -0.40], [-7.36, 5.21]],
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
                "init_x": 0.74,
                "init_y": 5.47,
                "init_a": -5.71,
                "velocity": 0.88,
                "goals": [[0.74, 5.47], [-4.69, 3.76], [6.71, -1.57], [-2.66, 2.59]],
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
                "init_x": 1.20,
                "init_y": 6.36,
                "init_a": -5.71,
                "velocity": 0.88,
                "goals": [[0.74, 5.47], [-4.72, 4.51], [0.84, 4.54], [-0.97, 0.56]],
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
                "init_x": -2.72,
                "init_y": -5.79,
                "init_a": 143.12,
                "velocity": 0.98,
                "goals": [[-1.83, -2.08], [7.51, 0.28], [0.05, 5.68], [2.78, -2.37]],
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
                "init_x": -2.48,
                "init_y": -3.08,
                "init_a": -120.31,
                "velocity": 0.82,
                "goals": [[4.02, 1.16], [1.96, -5.37], [-4.90, -1.49], [2.80, 4.82]],
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
                "init_x": -1.02,
                "init_y": 0.17,
                "init_a": 116.73,
                "velocity": 0.88,
                "goals": [[4.59, 0.20], [-1.52, 3.19], [-1.96, -0.29], [5.30, -3.68]],
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
                "init_x": -7.00,
                "init_y": -5.59,
                "init_a": -19.32,
                "velocity": 1.06,
                "goals": [[7.41, -0.38], [-0.59, -3.94], [7.19, -5.41], [-3.25, 5.53]],
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
                "goals": [[-7.31, -3.40], [-4.66, -3.84], [0.99, 1.95], [2.02, -4.82]],
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

def tests_adult_0_child_100_test_case_44_stopped_low(tester):
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
                "goals": [[-2.76, -1.11], [-4.51, 1.42], [-5.46, 3.74], [-7.53, -2.48]],
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
                "init_x": -2.09,
                "init_y": -0.36,
                "init_a": 62.70,
                "velocity": 1.02,
                "goals": [[-2.76, -1.11], [-5.79, -0.66], [-3.97, 2.12], [6.29, 5.10]],
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
                "init_x": 7.80,
                "init_y": 3.42,
                "init_a": 104.72,
                "velocity": 0.88,
                "goals": [[7.80, 3.42], [-0.44, -4.41], [5.42, 0.31], [-2.95, -2.88]],
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
                "init_x": 7.27,
                "init_y": 4.27,
                "init_a": 104.72,
                "velocity": 0.88,
                "goals": [[7.80, 3.42], [1.82, -3.10], [3.43, 1.03], [0.43, 5.45]],
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
                "init_x": -1.36,
                "init_y": 0.85,
                "init_a": 88.60,
                "velocity": 1.04,
                "goals": [[-3.59, -0.02], [-4.53, 3.41], [1.39, -1.85], [2.77, 4.12]],
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
                "init_x": -7.63,
                "init_y": -1.61,
                "init_a": -32.34,
                "velocity": 1.05,
                "goals": [[4.79, 1.72], [4.25, -0.59], [-7.89, -4.22], [1.71, -5.53]],
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
                "init_x": -3.05,
                "init_y": 5.53,
                "init_a": 127.45,
                "velocity": 1.09,
                "goals": [[5.39, 1.06], [6.71, -4.82], [5.72, -5.74], [-3.14, -5.58]],
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
                "init_x": 3.33,
                "init_y": 1.11,
                "init_a": -119.66,
                "velocity": 0.82,
                "goals": [[-7.51, 0.54], [6.21, 5.27], [4.62, 2.37], [-5.90, 2.35]],
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

def tests_adult_0_child_100_test_case_45_stopped_low(tester):
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
                "goals": [[3.96, -5.80], [-3.54, 1.18], [4.77, 1.79], [0.30, -0.59]],
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
                "init_x": 4.59,
                "init_y": -6.57,
                "init_a": -169.11,
                "velocity": 1.17,
                "goals": [[3.96, -5.80], [3.97, -4.39], [-2.50, -2.47], [-3.97, 1.03]],
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
                "init_x": 3.60,
                "init_y": -3.44,
                "init_a": 101.70,
                "velocity": 1.03,
                "goals": [[3.60, -3.44], [0.39, 5.04], [4.16, 3.99], [5.93, 2.24]],
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
                "goals": [[3.60, -3.44], [7.22, 0.33], [1.83, -4.96], [-7.29, -0.33]],
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
                "goals": [[-2.67, -1.91], [7.44, 1.25], [-6.76, 5.70], [-1.35, -1.99]],
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
                "init_x": 1.31,
                "init_y": -1.38,
                "init_a": -167.79,
                "velocity": 0.93,
                "goals": [[7.50, 3.94], [-7.74, 1.72], [-5.80, -4.56], [6.01, 2.97]],
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
                "init_x": -4.51,
                "init_y": 4.39,
                "init_a": -91.71,
                "velocity": 1.11,
                "goals": [[-0.77, 3.00], [-5.65, 0.78], [3.86, 5.57], [-5.14, -5.25]],
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

def tests_adult_0_child_100_test_case_46_walking_low(tester):
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
                "init_x": 5.95,
                "init_y": 1.00,
                "init_a": 19.67,
                "velocity": 1.01,
                "goals": [[-5.50, 3.69], [0.08, 4.59], [6.39, -1.03], [-4.42, 1.75]],
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
                "init_x": 6.33,
                "init_y": 0.08,
                "init_a": 19.67,
                "velocity": 1.01,
                "goals": [[-5.50, 3.69], [-7.85, 4.30], [-0.06, -1.91], [3.95, -3.90]],
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
                "init_x": 3.95,
                "init_y": -0.34,
                "init_a": 109.60,
                "velocity": 1.19,
                "goals": [[-3.89, -3.62], [-2.89, 0.13], [0.17, -2.52], [7.58, -0.26]],
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
                "init_x": 3.02,
                "init_y": -0.70,
                "init_a": 109.60,
                "velocity": 1.19,
                "goals": [[-3.89, -3.62], [7.06, 2.42], [1.97, -1.93], [-0.78, 2.50]],
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
                "init_x": -7.47,
                "init_y": -4.48,
                "init_a": 9.62,
                "velocity": 0.84,
                "goals": [[7.28, -3.23], [-5.46, -4.28], [4.61, 3.04], [4.33, 2.17]],
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
                "init_x": -0.96,
                "init_y": 0.98,
                "init_a": 29.90,
                "velocity": 1.00,
                "goals": [[7.74, 2.67], [-5.41, 2.24], [-0.62, 3.85], [2.30, 0.66]],
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
                "init_x": 2.85,
                "init_y": -1.12,
                "init_a": -92.14,
                "velocity": 1.01,
                "goals": [[-1.70, -4.43], [-5.96, 5.29], [-1.34, -1.08], [-3.10, -0.38]],
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
                "init_x": -1.52,
                "init_y": -3.75,
                "init_a": 93.07,
                "velocity": 0.90,
                "goals": [[4.64, -1.43], [-3.33, 5.92], [-2.79, 1.69], [2.76, 4.99]],
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

def tests_adult_0_child_100_test_case_47_walking_low(tester):
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
                "goals": [[5.38, -4.67], [-3.30, -1.81], [1.10, 5.32], [-2.70, 4.17]],
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
                "init_x": 1.52,
                "init_y": -1.39,
                "init_a": -83.82,
                "velocity": 0.96,
                "goals": [[5.38, -4.67], [-3.93, 3.09], [4.23, 3.44], [-2.55, -4.30]],
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
                "init_x": -7.28,
                "init_y": -3.32,
                "init_a": -46.08,
                "velocity": 1.09,
                "goals": [[-6.61, 4.32], [7.64, -2.85], [-5.65, -0.42], [7.94, -0.80]],
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
                "init_x": -7.14,
                "init_y": -2.33,
                "init_a": -46.08,
                "velocity": 1.09,
                "goals": [[-6.61, 4.32], [7.79, -1.84], [1.47, -1.83], [-1.58, -0.65]],
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
                "init_x": 7.60,
                "init_y": -4.23,
                "init_a": 35.99,
                "velocity": 1.04,
                "goals": [[3.88, -5.23], [-6.00, 1.02], [-4.22, -1.74], [-6.93, -2.74]],
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
                "init_x": 3.75,
                "init_y": -3.94,
                "init_a": -10.22,
                "velocity": 0.89,
                "goals": [[2.55, -2.24], [-2.15, -2.08], [-7.20, -5.29], [7.07, -3.58]],
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
                "goals": [[-3.71, 2.67], [2.68, -0.73], [-4.37, 3.88], [4.68, -2.22]],
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

def tests_adult_0_child_100_test_case_48_walking_low(tester):
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
                "goals": [[-5.16, -0.76], [-5.74, -4.29], [-4.01, 3.98], [-1.25, 2.12]],
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
                "goals": [[-5.16, -0.76], [4.93, -1.10], [-1.99, -2.05], [6.21, 0.15]],
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
                "init_x": -2.65,
                "init_y": 3.64,
                "init_a": -7.87,
                "velocity": 1.15,
                "goals": [[4.57, 3.77], [-1.03, 1.08], [2.35, -2.27], [7.83, -1.49]],
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
                "init_x": -1.82,
                "init_y": 3.09,
                "init_a": -7.87,
                "velocity": 1.15,
                "goals": [[4.57, 3.77], [3.77, 4.76], [4.90, 3.81], [1.89, 5.98]],
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
                "init_x": -1.24,
                "init_y": 1.32,
                "init_a": -165.93,
                "velocity": 0.95,
                "goals": [[-6.77, -1.10], [-2.04, -1.72], [-4.54, -4.11], [1.49, -5.35]],
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
                "init_x": 7.20,
                "init_y": -0.75,
                "init_a": 166.29,
                "velocity": 1.15,
                "goals": [[-1.18, -2.13], [-6.90, -1.02], [2.07, 2.13], [6.98, -1.35]],
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
                "init_x": 2.21,
                "init_y": 0.14,
                "init_a": -27.55,
                "velocity": 1.20,
                "goals": [[3.86, -5.84], [4.06, -3.67], [-3.34, -3.79], [5.44, -3.67]],
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

def tests_adult_0_child_100_test_case_49_walking_low(tester):
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
                "goals": [[4.09, 3.57], [-0.83, 0.97], [-7.80, -5.71], [2.59, -0.36]],
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
                "goals": [[4.09, 3.57], [1.24, 5.57], [4.71, -3.95], [6.56, -4.65]],
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
                "goals": [[4.49, 1.10], [-7.12, -1.05], [1.91, 2.91], [-2.93, -5.07]],
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
                "goals": [[4.49, 1.10], [-5.96, -0.83], [-6.77, 1.85], [-1.77, 0.59]],
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
                "init_x": 0.47,
                "init_y": 5.92,
                "init_a": -29.06,
                "velocity": 1.05,
                "goals": [[-5.51, 4.39], [-7.51, -0.64], [-6.38, -4.91], [6.45, -4.47]],
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
                "init_x": -7.71,
                "init_y": -3.31,
                "init_a": -104.12,
                "velocity": 0.80,
                "goals": [[-0.45, 3.79], [-1.65, -0.28], [1.12, -3.26], [4.00, -4.29]],
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
                "init_x": 1.94,
                "init_y": -1.27,
                "init_a": 27.10,
                "velocity": 1.09,
                "goals": [[-7.70, 3.21], [4.88, -3.08], [7.17, 0.26], [2.63, -2.74]],
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
                "init_x": 5.13,
                "init_y": -0.55,
                "init_a": -22.34,
                "velocity": 0.88,
                "goals": [[-4.68, -4.74], [-0.28, 3.81], [-4.74, 0.03], [-3.75, 1.86]],
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

def tests_adult_0_child_100_test_case_50_walking_low(tester):
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
                "goals": [[0.27, -0.82], [-2.17, -2.26], [-5.43, -4.96], [6.60, 1.78]],
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
                "init_x": 1.46,
                "init_y": 2.40,
                "init_a": -24.16,
                "velocity": 1.13,
                "goals": [[0.27, -0.82], [-6.61, -3.50], [2.67, -5.16], [6.86, 4.13]],
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
                "init_x": 5.74,
                "init_y": -1.10,
                "init_a": 12.39,
                "velocity": 0.85,
                "goals": [[3.12, 1.80], [-5.19, -1.99], [2.59, -0.87], [2.94, 1.88]],
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
                "init_x": 4.81,
                "init_y": -0.74,
                "init_a": 12.39,
                "velocity": 0.85,
                "goals": [[3.12, 1.80], [-5.54, 5.40], [-5.24, 0.31], [5.28, 1.22]],
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
                "init_x": -7.02,
                "init_y": -4.37,
                "init_a": 126.79,
                "velocity": 0.89,
                "goals": [[-2.10, -4.06], [2.60, 4.15], [0.94, -4.10], [3.18, 5.06]],
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
                "init_x": 3.36,
                "init_y": -5.69,
                "init_a": -61.18,
                "velocity": 1.13,
                "goals": [[0.01, -1.76], [-5.35, -1.61], [-4.07, 0.91], [-0.46, -1.43]],
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
                "init_x": 0.89,
                "init_y": 0.78,
                "init_a": -133.53,
                "velocity": 1.20,
                "goals": [[-3.98, -4.85], [1.35, 3.50], [1.38, 3.39], [-4.28, 3.64]],
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

def tests_adult_0_child_100_test_case_51_walking_low(tester):
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
                "init_x": -6.08,
                "init_y": -2.95,
                "init_a": -32.01,
                "velocity": 0.85,
                "goals": [[-3.53, 3.45], [7.17, -0.21], [-1.40, 3.41], [-5.19, -5.87]],
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
                "init_x": -5.30,
                "init_y": -2.34,
                "init_a": -32.01,
                "velocity": 0.85,
                "goals": [[-3.53, 3.45], [4.92, -0.73], [-4.33, 1.17], [-3.79, 2.55]],
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
                "init_x": 3.61,
                "init_y": 0.06,
                "init_a": 61.58,
                "velocity": 1.12,
                "goals": [[6.17, 2.16], [1.35, -0.92], [1.07, -1.08], [-1.83, -5.92]],
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
                "init_x": 3.67,
                "init_y": 1.06,
                "init_a": 61.58,
                "velocity": 1.12,
                "goals": [[6.17, 2.16], [-3.74, 1.56], [-6.88, 3.75], [-5.33, 1.43]],
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
                "init_x": 7.83,
                "init_y": -5.77,
                "init_a": -84.96,
                "velocity": 1.12,
                "goals": [[-4.41, -4.56], [0.89, -1.56], [-3.89, 3.00], [-0.06, -4.99]],
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
                "init_x": 0.35,
                "init_y": 1.25,
                "init_a": -72.79,
                "velocity": 0.80,
                "goals": [[-1.57, -3.14], [-5.32, -4.60], [5.25, -3.31], [5.67, 5.21]],
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
                "init_x": -2.33,
                "init_y": 4.33,
                "init_a": -174.49,
                "velocity": 1.19,
                "goals": [[-0.97, -2.10], [-7.78, -1.98], [5.26, 3.32], [4.66, -5.62]],
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
                "init_x": 2.12,
                "init_y": -0.91,
                "init_a": -39.51,
                "velocity": 0.88,
                "goals": [[-3.83, -3.31], [-1.12, 1.91], [-2.07, -4.68], [-2.80, -0.69]],
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

def tests_adult_0_child_100_test_case_52_stopped_low(tester):
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
                "goals": [[7.30, -0.08], [6.97, 1.88], [-4.04, -4.36], [6.16, -1.66]],
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
                "init_x": 8.28,
                "init_y": 0.13,
                "init_a": 58.20,
                "velocity": 0.83,
                "goals": [[7.30, -0.08], [3.33, 3.96], [6.03, 2.98], [-7.67, 1.94]],
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
                "init_x": -4.56,
                "init_y": -4.03,
                "init_a": -132.32,
                "velocity": 0.97,
                "goals": [[-4.56, -4.03], [-4.95, 4.25], [3.24, -2.31], [5.79, 3.74]],
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
                "init_x": -3.87,
                "init_y": -3.30,
                "init_a": -132.32,
                "velocity": 0.97,
                "goals": [[-4.56, -4.03], [-6.90, 1.32], [-2.96, 2.16], [-7.84, -2.99]],
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
                "init_x": -2.72,
                "init_y": -1.52,
                "init_a": 85.48,
                "velocity": 0.86,
                "goals": [[-2.39, 0.71], [-4.06, 2.88], [0.13, -0.65], [-6.27, -3.77]],
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
                "init_x": -2.94,
                "init_y": -1.14,
                "init_a": -29.67,
                "velocity": 1.15,
                "goals": [[1.08, 5.17], [5.90, 2.87], [-0.28, 3.14], [3.48, -4.07]],
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
                "init_x": 7.87,
                "init_y": 2.75,
                "init_a": -89.74,
                "velocity": 1.11,
                "goals": [[-2.40, -5.47], [-0.46, 0.74], [0.03, 1.55], [3.15, 0.22]],
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
                "goals": [[5.88, -4.46], [-2.96, -4.96], [-4.27, 3.07], [1.81, 4.91]],
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

def tests_adult_0_child_100_test_case_53_stopped_low(tester):
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
                "goals": [[-1.70, 4.83], [0.41, -2.30], [6.61, 0.63], [7.17, 2.54]],
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
                "init_x": -2.65,
                "init_y": 4.54,
                "init_a": -28.91,
                "velocity": 0.93,
                "goals": [[-1.70, 4.83], [2.88, -5.43], [-3.80, 2.39], [-2.54, -1.07]],
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
                "init_x": 0.75,
                "init_y": 0.98,
                "init_a": 22.73,
                "velocity": 0.92,
                "goals": [[0.75, 0.98], [-2.46, -2.19], [-4.14, -2.25], [-6.71, 2.02]],
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
                "init_x": 1.62,
                "init_y": 1.47,
                "init_a": 22.73,
                "velocity": 0.92,
                "goals": [[0.75, 0.98], [3.73, -2.74], [-0.02, -0.38], [-4.62, -3.91]],
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
                "init_x": 3.37,
                "init_y": 4.04,
                "init_a": -131.76,
                "velocity": 1.05,
                "goals": [[-1.15, -5.12], [5.26, -1.70], [7.14, 3.08], [-0.92, 3.40]],
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
                "goals": [[-4.48, 4.09], [-7.05, -4.46], [-2.47, -1.07], [7.33, 0.40]],
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
                "goals": [[4.14, -2.26], [-4.44, -5.04], [4.93, -4.99], [-2.55, -0.15]],
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

def tests_adult_0_child_100_test_case_54_walking_low(tester):
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
                "init_x": -7.63,
                "init_y": 0.82,
                "init_a": 101.38,
                "velocity": 0.98,
                "goals": [[3.98, 2.75], [-6.32, -1.25], [5.70, -4.79], [3.42, 2.94]],
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
                "init_x": -6.64,
                "init_y": 0.94,
                "init_a": 101.38,
                "velocity": 0.98,
                "goals": [[3.98, 2.75], [-3.82, 5.61], [-1.45, 3.42], [-2.79, 0.54]],
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
                "goals": [[7.44, -4.34], [3.65, -2.72], [7.99, -3.78], [4.32, -4.08]],
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
                "goals": [[7.44, -4.34], [4.72, -3.91], [-5.49, 4.71], [0.71, 1.21]],
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
                "init_x": -0.07,
                "init_y": 3.46,
                "init_a": -56.42,
                "velocity": 0.95,
                "goals": [[-2.62, 4.71], [1.84, 1.89], [-1.68, -1.45], [-1.20, 3.86]],
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
                "init_x": 5.06,
                "init_y": 5.78,
                "init_a": -14.62,
                "velocity": 0.84,
                "goals": [[-3.58, 3.91], [2.17, 3.58], [-6.18, -1.76], [-6.01, -2.93]],
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
                "init_x": -0.07,
                "init_y": -1.98,
                "init_a": -73.07,
                "velocity": 0.99,
                "goals": [[1.21, -0.99], [-5.40, -0.94], [4.01, -2.52], [1.44, 4.84]],
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
                "init_x": 3.87,
                "init_y": -5.52,
                "init_a": 66.98,
                "velocity": 0.80,
                "goals": [[-2.75, -1.68], [6.39, -3.49], [-6.69, 5.89], [-4.10, 1.33]],
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
                "init_x": -3.42,
                "init_y": 5.56,
                "init_a": -132.23,
                "velocity": 1.14,
                "goals": [[4.25, -1.15], [-2.41, 5.16], [-3.87, 3.79], [4.85, -3.50]],
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

def tests_adult_0_child_100_test_case_55_walking_low(tester):
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
                "goals": [[-1.34, -4.55], [-3.25, -1.97], [-5.90, -5.24], [-0.13, -5.57]],
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
                "init_x": 8.59,
                "init_y": -3.82,
                "init_a": -120.24,
                "velocity": 1.05,
                "goals": [[-1.34, -4.55], [3.21, 1.17], [1.82, 0.87], [7.98, -2.40]],
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
                "init_x": 0.59,
                "init_y": -4.57,
                "init_a": 45.41,
                "velocity": 1.09,
                "goals": [[1.47, -0.13], [7.91, -2.08], [-3.60, -0.32], [-3.37, -4.19]],
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
                "init_x": 0.04,
                "init_y": -3.74,
                "init_a": 45.41,
                "velocity": 1.09,
                "goals": [[1.47, -0.13], [-7.85, -0.79], [-2.99, 0.29], [-6.03, -4.44]],
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
                "init_x": -5.18,
                "init_y": 4.26,
                "init_a": 28.39,
                "velocity": 1.02,
                "goals": [[-6.21, 0.04], [-0.22, 0.96], [7.89, 5.74], [0.06, 5.12]],
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
                "init_x": 2.33,
                "init_y": -2.57,
                "init_a": -113.53,
                "velocity": 1.09,
                "goals": [[7.47, -0.66], [0.76, 1.03], [-7.82, 1.31], [-5.93, 1.28]],
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
                "goals": [[-5.99, -4.82], [-4.17, -5.71], [3.74, -4.81], [0.13, -3.22]],
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

def tests_adult_0_child_100_test_case_56_walking_low(tester):
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
                "goals": [[-4.69, 0.44], [6.68, -5.02], [-5.43, -4.24], [1.17, 1.93]],
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
                "init_x": -6.10,
                "init_y": 0.35,
                "init_a": -140.34,
                "velocity": 1.20,
                "goals": [[-4.69, 0.44], [1.95, 3.49], [1.81, -3.23], [7.13, -1.91]],
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
                "init_x": 5.17,
                "init_y": -1.24,
                "init_a": -143.67,
                "velocity": 0.83,
                "goals": [[2.94, -2.33], [-2.94, 0.21], [1.30, 3.52], [5.11, -2.46]],
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
                "init_x": 4.23,
                "init_y": -1.59,
                "init_a": -143.67,
                "velocity": 0.83,
                "goals": [[2.94, -2.33], [-7.91, 1.82], [-4.67, 0.87], [7.68, 3.49]],
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
                "init_x": 4.53,
                "init_y": 5.45,
                "init_a": 7.24,
                "velocity": 1.19,
                "goals": [[-4.13, -0.44], [3.80, -5.06], [-7.58, -4.14], [-3.71, -0.19]],
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
                "init_x": -3.66,
                "init_y": -4.57,
                "init_a": 106.14,
                "velocity": 1.08,
                "goals": [[0.23, -2.98], [4.08, -2.89], [3.99, -4.25], [-3.08, -3.76]],
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
                "goals": [[-6.44, -1.66], [-3.52, 2.89], [-7.81, 0.71], [-1.58, -2.68]],
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
                "init_x": 4.21,
                "init_y": 5.34,
                "init_a": -49.97,
                "velocity": 0.81,
                "goals": [[-1.52, 1.95], [2.38, -4.92], [-1.94, -3.15], [-5.23, 5.21]],
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

def tests_adult_0_child_100_test_case_57_stopped_low(tester):
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
                "goals": [[5.13, 2.94], [-0.42, -4.55], [-7.11, -4.40], [-1.96, -5.32]],
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
                "init_x": 5.47,
                "init_y": 2.00,
                "init_a": -121.11,
                "velocity": 0.97,
                "goals": [[5.13, 2.94], [3.14, 5.06], [-6.47, 4.18], [4.18, 5.63]],
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
                "init_x": -1.51,
                "init_y": -1.09,
                "init_a": 156.54,
                "velocity": 1.15,
                "goals": [[-1.51, -1.09], [-0.28, 3.29], [-1.96, -3.78], [-5.64, 2.01]],
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
                "init_x": -0.78,
                "init_y": -0.41,
                "init_a": 156.54,
                "velocity": 1.15,
                "goals": [[-1.51, -1.09], [1.96, 0.28], [-5.16, 1.65], [-1.95, 2.43]],
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
                "init_x": -2.62,
                "init_y": 2.11,
                "init_a": 48.72,
                "velocity": 0.94,
                "goals": [[0.75, -3.59], [-7.37, -4.53], [1.32, 3.66], [4.77, 5.07]],
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
                "goals": [[-1.12, 0.37], [-5.07, -4.10], [-7.65, 3.88], [1.07, 4.62]],
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
                "init_x": 6.44,
                "init_y": -0.88,
                "init_a": -107.32,
                "velocity": 0.97,
                "goals": [[-3.48, 5.36], [-0.30, 3.65], [-7.01, -4.78], [7.47, 3.15]],
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
                "init_x": 7.65,
                "init_y": -0.54,
                "init_a": -62.86,
                "velocity": 0.91,
                "goals": [[4.32, -0.49], [-4.28, 4.91], [-0.78, 1.85], [-0.10, -4.80]],
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
                "init_x": -3.79,
                "init_y": -0.57,
                "init_a": -173.83,
                "velocity": 0.89,
                "goals": [[6.57, -1.98], [-2.08, -3.66], [7.43, 4.19], [-3.66, -1.80]],
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

def tests_adult_0_child_100_test_case_58_walking_low(tester):
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
                "goals": [[7.59, -4.42], [-1.98, -0.65], [4.55, -0.61], [3.72, 0.57]],
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
                "init_x": -3.46,
                "init_y": -1.26,
                "init_a": -34.91,
                "velocity": 1.14,
                "goals": [[7.59, -4.42], [1.38, -1.56], [4.67, 0.68], [-7.15, 2.33]],
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
                "goals": [[-3.20, -2.39], [-1.40, -2.14], [7.35, 5.95], [-3.03, 0.07]],
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
                "init_x": 1.75,
                "init_y": -3.77,
                "init_a": 163.02,
                "velocity": 0.82,
                "goals": [[-3.20, -2.39], [-3.26, 0.64], [3.94, 4.39], [4.60, -6.00]],
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
                "init_x": -3.81,
                "init_y": -1.27,
                "init_a": 168.39,
                "velocity": 0.91,
                "goals": [[4.59, 5.70], [-2.39, -2.56], [0.44, -0.07], [-5.27, 3.78]],
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
                "init_x": 0.76,
                "init_y": 3.08,
                "init_a": 0.03,
                "velocity": 0.82,
                "goals": [[2.48, -3.77], [-6.30, 2.42], [6.21, 2.50], [4.09, -0.12]],
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
                "init_x": -7.48,
                "init_y": -4.83,
                "init_a": -0.69,
                "velocity": 1.17,
                "goals": [[7.10, 3.83], [-0.05, -5.08], [7.35, -5.88], [7.70, 2.30]],
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
                "init_x": 5.03,
                "init_y": -5.65,
                "init_a": -178.37,
                "velocity": 0.83,
                "goals": [[3.40, 5.09], [-1.44, -0.80], [1.53, 3.29], [-1.92, -1.47]],
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

def tests_adult_0_child_100_test_case_59_walking_low(tester):
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
                "init_x": -0.57,
                "init_y": 1.57,
                "init_a": -32.70,
                "velocity": 1.10,
                "goals": [[-1.20, 1.27], [-1.61, 5.43], [0.99, -4.12], [5.94, -5.21]],
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
                "init_x": 0.28,
                "init_y": 1.05,
                "init_a": -32.70,
                "velocity": 1.10,
                "goals": [[-1.20, 1.27], [2.94, 2.17], [-7.63, 1.20], [3.59, -5.91]],
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
                "goals": [[4.86, -0.21], [2.39, 3.40], [5.29, 3.42], [-4.29, -3.61]],
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
                "goals": [[4.86, -0.21], [1.21, 1.07], [-3.39, 3.85], [-1.80, -0.64]],
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
                "goals": [[-1.86, -2.58], [-2.33, 3.77], [4.11, -1.53], [7.74, -1.14]],
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
                "init_x": 7.71,
                "init_y": 3.33,
                "init_a": 157.25,
                "velocity": 0.82,
                "goals": [[-6.72, 5.69], [-0.46, -4.84], [-4.47, 5.32], [-6.25, 5.73]],
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
                "init_x": 0.92,
                "init_y": 2.55,
                "init_a": 173.98,
                "velocity": 1.09,
                "goals": [[0.63, 1.86], [1.04, 2.97], [-6.29, 5.19], [-4.49, -5.82]],
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
                "init_x": 1.27,
                "init_y": 5.45,
                "init_a": 79.35,
                "velocity": 0.81,
                "goals": [[2.70, 5.85], [-6.52, 3.50], [0.99, -1.04], [7.58, 0.47]],
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

def tests_adult_0_child_100_test_case_60_walking_low(tester):
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
                "goals": [[4.38, 5.93], [3.75, 5.11], [-2.91, 1.91], [6.41, 4.46]],
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
                "init_x": 0.28,
                "init_y": 2.23,
                "init_a": 54.76,
                "velocity": 1.11,
                "goals": [[4.38, 5.93], [-7.06, 3.25], [7.67, -4.81], [6.78, 3.94]],
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
                "init_x": -7.61,
                "init_y": 5.59,
                "init_a": 47.70,
                "velocity": 1.07,
                "goals": [[-3.66, -3.74], [7.17, 0.03], [-3.79, 5.05], [6.56, -4.84]],
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
                "init_x": -8.60,
                "init_y": 5.71,
                "init_a": 47.70,
                "velocity": 1.07,
                "goals": [[-3.66, -3.74], [-3.03, -0.26], [2.43, 1.11], [-4.36, -4.06]],
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
                "goals": [[-5.57, -4.46], [-2.53, 0.09], [-2.72, 2.84], [2.15, -2.80]],
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
                "goals": [[6.53, 0.30], [6.34, -5.09], [-6.07, 3.32], [3.18, -5.80]],
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
                "goals": [[3.93, -4.43], [2.09, -5.20], [-0.90, 5.52], [6.10, 3.54]],
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
                "init_x": 7.73,
                "init_y": -1.18,
                "init_a": -77.11,
                "velocity": 0.82,
                "goals": [[5.08, -1.60], [-1.23, -2.16], [3.17, 0.26], [3.82, 1.31]],
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

def tests_adult_0_child_100_test_case_61_stopped_low(tester):
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
                "goals": [[0.99, 2.09], [-6.75, -4.95], [-2.31, 0.30], [2.00, 1.62]],
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
                "init_x": 1.99,
                "init_y": 2.04,
                "init_a": -141.43,
                "velocity": 0.95,
                "goals": [[0.99, 2.09], [7.36, 1.17], [-0.61, 0.86], [-0.82, -5.72]],
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
                "init_x": 1.07,
                "init_y": -3.92,
                "init_a": -110.00,
                "velocity": 0.90,
                "goals": [[1.07, -3.92], [5.68, -2.10], [2.72, -2.22], [2.53, -1.89]],
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
                "goals": [[1.07, -3.92], [7.82, 2.50], [-1.99, 0.41], [-6.06, 0.55]],
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
                "goals": [[5.56, 3.38], [-4.33, -4.40], [-0.54, -3.90], [-4.63, -0.26]],
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
                "init_x": -1.56,
                "init_y": -5.40,
                "init_a": -170.98,
                "velocity": 1.11,
                "goals": [[4.71, -2.50], [2.39, -3.19], [7.59, 3.91], [5.82, 3.67]],
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
                "init_x": -7.50,
                "init_y": 2.54,
                "init_a": -89.36,
                "velocity": 1.13,
                "goals": [[-6.74, -0.84], [-5.09, -5.67], [-2.12, 0.62], [-5.02, 1.12]],
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

def tests_adult_0_child_100_test_case_62_stopped_low(tester):
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
                "goals": [[5.98, 4.30], [-0.16, -3.92], [4.30, -3.32], [5.54, -0.93]],
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
                "goals": [[5.98, 4.30], [0.99, -1.40], [-4.45, -3.43], [-2.72, -2.46]],
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
                "goals": [[-1.33, 1.09], [3.88, -3.14], [-3.71, -0.51], [2.69, -4.02]],
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
                "goals": [[-1.33, 1.09], [6.23, 5.83], [-1.18, 4.02], [1.66, 1.07]],
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
                "goals": [[-6.09, -0.56], [-5.18, 5.79], [-4.08, -3.19], [-4.46, -3.87]],
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
                "init_x": 4.42,
                "init_y": -2.28,
                "init_a": -147.87,
                "velocity": 0.97,
                "goals": [[1.69, -0.46], [-1.28, 0.36], [-5.25, -5.81], [-4.56, 1.88]],
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
                "init_x": -4.90,
                "init_y": -3.48,
                "init_a": -67.97,
                "velocity": 1.17,
                "goals": [[-2.12, 1.55], [6.55, -4.08], [5.36, 1.73], [2.49, -5.84]],
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
                "init_x": -2.25,
                "init_y": -1.12,
                "init_a": 14.93,
                "velocity": 1.08,
                "goals": [[3.27, 4.54], [-1.62, 4.29], [5.30, -4.79], [4.85, -4.35]],
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
                "init_x": 7.18,
                "init_y": 1.91,
                "init_a": -56.20,
                "velocity": 1.01,
                "goals": [[-3.15, 1.62], [-1.49, 3.01], [-5.00, -0.11], [1.25, -0.11]],
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

def tests_adult_0_child_100_test_case_63_walking_low(tester):
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
                "goals": [[5.86, -3.97], [3.31, 3.81], [2.05, 1.96], [3.64, 4.67]],
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
                "init_x": -2.25,
                "init_y": 2.82,
                "init_a": -148.32,
                "velocity": 1.08,
                "goals": [[5.86, -3.97], [-6.07, 0.69], [5.11, -5.55], [7.07, -2.43]],
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
                "init_x": -5.97,
                "init_y": -5.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goals": [[2.18, -1.61], [-4.26, 3.37], [-0.48, 5.92], [-4.45, -2.99]],
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
                "init_x": -6.03,
                "init_y": -6.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goals": [[2.18, -1.61], [-5.60, -3.92], [5.38, -2.85], [6.53, -1.49]],
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
                "init_x": 2.53,
                "init_y": 3.99,
                "init_a": -28.58,
                "velocity": 1.19,
                "goals": [[4.54, 4.23], [-6.76, 0.55], [-2.68, -4.27], [3.62, -1.43]],
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
                "init_x": -4.39,
                "init_y": 2.71,
                "init_a": -149.94,
                "velocity": 1.03,
                "goals": [[5.92, -2.47], [-3.54, -3.97], [7.70, -0.29], [-6.54, -1.42]],
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
                "init_x": -7.36,
                "init_y": 5.09,
                "init_a": -122.65,
                "velocity": 0.95,
                "goals": [[7.50, 1.97], [0.55, -3.50], [4.21, 2.38], [-6.94, 4.20]],
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

def tests_adult_0_child_100_test_case_64_stopped_low(tester):
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
                "goals": [[0.95, 2.92], [5.81, 0.34], [1.53, -3.11], [7.33, -5.48]],
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
                "init_x": 0.26,
                "init_y": 3.64,
                "init_a": -152.55,
                "velocity": 0.94,
                "goals": [[0.95, 2.92], [3.06, -0.58], [-5.45, -2.44], [-4.61, -3.32]],
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
                "init_x": 5.94,
                "init_y": 0.06,
                "init_a": -62.76,
                "velocity": 0.94,
                "goals": [[5.94, 0.06], [-2.98, 5.82], [-3.28, 5.87], [3.21, 2.49]],
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
                "init_x": 6.91,
                "init_y": 0.29,
                "init_a": -62.76,
                "velocity": 0.94,
                "goals": [[5.94, 0.06], [-5.17, -0.51], [0.85, -2.02], [-6.22, 0.86]],
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
                "init_x": -4.08,
                "init_y": -2.07,
                "init_a": 161.11,
                "velocity": 0.88,
                "goals": [[4.02, 3.59], [3.35, -1.67], [-5.06, -1.10], [7.82, 5.19]],
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
                "init_x": -7.46,
                "init_y": 4.27,
                "init_a": -21.70,
                "velocity": 1.18,
                "goals": [[2.94, 5.50], [-4.70, -2.18], [4.61, 0.15], [-4.78, 1.39]],
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
                "init_x": 2.12,
                "init_y": -5.78,
                "init_a": -142.86,
                "velocity": 1.10,
                "goals": [[-2.92, -2.55], [-3.27, 0.02], [3.64, -2.13], [-1.43, -5.73]],
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
                "init_x": -3.75,
                "init_y": 0.55,
                "init_a": -109.54,
                "velocity": 1.03,
                "goals": [[-1.26, -1.95], [-4.08, 0.51], [-6.60, -5.95], [-6.26, 0.60]],
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

def tests_adult_0_child_100_test_case_65_stopped_low(tester):
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
                "goals": [[3.61, -4.95], [-0.40, 2.29], [2.49, 5.53], [-2.47, -0.14]],
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
                "init_x": 2.64,
                "init_y": -5.16,
                "init_a": -6.29,
                "velocity": 0.99,
                "goals": [[3.61, -4.95], [2.85, -4.25], [-4.72, 3.86], [-6.15, -2.84]],
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
                "init_x": -0.85,
                "init_y": -1.73,
                "init_a": -0.49,
                "velocity": 0.82,
                "goals": [[-0.85, -1.73], [0.34, 4.20], [-5.86, -0.19], [6.70, 1.75]],
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
                "init_x": -1.32,
                "init_y": -2.61,
                "init_a": -0.49,
                "velocity": 0.82,
                "goals": [[-0.85, -1.73], [5.21, -0.95], [-1.19, -1.98], [4.65, 4.94]],
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
                "init_x": 5.80,
                "init_y": 2.56,
                "init_a": -26.85,
                "velocity": 1.05,
                "goals": [[-1.73, -5.06], [-5.57, -0.59], [0.64, 3.10], [-3.04, -4.87]],
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
                "init_x": 3.53,
                "init_y": -4.42,
                "init_a": 131.50,
                "velocity": 1.16,
                "goals": [[-5.68, -0.30], [-5.10, 3.39], [4.58, 4.92], [-3.73, -4.50]],
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
                "init_x": 0.62,
                "init_y": 2.74,
                "init_a": 44.82,
                "velocity": 1.10,
                "goals": [[0.76, 5.05], [-2.26, 5.34], [-3.80, 2.20], [1.65, 3.39]],
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
                "init_x": -3.62,
                "init_y": -5.83,
                "init_a": -68.59,
                "velocity": 0.87,
                "goals": [[7.60, -5.23], [-3.65, -1.13], [-6.26, 1.31], [-3.39, -0.43]],
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
                "init_x": -1.89,
                "init_y": -0.85,
                "init_a": 37.13,
                "velocity": 1.05,
                "goals": [[-6.71, 3.74], [-6.82, -4.10], [6.29, 4.51], [0.07, 2.80]],
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

def tests_adult_0_child_100_test_case_66_walking_low(tester):
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
                "goals": [[6.32, 2.26], [2.02, -5.92], [2.24, 2.20], [0.06, 0.63]],
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
                "init_x": 2.21,
                "init_y": -5.72,
                "init_a": -57.89,
                "velocity": 0.85,
                "goals": [[6.32, 2.26], [4.88, -3.01], [3.22, -4.35], [0.06, -0.74]],
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
                "init_x": -7.72,
                "init_y": -5.03,
                "init_a": -61.58,
                "velocity": 0.95,
                "goals": [[-3.30, -0.14], [-5.54, -3.90], [-0.81, -0.94], [-1.83, 2.68]],
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
                "init_x": -6.97,
                "init_y": -5.70,
                "init_a": -61.58,
                "velocity": 0.95,
                "goals": [[-3.30, -0.14], [-0.81, -2.29], [-2.79, -5.88], [1.99, 0.93]],
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
                "goals": [[5.15, 3.97], [-4.52, -4.80], [-6.43, -3.38], [-4.35, -0.04]],
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
                "init_x": -4.72,
                "init_y": -1.73,
                "init_a": -102.43,
                "velocity": 1.06,
                "goals": [[-5.82, -2.46], [7.60, -3.09], [-4.24, 1.68], [6.23, 1.95]],
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
                "init_x": 1.03,
                "init_y": 4.61,
                "init_a": 77.00,
                "velocity": 1.00,
                "goals": [[-6.64, -3.90], [-3.61, 5.38], [-0.26, -2.99], [-5.63, 1.72]],
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
                "init_x": 5.84,
                "init_y": 5.80,
                "init_a": -158.78,
                "velocity": 1.18,
                "goals": [[-2.54, -0.78], [-0.26, -1.36], [-2.60, 3.56], [-0.90, -1.62]],
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

def tests_adult_0_child_100_test_case_67_stopped_low(tester):
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
                "init_x": 3.58,
                "init_y": -5.58,
                "init_a": 151.44,
                "velocity": 1.15,
                "goals": [[3.58, -5.58], [-0.33, 3.65], [-1.80, -2.39], [-3.41, 2.14]],
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
                "init_x": 3.40,
                "init_y": -4.60,
                "init_a": 151.44,
                "velocity": 1.15,
                "goals": [[3.58, -5.58], [5.27, -0.92], [2.37, -0.11], [2.54, 3.82]],
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
                "init_x": 6.06,
                "init_y": -4.08,
                "init_a": -98.20,
                "velocity": 1.20,
                "goals": [[6.06, -4.08], [-7.32, 2.81], [-1.48, 4.68], [-4.49, 0.42]],
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
                "init_x": 6.67,
                "init_y": -4.87,
                "init_a": -98.20,
                "velocity": 1.20,
                "goals": [[6.06, -4.08], [-7.38, 0.44], [-3.10, -0.26], [-2.58, 3.52]],
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
                "init_x": -2.14,
                "init_y": -0.24,
                "init_a": 16.58,
                "velocity": 1.18,
                "goals": [[-3.04, -2.77], [-0.83, 0.06], [6.10, 0.99], [1.06, -2.80]],
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
                "init_x": 6.91,
                "init_y": -0.27,
                "init_a": 165.91,
                "velocity": 1.16,
                "goals": [[-3.49, 5.11], [-4.76, -3.00], [3.72, 0.60], [-7.18, 2.25]],
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
                "init_x": 2.94,
                "init_y": 2.24,
                "init_a": -9.44,
                "velocity": 0.88,
                "goals": [[-4.85, 2.66], [6.15, -5.96], [4.37, 0.10], [-0.96, 1.72]],
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

def tests_adult_0_child_100_test_case_68_stopped_low(tester):
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
                "init_x": -1.49,
                "init_y": -0.10,
                "init_a": -39.32,
                "velocity": 1.01,
                "goals": [[-1.49, -0.10], [-7.64, -1.74], [-4.75, -0.07], [-6.58, 0.30]],
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
                "init_x": -0.79,
                "init_y": -0.82,
                "init_a": -39.32,
                "velocity": 1.01,
                "goals": [[-1.49, -0.10], [2.94, 1.32], [4.38, -1.22], [5.72, -2.73]],
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
                "init_x": -4.26,
                "init_y": 2.08,
                "init_a": -104.95,
                "velocity": 0.82,
                "goals": [[-4.26, 2.08], [0.59, -4.54], [7.02, -2.48], [0.82, -4.85]],
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
                "goals": [[-4.26, 2.08], [-2.90, 3.13], [5.09, -0.41], [5.29, 1.85]],
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
                "init_x": 3.91,
                "init_y": -2.99,
                "init_a": -64.56,
                "velocity": 0.99,
                "goals": [[7.74, -1.73], [5.44, 0.21], [7.36, -1.40], [1.66, -1.94]],
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
                "init_x": -6.31,
                "init_y": -5.70,
                "init_a": 121.34,
                "velocity": 1.11,
                "goals": [[2.91, 4.38], [4.30, -1.90], [-7.91, -2.23], [0.77, -5.82]],
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
                "init_x": -4.17,
                "init_y": 4.17,
                "init_a": 4.82,
                "velocity": 1.16,
                "goals": [[-2.58, 4.79], [-7.95, -1.02], [-5.53, -4.73], [5.80, -4.98]],
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
                "init_x": -1.55,
                "init_y": -1.39,
                "init_a": -159.26,
                "velocity": 1.05,
                "goals": [[-5.41, 2.67], [4.39, 4.84], [1.13, -3.28], [-2.70, -5.03]],
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

def tests_adult_0_child_100_test_case_69_stopped_low(tester):
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
                "goals": [[4.76, -5.57], [-2.30, -5.47], [1.96, 5.69], [-4.86, -3.57]],
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
                "init_x": 3.86,
                "init_y": -5.99,
                "init_a": -169.40,
                "velocity": 0.82,
                "goals": [[4.76, -5.57], [0.75, 4.58], [-3.06, -3.28], [-1.35, -2.34]],
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
                "init_x": -3.47,
                "init_y": 5.59,
                "init_a": 19.05,
                "velocity": 0.91,
                "goals": [[-3.47, 5.59], [2.71, -2.61], [-3.10, 3.97], [4.89, 0.07]],
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
                "init_x": -2.53,
                "init_y": 5.93,
                "init_a": 19.05,
                "velocity": 0.91,
                "goals": [[-3.47, 5.59], [5.06, 3.29], [7.76, -2.27], [-3.94, 5.03]],
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
                "init_x": -6.94,
                "init_y": 4.92,
                "init_a": 29.78,
                "velocity": 0.87,
                "goals": [[-0.14, -4.01], [6.76, 4.85], [-7.64, 4.96], [6.59, -4.64]],
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
                "init_x": 2.35,
                "init_y": -2.53,
                "init_a": 154.49,
                "velocity": 0.96,
                "goals": [[3.00, -2.26], [-3.96, -5.79], [5.81, 3.47], [-1.99, 4.70]],
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
                "init_x": -4.31,
                "init_y": 1.90,
                "init_a": -11.53,
                "velocity": 1.20,
                "goals": [[-7.32, -3.19], [4.26, -4.19], [-6.50, -3.72], [4.77, 1.27]],
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

def tests_adult_0_child_100_test_case_70_walking_low(tester):
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
                "init_x": -7.95,
                "init_y": 1.51,
                "init_a": -118.43,
                "velocity": 1.01,
                "goals": [[-1.43, 4.61], [3.56, -1.03], [-5.53, -4.33], [-3.13, 0.75]],
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
                "init_x": -7.03,
                "init_y": 1.90,
                "init_a": -118.43,
                "velocity": 1.01,
                "goals": [[-1.43, 4.61], [-4.31, -5.44], [0.25, 0.95], [-6.99, -5.08]],
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
                "goals": [[6.39, 4.01], [-0.43, -0.04], [-0.56, 3.35], [-0.38, 1.06]],
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
                "init_x": 1.86,
                "init_y": -5.12,
                "init_a": -118.41,
                "velocity": 1.03,
                "goals": [[6.39, 4.01], [-6.02, -1.01], [1.95, 5.50], [7.79, 5.98]],
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
                "init_x": -0.40,
                "init_y": -3.58,
                "init_a": -6.10,
                "velocity": 1.07,
                "goals": [[-2.32, 2.16], [-5.60, -1.09], [0.16, -5.98], [-4.04, -3.54]],
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
                "init_x": 5.67,
                "init_y": -1.32,
                "init_a": -96.86,
                "velocity": 1.13,
                "goals": [[4.80, 1.90], [-6.44, 2.38], [4.92, 4.83], [3.62, -2.71]],
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
                "init_x": 1.54,
                "init_y": 3.23,
                "init_a": -129.37,
                "velocity": 1.11,
                "goals": [[2.36, -4.09], [0.53, 0.35], [4.42, 0.37], [5.16, -0.42]],
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
                "goals": [[3.01, -1.64], [1.89, 0.78], [-1.39, 1.20], [5.61, -1.73]],
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
                "init_x": 3.45,
                "init_y": -0.87,
                "init_a": -140.80,
                "velocity": 0.83,
                "goals": [[1.40, 1.41], [4.44, -0.20], [-5.48, 3.49], [-5.18, -5.71]],
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

def tests_adult_0_child_100_test_case_71_stopped_low(tester):
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
                "goals": [[-6.42, -2.08], [-5.08, -5.91], [3.07, 2.52], [3.58, -3.96]],
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
                "init_x": -6.56,
                "init_y": -1.09,
                "init_a": 113.86,
                "velocity": 1.15,
                "goals": [[-6.42, -2.08], [5.21, 2.90], [-0.75, -3.69], [-1.93, 3.85]],
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
                "init_x": 5.10,
                "init_y": 3.40,
                "init_a": -32.45,
                "velocity": 0.85,
                "goals": [[5.10, 3.40], [1.83, -5.14], [2.31, -0.04], [-4.90, 5.85]],
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
                "init_x": 5.51,
                "init_y": 4.31,
                "init_a": -32.45,
                "velocity": 0.85,
                "goals": [[5.10, 3.40], [7.79, -5.95], [-6.29, 0.84], [-5.73, 1.36]],
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
                "init_x": -1.34,
                "init_y": -3.02,
                "init_a": 143.97,
                "velocity": 0.90,
                "goals": [[5.00, 4.08], [6.23, 5.29], [6.18, 4.31], [2.05, 2.00]],
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
                "init_x": 4.91,
                "init_y": 3.55,
                "init_a": 106.90,
                "velocity": 0.97,
                "goals": [[-1.46, -5.78], [-7.53, 2.87], [-1.33, 0.35], [3.26, 4.56]],
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
                "init_x": 2.89,
                "init_y": 2.74,
                "init_a": -141.03,
                "velocity": 0.86,
                "goals": [[4.39, 1.89], [-3.97, 3.46], [4.28, 0.05], [-1.74, 1.19]],
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
                "init_x": -5.19,
                "init_y": 3.11,
                "init_a": 10.38,
                "velocity": 1.14,
                "goals": [[-5.77, -4.61], [-3.53, -2.26], [7.66, 2.62], [2.07, 3.81]],
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
                "init_x": -4.61,
                "init_y": -2.80,
                "init_a": 109.36,
                "velocity": 1.04,
                "goals": [[3.58, -3.18], [3.23, 0.84], [5.43, 2.91], [-0.40, -3.05]],
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

def tests_adult_0_child_100_test_case_72_walking_low(tester):
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
                "goals": [[-7.07, -5.05], [3.08, 1.93], [-1.50, 4.96], [1.15, -2.05]],
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
                "init_x": 6.52,
                "init_y": -0.71,
                "init_a": -148.95,
                "velocity": 1.15,
                "goals": [[-7.07, -5.05], [5.73, 0.91], [0.71, -2.85], [2.46, -4.88]],
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
                "init_x": -7.19,
                "init_y": -2.77,
                "init_a": 93.35,
                "velocity": 0.82,
                "goals": [[1.89, -3.33], [4.59, -3.99], [-5.56, -1.39], [1.30, -2.01]],
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
                "init_x": -6.79,
                "init_y": -1.85,
                "init_a": 93.35,
                "velocity": 0.82,
                "goals": [[1.89, -3.33], [-5.56, 2.93], [-0.05, -1.85], [-1.94, 1.73]],
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
                "init_x": 1.73,
                "init_y": 1.94,
                "init_a": 138.40,
                "velocity": 0.87,
                "goals": [[5.54, -2.21], [6.18, 5.86], [-1.81, -5.22], [-6.95, 2.35]],
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
                "init_x": -1.08,
                "init_y": -2.17,
                "init_a": -34.84,
                "velocity": 1.12,
                "goals": [[2.74, 5.39], [6.46, -2.33], [4.72, 1.50], [4.15, 2.46]],
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
                "init_x": -3.56,
                "init_y": 3.81,
                "init_a": -153.11,
                "velocity": 0.91,
                "goals": [[-1.63, -2.04], [6.38, 2.02], [7.39, 0.38], [-7.33, 0.72]],
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
                "init_x": -2.76,
                "init_y": 2.48,
                "init_a": -108.38,
                "velocity": 0.85,
                "goals": [[7.02, 1.04], [4.18, 1.42], [-1.66, 5.40], [6.51, 0.32]],
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
                "init_x": -5.74,
                "init_y": 4.17,
                "init_a": 160.26,
                "velocity": 1.18,
                "goals": [[-4.82, 1.87], [4.14, -5.92], [0.40, 3.01], [-6.85, 4.77]],
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

def tests_adult_0_child_100_test_case_73_walking_low(tester):
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
                "init_x": -2.40,
                "init_y": -1.35,
                "init_a": -138.70,
                "velocity": 0.97,
                "goals": [[-2.56, -1.93], [2.53, -3.10], [2.35, -1.39], [2.96, -3.84]],
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
                "init_x": -2.52,
                "init_y": -2.34,
                "init_a": -138.70,
                "velocity": 0.97,
                "goals": [[-2.56, -1.93], [2.03, -5.69], [5.94, 0.94], [3.12, 5.08]],
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
                "init_x": -4.23,
                "init_y": -5.25,
                "init_a": 15.15,
                "velocity": 1.16,
                "goals": [[-7.16, 0.16], [-1.54, 3.92], [5.53, 4.31], [-0.50, 2.01]],
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
                "goals": [[-7.16, 0.16], [7.15, 3.65], [2.91, 1.83], [-7.29, 5.11]],
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
                "goals": [[1.08, 3.88], [-5.05, -1.81], [-6.65, -1.60], [7.63, -1.97]],
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
                "goals": [[-6.72, -4.09], [4.75, 3.95], [-4.72, -0.62], [-0.82, -2.63]],
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
                "init_x": -2.13,
                "init_y": -2.73,
                "init_a": -125.38,
                "velocity": 1.08,
                "goals": [[6.91, 4.11], [-7.22, -3.42], [-6.73, -2.59], [1.95, 4.38]],
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

def tests_adult_0_child_100_test_case_74_walking_low(tester):
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
                "init_x": 7.02,
                "init_y": 3.42,
                "init_a": 112.30,
                "velocity": 0.90,
                "goals": [[-6.21, 2.48], [3.76, -2.54], [5.27, -5.08], [-0.28, -2.97]],
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
                "init_x": 7.08,
                "init_y": 4.42,
                "init_a": 112.30,
                "velocity": 0.90,
                "goals": [[-6.21, 2.48], [4.46, 3.85], [7.08, 0.18], [5.39, -2.77]],
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
                "init_x": 0.70,
                "init_y": 5.00,
                "init_a": 59.38,
                "velocity": 0.83,
                "goals": [[2.09, 3.91], [5.63, -1.58], [-4.67, -3.38], [-6.27, -0.84]],
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
                "init_x": 1.03,
                "init_y": 4.06,
                "init_a": 59.38,
                "velocity": 0.83,
                "goals": [[2.09, 3.91], [-7.75, -2.08], [-0.47, -2.04], [-0.15, 4.58]],
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
                "goals": [[7.20, -2.26], [4.18, -4.18], [0.62, -5.10], [3.71, -5.85]],
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
                "init_x": -0.94,
                "init_y": 4.46,
                "init_a": -80.30,
                "velocity": 0.92,
                "goals": [[7.66, -1.08], [7.49, 0.98], [4.86, -2.21], [-7.51, 2.12]],
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
                "init_x": 6.72,
                "init_y": 1.94,
                "init_a": 73.06,
                "velocity": 0.81,
                "goals": [[1.90, 2.82], [-6.15, -4.46], [-1.93, -4.51], [3.47, 4.51]],
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
                "init_x": -6.84,
                "init_y": 2.69,
                "init_a": -165.69,
                "velocity": 0.89,
                "goals": [[4.55, 4.44], [-1.64, 3.88], [-7.59, -6.00], [7.36, 3.24]],
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

def tests_adult_0_child_100_test_case_75_stopped_low(tester):
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
                "init_x": 1.56,
                "init_y": -2.16,
                "init_a": 168.66,
                "velocity": 0.84,
                "goals": [[1.56, -2.16], [-3.37, 4.06], [1.41, 5.82], [-7.03, -2.02]],
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
                "init_x": 0.87,
                "init_y": -1.43,
                "init_a": 168.66,
                "velocity": 0.84,
                "goals": [[1.56, -2.16], [-0.17, -2.94], [7.06, -1.55], [-6.09, -5.31]],
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
                "init_x": -5.66,
                "init_y": 1.49,
                "init_a": 20.31,
                "velocity": 1.08,
                "goals": [[-5.66, 1.49], [-1.35, 5.57], [6.11, -5.74], [6.46, -2.25]],
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
                "init_x": -6.20,
                "init_y": 0.64,
                "init_a": 20.31,
                "velocity": 1.08,
                "goals": [[-5.66, 1.49], [2.33, 1.98], [7.08, 5.92], [-3.80, 1.40]],
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
                "init_x": -6.05,
                "init_y": 2.62,
                "init_a": 63.46,
                "velocity": 0.88,
                "goals": [[-4.47, -5.62], [5.21, 4.64], [-6.49, -3.81], [-3.15, 1.96]],
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
                "init_x": 4.41,
                "init_y": 3.26,
                "init_a": -171.37,
                "velocity": 1.14,
                "goals": [[-6.55, 0.80], [-0.65, 3.77], [1.33, 4.95], [-2.54, 5.03]],
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
                "init_x": -6.50,
                "init_y": -1.97,
                "init_a": -13.49,
                "velocity": 1.07,
                "goals": [[3.56, -3.20], [1.10, 0.36], [-3.56, 1.20], [3.17, -3.68]],
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
                "init_x": -5.24,
                "init_y": 2.69,
                "init_a": 113.86,
                "velocity": 0.84,
                "goals": [[3.58, 4.22], [6.02, 1.15], [1.35, -3.25], [0.71, -2.35]],
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
                "init_x": 4.29,
                "init_y": -4.80,
                "init_a": -22.85,
                "velocity": 0.91,
                "goals": [[0.64, 1.01], [6.13, -2.09], [-1.12, -0.85], [-0.93, -6.00]],
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

def tests_adult_0_child_100_test_case_76_stopped_low(tester):
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
                "init_x": 5.08,
                "init_y": -3.31,
                "init_a": -107.43,
                "velocity": 1.03,
                "goals": [[5.08, -3.31], [-6.19, 2.28], [7.46, 0.98], [5.90, -4.98]],
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
                "init_x": 4.78,
                "init_y": -2.36,
                "init_a": -107.43,
                "velocity": 1.03,
                "goals": [[5.08, -3.31], [4.85, -2.21], [-7.45, -3.71], [3.66, 0.34]],
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
                "init_x": 4.12,
                "init_y": -2.17,
                "init_a": 49.74,
                "velocity": 1.12,
                "goals": [[4.12, -2.17], [6.80, 3.19], [-1.65, -0.04], [-4.90, -1.64]],
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
                "init_x": 4.25,
                "init_y": -3.16,
                "init_a": 49.74,
                "velocity": 1.12,
                "goals": [[4.12, -2.17], [1.93, -2.27], [7.36, 2.56], [1.07, -0.32]],
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
                "init_x": 1.91,
                "init_y": 5.47,
                "init_a": -147.17,
                "velocity": 0.99,
                "goals": [[1.40, -4.32], [-6.04, 0.25], [1.33, 4.30], [3.63, 0.63]],
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
                "init_x": 1.94,
                "init_y": -1.27,
                "init_a": 143.26,
                "velocity": 1.07,
                "goals": [[-0.12, -5.71], [6.04, 3.82], [4.16, -4.45], [6.60, -4.82]],
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
                "init_x": -7.35,
                "init_y": -2.49,
                "init_a": 117.15,
                "velocity": 0.81,
                "goals": [[-6.45, 1.16], [3.06, -2.41], [3.37, 0.51], [-0.48, -3.32]],
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
                "init_x": -5.61,
                "init_y": -2.48,
                "init_a": 70.00,
                "velocity": 1.04,
                "goals": [[-2.79, 5.21], [-6.53, 5.39], [-4.94, 4.30], [-4.84, 5.08]],
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
                "init_x": 1.42,
                "init_y": 1.96,
                "init_a": -85.06,
                "velocity": 1.10,
                "goals": [[-7.96, -4.40], [-1.62, -4.74], [6.52, 4.48], [-1.25, -3.25]],
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

def tests_adult_0_child_100_test_case_77_walking_low(tester):
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
                "init_x": -5.55,
                "init_y": -0.38,
                "init_a": 21.62,
                "velocity": 1.09,
                "goals": [[3.54, 3.83], [4.89, -5.60], [6.09, 4.06], [-2.30, -4.81]],
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
                "init_x": -5.74,
                "init_y": -1.36,
                "init_a": 21.62,
                "velocity": 1.09,
                "goals": [[3.54, 3.83], [3.50, -2.67], [0.41, -4.24], [5.57, -1.72]],
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
                "goals": [[-0.41, -4.71], [-5.11, -0.33], [6.51, -5.00], [6.11, 2.09]],
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
                "goals": [[-0.41, -4.71], [7.20, -3.90], [2.38, 2.62], [-0.84, 3.70]],
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
                "goals": [[-7.83, -2.70], [-7.88, 4.80], [6.92, -0.67], [6.46, 2.99]],
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
                "init_x": 4.64,
                "init_y": -1.38,
                "init_a": 131.93,
                "velocity": 1.09,
                "goals": [[2.86, -3.32], [-3.07, -2.63], [-4.85, -0.48], [5.24, 5.98]],
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
                "init_x": -4.62,
                "init_y": 5.33,
                "init_a": 11.41,
                "velocity": 0.95,
                "goals": [[6.82, 2.03], [-1.41, 2.49], [-2.40, -2.99], [4.71, -2.25]],
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

def tests_adult_0_child_100_test_case_78_stopped_low(tester):
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
                "goals": [[0.22, -1.46], [-6.71, -3.85], [-6.86, -0.73], [5.83, -0.70]],
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
                "init_x": 0.99,
                "init_y": -2.10,
                "init_a": 178.09,
                "velocity": 0.94,
                "goals": [[0.22, -1.46], [5.10, -3.02], [2.18, -3.07], [-1.11, 3.17]],
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
                "init_x": -2.86,
                "init_y": -0.75,
                "init_a": -5.96,
                "velocity": 1.13,
                "goals": [[-2.86, -0.75], [4.84, 2.44], [1.63, -5.55], [7.82, 4.14]],
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
                "init_x": -3.84,
                "init_y": -0.57,
                "init_a": -5.96,
                "velocity": 1.13,
                "goals": [[-2.86, -0.75], [0.08, -1.33], [5.71, 0.56], [2.90, 5.45]],
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
                "init_x": 3.81,
                "init_y": 0.45,
                "init_a": 124.19,
                "velocity": 0.93,
                "goals": [[2.35, 4.29], [2.01, 4.82], [-3.23, -4.94], [-2.83, 0.96]],
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
                "init_x": -1.43,
                "init_y": -4.88,
                "init_a": -71.90,
                "velocity": 0.83,
                "goals": [[5.33, -1.40], [4.51, -5.83], [2.81, 4.76], [2.61, 5.29]],
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
                "init_x": 6.47,
                "init_y": -3.56,
                "init_a": -31.43,
                "velocity": 0.94,
                "goals": [[1.28, 2.97], [2.19, -3.56], [0.95, 5.07], [-6.19, 0.20]],
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

def tests_adult_0_child_100_test_case_79_stopped_low(tester):
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
                "goals": [[-7.54, -5.32], [-2.54, -4.49], [-6.48, 3.97], [2.50, 3.21]],
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
                "init_x": -8.44,
                "init_y": -4.88,
                "init_a": -114.62,
                "velocity": 0.92,
                "goals": [[-7.54, -5.32], [-7.69, 5.59], [-0.86, -0.89], [0.42, 1.46]],
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
                "init_x": 2.87,
                "init_y": -3.35,
                "init_a": 109.27,
                "velocity": 0.89,
                "goals": [[2.87, -3.35], [-2.58, 2.41], [-1.96, 4.65], [7.34, -3.35]],
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
                "init_x": 3.87,
                "init_y": -3.25,
                "init_a": 109.27,
                "velocity": 0.89,
                "goals": [[2.87, -3.35], [2.82, -2.93], [-5.82, -3.95], [3.63, -5.95]],
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
                "goals": [[-1.54, 5.84], [4.70, -1.57], [3.46, 4.69], [6.23, -2.53]],
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
                "init_x": 0.22,
                "init_y": -2.80,
                "init_a": -88.28,
                "velocity": 0.92,
                "goals": [[4.17, -2.22], [-1.41, 3.90], [0.60, 1.73], [0.42, 3.41]],
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
                "init_x": -2.82,
                "init_y": 5.92,
                "init_a": 165.60,
                "velocity": 1.19,
                "goals": [[1.65, -5.65], [3.43, -5.76], [5.02, -5.25], [-7.77, 0.16]],
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

def tests_adult_0_child_100_test_case_80_stopped_low(tester):
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
                "goals": [[-6.45, -3.35], [-1.13, 0.76], [6.09, -2.40], [3.73, -4.30]],
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
                "init_x": -7.11,
                "init_y": -4.10,
                "init_a": -144.16,
                "velocity": 0.82,
                "goals": [[-6.45, -3.35], [3.34, -1.45], [6.33, 1.77], [2.28, 0.51]],
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
                "init_x": 2.43,
                "init_y": 4.67,
                "init_a": -42.80,
                "velocity": 1.11,
                "goals": [[2.43, 4.67], [7.87, 2.43], [0.34, -3.77], [-6.06, 4.32]],
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
                "init_x": 3.28,
                "init_y": 5.20,
                "init_a": -42.80,
                "velocity": 1.11,
                "goals": [[2.43, 4.67], [-7.56, -5.31], [-7.15, -3.24], [3.21, 5.38]],
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
                "init_x": 0.82,
                "init_y": 4.40,
                "init_a": -82.79,
                "velocity": 0.92,
                "goals": [[3.44, 5.40], [6.20, 2.89], [0.62, -0.04], [-5.53, 5.40]],
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
                "init_x": -6.96,
                "init_y": 2.58,
                "init_a": -26.58,
                "velocity": 0.87,
                "goals": [[6.46, 2.05], [6.94, 4.84], [0.83, 2.17], [6.80, 4.64]],
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
                "init_x": 0.70,
                "init_y": 1.83,
                "init_a": -81.47,
                "velocity": 1.15,
                "goals": [[-0.51, -1.76], [1.52, 3.31], [1.66, -3.22], [-6.83, 5.08]],
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
                "init_x": 1.16,
                "init_y": 1.19,
                "init_a": -37.19,
                "velocity": 0.86,
                "goals": [[-5.04, 3.70], [1.59, 5.99], [5.00, -0.21], [0.76, -4.26]],
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
                "init_x": -6.89,
                "init_y": -5.73,
                "init_a": 35.40,
                "velocity": 1.16,
                "goals": [[-3.32, -2.47], [-7.77, 5.01], [0.46, -4.25], [3.19, 5.07]],
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

def tests_adult_0_child_100_test_case_81_stopped_low(tester):
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
                "goals": [[-0.01, 0.38], [-3.31, 5.15], [1.32, -5.46], [0.18, -0.71]],
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
                "init_x": 0.30,
                "init_y": 1.33,
                "init_a": 132.14,
                "velocity": 0.82,
                "goals": [[-0.01, 0.38], [4.84, 4.29], [-0.91, -1.43], [-5.36, -1.25]],
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
                "init_x": 6.90,
                "init_y": -2.01,
                "init_a": 32.91,
                "velocity": 1.03,
                "goals": [[6.90, -2.01], [5.25, -5.69], [-6.50, 3.16], [-4.93, 0.00]],
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
                "init_x": 6.39,
                "init_y": -2.87,
                "init_a": 32.91,
                "velocity": 1.03,
                "goals": [[6.90, -2.01], [-5.97, -2.20], [-3.01, 5.25], [-1.15, -2.66]],
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
                "init_x": 4.94,
                "init_y": -0.32,
                "init_a": 79.63,
                "velocity": 1.15,
                "goals": [[2.01, -4.61], [-4.01, -5.64], [7.51, -5.78], [3.75, 0.24]],
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
                "init_x": -0.23,
                "init_y": -3.67,
                "init_a": -46.76,
                "velocity": 1.10,
                "goals": [[3.27, 5.92], [6.45, 5.49], [-1.82, 2.91], [6.46, 0.80]],
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
                "init_x": -3.00,
                "init_y": -0.40,
                "init_a": -135.14,
                "velocity": 1.03,
                "goals": [[1.92, -4.35], [2.31, 1.37], [-4.36, -0.60], [3.65, -1.37]],
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

def tests_adult_0_child_100_test_case_82_walking_low(tester):
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
                "init_x": -4.39,
                "init_y": 5.50,
                "init_a": 158.47,
                "velocity": 0.95,
                "goals": [[3.52, -4.79], [-1.00, -4.95], [-4.42, 2.68], [-6.52, -1.92]],
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
                "init_x": -5.36,
                "init_y": 5.25,
                "init_a": 158.47,
                "velocity": 0.95,
                "goals": [[3.52, -4.79], [-2.19, 1.10], [-4.27, -5.35], [-3.68, 3.60]],
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
                "init_x": 2.30,
                "init_y": -4.23,
                "init_a": 125.02,
                "velocity": 1.15,
                "goals": [[-6.63, -1.13], [-7.63, -5.19], [0.55, -2.54], [5.57, -6.00]],
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
                "init_x": 2.18,
                "init_y": -3.24,
                "init_a": 125.02,
                "velocity": 1.15,
                "goals": [[-6.63, -1.13], [4.18, -4.41], [-7.93, -0.37], [6.96, -3.83]],
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
                "init_x": -0.53,
                "init_y": 4.79,
                "init_a": 106.27,
                "velocity": 1.19,
                "goals": [[-7.03, -5.84], [-0.86, -5.11], [6.32, 0.39], [-2.45, 3.31]],
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
                "goals": [[5.11, -2.87], [2.93, 2.30], [-5.89, -2.12], [-4.05, 3.45]],
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
                "init_x": -0.52,
                "init_y": 0.27,
                "init_a": -18.49,
                "velocity": 0.96,
                "goals": [[7.43, -5.48], [-5.86, -0.14], [-3.03, -5.68], [3.93, -2.65]],
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
                "init_x": 7.10,
                "init_y": 5.31,
                "init_a": -42.58,
                "velocity": 1.20,
                "goals": [[-2.11, 1.32], [-3.49, -0.64], [6.38, -5.50], [7.10, 0.70]],
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
                "goals": [[7.43, 2.56], [3.01, -4.53], [-5.92, -1.78], [6.18, 5.38]],
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

def tests_adult_0_child_100_test_case_83_stopped_low(tester):
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
                "goals": [[0.90, 3.69], [1.38, 3.02], [-6.40, 1.20], [3.29, 0.89]],
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
                "init_x": 0.05,
                "init_y": 4.21,
                "init_a": 93.93,
                "velocity": 0.99,
                "goals": [[0.90, 3.69], [-2.37, -1.21], [3.17, 4.19], [7.16, 0.87]],
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
                "init_x": 7.38,
                "init_y": 1.35,
                "init_a": 11.15,
                "velocity": 1.18,
                "goals": [[7.38, 1.35], [6.45, 1.12], [2.01, 1.20], [-3.57, -0.64]],
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
                "init_x": 8.19,
                "init_y": 1.95,
                "init_a": 11.15,
                "velocity": 1.18,
                "goals": [[7.38, 1.35], [2.63, -3.76], [-2.93, -2.68], [-6.47, -1.32]],
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
                "init_x": -0.03,
                "init_y": -2.38,
                "init_a": -82.60,
                "velocity": 1.15,
                "goals": [[-2.84, 1.33], [-6.52, -0.65], [-0.58, -4.65], [-1.45, -5.66]],
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
                "init_x": -1.13,
                "init_y": -0.33,
                "init_a": 96.01,
                "velocity": 1.08,
                "goals": [[3.75, 4.82], [-4.47, 0.78], [5.34, -0.02], [-1.74, 1.07]],
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
                "init_x": 4.92,
                "init_y": -1.49,
                "init_a": 123.12,
                "velocity": 1.06,
                "goals": [[7.39, -0.30], [2.27, -0.91], [3.81, -3.79], [-1.36, -2.22]],
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
                "init_x": 1.83,
                "init_y": 2.97,
                "init_a": 163.63,
                "velocity": 0.97,
                "goals": [[2.01, -2.50], [-6.07, -3.42], [5.07, -3.23], [-5.65, 1.26]],
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

def tests_adult_0_child_100_test_case_84_walking_low(tester):
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
                "goals": [[-5.76, -4.85], [6.65, 4.86], [-5.63, 1.93], [3.14, 0.25]],
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
                "init_x": 7.91,
                "init_y": -2.52,
                "init_a": 23.60,
                "velocity": 0.92,
                "goals": [[-5.76, -4.85], [5.65, 4.42], [-4.52, -5.00], [-5.26, -0.37]],
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
                "init_x": -7.70,
                "init_y": 0.75,
                "init_a": -40.36,
                "velocity": 0.84,
                "goals": [[-4.51, -5.16], [-3.90, 5.08], [-4.47, 3.80], [-5.45, -1.39]],
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
                "init_x": -8.04,
                "init_y": -0.19,
                "init_a": -40.36,
                "velocity": 0.84,
                "goals": [[-4.51, -5.16], [-7.45, 1.68], [6.96, 5.46], [4.39, -3.54]],
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
                "init_x": -4.87,
                "init_y": 3.19,
                "init_a": -53.55,
                "velocity": 1.04,
                "goals": [[-2.14, -0.38], [-7.93, 4.95], [-6.21, -0.92], [-7.06, -2.46]],
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
                "init_x": 5.43,
                "init_y": -1.02,
                "init_a": -141.47,
                "velocity": 0.87,
                "goals": [[6.90, 1.19], [-3.89, -2.79], [4.78, -0.10], [-7.58, -0.04]],
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
                "init_x": -7.49,
                "init_y": 1.54,
                "init_a": -117.03,
                "velocity": 1.07,
                "goals": [[0.48, -0.81], [-1.86, 5.74], [-5.58, 2.09], [2.46, -1.27]],
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
                "init_x": -1.38,
                "init_y": 3.85,
                "init_a": -135.10,
                "velocity": 0.88,
                "goals": [[-4.01, -3.23], [-5.88, 0.72], [5.93, 3.25], [-7.70, -1.48]],
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

def tests_adult_0_child_100_test_case_85_stopped_low(tester):
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
                "goals": [[-3.02, 2.61], [4.61, 2.60], [-4.61, 3.30], [6.07, 2.08]],
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
                "goals": [[-3.02, 2.61], [-3.42, 4.29], [7.93, -3.45], [-5.49, -0.34]],
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
                "init_x": -0.32,
                "init_y": -2.63,
                "init_a": 167.90,
                "velocity": 0.87,
                "goals": [[-0.32, -2.63], [7.77, -3.84], [-7.85, 2.48], [-2.29, 5.67]],
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
                "init_x": -1.16,
                "init_y": -2.08,
                "init_a": 167.90,
                "velocity": 0.87,
                "goals": [[-0.32, -2.63], [-4.67, -2.89], [-5.06, 3.81], [-0.35, 4.14]],
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
                "init_x": 3.11,
                "init_y": -2.19,
                "init_a": 110.59,
                "velocity": 1.06,
                "goals": [[-7.51, 4.66], [-1.24, 5.95], [5.12, -1.90], [7.00, 0.93]],
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
                "init_x": -1.30,
                "init_y": -4.00,
                "init_a": 157.21,
                "velocity": 1.19,
                "goals": [[-4.06, 5.03], [-4.73, 5.35], [6.51, -1.70], [-6.51, 1.19]],
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
                "init_x": -3.48,
                "init_y": 2.99,
                "init_a": 107.28,
                "velocity": 0.93,
                "goals": [[2.41, -2.97], [-0.94, 3.58], [0.75, 2.82], [-5.34, -0.10]],
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
                "goals": [[6.59, 2.15], [3.97, 1.37], [2.66, 0.02], [7.91, -0.01]],
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

def tests_adult_0_child_100_test_case_86_walking_low(tester):
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
                "goals": [[5.91, 3.44], [-2.61, 5.07], [4.19, -2.30], [-6.87, -3.62]],
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
                "init_x": 2.12,
                "init_y": -3.29,
                "init_a": -165.80,
                "velocity": 1.03,
                "goals": [[5.91, 3.44], [-3.59, 0.98], [-2.62, -0.85], [4.56, 5.22]],
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
                "init_x": 2.69,
                "init_y": -4.53,
                "init_a": -24.70,
                "velocity": 0.82,
                "goals": [[-7.61, 4.56], [7.70, -0.32], [-0.48, -1.27], [-6.10, -0.37]],
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
                "init_x": 2.02,
                "init_y": -3.79,
                "init_a": -24.70,
                "velocity": 0.82,
                "goals": [[-7.61, 4.56], [-6.43, 1.95], [2.29, 3.60], [3.45, 0.48]],
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
                "goals": [[-4.90, 1.72], [6.99, -5.27], [-3.54, 5.17], [4.92, -5.39]],
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
                "init_x": 1.31,
                "init_y": 4.99,
                "init_a": -140.64,
                "velocity": 0.80,
                "goals": [[2.81, -0.67], [7.15, 2.36], [1.65, 4.96], [5.48, 2.44]],
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
                "init_x": 7.49,
                "init_y": -3.22,
                "init_a": -15.36,
                "velocity": 0.98,
                "goals": [[5.80, -4.98], [-7.05, 1.70], [7.57, -3.76], [-5.04, 5.59]],
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

def tests_adult_0_child_100_test_case_87_stopped_low(tester):
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
                "init_x": -3.80,
                "init_y": -5.85,
                "init_a": -131.38,
                "velocity": 0.94,
                "goals": [[-3.80, -5.85], [0.14, -1.03], [1.65, 4.24], [-6.30, 4.84]],
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
                "init_x": -4.68,
                "init_y": -5.37,
                "init_a": -131.38,
                "velocity": 0.94,
                "goals": [[-3.80, -5.85], [-0.40, -0.26], [-0.85, 5.46], [7.46, -3.13]],
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
                "init_x": 6.33,
                "init_y": 4.53,
                "init_a": 32.49,
                "velocity": 1.06,
                "goals": [[6.33, 4.53], [-1.82, 5.26], [1.35, -5.33], [-4.95, -4.86]],
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
                "goals": [[6.33, 4.53], [-7.73, -0.06], [2.98, -5.83], [0.26, 0.35]],
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
                "init_x": -0.36,
                "init_y": 4.54,
                "init_a": 64.20,
                "velocity": 1.03,
                "goals": [[-5.60, -0.41], [4.88, 2.58], [0.42, 1.88], [2.77, 1.38]],
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
                "init_x": -2.97,
                "init_y": -1.95,
                "init_a": 19.91,
                "velocity": 1.16,
                "goals": [[-1.07, -3.22], [1.87, -4.73], [-1.80, -5.79], [5.34, -2.96]],
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
                "goals": [[3.74, 4.37], [1.66, -3.63], [5.36, 4.94], [-6.25, -0.54]],
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

def tests_adult_0_child_100_test_case_88_walking_low(tester):
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
                "goals": [[2.02, -4.36], [5.11, -4.24], [7.18, 2.51], [3.63, -3.83]],
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
                "init_x": 5.14,
                "init_y": 4.63,
                "init_a": -52.22,
                "velocity": 1.14,
                "goals": [[2.02, -4.36], [-4.70, -2.63], [-7.96, -4.82], [-4.55, -2.66]],
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
                "init_x": 0.93,
                "init_y": 2.39,
                "init_a": -88.80,
                "velocity": 0.97,
                "goals": [[-6.64, -0.16], [2.22, -1.17], [5.89, -1.96], [-6.86, 2.63]],
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
                "init_x": 1.79,
                "init_y": 2.89,
                "init_a": -88.80,
                "velocity": 0.97,
                "goals": [[-6.64, -0.16], [-6.47, 2.91], [-5.89, -4.21], [4.51, 1.35]],
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
                "init_x": 4.32,
                "init_y": -2.58,
                "init_a": -126.36,
                "velocity": 0.88,
                "goals": [[-0.34, -0.49], [-1.94, 4.72], [-6.48, -4.52], [0.94, 5.43]],
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
                "init_x": 0.59,
                "init_y": 0.46,
                "init_a": -80.05,
                "velocity": 1.08,
                "goals": [[-6.28, -0.19], [-2.62, 4.17], [-3.01, 0.14], [6.71, 0.28]],
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
                "init_x": 1.52,
                "init_y": -1.11,
                "init_a": 147.69,
                "velocity": 0.99,
                "goals": [[-1.04, 1.45], [-3.86, -1.10], [5.49, -4.79], [-5.87, -1.71]],
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
                "init_x": -6.08,
                "init_y": -0.56,
                "init_a": 135.74,
                "velocity": 1.04,
                "goals": [[-4.79, 5.44], [-4.31, 3.52], [-0.50, -4.60], [1.42, 2.94]],
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
                "init_x": 3.71,
                "init_y": -4.46,
                "init_a": 159.18,
                "velocity": 1.03,
                "goals": [[-6.25, 5.48], [1.48, 2.11], [4.13, 5.25], [1.10, 4.98]],
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

def tests_adult_0_child_100_test_case_89_stopped_low(tester):
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
                "goals": [[-1.11, 2.39], [-1.85, 0.27], [0.60, 2.26], [-1.98, -0.21]],
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
                "init_x": -1.06,
                "init_y": 1.39,
                "init_a": 128.30,
                "velocity": 1.00,
                "goals": [[-1.11, 2.39], [5.02, -5.01], [-5.07, -4.43], [-5.59, -4.76]],
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
                "init_x": -4.99,
                "init_y": -0.89,
                "init_a": -171.32,
                "velocity": 0.93,
                "goals": [[-4.99, -0.89], [4.42, -1.71], [0.51, 1.32], [-5.67, -2.99]],
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
                "init_x": -3.99,
                "init_y": -0.92,
                "init_a": -171.32,
                "velocity": 0.93,
                "goals": [[-4.99, -0.89], [0.67, 0.96], [4.36, -1.01], [1.80, -1.73]],
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
                "init_x": 3.10,
                "init_y": 1.38,
                "init_a": -62.99,
                "velocity": 0.83,
                "goals": [[7.40, 5.93], [-1.64, 2.55], [2.33, -1.50], [-0.88, -1.14]],
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
                "init_x": -0.01,
                "init_y": -1.61,
                "init_a": -37.60,
                "velocity": 0.87,
                "goals": [[3.92, 0.83], [-0.85, -1.21], [0.81, 4.14], [7.75, -2.72]],
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
                "init_x": -5.04,
                "init_y": 1.09,
                "init_a": 172.38,
                "velocity": 1.10,
                "goals": [[2.50, 4.84], [-2.44, -2.08], [-2.50, 0.32], [1.71, -0.43]],
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

def tests_adult_0_child_100_test_case_90_stopped_low(tester):
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
                "goals": [[-0.93, 2.74], [6.89, 0.15], [1.44, -3.73], [1.00, -5.93]],
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
                "goals": [[-0.93, 2.74], [-0.48, -2.62], [-3.81, -0.94], [-1.33, 4.65]],
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
                "init_x": -3.13,
                "init_y": -4.23,
                "init_a": 112.02,
                "velocity": 1.17,
                "goals": [[-3.13, -4.23], [0.32, -5.37], [4.56, -4.06], [-5.03, -4.09]],
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
                "init_x": -4.07,
                "init_y": -3.89,
                "init_a": 112.02,
                "velocity": 1.17,
                "goals": [[-3.13, -4.23], [1.60, -1.20], [-0.08, -3.24], [-5.43, -2.93]],
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
                "init_x": -3.05,
                "init_y": 5.39,
                "init_a": -4.71,
                "velocity": 1.16,
                "goals": [[2.37, 2.14], [1.75, 1.65], [2.70, -1.87], [-4.99, -4.17]],
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
                "init_x": 1.51,
                "init_y": 2.65,
                "init_a": 171.66,
                "velocity": 1.19,
                "goals": [[-5.78, -4.44], [-6.42, -3.59], [6.06, -1.73], [6.90, 1.28]],
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
                "goals": [[-0.87, -4.13], [5.38, 3.50], [-5.41, 1.66], [0.08, -1.12]],
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
                "init_x": 4.08,
                "init_y": -1.20,
                "init_a": 108.61,
                "velocity": 0.96,
                "goals": [[4.47, -1.06], [0.55, 0.10], [-5.56, -0.92], [5.38, -5.29]],
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
                "init_x": -6.10,
                "init_y": -1.78,
                "init_a": 109.96,
                "velocity": 0.99,
                "goals": [[-6.33, -5.26], [-5.10, -2.75], [-5.05, 2.17], [-6.59, 0.01]],
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

def tests_adult_0_child_100_test_case_91_walking_low(tester):
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
                "goals": [[-0.01, -0.15], [4.01, -1.86], [-5.89, 3.95], [3.13, 4.13]],
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
                "init_x": -5.72,
                "init_y": 4.76,
                "init_a": 178.13,
                "velocity": 0.82,
                "goals": [[-0.01, -0.15], [-1.84, -5.45], [-7.22, 3.66], [-7.85, -4.09]],
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
                "init_x": 4.61,
                "init_y": 4.42,
                "init_a": 133.09,
                "velocity": 1.18,
                "goals": [[6.72, -5.01], [-0.44, -2.19], [0.12, 2.00], [2.41, 3.29]],
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
                "init_x": 5.56,
                "init_y": 4.74,
                "init_a": 133.09,
                "velocity": 1.18,
                "goals": [[6.72, -5.01], [-5.21, -5.23], [4.53, -3.38], [-5.61, -5.95]],
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
                "init_x": -1.31,
                "init_y": 5.46,
                "init_a": 16.99,
                "velocity": 1.16,
                "goals": [[5.20, -5.13], [-6.29, -1.17], [1.79, -1.38], [0.30, 3.13]],
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
                "goals": [[4.01, -5.30], [7.70, -0.40], [1.65, -5.66], [-5.17, -1.53]],
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
                "init_x": -0.55,
                "init_y": 3.87,
                "init_a": -31.16,
                "velocity": 0.83,
                "goals": [[-3.96, -5.81], [-7.05, 5.90], [-1.53, -5.80], [-4.02, -1.93]],
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

def tests_adult_0_child_100_test_case_92_stopped_low(tester):
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
                "goals": [[5.47, -3.71], [6.80, 4.08], [7.31, -5.31], [5.90, -5.58]],
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
                "init_x": 4.71,
                "init_y": -4.37,
                "init_a": -69.69,
                "velocity": 0.98,
                "goals": [[5.47, -3.71], [5.24, 0.75], [0.95, 3.13], [3.76, -0.15]],
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
                "goals": [[-0.37, 0.08], [6.22, -4.49], [-4.47, 4.83], [7.22, 4.48]],
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
                "init_x": -1.35,
                "init_y": -0.14,
                "init_a": 1.28,
                "velocity": 0.84,
                "goals": [[-0.37, 0.08], [-6.58, 4.32], [7.53, -5.88], [7.70, -2.88]],
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
                "init_x": -4.59,
                "init_y": -0.39,
                "init_a": 90.21,
                "velocity": 1.16,
                "goals": [[-2.14, -4.77], [7.68, 0.26], [5.80, 3.03], [-4.79, -0.87]],
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
                "goals": [[-4.70, -1.62], [5.77, -0.09], [0.73, -2.81], [7.21, -2.44]],
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
                "init_x": 0.48,
                "init_y": 3.41,
                "init_a": 76.50,
                "velocity": 0.94,
                "goals": [[2.90, -5.64], [0.81, 2.66], [6.27, -5.66], [1.76, 3.07]],
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
                "init_x": -4.26,
                "init_y": 0.30,
                "init_a": 2.06,
                "velocity": 1.14,
                "goals": [[-7.29, 1.54], [1.22, 0.15], [-7.14, -4.01], [-7.62, 1.25]],
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
                "init_x": -1.18,
                "init_y": -5.33,
                "init_a": 9.47,
                "velocity": 1.17,
                "goals": [[3.51, 2.65], [1.83, -5.60], [-6.74, -4.20], [5.18, 2.56]],
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

def tests_adult_0_child_100_test_case_93_stopped_low(tester):
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
                "goals": [[7.19, 4.52], [0.54, -0.44], [0.06, -3.35], [1.32, -2.79]],
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
                "goals": [[7.19, 4.52], [-7.78, -4.45], [-3.15, 4.15], [-3.59, 5.04]],
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
                "goals": [[3.63, -0.21], [-6.06, -4.19], [-4.66, -5.40], [-2.83, -1.73]],
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
                "init_x": 2.65,
                "init_y": -0.41,
                "init_a": -58.74,
                "velocity": 1.12,
                "goals": [[3.63, -0.21], [2.12, 0.72], [0.61, -4.81], [5.56, -4.96]],
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
                "init_x": 1.12,
                "init_y": 3.41,
                "init_a": -123.42,
                "velocity": 1.03,
                "goals": [[6.11, 2.16], [2.41, 0.57], [-6.51, -3.25], [-2.28, -5.12]],
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
                "init_x": -6.17,
                "init_y": 0.19,
                "init_a": -115.45,
                "velocity": 1.12,
                "goals": [[1.08, 5.96], [5.09, 4.77], [7.64, -2.77], [7.94, -4.34]],
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
                "init_x": 0.25,
                "init_y": -5.08,
                "init_a": 45.43,
                "velocity": 0.88,
                "goals": [[4.40, 5.35], [-7.70, -1.27], [-7.94, -0.67], [6.84, 0.68]],
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
                "init_x": 7.59,
                "init_y": 0.53,
                "init_a": 16.78,
                "velocity": 1.02,
                "goals": [[6.53, 0.29], [-4.83, -3.97], [4.10, 2.05], [6.51, 3.30]],
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
                "init_x": -4.22,
                "init_y": 0.97,
                "init_a": -165.06,
                "velocity": 1.01,
                "goals": [[3.05, -1.26], [-0.35, -4.08], [5.16, 0.44], [-2.15, -4.89]],
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

def tests_adult_0_child_100_test_case_94_stopped_low(tester):
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
                "goals": [[2.89, 5.31], [2.15, 5.87], [3.06, 1.72], [6.15, -0.18]],
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
                "init_x": 2.51,
                "init_y": 6.23,
                "init_a": -3.78,
                "velocity": 0.94,
                "goals": [[2.89, 5.31], [-0.36, 5.68], [0.32, 2.30], [-7.06, -3.83]],
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
                "init_x": -1.03,
                "init_y": -4.62,
                "init_a": -91.41,
                "velocity": 1.02,
                "goals": [[-1.03, -4.62], [7.04, 4.92], [0.29, 0.41], [1.51, -3.50]],
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
                "init_x": -1.34,
                "init_y": -5.57,
                "init_a": -91.41,
                "velocity": 1.02,
                "goals": [[-1.03, -4.62], [6.12, 4.33], [6.57, -5.55], [2.59, -5.37]],
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
                "init_x": -1.90,
                "init_y": 5.02,
                "init_a": 75.35,
                "velocity": 1.19,
                "goals": [[-2.50, -0.70], [-0.73, -5.92], [-1.48, 3.01], [4.27, -3.94]],
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
                "init_x": 4.65,
                "init_y": -1.35,
                "init_a": -47.70,
                "velocity": 0.97,
                "goals": [[-0.98, 0.12], [-1.00, -2.77], [-5.23, 3.62], [5.11, 3.96]],
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
                "init_x": 7.03,
                "init_y": 0.65,
                "init_a": 128.01,
                "velocity": 0.83,
                "goals": [[3.19, 3.93], [-1.28, -2.75], [6.30, -0.21], [0.25, 5.83]],
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
                "init_x": -4.12,
                "init_y": 3.61,
                "init_a": 116.14,
                "velocity": 1.14,
                "goals": [[-3.49, 2.80], [-7.40, -1.39], [2.61, 4.20], [-5.20, -1.33]],
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

def tests_adult_0_child_100_test_case_95_stopped_low(tester):
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
                "init_x": -6.54,
                "init_y": -0.30,
                "init_a": 117.02,
                "velocity": 0.80,
                "goals": [[-6.54, -0.30], [-0.90, 2.30], [0.02, -1.22], [-6.91, 1.54]],
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
                "init_x": -5.74,
                "init_y": -0.90,
                "init_a": 117.02,
                "velocity": 0.80,
                "goals": [[-6.54, -0.30], [2.56, 2.16], [6.68, -3.81], [6.68, 3.40]],
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
                "init_x": -6.81,
                "init_y": -0.97,
                "init_a": 177.66,
                "velocity": 1.08,
                "goals": [[-6.81, -0.97], [2.31, -2.44], [5.25, 0.01], [-6.09, 1.92]],
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
                "init_x": -7.68,
                "init_y": -0.48,
                "init_a": 177.66,
                "velocity": 1.08,
                "goals": [[-6.81, -0.97], [-1.42, -3.65], [4.35, 3.96], [-0.24, 4.20]],
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
                "init_x": 7.57,
                "init_y": -4.10,
                "init_a": -113.57,
                "velocity": 1.03,
                "goals": [[0.55, -4.29], [4.18, 2.86], [-7.11, 5.75], [3.27, -4.65]],
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
                "init_x": -0.46,
                "init_y": -2.46,
                "init_a": -1.37,
                "velocity": 0.86,
                "goals": [[7.35, -2.17], [-3.50, 5.37], [-1.74, -0.06], [-6.83, 2.13]],
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
                "init_x": 2.18,
                "init_y": -5.24,
                "init_a": -30.90,
                "velocity": 1.11,
                "goals": [[4.81, -1.33], [-2.64, 0.52], [1.05, 0.72], [1.56, 0.66]],
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
                "init_x": 3.28,
                "init_y": 0.47,
                "init_a": -116.73,
                "velocity": 0.82,
                "goals": [[-5.06, -5.11], [0.69, -0.89], [-4.79, -3.47], [3.22, 5.36]],
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

def tests_adult_0_child_100_test_case_96_stopped_low(tester):
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
                "goals": [[1.16, 3.19], [-7.37, -0.62], [-4.88, -4.32], [-7.17, 4.12]],
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
                "init_x": 0.16,
                "init_y": 3.15,
                "init_a": -13.09,
                "velocity": 1.08,
                "goals": [[1.16, 3.19], [-4.49, -3.34], [1.66, -3.64], [1.70, 5.63]],
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
                "goals": [[-5.08, 0.38], [-4.35, 3.88], [2.73, 3.06], [6.63, -2.12]],
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
                "init_x": -4.53,
                "init_y": 1.21,
                "init_a": 39.15,
                "velocity": 1.09,
                "goals": [[-5.08, 0.38], [2.45, 1.73], [-7.62, -0.74], [1.52, -5.91]],
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
                "init_x": 6.64,
                "init_y": -3.32,
                "init_a": 151.53,
                "velocity": 1.05,
                "goals": [[-7.73, 3.23], [7.42, 1.58], [2.70, 4.74], [-6.11, -3.76]],
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
                "init_x": 1.24,
                "init_y": -2.78,
                "init_a": 87.39,
                "velocity": 1.01,
                "goals": [[-7.51, 2.53], [-5.60, 1.83], [-1.49, -2.43], [3.85, -0.96]],
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
                "init_x": 2.61,
                "init_y": 1.70,
                "init_a": -156.35,
                "velocity": 0.98,
                "goals": [[-2.04, -1.36], [-5.22, -4.59], [2.99, 4.70], [-4.12, 3.50]],
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
                "init_x": -6.06,
                "init_y": -0.40,
                "init_a": -73.50,
                "velocity": 0.95,
                "goals": [[6.30, 0.59], [3.58, -5.32], [-5.06, -0.33], [-4.44, -5.84]],
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
                "init_x": 1.80,
                "init_y": 0.20,
                "init_a": 101.91,
                "velocity": 1.04,
                "goals": [[3.25, -5.64], [4.76, -4.10], [6.48, 2.57], [3.58, 5.73]],
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

def tests_adult_0_child_100_test_case_97_stopped_low(tester):
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
                "goals": [[-3.96, -5.71], [4.38, 1.73], [-6.31, 4.01], [-6.71, -1.19]],
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
                "init_x": -4.92,
                "init_y": -5.98,
                "init_a": 135.41,
                "velocity": 1.14,
                "goals": [[-3.96, -5.71], [-7.03, 0.77], [6.53, 2.95], [-3.39, 4.86]],
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
                "init_x": -7.00,
                "init_y": -3.76,
                "init_a": -8.34,
                "velocity": 0.81,
                "goals": [[-7.00, -3.76], [4.47, -3.41], [2.96, -0.11], [3.59, 4.54]],
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
                "init_x": -7.64,
                "init_y": -4.53,
                "init_a": -8.34,
                "velocity": 0.81,
                "goals": [[-7.00, -3.76], [-4.42, -4.10], [-1.83, 4.33], [5.49, 0.90]],
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
                "init_x": 5.13,
                "init_y": -3.01,
                "init_a": 60.90,
                "velocity": 0.86,
                "goals": [[-0.68, 2.36], [7.80, 0.93], [-4.75, -0.22], [-1.83, 2.53]],
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
                "init_x": -2.89,
                "init_y": -3.42,
                "init_a": -142.97,
                "velocity": 0.81,
                "goals": [[3.83, -0.08], [3.34, -1.07], [-3.88, -4.88], [1.50, 1.86]],
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
                "init_x": -5.71,
                "init_y": -1.43,
                "init_a": 100.92,
                "velocity": 1.16,
                "goals": [[-0.39, 3.59], [6.88, 5.89], [6.07, 0.94], [2.35, 5.54]],
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
                "init_x": 7.54,
                "init_y": -4.39,
                "init_a": -84.48,
                "velocity": 1.05,
                "goals": [[-0.54, 0.07], [-4.08, -1.19], [-7.21, 2.38], [-7.47, -5.06]],
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

def tests_adult_0_child_100_test_case_98_stopped_low(tester):
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
                "goals": [[-3.25, -0.40], [3.29, -4.69], [-6.52, -5.08], [-0.90, -1.04]],
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
                "init_x": -2.85,
                "init_y": -1.31,
                "init_a": 89.69,
                "velocity": 0.96,
                "goals": [[-3.25, -0.40], [5.35, -4.16], [1.70, 5.43], [-6.42, 0.27]],
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
                "init_x": 1.70,
                "init_y": -2.21,
                "init_a": -121.14,
                "velocity": 1.12,
                "goals": [[1.70, -2.21], [1.77, 4.59], [4.15, -0.05], [-3.88, -5.79]],
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
                "goals": [[1.70, -2.21], [0.63, 3.78], [2.31, 1.80], [-5.71, -4.05]],
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
                "goals": [[-1.70, 5.79], [6.97, -3.64], [7.70, 1.01], [-7.37, 0.07]],
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
                "init_x": -0.43,
                "init_y": -0.68,
                "init_a": -90.57,
                "velocity": 0.92,
                "goals": [[2.65, 2.42], [-4.96, 4.62], [-1.89, -2.83], [-3.30, -1.86]],
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
                "init_x": -6.41,
                "init_y": -0.93,
                "init_a": -177.47,
                "velocity": 0.96,
                "goals": [[-7.36, 1.83], [-7.96, 2.87], [7.76, -4.47], [5.23, 4.85]],
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

def tests_adult_0_child_100_test_case_99_stopped_low(tester):
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
                "goals": [[0.77, 2.15], [-1.47, 1.42], [7.87, -0.79], [4.91, 3.07]],
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
                "init_x": 1.52,
                "init_y": 1.48,
                "init_a": -109.88,
                "velocity": 0.91,
                "goals": [[0.77, 2.15], [-6.28, -0.63], [-3.21, -1.16], [-6.79, 2.06]],
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
                "init_x": 7.24,
                "init_y": -5.77,
                "init_a": 94.01,
                "velocity": 0.99,
                "goals": [[7.24, -5.77], [-2.60, 0.11], [1.84, 5.18], [7.66, -3.00]],
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
                "init_x": 6.65,
                "init_y": -6.58,
                "init_a": 94.01,
                "velocity": 0.99,
                "goals": [[7.24, -5.77], [2.39, 4.82], [1.20, 4.85], [-6.40, 2.46]],
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
                "init_x": -4.35,
                "init_y": -5.99,
                "init_a": -43.20,
                "velocity": 0.94,
                "goals": [[-1.22, 2.65], [-5.58, -3.55], [-6.17, -5.45], [4.49, -2.48]],
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
                "init_x": 1.32,
                "init_y": -0.58,
                "init_a": -131.71,
                "velocity": 1.19,
                "goals": [[4.83, -4.33], [1.71, -5.79], [5.44, -1.31], [3.26, 1.77]],
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
                "init_x": -1.35,
                "init_y": 0.48,
                "init_a": -56.40,
                "velocity": 1.13,
                "goals": [[1.13, 1.01], [0.91, 2.95], [-1.92, -0.64], [7.07, -4.19]],
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

def tests_adult_0_child_100_test_case_100_stopped_low(tester):
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
                "init_x": -0.41,
                "init_y": -3.70,
                "init_a": -13.51,
                "velocity": 1.16,
                "goals": [[-0.41, -3.70], [-1.05, 1.16], [-3.99, -0.32], [-5.81, 3.35]],
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
                "init_x": -1.04,
                "init_y": -2.93,
                "init_a": -13.51,
                "velocity": 1.16,
                "goals": [[-0.41, -3.70], [4.74, -5.48], [6.96, -2.90], [-6.58, -2.98]],
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
                "init_x": 5.70,
                "init_y": 0.93,
                "init_a": -126.29,
                "velocity": 0.96,
                "goals": [[5.70, 0.93], [0.10, 1.21], [0.78, -5.85], [3.39, 0.51]],
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
                "init_x": 5.22,
                "init_y": 0.05,
                "init_a": -126.29,
                "velocity": 0.96,
                "goals": [[5.70, 0.93], [5.67, 2.58], [-4.09, -3.28], [-0.62, -2.22]],
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
                "init_x": -3.19,
                "init_y": -5.62,
                "init_a": 116.97,
                "velocity": 0.92,
                "goals": [[6.21, 4.51], [-2.62, -1.84], [-2.17, -3.77], [1.05, 0.28]],
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
                "init_x": 3.14,
                "init_y": 3.78,
                "init_a": 161.95,
                "velocity": 1.07,
                "goals": [[7.95, -5.10], [4.80, 4.64], [1.72, 4.04], [-3.04, -0.21]],
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
                "init_x": 5.09,
                "init_y": 2.93,
                "init_a": 84.73,
                "velocity": 1.13,
                "goals": [[-5.01, 1.58], [-7.02, -0.57], [-2.15, -5.25], [-4.88, 4.29]],
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
                "init_x": -6.23,
                "init_y": -4.61,
                "init_a": 140.29,
                "velocity": 1.12,
                "goals": [[0.60, -4.75], [5.30, 5.28], [0.08, -3.22], [-3.12, 0.89]],
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
