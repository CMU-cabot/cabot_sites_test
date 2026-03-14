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

def tests_adult_0_child_100_test_case_01_stopped_medium(tester):
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
                "goals": [[5.80, -4.90], [-2.48, 5.38], [1.38, -4.37], [2.38, -4.77]],
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
                "init_x": 4.97,
                "init_y": -4.35,
                "init_a": 144.90,
                "velocity": 1.11,
                "goals": [[5.80, -4.90], [-0.48, -1.35], [-3.14, 2.29], [4.80, -0.05]],
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
                "init_x": -2.01,
                "init_y": 1.79,
                "init_a": -35.67,
                "velocity": 1.03,
                "goals": [[-2.01, 1.79], [-0.63, -1.53], [-5.03, -4.93], [4.03, -3.09]],
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
                "init_x": -2.79,
                "init_y": 1.16,
                "init_a": -35.67,
                "velocity": 1.03,
                "goals": [[-2.01, 1.79], [1.51, -1.15], [-1.80, 4.29], [3.62, 3.37]],
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
                "init_x": -7.19,
                "init_y": 2.85,
                "init_a": 75.06,
                "velocity": 1.17,
                "goals": [[3.44, 1.87], [6.24, 5.68], [2.53, 1.88], [3.78, 3.83]],
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
                "init_x": 3.79,
                "init_y": -3.75,
                "init_a": 0.71,
                "velocity": 1.16,
                "goals": [[6.82, -5.77], [-0.88, -1.04], [6.75, 3.64], [-4.48, -0.82]],
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
                "init_x": 6.04,
                "init_y": 1.64,
                "init_a": -33.61,
                "velocity": 0.85,
                "goals": [[5.80, -2.95], [5.23, -2.99], [7.44, 3.88], [-3.82, 5.25]],
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

def tests_adult_0_child_100_test_case_02_walking_medium(tester):
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
                "goals": [[6.16, 1.18], [2.28, -5.79], [-6.62, -4.50], [0.20, 1.50]],
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
                "init_x": 1.00,
                "init_y": 3.34,
                "init_a": 23.06,
                "velocity": 1.15,
                "goals": [[6.16, 1.18], [-1.13, -5.73], [-4.73, 1.25], [-4.72, -5.64]],
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
                "init_x": -3.72,
                "init_y": 0.15,
                "init_a": 177.69,
                "velocity": 0.82,
                "goals": [[4.38, 4.53], [-1.19, 0.15], [-1.36, -3.89], [-3.62, 0.91]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.38, 4.53], [-1.30, -2.22], [0.32, -4.38], [-0.78, -0.54]],
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
                "init_x": -7.76,
                "init_y": -3.90,
                "init_a": -37.15,
                "velocity": 0.95,
                "goals": [[0.24, 0.89], [-7.75, -5.39], [7.80, -5.11], [-0.18, -1.95]],
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
                "init_x": -1.64,
                "init_y": -5.50,
                "init_a": 14.01,
                "velocity": 0.81,
                "goals": [[-2.39, 5.83], [4.68, 1.58], [7.46, -1.94], [-7.80, -1.22]],
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
                "init_x": 5.56,
                "init_y": 5.06,
                "init_a": 34.64,
                "velocity": 1.20,
                "goals": [[4.60, -2.08], [-1.66, -5.91], [-5.79, 0.33], [-1.45, -0.51]],
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
                "init_x": -3.81,
                "init_y": -1.39,
                "init_a": -21.06,
                "velocity": 0.82,
                "goals": [[-3.37, -1.33], [6.59, -2.45], [-0.28, 2.55], [-4.76, -3.27]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.25, -5.43], [-6.12, 1.55], [-6.46, 5.54], [5.44, 4.15]],
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

def tests_adult_0_child_100_test_case_03_walking_medium(tester):
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
                "goals": [[-7.21, -5.96], [-3.49, 0.18], [-2.72, 1.03], [3.47, -2.52]],
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
                "init_x": -8.40,
                "init_y": 3.89,
                "init_a": 85.34,
                "velocity": 0.98,
                "goals": [[-7.21, -5.96], [-4.12, -0.53], [-1.14, 3.44], [-2.24, -2.69]],
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
                "init_x": 1.51,
                "init_y": 4.59,
                "init_a": -24.94,
                "velocity": 1.11,
                "goals": [[-1.61, 1.37], [-2.26, 1.54], [3.37, -1.43], [-7.03, 3.55]],
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
                "init_x": 1.85,
                "init_y": 5.53,
                "init_a": -24.94,
                "velocity": 1.11,
                "goals": [[-1.61, 1.37], [-4.76, 2.61], [-0.21, 5.47], [-6.84, -0.37]],
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
                "init_x": -6.30,
                "init_y": -0.73,
                "init_a": -25.18,
                "velocity": 0.95,
                "goals": [[-2.94, 1.75], [-1.97, -5.83], [-7.44, 1.21], [-7.75, -3.64]],
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
                "init_x": -7.31,
                "init_y": -2.03,
                "init_a": 113.87,
                "velocity": 0.90,
                "goals": [[-1.00, 5.09], [-5.07, 5.26], [-6.54, 0.22], [-5.00, -3.40]],
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
                "init_x": -6.71,
                "init_y": 5.82,
                "init_a": 28.16,
                "velocity": 0.87,
                "goals": [[-7.09, 2.88], [-2.49, -4.17], [-5.73, 2.79], [-0.20, -2.74]],
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

def tests_adult_0_child_100_test_case_04_stopped_medium(tester):
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
                "goals": [[-1.18, -0.11], [4.18, -5.73], [-4.25, 5.20], [-2.02, 4.65]],
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
                "init_x": -0.81,
                "init_y": -1.04,
                "init_a": -0.28,
                "velocity": 1.14,
                "goals": [[-1.18, -0.11], [2.82, -3.62], [4.34, 3.96], [-3.30, 2.12]],
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
                "init_x": 4.13,
                "init_y": 5.30,
                "init_a": 28.19,
                "velocity": 1.11,
                "goals": [[4.13, 5.30], [-5.48, -4.67], [-5.04, -5.06], [-5.35, -2.26]],
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
                "init_x": 4.17,
                "init_y": 6.30,
                "init_a": 28.19,
                "velocity": 1.11,
                "goals": [[4.13, 5.30], [-6.42, -1.93], [4.78, -3.62], [4.60, 2.45]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.83, 2.41], [3.76, 1.14], [-7.29, -0.74], [-6.06, -0.72]],
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
                "init_x": 6.78,
                "init_y": -4.93,
                "init_a": 37.63,
                "velocity": 1.18,
                "goals": [[2.10, -1.79], [5.18, -0.40], [0.23, 4.22], [0.19, 2.87]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.02, -1.83], [5.86, 0.24], [1.33, -3.22], [6.49, -5.45]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.73, 1.04], [7.25, 1.11], [-5.85, 4.00], [-3.55, -4.59]],
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

def tests_adult_0_child_100_test_case_05_walking_medium(tester):
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
                "goals": [[-0.81, -2.77], [6.93, 4.96], [6.20, 5.95], [-7.44, 0.13]],
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
                "init_x": 4.97,
                "init_y": -5.94,
                "init_a": 94.19,
                "velocity": 1.09,
                "goals": [[-0.81, -2.77], [-3.63, -3.07], [-2.07, 5.65], [-7.75, 1.61]],
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
                "init_x": -3.05,
                "init_y": -2.34,
                "init_a": -79.94,
                "velocity": 1.08,
                "goals": [[-2.10, 1.87], [-3.65, -3.75], [0.33, -0.93], [6.22, -1.60]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.10, 1.87], [7.93, 0.59], [0.70, 3.39], [-0.45, -3.33]],
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
                "init_x": 1.24,
                "init_y": -2.50,
                "init_a": -148.03,
                "velocity": 1.16,
                "goals": [[2.12, 4.59], [-1.34, 3.92], [-4.48, -4.69], [4.70, 2.83]],
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
                "init_x": -7.19,
                "init_y": -5.29,
                "init_a": -2.85,
                "velocity": 1.07,
                "goals": [[-2.98, 5.35], [-5.42, 2.38], [-7.16, 3.36], [-0.15, -3.02]],
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
                "init_x": -1.05,
                "init_y": -1.04,
                "init_a": 14.46,
                "velocity": 0.95,
                "goals": [[-0.44, 1.54], [3.20, 4.05], [4.54, 1.44], [3.12, -0.07]],
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
                "init_x": -2.66,
                "init_y": 4.36,
                "init_a": 118.49,
                "velocity": 1.02,
                "goals": [[0.77, -2.60], [4.21, -0.84], [7.36, -4.29], [7.61, 0.74]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.75, 5.18], [-2.01, -2.37], [-5.72, 1.37], [-5.54, -3.62]],
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

def tests_adult_0_child_100_test_case_06_walking_medium(tester):
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
                "goals": [[-2.54, -5.99], [6.49, -5.28], [-6.33, -4.39], [-0.13, 3.49]],
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
                "init_x": -0.23,
                "init_y": 6.03,
                "init_a": 115.33,
                "velocity": 1.06,
                "goals": [[-2.54, -5.99], [2.78, -3.58], [1.15, -2.56], [0.30, -3.54]],
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
                "init_x": 5.75,
                "init_y": -1.75,
                "init_a": 85.54,
                "velocity": 1.06,
                "goals": [[4.82, 2.34], [-7.25, -5.52], [5.60, -1.85], [-6.50, 5.85]],
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
                "init_x": 6.33,
                "init_y": -0.94,
                "init_a": 85.54,
                "velocity": 1.06,
                "goals": [[4.82, 2.34], [6.09, 2.10], [2.73, -2.38], [5.70, 1.71]],
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
                "init_x": -0.08,
                "init_y": 1.93,
                "init_a": 16.11,
                "velocity": 1.16,
                "goals": [[-3.12, -0.43], [-4.50, 5.43], [7.88, -5.41], [-2.92, -0.36]],
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
                "init_x": -6.33,
                "init_y": -3.83,
                "init_a": 126.00,
                "velocity": 0.85,
                "goals": [[-5.04, -1.36], [-5.26, -4.93], [-0.69, -0.09], [-3.83, 4.34]],
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
                "init_x": -2.67,
                "init_y": 5.84,
                "init_a": 11.02,
                "velocity": 0.83,
                "goals": [[-5.00, 3.06], [6.83, -1.15], [-0.55, 2.59], [7.22, 3.94]],
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

def tests_adult_0_child_100_test_case_07_walking_medium(tester):
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
                "goals": [[5.00, 4.61], [4.72, -2.89], [-7.00, 5.58], [7.14, 3.34]],
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
                "init_x": 6.14,
                "init_y": 3.18,
                "init_a": 59.94,
                "velocity": 0.92,
                "goals": [[5.00, 4.61], [-5.20, 1.56], [2.75, 1.20], [4.60, 1.81]],
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
                "init_x": -7.07,
                "init_y": -2.22,
                "init_a": 70.22,
                "velocity": 1.00,
                "goals": [[4.51, -1.38], [5.97, 3.99], [0.82, -3.49], [-0.71, -5.52]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.51, -1.38], [-1.66, -0.38], [-2.84, 0.84], [-7.79, 3.38]],
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
                "init_x": -1.92,
                "init_y": -4.91,
                "init_a": 87.02,
                "velocity": 0.97,
                "goals": [[-5.39, 1.51], [-4.88, 0.33], [6.40, -1.59], [7.62, -3.15]],
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
                "init_x": -1.04,
                "init_y": 2.09,
                "init_a": 44.20,
                "velocity": 1.03,
                "goals": [[-2.75, 3.33], [-1.46, 5.09], [-0.06, -4.06], [-6.82, 4.13]],
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
                "init_x": 0.63,
                "init_y": -2.88,
                "init_a": -108.06,
                "velocity": 1.02,
                "goals": [[-4.87, -0.78], [4.35, -3.23], [-3.07, 2.29], [-4.76, -2.42]],
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
                "init_x": 4.03,
                "init_y": -1.12,
                "init_a": 56.72,
                "velocity": 1.17,
                "goals": [[-6.89, 1.07], [-0.17, -1.52], [4.47, -4.87], [4.62, -5.63]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[1.77, -2.67], [-0.07, -5.42], [-2.05, -4.56], [-5.05, 1.99]],
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

def tests_adult_0_child_100_test_case_08_stopped_medium(tester):
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
                "goals": [[-4.77, 0.91], [-2.57, 0.54], [-1.82, 0.40], [6.76, -3.25]],
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
                "init_x": -5.72,
                "init_y": 1.23,
                "init_a": 62.71,
                "velocity": 0.97,
                "goals": [[-4.77, 0.91], [-6.63, 5.64], [-1.81, 5.06], [6.21, 0.61]],
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
                "init_x": -4.52,
                "init_y": 3.30,
                "init_a": -75.07,
                "velocity": 0.82,
                "goals": [[-4.52, 3.30], [5.24, 5.74], [-6.43, 0.88], [4.62, 2.78]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.52, 3.30], [7.46, -1.50], [7.92, -3.34], [-0.45, 5.40]],
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
                "init_x": -7.27,
                "init_y": 1.76,
                "init_a": -23.37,
                "velocity": 0.91,
                "goals": [[-7.91, 3.36], [-5.57, 5.66], [-6.00, -2.22], [6.07, -0.15]],
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
                "init_x": 5.67,
                "init_y": -4.14,
                "init_a": 57.15,
                "velocity": 0.98,
                "goals": [[7.70, 2.37], [-1.20, 0.37], [-2.74, -0.63], [3.29, 4.37]],
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
                "init_x": 5.22,
                "init_y": -2.80,
                "init_a": 65.60,
                "velocity": 0.86,
                "goals": [[-1.73, -4.73], [2.83, -3.04], [0.39, 5.39], [-3.59, -2.14]],
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
                "init_x": 3.97,
                "init_y": 5.97,
                "init_a": -47.11,
                "velocity": 0.91,
                "goals": [[0.95, 1.70], [0.55, -3.14], [-0.89, 5.58], [2.45, -1.92]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.18, -1.03], [6.60, 0.61], [-3.76, 0.55], [3.02, -5.71]],
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

def tests_adult_0_child_100_test_case_09_stopped_medium(tester):
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
                "goals": [[-3.95, 0.77], [-5.54, 1.74], [-2.00, -5.31], [-7.10, -2.12]],
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
                "init_x": -4.95,
                "init_y": 0.69,
                "init_a": 24.94,
                "velocity": 1.04,
                "goals": [[-3.95, 0.77], [-7.25, 3.66], [0.44, 2.17], [-1.53, -1.73]],
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
                "init_x": 3.40,
                "init_y": 4.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goals": [[3.40, 4.71], [0.59, 4.47], [-0.22, 2.60], [-0.88, -3.17]],
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
                "init_x": 3.46,
                "init_y": 3.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goals": [[3.40, 4.71], [-0.52, 3.37], [3.44, -3.12], [-4.14, 4.65]],
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
                "init_x": 7.69,
                "init_y": -2.51,
                "init_a": 31.90,
                "velocity": 0.83,
                "goals": [[-0.89, 3.23], [0.58, 3.52], [5.60, -1.08], [-4.35, 4.85]],
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
                "init_x": 1.74,
                "init_y": 2.87,
                "init_a": -38.87,
                "velocity": 1.12,
                "goals": [[-4.04, 4.58], [0.94, 1.08], [1.67, 3.44], [-2.85, 1.61]],
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
                "init_x": -3.11,
                "init_y": -4.04,
                "init_a": 48.60,
                "velocity": 1.01,
                "goals": [[0.32, -3.93], [6.06, 5.46], [6.84, 3.32], [0.35, -4.13]],
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

def tests_adult_0_child_100_test_case_10_stopped_medium(tester):
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
                "goals": [[5.20, -4.57], [-7.44, -1.12], [7.04, 5.55], [-2.31, -2.24]],
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
                "init_x": 4.26,
                "init_y": -4.91,
                "init_a": 29.17,
                "velocity": 1.06,
                "goals": [[5.20, -4.57], [0.26, -5.18], [-5.31, -4.03], [-3.52, -5.44]],
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
                "init_x": -5.86,
                "init_y": 5.62,
                "init_a": -172.39,
                "velocity": 0.92,
                "goals": [[-5.86, 5.62], [1.25, -0.25], [-2.62, 0.31], [3.83, 0.58]],
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
                "init_x": -5.79,
                "init_y": 6.61,
                "init_a": -172.39,
                "velocity": 0.92,
                "goals": [[-5.86, 5.62], [2.58, -0.34], [-3.26, -4.78], [-3.74, -0.98]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.28, -1.59], [-5.62, 3.47], [-4.59, 2.69], [5.08, -4.75]],
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
                "init_x": -3.09,
                "init_y": -5.54,
                "init_a": -4.08,
                "velocity": 1.19,
                "goals": [[2.28, 0.07], [-3.72, 1.32], [4.96, -3.51], [6.83, -0.49]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.14, -4.66], [-4.28, -1.64], [2.09, 4.21], [7.97, -5.39]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.85, -2.97], [-7.61, -5.32], [7.02, -5.16], [-4.57, 5.87]],
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

def tests_adult_0_child_100_test_case_11_walking_medium(tester):
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
                "goals": [[3.12, 4.98], [3.74, 4.14], [7.15, -5.10], [1.18, -5.37]],
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
                "init_x": 4.53,
                "init_y": -3.17,
                "init_a": 150.95,
                "velocity": 1.20,
                "goals": [[3.12, 4.98], [6.32, -4.31], [-4.26, -5.03], [7.75, -5.82]],
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
                "init_x": 2.23,
                "init_y": -1.79,
                "init_a": -56.37,
                "velocity": 0.91,
                "goals": [[5.16, -3.74], [-4.33, -3.81], [-2.70, 0.93], [3.55, -3.38]],
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
                "init_x": 1.72,
                "init_y": -2.65,
                "init_a": -56.37,
                "velocity": 0.91,
                "goals": [[5.16, -3.74], [7.32, 0.22], [-6.00, 1.80], [-6.50, -0.06]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.05, -0.44], [-6.98, 2.24], [6.53, 0.45], [-3.39, -2.00]],
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
                "init_x": 6.19,
                "init_y": -4.13,
                "init_a": 45.66,
                "velocity": 1.12,
                "goals": [[-5.39, -3.91], [-1.56, -5.38], [7.10, -4.78], [-4.97, 1.78]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.40, -3.83], [-6.99, -5.72], [7.08, -4.04], [-3.38, -3.30]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.17, 4.38], [0.48, -4.04], [3.02, -0.36], [-7.91, 4.57]],
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

def tests_adult_0_child_100_test_case_12_walking_medium(tester):
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
                "goals": [[2.00, 5.30], [-2.39, 2.61], [4.99, -1.45], [-3.84, -0.95]],
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
                "init_x": -2.78,
                "init_y": 0.69,
                "init_a": 5.14,
                "velocity": 0.85,
                "goals": [[2.00, 5.30], [0.84, 4.48], [-6.44, -1.36], [1.01, -0.33]],
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
                "init_x": -2.07,
                "init_y": 4.67,
                "init_a": -122.78,
                "velocity": 0.95,
                "goals": [[0.82, 5.85], [-7.49, -5.96], [4.24, -5.86], [5.67, -4.54]],
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
                "init_x": -2.55,
                "init_y": 3.79,
                "init_a": -122.78,
                "velocity": 0.95,
                "goals": [[0.82, 5.85], [2.67, 1.66], [-0.00, 1.07], [4.97, 1.75]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.95, 3.92], [-2.11, 0.88], [1.97, 4.22], [-0.22, 0.72]],
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
                "init_x": 5.74,
                "init_y": 2.92,
                "init_a": 156.64,
                "velocity": 0.91,
                "goals": [[5.04, -0.76], [3.76, 4.14], [2.89, -3.33], [-0.29, -4.26]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.74, 4.69], [5.05, 2.98], [-2.87, 5.34], [0.09, 5.29]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.32, 3.00], [2.56, 2.35], [-3.50, -3.03], [0.34, -4.30]],
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

def tests_adult_0_child_100_test_case_13_stopped_medium(tester):
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
                "goals": [[-5.18, -3.02], [7.61, 1.72], [7.22, 2.28], [-7.02, 2.88]],
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
                "init_x": -4.97,
                "init_y": -4.00,
                "init_a": -128.85,
                "velocity": 1.14,
                "goals": [[-5.18, -3.02], [2.34, -3.43], [2.27, 3.25], [-5.40, -0.32]],
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
                "init_x": 2.06,
                "init_y": 1.40,
                "init_a": 106.78,
                "velocity": 0.88,
                "goals": [[2.06, 1.40], [-6.66, 5.37], [-2.32, 4.81], [4.20, 5.16]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[2.06, 1.40], [-0.37, 3.19], [3.92, 3.60], [4.40, 5.95]],
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
                "init_x": 6.06,
                "init_y": 5.55,
                "init_a": -17.15,
                "velocity": 1.03,
                "goals": [[-5.29, -3.55], [-3.80, 4.55], [-1.61, -2.98], [-5.86, 4.75]],
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
                "init_x": 6.92,
                "init_y": 1.84,
                "init_a": -51.62,
                "velocity": 1.18,
                "goals": [[6.06, 2.43], [-1.19, 2.02], [-7.59, 2.67], [6.08, 0.74]],
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
                "init_x": -5.44,
                "init_y": 0.03,
                "init_a": -80.05,
                "velocity": 0.87,
                "goals": [[-0.64, -2.50], [4.92, 1.26], [5.67, 2.64], [-2.94, -4.24]],
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
                "init_x": -5.01,
                "init_y": 3.68,
                "init_a": 126.76,
                "velocity": 1.20,
                "goals": [[4.85, 3.54], [7.69, -5.18], [-3.79, 0.18], [-1.57, 4.56]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.41, -2.51], [3.51, -4.08], [6.17, 3.62], [0.72, 3.77]],
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

def tests_adult_0_child_100_test_case_14_stopped_medium(tester):
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
                "goals": [[-3.73, -3.35], [-2.66, -1.21], [-4.50, -0.25], [5.74, -1.32]],
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
                "init_x": -3.90,
                "init_y": -2.36,
                "init_a": 136.60,
                "velocity": 1.16,
                "goals": [[-3.73, -3.35], [2.92, 4.06], [-0.61, 5.05], [-3.34, -0.44]],
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
                "init_x": -6.95,
                "init_y": 1.56,
                "init_a": 137.97,
                "velocity": 0.85,
                "goals": [[-6.95, 1.56], [0.60, 0.02], [3.73, 2.28], [4.41, 4.83]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.95, 1.56], [2.50, -2.17], [-2.88, -1.79], [0.44, -2.66]],
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
                "init_x": 1.40,
                "init_y": -5.05,
                "init_a": -153.73,
                "velocity": 1.03,
                "goals": [[-0.21, -3.28], [-6.98, -2.08], [4.80, -3.17], [-6.24, 4.33]],
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
                "init_x": -7.06,
                "init_y": -4.05,
                "init_a": 50.62,
                "velocity": 0.83,
                "goals": [[-0.68, -5.89], [-1.09, 3.11], [-0.35, -2.04], [-2.23, -4.06]],
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
                "init_x": -5.92,
                "init_y": 3.31,
                "init_a": 121.20,
                "velocity": 0.80,
                "goals": [[-4.43, -3.09], [7.14, -1.56], [4.02, 0.75], [3.10, 5.12]],
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
                "init_x": 3.47,
                "init_y": 1.08,
                "init_a": 55.72,
                "velocity": 1.16,
                "goals": [[-1.23, -4.95], [-5.47, -2.83], [-3.15, -2.82], [5.07, 4.75]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.14, -4.00], [6.81, 4.95], [-3.23, -5.65], [-1.93, -2.69]],
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

def tests_adult_0_child_100_test_case_15_walking_medium(tester):
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
                "goals": [[5.44, 1.84], [5.76, -0.71], [2.21, 0.67], [3.29, 3.84]],
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
                "init_x": 6.17,
                "init_y": 0.34,
                "init_a": 95.69,
                "velocity": 0.99,
                "goals": [[5.44, 1.84], [3.26, 1.38], [-2.27, 0.52], [7.66, -3.42]],
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
                "init_x": 5.92,
                "init_y": 4.80,
                "init_a": 2.99,
                "velocity": 0.99,
                "goals": [[-4.50, -1.26], [4.85, -3.07], [-7.45, 3.62], [-2.39, -2.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.50, -1.26], [-5.78, 5.04], [-6.19, -3.22], [3.76, 2.38]],
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
                "init_x": -7.58,
                "init_y": -4.36,
                "init_a": -55.50,
                "velocity": 0.91,
                "goals": [[1.80, 1.34], [-1.89, 3.53], [-1.53, 0.98], [7.73, 0.96]],
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
                "init_x": 0.78,
                "init_y": 3.56,
                "init_a": 174.91,
                "velocity": 1.09,
                "goals": [[-0.84, 0.85], [0.29, 2.46], [7.99, 1.79], [-1.59, -0.73]],
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
                "init_x": 2.30,
                "init_y": 1.15,
                "init_a": -105.55,
                "velocity": 1.01,
                "goals": [[-1.51, 1.04], [6.60, 2.30], [2.78, -5.51], [-7.80, 1.90]],
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
                "init_x": -5.11,
                "init_y": -4.95,
                "init_a": -58.14,
                "velocity": 1.14,
                "goals": [[3.89, 6.00], [-5.68, -2.74], [-4.07, 1.28], [-3.62, -1.46]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.48, -1.67], [2.86, 3.41], [3.62, -5.04], [2.82, 5.62]],
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

def tests_adult_0_child_100_test_case_16_stopped_medium(tester):
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
                "goals": [[-6.55, -5.05], [-2.23, 5.02], [-0.58, -5.68], [1.07, 5.47]],
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
                "init_x": -5.79,
                "init_y": -5.71,
                "init_a": -118.17,
                "velocity": 1.03,
                "goals": [[-6.55, -5.05], [6.55, -5.37], [5.32, 1.06], [7.64, -5.72]],
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
                "init_x": 1.69,
                "init_y": 0.82,
                "init_a": 171.03,
                "velocity": 1.16,
                "goals": [[1.69, 0.82], [-4.09, -1.98], [-6.68, -3.78], [-1.94, -3.25]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[1.69, 0.82], [-5.13, -2.62], [4.49, -0.39], [3.21, 1.90]],
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
                "init_x": -5.89,
                "init_y": -0.13,
                "init_a": -179.07,
                "velocity": 1.13,
                "goals": [[2.77, 5.66], [-0.92, 4.90], [1.32, 2.30], [-5.48, -3.63]],
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
                "init_x": 3.79,
                "init_y": 2.79,
                "init_a": -162.55,
                "velocity": 1.00,
                "goals": [[2.95, -1.66], [4.12, -5.78], [-0.55, -2.72], [3.82, -3.53]],
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
                "init_x": -2.95,
                "init_y": -5.65,
                "init_a": 13.28,
                "velocity": 0.85,
                "goals": [[-5.43, 0.96], [2.19, 1.87], [-3.99, -4.27], [1.46, 2.64]],
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
                "init_x": -3.30,
                "init_y": 0.16,
                "init_a": -27.20,
                "velocity": 1.11,
                "goals": [[-1.50, -3.07], [-5.93, 1.90], [0.15, -4.97], [-6.90, 3.71]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.46, -4.98], [-1.68, 2.71], [-0.21, -4.77], [3.02, 4.58]],
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

def tests_adult_0_child_100_test_case_17_walking_medium(tester):
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
                "goals": [[-6.97, -2.49], [-5.56, 0.94], [3.03, -5.52], [5.26, -5.46]],
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
                "init_x": -3.30,
                "init_y": -1.21,
                "init_a": -145.87,
                "velocity": 1.13,
                "goals": [[-6.97, -2.49], [-7.49, 4.36], [3.58, 4.20], [-6.09, 5.84]],
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
                "init_x": -4.76,
                "init_y": 3.02,
                "init_a": -13.14,
                "velocity": 0.90,
                "goals": [[-2.68, -5.19], [-5.36, -4.40], [7.94, 3.44], [-2.82, 1.22]],
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
                "init_x": -3.76,
                "init_y": 2.96,
                "init_a": -13.14,
                "velocity": 0.90,
                "goals": [[-2.68, -5.19], [-5.29, 3.69], [-2.98, 3.41], [0.98, 3.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.68, -0.28], [2.96, -0.22], [-7.82, 2.03], [-4.13, 5.89]],
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
                "init_x": 3.31,
                "init_y": 1.19,
                "init_a": 104.55,
                "velocity": 0.97,
                "goals": [[3.73, 2.84], [-1.88, 2.27], [6.84, -1.73], [3.51, -0.07]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.72, 3.98], [-4.64, -1.36], [7.48, 5.24], [6.49, -4.96]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.25, -3.98], [7.09, -2.16], [-5.41, -1.48], [-6.12, 4.55]],
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

def tests_adult_0_child_100_test_case_18_stopped_medium(tester):
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
                "goals": [[0.31, -0.60], [2.10, 1.22], [5.11, -2.70], [-5.00, 0.35]],
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
                "init_x": 0.95,
                "init_y": 0.17,
                "init_a": -84.34,
                "velocity": 1.01,
                "goals": [[0.31, -0.60], [3.02, 4.56], [-2.84, 0.23], [-7.73, 3.88]],
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
                "goals": [[3.35, 1.81], [7.89, 1.12], [1.02, -5.72], [6.25, 0.41]],
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
                "init_x": 2.36,
                "init_y": 1.96,
                "init_a": -69.12,
                "velocity": 0.81,
                "goals": [[3.35, 1.81], [5.90, -2.48], [0.69, -4.49], [-2.86, -2.47]],
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
                "init_x": -6.30,
                "init_y": 5.99,
                "init_a": 23.99,
                "velocity": 0.92,
                "goals": [[-3.81, -4.17], [5.27, 3.47], [-5.47, -0.89], [4.56, 1.55]],
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
                "init_x": 1.39,
                "init_y": 4.85,
                "init_a": 70.57,
                "velocity": 0.82,
                "goals": [[-5.02, 0.17], [4.90, -0.25], [-6.82, 1.43], [1.72, -4.68]],
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
                "init_x": 5.93,
                "init_y": 1.89,
                "init_a": 133.26,
                "velocity": 0.91,
                "goals": [[2.02, 4.88], [4.79, -3.69], [-3.86, 4.50], [0.08, -2.28]],
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

def tests_adult_0_child_100_test_case_19_stopped_medium(tester):
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
                "goals": [[3.29, -3.00], [-6.76, 5.12], [-1.77, -4.48], [-4.42, 0.17]],
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
                "init_x": 2.72,
                "init_y": -2.18,
                "init_a": -0.90,
                "velocity": 1.01,
                "goals": [[3.29, -3.00], [2.24, -4.66], [4.14, -0.05], [-2.05, -5.33]],
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
                "init_x": -1.04,
                "init_y": -5.18,
                "init_a": 1.05,
                "velocity": 0.83,
                "goals": [[-1.04, -5.18], [4.31, 4.54], [-1.63, 2.92], [-5.67, -1.81]],
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
                "init_x": -1.94,
                "init_y": -5.60,
                "init_a": 1.05,
                "velocity": 0.83,
                "goals": [[-1.04, -5.18], [-2.70, 4.26], [1.80, -0.50], [-1.19, -3.39]],
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
                "init_x": -3.39,
                "init_y": 5.43,
                "init_a": 0.93,
                "velocity": 1.18,
                "goals": [[6.62, 1.26], [7.43, -1.35], [-4.40, 1.94], [4.66, 0.17]],
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
                "init_x": 3.01,
                "init_y": -2.28,
                "init_a": 87.85,
                "velocity": 1.08,
                "goals": [[-1.40, 0.87], [6.13, -1.59], [-2.73, -5.70], [1.65, -1.67]],
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
                "init_x": 4.67,
                "init_y": 3.01,
                "init_a": 111.63,
                "velocity": 0.99,
                "goals": [[6.14, -5.92], [-0.28, 3.12], [-3.10, -1.73], [1.93, -5.10]],
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

def tests_adult_0_child_100_test_case_20_stopped_medium(tester):
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
                "goals": [[-1.01, -0.02], [-3.55, 0.41], [0.50, 5.83], [2.52, 4.42]],
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
                "init_x": -1.91,
                "init_y": -0.46,
                "init_a": 23.00,
                "velocity": 0.99,
                "goals": [[-1.01, -0.02], [-1.43, -2.71], [-0.18, 1.40], [-2.38, -4.00]],
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
                "init_x": 1.23,
                "init_y": -1.65,
                "init_a": 125.73,
                "velocity": 1.08,
                "goals": [[1.23, -1.65], [-2.36, -3.51], [0.53, 3.62], [3.61, 5.34]],
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
                "init_x": 1.15,
                "init_y": -0.65,
                "init_a": 125.73,
                "velocity": 1.08,
                "goals": [[1.23, -1.65], [-4.83, 2.86], [-4.21, 0.30], [3.12, 0.82]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.90, -1.13], [-2.38, 0.14], [5.71, 0.60], [-5.14, -3.81]],
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
                "init_x": -3.53,
                "init_y": 3.64,
                "init_a": 54.42,
                "velocity": 0.88,
                "goals": [[4.04, 3.23], [-7.00, 4.35], [-1.68, -1.87], [0.31, -4.56]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.74, 4.24], [-1.85, 1.91], [-3.34, -0.22], [-4.34, 2.22]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.91, 1.40], [-1.33, 0.09], [0.63, -4.30], [-1.12, -4.04]],
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

def tests_adult_0_child_100_test_case_21_stopped_medium(tester):
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
                "goals": [[4.30, 5.63], [3.33, 2.93], [0.66, -3.72], [-6.59, -2.67]],
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
                "init_x": 5.25,
                "init_y": 5.30,
                "init_a": 38.92,
                "velocity": 0.90,
                "goals": [[4.30, 5.63], [3.50, -0.22], [-1.97, 3.84], [5.39, 4.85]],
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
                "init_x": 6.83,
                "init_y": -5.95,
                "init_a": 44.68,
                "velocity": 0.92,
                "goals": [[6.83, -5.95], [-7.22, 5.80], [-1.64, 1.10], [-4.00, -4.65]],
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
                "init_x": 7.19,
                "init_y": -5.01,
                "init_a": 44.68,
                "velocity": 0.92,
                "goals": [[6.83, -5.95], [3.06, -4.28], [-2.39, -0.44], [0.70, 3.40]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-7.17, 3.33], [2.63, -5.07], [-3.86, 0.03], [-7.68, 2.99]],
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
                "init_x": -4.96,
                "init_y": 1.86,
                "init_a": 1.30,
                "velocity": 1.15,
                "goals": [[-0.53, 1.73], [6.55, -3.33], [7.38, 1.06], [-7.14, -0.10]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.95, 4.01], [0.89, 3.84], [7.64, -3.04], [3.34, -4.18]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.26, -5.17], [6.31, 5.48], [4.88, -1.77], [0.01, -5.23]],
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

def tests_adult_0_child_100_test_case_22_walking_medium(tester):
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
                "goals": [[-0.12, -5.67], [7.71, -1.02], [-4.01, -0.23], [-2.06, -4.65]],
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
                "init_x": 1.10,
                "init_y": 2.08,
                "init_a": -62.86,
                "velocity": 1.02,
                "goals": [[-0.12, -5.67], [-7.07, -2.69], [1.05, -4.52], [4.88, 3.67]],
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
                "init_x": -1.89,
                "init_y": -5.67,
                "init_a": 91.46,
                "velocity": 1.08,
                "goals": [[-1.87, 1.12], [-5.85, -1.80], [0.24, 3.15], [-0.55, -5.83]],
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
                "init_x": -2.35,
                "init_y": -6.56,
                "init_a": 91.46,
                "velocity": 1.08,
                "goals": [[-1.87, 1.12], [1.85, 1.18], [6.00, 1.76], [-4.75, -2.31]],
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
                "init_x": -1.17,
                "init_y": 0.70,
                "init_a": -150.22,
                "velocity": 1.11,
                "goals": [[3.01, -0.47], [2.88, 5.21], [6.36, 2.13], [5.14, -1.70]],
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
                "init_x": -3.78,
                "init_y": -1.27,
                "init_a": -128.07,
                "velocity": 0.88,
                "goals": [[3.60, 1.88], [-5.67, 3.76], [7.24, 2.88], [6.64, 4.61]],
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
                "init_x": -0.28,
                "init_y": 3.41,
                "init_a": -123.21,
                "velocity": 1.03,
                "goals": [[6.26, -3.32], [7.85, 4.78], [-3.18, 1.26], [-6.35, -4.50]],
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

def tests_adult_0_child_100_test_case_23_walking_medium(tester):
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
                "goals": [[-2.58, 4.91], [-7.77, 4.52], [3.96, -5.94], [0.62, 4.29]],
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
                "init_x": 6.96,
                "init_y": 5.19,
                "init_a": -165.93,
                "velocity": 0.86,
                "goals": [[-2.58, 4.91], [7.30, -2.76], [0.96, 2.71], [-6.41, -1.88]],
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
                "init_x": 0.05,
                "init_y": 5.36,
                "init_a": -21.93,
                "velocity": 1.07,
                "goals": [[4.46, 0.65], [-0.15, 3.63], [3.13, 0.24], [-4.62, -1.28]],
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
                "init_x": 0.63,
                "init_y": 4.55,
                "init_a": -21.93,
                "velocity": 1.07,
                "goals": [[4.46, 0.65], [-7.29, 3.86], [-3.81, -5.48], [-5.88, 0.71]],
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
                "init_x": 3.57,
                "init_y": 1.19,
                "init_a": 162.94,
                "velocity": 1.19,
                "goals": [[6.19, -3.25], [-4.38, 0.56], [2.79, -4.25], [-0.44, 1.87]],
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
                "init_x": 4.09,
                "init_y": 3.93,
                "init_a": 95.03,
                "velocity": 1.03,
                "goals": [[-6.87, -3.96], [-4.28, 1.22], [0.16, 4.05], [2.35, -0.02]],
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
                "init_x": 2.07,
                "init_y": 2.94,
                "init_a": -137.50,
                "velocity": 0.92,
                "goals": [[1.23, -5.29], [6.62, -1.70], [4.78, -1.45], [7.76, -3.22]],
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

def tests_adult_0_child_100_test_case_24_walking_medium(tester):
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
                "goals": [[3.12, 0.02], [3.93, 3.68], [-2.59, 4.17], [-2.92, -2.65]],
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
                "init_x": 1.98,
                "init_y": -1.50,
                "init_a": -157.41,
                "velocity": 0.85,
                "goals": [[3.12, 0.02], [0.43, -1.49], [0.97, 1.41], [-5.06, 0.63]],
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
                "init_x": 2.15,
                "init_y": 5.85,
                "init_a": -93.65,
                "velocity": 0.93,
                "goals": [[-1.54, -1.08], [-6.36, 3.68], [-3.70, 4.09], [-1.27, 5.89]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.54, -1.08], [-4.26, -2.77], [6.95, -3.53], [3.07, 4.37]],
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
                "init_x": -0.01,
                "init_y": 5.22,
                "init_a": 28.78,
                "velocity": 1.05,
                "goals": [[6.34, 4.38], [6.31, 4.82], [7.94, 2.40], [-0.24, 3.67]],
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
                "init_x": 0.89,
                "init_y": -0.98,
                "init_a": 47.75,
                "velocity": 1.15,
                "goals": [[-7.70, 1.71], [-1.03, 4.35], [5.30, 4.89], [-3.15, 4.80]],
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
                "init_x": 5.49,
                "init_y": -1.87,
                "init_a": -14.55,
                "velocity": 0.91,
                "goals": [[-1.22, -4.48], [-4.47, -0.49], [-5.64, -0.47], [6.57, 5.12]],
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
                "init_x": -2.91,
                "init_y": 3.99,
                "init_a": -60.64,
                "velocity": 0.98,
                "goals": [[0.34, 0.72], [-7.94, -2.28], [-0.80, 5.90], [-0.95, 0.25]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.66, 1.79], [-7.15, 0.25], [0.55, 1.05], [3.57, -3.65]],
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

def tests_adult_0_child_100_test_case_25_stopped_medium(tester):
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
                "goals": [[1.98, 0.61], [0.03, 1.77], [-7.63, -0.57], [-0.96, 0.97]],
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
                "init_x": 1.50,
                "init_y": -0.27,
                "init_a": 119.93,
                "velocity": 0.87,
                "goals": [[1.98, 0.61], [3.81, -5.23], [-1.60, 4.22], [7.33, -4.94]],
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
                "init_x": -0.45,
                "init_y": -2.13,
                "init_a": -100.62,
                "velocity": 0.84,
                "goals": [[-0.45, -2.13], [-4.75, 3.24], [-1.50, 5.42], [-1.42, -2.87]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.45, -2.13], [-1.38, -2.16], [0.34, -3.90], [0.38, 1.33]],
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
                "init_x": -3.83,
                "init_y": -3.75,
                "init_a": -121.25,
                "velocity": 1.04,
                "goals": [[-4.95, -2.61], [-7.26, 4.92], [-3.62, -5.21], [-4.60, -3.12]],
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
                "init_x": -4.26,
                "init_y": -5.25,
                "init_a": 99.04,
                "velocity": 0.84,
                "goals": [[-7.89, -4.06], [6.02, 4.87], [-6.07, -4.38], [2.19, -2.02]],
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
                "init_x": 7.23,
                "init_y": -1.59,
                "init_a": 46.51,
                "velocity": 0.89,
                "goals": [[5.63, -3.67], [-3.49, 5.58], [3.42, -0.13], [2.56, 5.13]],
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
                "init_x": -3.42,
                "init_y": 5.15,
                "init_a": -78.10,
                "velocity": 1.13,
                "goals": [[1.13, -2.94], [4.48, -4.36], [0.88, 3.48], [3.43, 4.21]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.59, -5.29], [2.57, 1.98], [-2.99, 0.88], [2.59, 0.46]],
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

def tests_adult_0_child_100_test_case_26_walking_medium(tester):
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
                "goals": [[-0.24, 1.05], [-3.94, -0.55], [-4.76, 4.71], [-5.96, 5.76]],
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
                "init_x": 7.22,
                "init_y": 1.88,
                "init_a": 22.56,
                "velocity": 0.92,
                "goals": [[-0.24, 1.05], [-2.68, -1.44], [-0.20, -0.41], [5.02, -2.41]],
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
                "init_x": 6.47,
                "init_y": -0.85,
                "init_a": -53.33,
                "velocity": 1.09,
                "goals": [[-0.05, 0.07], [-4.63, -1.13], [6.09, -5.36], [2.01, 4.56]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.05, 0.07], [1.80, 0.70], [3.25, 5.17], [3.91, 3.53]],
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
                "init_x": 4.99,
                "init_y": 0.11,
                "init_a": -69.24,
                "velocity": 0.98,
                "goals": [[6.90, -0.14], [2.31, -3.39], [0.75, 4.84], [-6.64, -0.91]],
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
                "init_x": 6.17,
                "init_y": 3.30,
                "init_a": 109.81,
                "velocity": 1.09,
                "goals": [[4.50, -5.67], [-0.93, -4.67], [6.40, 2.53], [-0.98, -3.42]],
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
                "init_x": -2.68,
                "init_y": -1.87,
                "init_a": -4.14,
                "velocity": 0.83,
                "goals": [[-5.08, 0.98], [-6.01, 1.91], [0.14, 0.05], [4.85, -3.42]],
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
                "init_x": -4.69,
                "init_y": 0.38,
                "init_a": -109.84,
                "velocity": 1.10,
                "goals": [[5.59, -2.72], [2.73, 2.83], [0.98, -1.60], [-5.44, 1.66]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.79, 0.87], [1.91, 0.97], [5.38, 2.92], [6.49, 0.57]],
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

def tests_adult_0_child_100_test_case_27_walking_medium(tester):
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
                "goals": [[2.09, -0.92], [-1.92, -4.44], [-6.39, 4.09], [4.30, -5.17]],
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
                "init_x": -3.67,
                "init_y": 6.18,
                "init_a": -174.00,
                "velocity": 0.93,
                "goals": [[2.09, -0.92], [-6.10, -5.43], [-1.00, -5.38], [5.64, -3.36]],
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
                "init_x": -1.36,
                "init_y": 5.49,
                "init_a": 64.23,
                "velocity": 1.17,
                "goals": [[-2.03, -4.57], [5.07, -1.40], [3.11, 3.55], [-4.53, -3.53]],
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
                "init_x": -1.62,
                "init_y": 4.53,
                "init_a": 64.23,
                "velocity": 1.17,
                "goals": [[-2.03, -4.57], [7.01, 4.45], [-1.81, -0.73], [0.03, 2.44]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.11, 2.08], [1.06, 3.31], [-6.13, 5.31], [2.28, -0.18]],
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
                "init_x": -2.54,
                "init_y": -2.62,
                "init_a": 121.59,
                "velocity": 0.97,
                "goals": [[-3.08, 0.68], [-7.17, 0.46], [-7.41, 5.79], [-6.07, 4.29]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.06, 3.71], [-2.34, -3.80], [7.59, -2.58], [-4.86, 0.94]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.70, 0.17], [-4.32, -3.81], [4.42, -0.05], [4.18, 2.19]],
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

def tests_adult_0_child_100_test_case_28_walking_medium(tester):
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
                "goals": [[7.97, -3.35], [-7.64, -5.33], [-6.76, 2.42], [-6.64, -2.17]],
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
                "init_x": 4.93,
                "init_y": 2.82,
                "init_a": -131.64,
                "velocity": 1.17,
                "goals": [[7.97, -3.35], [2.43, 0.73], [-4.19, -2.71], [2.45, 2.48]],
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
                "init_x": -0.31,
                "init_y": -3.52,
                "init_a": 0.96,
                "velocity": 0.84,
                "goals": [[7.13, 3.79], [-5.23, -3.46], [1.12, 0.84], [1.25, 4.91]],
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
                "init_x": 0.23,
                "init_y": -2.68,
                "init_a": 0.96,
                "velocity": 0.84,
                "goals": [[7.13, 3.79], [-2.25, 1.19], [6.61, 5.95], [-0.79, 5.36]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.91, 0.30], [1.17, -0.87], [7.08, 1.94], [-3.21, 3.17]],
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
                "init_x": 4.61,
                "init_y": 4.57,
                "init_a": -31.19,
                "velocity": 1.02,
                "goals": [[6.86, -1.43], [-0.13, 2.78], [-7.22, 4.17], [-6.33, -2.63]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.69, 2.62], [4.15, -5.09], [-0.68, -0.24], [-0.57, -1.15]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.27, 5.21], [-4.96, 4.11], [-4.07, 2.42], [3.57, 1.50]],
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

def tests_adult_0_child_100_test_case_29_walking_medium(tester):
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
                "goals": [[-4.89, 0.56], [5.89, -3.26], [4.82, 0.07], [-6.08, -1.22]],
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
                "init_x": -0.79,
                "init_y": -3.87,
                "init_a": 107.23,
                "velocity": 0.97,
                "goals": [[-4.89, 0.56], [0.56, 4.53], [5.64, 3.38], [-5.32, -5.61]],
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
                "init_x": -7.30,
                "init_y": -0.17,
                "init_a": -176.10,
                "velocity": 0.99,
                "goals": [[2.76, 2.61], [-3.93, 2.73], [2.31, 4.81], [0.81, -3.44]],
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
                "goals": [[2.76, 2.61], [0.79, -3.12], [1.73, 0.53], [-3.77, 4.24]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.09, 2.16], [-2.90, 1.74], [5.01, -1.43], [6.23, -1.37]],
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
                "init_x": -7.25,
                "init_y": -3.25,
                "init_a": -91.75,
                "velocity": 0.82,
                "goals": [[-1.68, -1.92], [6.76, -4.93], [-1.07, 3.44], [0.03, 1.57]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.65, -0.63], [-0.26, -2.61], [4.07, 2.81], [5.60, -0.75]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.85, 3.41], [0.65, -5.96], [2.17, -3.99], [3.26, 3.49]],
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

def tests_adult_0_child_100_test_case_30_stopped_medium(tester):
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
                "goals": [[-2.44, -0.34], [-6.83, -3.56], [0.75, 3.37], [-1.30, -3.42]],
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
                "init_x": -2.67,
                "init_y": -1.31,
                "init_a": 56.83,
                "velocity": 1.19,
                "goals": [[-2.44, -0.34], [5.73, -4.88], [2.02, -1.93], [0.15, 1.25]],
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
                "init_x": 0.25,
                "init_y": -2.83,
                "init_a": 43.47,
                "velocity": 1.10,
                "goals": [[0.25, -2.83], [-2.91, -4.75], [5.56, 1.25], [-1.25, -4.17]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.25, -2.83], [-0.09, -2.32], [4.61, 1.41], [7.70, -1.94]],
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
                "init_x": -6.36,
                "init_y": 4.99,
                "init_a": 154.58,
                "velocity": 1.03,
                "goals": [[-7.14, 5.87], [6.16, -0.59], [4.39, 1.08], [-0.48, 2.13]],
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
                "init_x": -1.63,
                "init_y": 5.94,
                "init_a": -85.45,
                "velocity": 1.09,
                "goals": [[0.18, -3.26], [0.51, 2.06], [-0.29, 1.55], [-5.53, 5.15]],
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
                "init_x": -0.44,
                "init_y": -5.91,
                "init_a": -77.71,
                "velocity": 1.05,
                "goals": [[-0.62, 2.05], [1.59, -2.25], [-0.30, -4.13], [4.92, -2.98]],
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
                "init_x": -2.52,
                "init_y": 2.61,
                "init_a": 135.16,
                "velocity": 0.83,
                "goals": [[6.23, 1.80], [7.01, 3.26], [2.62, 0.78], [4.13, -3.87]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.39, -0.36], [3.35, -3.43], [-3.84, -0.28], [-1.78, 1.37]],
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

def tests_adult_0_child_100_test_case_31_stopped_medium(tester):
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
                "goals": [[7.04, -2.91], [-3.18, 1.51], [-7.73, -4.79], [0.15, -1.66]],
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
                "init_x": 7.24,
                "init_y": -3.89,
                "init_a": 172.22,
                "velocity": 1.19,
                "goals": [[7.04, -2.91], [-1.30, -0.18], [-0.44, -3.87], [-1.60, -4.58]],
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
                "init_x": 1.59,
                "init_y": -2.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goals": [[1.59, -2.71], [0.75, 1.48], [2.37, 0.12], [-2.09, -1.74]],
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
                "init_x": 1.52,
                "init_y": -1.71,
                "init_a": 164.25,
                "velocity": 1.13,
                "goals": [[1.59, -2.71], [0.12, 4.48], [3.84, 0.17], [-7.24, -5.88]],
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
                "init_x": 5.07,
                "init_y": -4.24,
                "init_a": -170.53,
                "velocity": 1.13,
                "goals": [[3.23, 2.89], [-1.40, -2.68], [4.75, -5.26], [-0.69, 4.29]],
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
                "init_x": 0.46,
                "init_y": 4.54,
                "init_a": -162.87,
                "velocity": 1.13,
                "goals": [[7.42, -5.70], [4.07, 5.77], [3.60, 5.55], [4.39, -6.00]],
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
                "init_x": -7.13,
                "init_y": -3.15,
                "init_a": 52.77,
                "velocity": 0.84,
                "goals": [[-6.95, 1.01], [6.86, -4.32], [2.54, -1.83], [-2.48, 4.64]],
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

def tests_adult_0_child_100_test_case_32_stopped_medium(tester):
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
                "goals": [[-7.93, 5.89], [5.71, -2.96], [-6.49, 0.50], [6.31, 4.19]],
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
                "init_x": -8.69,
                "init_y": 6.54,
                "init_a": 65.90,
                "velocity": 0.94,
                "goals": [[-7.93, 5.89], [-2.35, -5.34], [6.28, 2.96], [-4.88, 1.75]],
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
                "init_x": 7.53,
                "init_y": 4.18,
                "init_a": -95.26,
                "velocity": 0.95,
                "goals": [[7.53, 4.18], [0.92, 3.04], [3.00, 1.98], [4.84, -1.83]],
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
                "init_x": 7.63,
                "init_y": 3.18,
                "init_a": -95.26,
                "velocity": 0.95,
                "goals": [[7.53, 4.18], [2.99, 2.74], [5.19, -1.38], [7.75, -1.34]],
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
                "init_x": -4.20,
                "init_y": 4.82,
                "init_a": -92.35,
                "velocity": 1.14,
                "goals": [[2.00, 0.65], [-0.09, -0.98], [1.16, -3.22], [3.59, -5.41]],
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
                "init_x": 4.64,
                "init_y": 5.70,
                "init_a": 46.97,
                "velocity": 0.91,
                "goals": [[-1.01, -4.80], [-4.06, 5.88], [2.18, -2.50], [-4.53, 4.10]],
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
                "init_x": -6.42,
                "init_y": -5.74,
                "init_a": 120.10,
                "velocity": 0.97,
                "goals": [[-6.88, 3.92], [1.21, 5.73], [-6.17, 2.42], [1.46, -0.68]],
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

def tests_adult_0_child_100_test_case_33_stopped_medium(tester):
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
                "goals": [[6.00, 5.47], [-7.50, -2.92], [-4.59, 1.02], [4.32, 5.01]],
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
                "init_x": 6.38,
                "init_y": 6.39,
                "init_a": -77.91,
                "velocity": 1.04,
                "goals": [[6.00, 5.47], [6.37, -5.61], [-7.35, 0.32], [-7.02, 1.20]],
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
                "init_x": 5.78,
                "init_y": 4.94,
                "init_a": -171.83,
                "velocity": 0.87,
                "goals": [[5.78, 4.94], [-2.86, -1.13], [0.70, -2.10], [-6.14, -3.43]],
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
                "init_x": 5.41,
                "init_y": 4.02,
                "init_a": -171.83,
                "velocity": 0.87,
                "goals": [[5.78, 4.94], [7.52, -1.48], [-3.37, 1.76], [5.57, -1.86]],
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
                "init_x": 2.38,
                "init_y": -3.94,
                "init_a": -33.94,
                "velocity": 1.07,
                "goals": [[6.19, 4.16], [2.45, -1.37], [3.42, 5.49], [-3.40, -0.77]],
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
                "init_x": 2.94,
                "init_y": 0.43,
                "init_a": 164.76,
                "velocity": 0.89,
                "goals": [[5.30, -5.56], [2.43, -4.03], [1.84, -1.97], [1.67, -1.40]],
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
                "init_x": -5.46,
                "init_y": 1.94,
                "init_a": -98.62,
                "velocity": 1.08,
                "goals": [[-5.18, 5.63], [5.95, 2.94], [2.55, 5.14], [-3.07, -2.53]],
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

def tests_adult_0_child_100_test_case_34_walking_medium(tester):
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
                "goals": [[-5.18, -5.55], [-4.24, 4.22], [3.20, 2.50], [-0.43, 3.86]],
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
                "init_x": -4.20,
                "init_y": 0.80,
                "init_a": -177.86,
                "velocity": 0.96,
                "goals": [[-5.18, -5.55], [-2.13, 3.95], [-1.24, -1.38], [6.81, -1.88]],
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
                "init_x": 2.93,
                "init_y": 0.83,
                "init_a": 172.32,
                "velocity": 0.81,
                "goals": [[-4.45, 4.16], [4.40, 5.41], [4.77, 4.52], [-5.25, -2.64]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.45, 4.16], [7.10, -2.37], [-7.71, -2.23], [0.76, -0.27]],
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
                "init_x": -6.75,
                "init_y": 1.31,
                "init_a": 14.10,
                "velocity": 0.98,
                "goals": [[-7.44, 5.58], [4.64, -4.81], [2.52, 1.81], [6.16, -1.03]],
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
                "init_x": -0.58,
                "init_y": -2.74,
                "init_a": -95.95,
                "velocity": 1.05,
                "goals": [[6.34, 4.21], [-5.46, -1.54], [5.32, 0.54], [-2.09, 4.88]],
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
                "init_x": 2.99,
                "init_y": -5.51,
                "init_a": 147.11,
                "velocity": 1.08,
                "goals": [[-0.71, -1.26], [4.92, -3.91], [2.96, -5.89], [-0.35, -5.35]],
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
                "init_x": 1.41,
                "init_y": -1.12,
                "init_a": 24.71,
                "velocity": 0.98,
                "goals": [[6.40, -4.51], [7.86, -0.38], [1.71, -0.90], [4.11, 3.69]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.91, -1.61], [7.85, -2.26], [-0.15, -3.37], [-1.09, -1.97]],
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

def tests_adult_0_child_100_test_case_35_stopped_medium(tester):
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
                "goals": [[4.61, 0.41], [4.91, -1.73], [4.46, -5.12], [-0.76, -2.50]],
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
                "init_x": 4.91,
                "init_y": -0.54,
                "init_a": 131.72,
                "velocity": 1.13,
                "goals": [[4.61, 0.41], [-0.66, 4.94], [-6.32, -5.93], [-0.31, -4.94]],
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
                "init_x": -5.93,
                "init_y": 3.07,
                "init_a": 83.88,
                "velocity": 0.93,
                "goals": [[-5.93, 3.07], [-4.69, 2.16], [3.94, 2.35], [-3.55, 1.53]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.93, 3.07], [-4.46, 5.66], [-0.44, 5.80], [4.17, -5.25]],
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
                "init_x": 4.96,
                "init_y": -5.06,
                "init_a": 104.22,
                "velocity": 1.04,
                "goals": [[1.45, -4.68], [6.52, 2.83], [5.41, -0.86], [3.49, -5.72]],
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
                "init_x": 6.86,
                "init_y": 2.78,
                "init_a": 151.85,
                "velocity": 0.90,
                "goals": [[0.18, -5.87], [6.64, -0.51], [7.98, 5.62], [-2.51, -3.32]],
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
                "init_x": -3.53,
                "init_y": 0.65,
                "init_a": -46.43,
                "velocity": 1.12,
                "goals": [[2.38, -3.10], [-7.04, -1.00], [-5.36, -0.36], [5.71, 0.89]],
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
                "init_x": -6.06,
                "init_y": -5.40,
                "init_a": -123.89,
                "velocity": 1.13,
                "goals": [[-4.51, -1.89], [-2.96, 0.38], [4.12, 3.85], [-3.05, 0.81]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-7.93, 4.58], [-5.09, -2.35], [-3.49, 4.64], [4.23, -4.10]],
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

def tests_adult_0_child_100_test_case_36_stopped_medium(tester):
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
                "goals": [[-3.70, -3.49], [-5.88, -0.54], [-0.79, 4.45], [-1.74, 5.82]],
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
                "init_x": -3.23,
                "init_y": -2.61,
                "init_a": 63.37,
                "velocity": 1.05,
                "goals": [[-3.70, -3.49], [-6.29, 3.99], [-5.47, 5.81], [4.10, 1.41]],
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
                "init_x": -4.13,
                "init_y": 5.07,
                "init_a": -14.49,
                "velocity": 0.86,
                "goals": [[-4.13, 5.07], [0.33, 3.15], [1.65, -1.27], [4.80, 4.50]],
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
                "init_x": -3.71,
                "init_y": 5.98,
                "init_a": -14.49,
                "velocity": 0.86,
                "goals": [[-4.13, 5.07], [-6.06, 0.05], [2.87, 0.76], [2.89, 0.40]],
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
                "init_x": -3.07,
                "init_y": 3.47,
                "init_a": 142.87,
                "velocity": 1.03,
                "goals": [[2.08, 4.91], [1.93, 5.17], [0.92, -0.78], [-4.67, -0.31]],
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
                "init_x": -3.22,
                "init_y": -5.94,
                "init_a": 133.05,
                "velocity": 1.16,
                "goals": [[3.43, -1.16], [-2.85, -2.72], [-6.75, 1.81], [3.60, 3.31]],
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
                "init_x": 1.74,
                "init_y": -1.65,
                "init_a": -85.48,
                "velocity": 0.93,
                "goals": [[-0.60, 3.76], [3.35, 0.35], [-7.29, 2.14], [7.22, 1.43]],
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

def tests_adult_0_child_100_test_case_37_stopped_medium(tester):
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
                "goals": [[-5.02, 0.90], [6.78, 0.68], [4.98, 4.41], [-6.64, -5.50]],
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
                "init_x": -5.46,
                "init_y": 0.00,
                "init_a": 102.07,
                "velocity": 1.19,
                "goals": [[-5.02, 0.90], [1.75, 4.97], [7.25, -1.06], [4.29, -2.96]],
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
                "init_x": -7.28,
                "init_y": 2.94,
                "init_a": 59.29,
                "velocity": 0.97,
                "goals": [[-7.28, 2.94], [1.34, -4.66], [7.86, -1.27], [4.28, -5.17]],
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
                "init_x": -6.35,
                "init_y": 2.58,
                "init_a": 59.29,
                "velocity": 0.97,
                "goals": [[-7.28, 2.94], [5.80, -5.94], [4.43, -2.03], [1.71, -1.25]],
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
                "init_x": 7.76,
                "init_y": -2.95,
                "init_a": -30.08,
                "velocity": 1.03,
                "goals": [[-6.22, -1.79], [4.84, -2.23], [-7.52, -2.78], [-5.55, 2.18]],
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
                "init_x": -5.77,
                "init_y": -1.62,
                "init_a": -60.49,
                "velocity": 0.85,
                "goals": [[-2.91, 4.58], [7.79, 3.73], [-2.83, 0.51], [-3.39, 1.76]],
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
                "init_x": 0.52,
                "init_y": 0.25,
                "init_a": 126.27,
                "velocity": 0.88,
                "goals": [[-7.76, -1.67], [6.92, -4.34], [7.70, 1.30], [-7.28, -5.36]],
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

def tests_adult_0_child_100_test_case_38_stopped_medium(tester):
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
                "goals": [[4.49, 2.40], [-4.64, 5.95], [4.78, -4.00], [-1.22, -1.72]],
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
                "init_x": 5.04,
                "init_y": 3.23,
                "init_a": -69.41,
                "velocity": 1.05,
                "goals": [[4.49, 2.40], [-4.90, -0.53], [-0.77, -6.00], [-5.97, 3.00]],
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
                "init_x": 5.17,
                "init_y": 1.61,
                "init_a": 172.86,
                "velocity": 1.16,
                "goals": [[5.17, 1.61], [-3.78, -4.14], [-1.84, 0.98], [-3.86, -2.97]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.17, 1.61], [-2.40, -4.88], [1.06, 4.85], [5.68, -2.91]],
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
                "goals": [[0.17, 1.64], [3.07, 3.80], [7.64, -4.21], [0.39, -3.54]],
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
                "init_x": -1.18,
                "init_y": 2.18,
                "init_a": 59.07,
                "velocity": 0.99,
                "goals": [[2.19, -4.41], [7.08, -4.27], [-7.49, 1.37], [-1.46, 3.03]],
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
                "init_x": 4.68,
                "init_y": -2.56,
                "init_a": -100.52,
                "velocity": 0.92,
                "goals": [[1.65, 3.36], [-4.52, 1.63], [3.66, -2.83], [-0.19, -2.10]],
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
                "init_x": 7.46,
                "init_y": 2.45,
                "init_a": -39.44,
                "velocity": 0.87,
                "goals": [[-1.23, -5.42], [6.42, -3.85], [1.73, 0.99], [-3.10, -2.24]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.65, -3.74], [-4.75, 4.25], [5.25, -3.26], [-1.29, 5.58]],
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

def tests_adult_0_child_100_test_case_39_stopped_medium(tester):
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
                "goals": [[1.38, -2.33], [3.83, -0.20], [-3.45, -0.07], [7.07, -5.67]],
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
                "goals": [[1.38, -2.33], [1.35, 4.22], [-4.17, -0.93], [5.93, -0.36]],
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
                "init_x": 5.80,
                "init_y": -4.06,
                "init_a": -161.85,
                "velocity": 1.12,
                "goals": [[5.80, -4.06], [-4.85, 3.16], [5.56, -3.57], [2.14, 2.79]],
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
                "init_x": 4.82,
                "init_y": -3.83,
                "init_a": -161.85,
                "velocity": 1.12,
                "goals": [[5.80, -4.06], [-5.27, -1.23], [3.93, -1.39], [1.79, 1.17]],
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
                "init_x": -5.39,
                "init_y": -1.09,
                "init_a": 97.81,
                "velocity": 1.15,
                "goals": [[-4.19, -4.10], [-5.56, 2.19], [-2.29, -4.15], [-4.30, 3.41]],
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
                "init_x": 2.84,
                "init_y": 1.30,
                "init_a": 153.54,
                "velocity": 0.90,
                "goals": [[0.87, 1.56], [-3.91, -0.92], [0.32, 0.47], [7.09, 0.21]],
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
                "init_x": 6.97,
                "init_y": 3.81,
                "init_a": 49.81,
                "velocity": 1.20,
                "goals": [[-5.73, -1.01], [4.54, -4.76], [-5.66, 1.13], [-4.08, -0.99]],
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

def tests_adult_0_child_100_test_case_40_stopped_medium(tester):
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
                "goals": [[6.87, 3.87], [-6.23, -4.84], [-1.72, 0.23], [-3.83, 3.11]],
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
                "init_x": 6.65,
                "init_y": 2.89,
                "init_a": -124.82,
                "velocity": 1.11,
                "goals": [[6.87, 3.87], [4.35, 2.61], [-5.95, 2.04], [7.12, -1.55]],
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
                "init_x": -6.86,
                "init_y": 1.23,
                "init_a": 59.64,
                "velocity": 1.16,
                "goals": [[-6.86, 1.23], [7.10, -1.37], [-3.46, 3.31], [0.07, 0.35]],
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
                "init_x": -6.40,
                "init_y": 2.12,
                "init_a": 59.64,
                "velocity": 1.16,
                "goals": [[-6.86, 1.23], [4.35, -2.92], [1.74, -1.72], [1.35, 4.15]],
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
                "init_x": 5.32,
                "init_y": 1.57,
                "init_a": -165.67,
                "velocity": 1.03,
                "goals": [[-6.97, -5.11], [-2.57, 0.95], [-3.92, -1.20], [1.34, -2.01]],
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
                "init_x": 6.78,
                "init_y": 5.11,
                "init_a": -139.55,
                "velocity": 0.82,
                "goals": [[-4.59, 2.13], [-4.87, -1.59], [6.62, -1.27], [-4.38, 5.93]],
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
                "init_x": -0.61,
                "init_y": -3.01,
                "init_a": 29.82,
                "velocity": 0.89,
                "goals": [[-6.62, -4.67], [-0.44, 3.63], [-7.65, 3.28], [3.14, -1.60]],
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

def tests_adult_0_child_100_test_case_41_walking_medium(tester):
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
                "goals": [[-7.88, 3.45], [4.47, 3.92], [0.89, 5.88], [-7.81, -0.00]],
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
                "init_x": 2.82,
                "init_y": -3.50,
                "init_a": -132.76,
                "velocity": 0.88,
                "goals": [[-7.88, 3.45], [-6.65, 4.69], [7.12, -0.94], [-3.33, -4.09]],
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
                "init_x": 5.60,
                "init_y": 0.49,
                "init_a": -8.20,
                "velocity": 0.96,
                "goals": [[-2.82, -2.38], [2.37, 2.86], [6.37, 0.95], [4.85, -0.12]],
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
                "init_x": 4.73,
                "init_y": -0.01,
                "init_a": -8.20,
                "velocity": 0.96,
                "goals": [[-2.82, -2.38], [0.52, 2.99], [0.04, 1.60], [3.19, 5.62]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.38, -4.28], [-3.40, 1.65], [-4.97, 5.88], [3.85, 0.16]],
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
                "init_x": -5.49,
                "init_y": -5.63,
                "init_a": 28.58,
                "velocity": 1.16,
                "goals": [[5.00, -2.36], [-6.95, -0.09], [4.14, 2.74], [5.15, 1.93]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.49, -2.47], [-3.57, -3.73], [-7.60, 2.23], [-0.47, -5.59]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.50, -1.06], [-5.32, 0.01], [7.53, -1.92], [-2.40, -0.48]],
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

def tests_adult_0_child_100_test_case_42_walking_medium(tester):
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
                "goals": [[2.37, 5.22], [1.10, -1.25], [-6.39, 2.81], [5.00, -3.30]],
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
                "init_x": -3.48,
                "init_y": -2.71,
                "init_a": -52.09,
                "velocity": 0.92,
                "goals": [[2.37, 5.22], [-5.59, -5.21], [4.44, 0.36], [4.66, 0.59]],
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
                "init_x": -6.76,
                "init_y": 1.80,
                "init_a": -27.05,
                "velocity": 1.08,
                "goals": [[7.65, 3.40], [-5.39, 2.02], [6.57, -2.90], [-1.36, 0.62]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.65, 3.40], [0.34, -3.60], [-1.97, -3.23], [3.97, 5.03]],
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
                "goals": [[-7.48, -1.91], [-2.56, -2.93], [-7.73, -1.90], [5.02, 3.27]],
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
                "init_x": 5.73,
                "init_y": -5.77,
                "init_a": -153.13,
                "velocity": 1.06,
                "goals": [[-2.54, 4.18], [-0.29, -4.15], [1.21, 1.78], [4.51, -0.02]],
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
                "init_x": -3.68,
                "init_y": -5.24,
                "init_a": 11.42,
                "velocity": 1.14,
                "goals": [[-5.03, 2.17], [-0.80, 2.80], [-3.84, 5.07], [-3.84, 4.54]],
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
                "init_x": 2.76,
                "init_y": 0.58,
                "init_a": 88.37,
                "velocity": 1.09,
                "goals": [[4.50, 4.39], [-4.83, -3.53], [0.95, 5.12], [-5.04, -4.57]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.23, 5.54], [0.24, 2.16], [-1.99, 4.60], [6.33, 0.69]],
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

def tests_adult_0_child_100_test_case_43_stopped_medium(tester):
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
                "goals": [[6.02, 3.19], [5.14, 4.54], [1.38, 1.93], [7.39, -4.18]],
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
                "init_x": 6.94,
                "init_y": 2.81,
                "init_a": -51.83,
                "velocity": 0.92,
                "goals": [[6.02, 3.19], [3.19, 3.15], [0.83, -0.84], [7.13, -1.99]],
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
                "init_x": 0.74,
                "init_y": 5.47,
                "init_a": -5.71,
                "velocity": 0.88,
                "goals": [[0.74, 5.47], [2.18, -4.55], [-2.36, 3.22], [4.71, -0.54]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.74, 5.47], [-1.40, 0.85], [1.62, -3.48], [-6.38, 1.09]],
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
                "init_x": -2.72,
                "init_y": -5.79,
                "init_a": 143.12,
                "velocity": 0.98,
                "goals": [[-1.83, -2.08], [7.37, -0.55], [6.51, -1.38], [3.65, 1.90]],
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
                "init_x": -2.48,
                "init_y": -3.08,
                "init_a": -120.31,
                "velocity": 0.82,
                "goals": [[4.02, 1.16], [2.01, 3.84], [6.48, 1.26], [-1.14, 4.90]],
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
                "init_x": -1.02,
                "init_y": 0.17,
                "init_a": 116.73,
                "velocity": 0.88,
                "goals": [[4.59, 0.20], [1.93, -3.16], [7.63, 1.12], [0.33, 1.90]],
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
                "init_x": -7.00,
                "init_y": -5.59,
                "init_a": -19.32,
                "velocity": 1.06,
                "goals": [[7.41, -0.38], [7.58, -1.18], [4.53, 3.29], [3.24, 4.23]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-7.31, -3.40], [6.32, 2.83], [-5.18, 4.85], [-6.58, 5.21]],
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

def tests_adult_0_child_100_test_case_44_stopped_medium(tester):
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
                "goals": [[-2.76, -1.11], [4.46, 3.61], [-3.59, -5.36], [-3.48, -3.44]],
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
                "init_x": -2.09,
                "init_y": -0.36,
                "init_a": 62.70,
                "velocity": 1.02,
                "goals": [[-2.76, -1.11], [0.81, -0.80], [0.31, 2.88], [-7.50, -1.11]],
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
                "init_x": 7.80,
                "init_y": 3.42,
                "init_a": 104.72,
                "velocity": 0.88,
                "goals": [[7.80, 3.42], [7.24, -3.71], [-2.47, -0.17], [5.32, 2.51]],
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
                "init_x": 7.27,
                "init_y": 4.27,
                "init_a": 104.72,
                "velocity": 0.88,
                "goals": [[7.80, 3.42], [-5.05, 3.68], [4.90, 4.44], [7.94, 3.98]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.59, -0.02], [6.14, 4.95], [0.43, 3.87], [-6.83, 1.24]],
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
                "init_x": -7.63,
                "init_y": -1.61,
                "init_a": -32.34,
                "velocity": 1.05,
                "goals": [[4.79, 1.72], [1.51, 4.16], [0.35, 4.54], [-2.64, -4.26]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.39, 1.06], [-5.30, 3.22], [1.96, 0.16], [-4.26, -1.10]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-7.51, 0.54], [-6.22, 2.93], [-2.33, -2.52], [3.70, -4.77]],
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

def tests_adult_0_child_100_test_case_45_stopped_medium(tester):
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
                "goals": [[3.96, -5.80], [0.84, -1.26], [-7.01, -4.76], [2.64, 3.85]],
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
                "init_x": 4.59,
                "init_y": -6.57,
                "init_a": -169.11,
                "velocity": 1.17,
                "goals": [[3.96, -5.80], [-0.17, -4.39], [-2.21, -2.31], [-3.30, -3.54]],
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
                "init_x": 3.60,
                "init_y": -3.44,
                "init_a": 101.70,
                "velocity": 1.03,
                "goals": [[3.60, -3.44], [3.31, -1.28], [-4.03, 1.91], [1.45, 4.47]],
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
                "init_x": 2.66,
                "init_y": -3.11,
                "init_a": 101.70,
                "velocity": 1.03,
                "goals": [[3.60, -3.44], [-6.71, 4.79], [-3.14, 1.02], [-7.95, -3.34]],
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
                "init_x": -3.42,
                "init_y": -0.42,
                "init_a": -141.34,
                "velocity": 0.91,
                "goals": [[-2.67, -1.91], [-1.94, 2.14], [-7.65, -2.40], [5.76, 2.65]],
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
                "init_x": 1.31,
                "init_y": -1.38,
                "init_a": -167.79,
                "velocity": 0.93,
                "goals": [[7.50, 3.94], [-0.41, 5.42], [-0.12, 5.65], [-7.96, 2.72]],
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
                "init_x": -4.51,
                "init_y": 4.39,
                "init_a": -91.71,
                "velocity": 1.11,
                "goals": [[-0.77, 3.00], [2.18, -4.30], [4.42, -3.39], [-6.06, -3.52]],
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

def tests_adult_0_child_100_test_case_46_walking_medium(tester):
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
                "goals": [[-5.50, 3.69], [-3.37, 4.15], [-4.40, -4.16], [4.22, -1.67]],
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
                "init_x": 6.33,
                "init_y": 0.08,
                "init_a": 19.67,
                "velocity": 1.01,
                "goals": [[-5.50, 3.69], [-7.37, 0.43], [3.46, -0.42], [-3.98, -3.60]],
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
                "init_x": 3.95,
                "init_y": -0.34,
                "init_a": 109.60,
                "velocity": 1.19,
                "goals": [[-3.89, -3.62], [-5.47, -4.61], [1.81, -2.35], [-0.03, 1.67]],
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
                "init_x": 3.02,
                "init_y": -0.70,
                "init_a": 109.60,
                "velocity": 1.19,
                "goals": [[-3.89, -3.62], [6.04, -4.66], [6.12, 5.39], [7.57, 2.25]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.28, -3.23], [-1.83, 1.16], [6.55, 1.32], [0.24, 0.11]],
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
                "init_x": -0.96,
                "init_y": 0.98,
                "init_a": 29.90,
                "velocity": 1.00,
                "goals": [[7.74, 2.67], [1.82, 2.55], [0.04, 1.00], [5.26, -4.29]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.70, -4.43], [-4.75, 1.78], [0.90, 2.37], [5.02, -2.53]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.64, -1.43], [-5.38, 3.55], [1.38, -0.42], [4.01, 0.49]],
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

def tests_adult_0_child_100_test_case_47_walking_medium(tester):
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
                "goals": [[5.38, -4.67], [-5.88, 2.49], [-6.45, -2.07], [3.36, 1.34]],
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
                "init_x": 1.52,
                "init_y": -1.39,
                "init_a": -83.82,
                "velocity": 0.96,
                "goals": [[5.38, -4.67], [-7.20, -0.66], [0.76, -1.38], [4.56, -4.34]],
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
                "init_x": -7.28,
                "init_y": -3.32,
                "init_a": -46.08,
                "velocity": 1.09,
                "goals": [[-6.61, 4.32], [3.84, -1.76], [-0.92, 4.13], [1.04, 3.00]],
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
                "init_x": -7.14,
                "init_y": -2.33,
                "init_a": -46.08,
                "velocity": 1.09,
                "goals": [[-6.61, 4.32], [-5.62, 5.19], [-3.93, 4.72], [-6.36, 1.91]],
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
                "init_x": 7.60,
                "init_y": -4.23,
                "init_a": 35.99,
                "velocity": 1.04,
                "goals": [[3.88, -5.23], [3.20, -5.77], [-2.68, -3.63], [3.78, 1.36]],
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
                "init_x": 3.75,
                "init_y": -3.94,
                "init_a": -10.22,
                "velocity": 0.89,
                "goals": [[2.55, -2.24], [-2.18, 1.38], [4.17, 4.22], [2.28, 5.87]],
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
                "init_x": 6.25,
                "init_y": -3.42,
                "init_a": -149.79,
                "velocity": 0.95,
                "goals": [[-3.71, 2.67], [4.94, 2.12], [3.23, 5.40], [0.48, 1.92]],
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

def tests_adult_0_child_100_test_case_48_walking_medium(tester):
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
                "goals": [[-5.16, -0.76], [-0.04, 1.28], [-4.82, -2.01], [-5.44, 2.71]],
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
                "init_x": 0.66,
                "init_y": 1.80,
                "init_a": -173.12,
                "velocity": 0.92,
                "goals": [[-5.16, -0.76], [2.17, -2.50], [4.82, 0.04], [2.52, -0.27]],
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
                "init_x": -2.65,
                "init_y": 3.64,
                "init_a": -7.87,
                "velocity": 1.15,
                "goals": [[4.57, 3.77], [5.96, 1.91], [6.24, 0.53], [0.12, -4.70]],
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
                "init_x": -1.82,
                "init_y": 3.09,
                "init_a": -7.87,
                "velocity": 1.15,
                "goals": [[4.57, 3.77], [2.55, -3.08], [-7.60, -4.61], [2.58, -5.85]],
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
                "init_x": -1.24,
                "init_y": 1.32,
                "init_a": -165.93,
                "velocity": 0.95,
                "goals": [[-6.77, -1.10], [-2.02, -2.09], [-2.27, 4.70], [-4.45, -4.19]],
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
                "init_x": 7.20,
                "init_y": -0.75,
                "init_a": 166.29,
                "velocity": 1.15,
                "goals": [[-1.18, -2.13], [-2.92, 0.91], [-0.44, -0.54], [4.36, 4.92]],
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
                "init_x": 2.21,
                "init_y": 0.14,
                "init_a": -27.55,
                "velocity": 1.20,
                "goals": [[3.86, -5.84], [4.64, 4.96], [-3.15, -0.02], [5.60, 5.04]],
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

def tests_adult_0_child_100_test_case_49_walking_medium(tester):
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
                "goals": [[4.09, 3.57], [0.03, -1.42], [-7.65, 1.03], [-0.02, -0.30]],
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
                "init_x": 5.26,
                "init_y": -0.57,
                "init_a": -56.41,
                "velocity": 0.89,
                "goals": [[4.09, 3.57], [6.39, 1.32], [6.24, -0.97], [3.63, -5.63]],
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
                "init_x": 3.19,
                "init_y": 4.24,
                "init_a": 34.19,
                "velocity": 1.07,
                "goals": [[4.49, 1.10], [0.29, -5.42], [5.17, 3.68], [-7.74, -1.39]],
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
                "init_x": 2.30,
                "init_y": 3.79,
                "init_a": 34.19,
                "velocity": 1.07,
                "goals": [[4.49, 1.10], [5.38, 0.13], [-4.62, 4.68], [-0.47, -3.00]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.51, 4.39], [-0.63, -5.57], [-4.21, -5.25], [-0.36, 4.28]],
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
                "init_x": -7.71,
                "init_y": -3.31,
                "init_a": -104.12,
                "velocity": 0.80,
                "goals": [[-0.45, 3.79], [-7.37, -4.97], [-5.27, 4.02], [-7.85, 1.66]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-7.70, 3.21], [-3.50, -4.02], [2.55, -3.37], [-4.46, 5.77]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.68, -4.74], [-2.40, -5.58], [2.24, -2.52], [-5.96, 0.75]],
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

def tests_adult_0_child_100_test_case_50_walking_medium(tester):
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
                "goals": [[0.27, -0.82], [-1.35, 2.35], [-7.14, -5.84], [-6.68, -3.10]],
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
                "init_x": 1.46,
                "init_y": 2.40,
                "init_a": -24.16,
                "velocity": 1.13,
                "goals": [[0.27, -0.82], [-4.09, 5.23], [4.05, 5.03], [2.78, -0.29]],
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
                "init_x": 5.74,
                "init_y": -1.10,
                "init_a": 12.39,
                "velocity": 0.85,
                "goals": [[3.12, 1.80], [-4.47, -2.75], [-6.68, 0.45], [0.80, 2.30]],
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
                "init_x": 4.81,
                "init_y": -0.74,
                "init_a": 12.39,
                "velocity": 0.85,
                "goals": [[3.12, 1.80], [-4.92, -2.03], [-0.22, -2.30], [3.52, 2.85]],
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
                "init_x": -7.02,
                "init_y": -4.37,
                "init_a": 126.79,
                "velocity": 0.89,
                "goals": [[-2.10, -4.06], [-6.63, -2.62], [-2.65, -2.28], [-6.27, -3.65]],
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
                "init_x": 3.36,
                "init_y": -5.69,
                "init_a": -61.18,
                "velocity": 1.13,
                "goals": [[0.01, -1.76], [-2.02, 5.62], [-5.39, -5.36], [-2.53, 1.48]],
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
                "init_x": 0.89,
                "init_y": 0.78,
                "init_a": -133.53,
                "velocity": 1.20,
                "goals": [[-3.98, -4.85], [0.96, 4.62], [-6.78, -4.23], [0.33, 1.29]],
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

def tests_adult_0_child_100_test_case_51_walking_medium(tester):
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
                "goals": [[-3.53, 3.45], [-1.78, -2.15], [-1.72, -5.34], [-7.93, -4.29]],
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
                "init_x": -5.30,
                "init_y": -2.34,
                "init_a": -32.01,
                "velocity": 0.85,
                "goals": [[-3.53, 3.45], [-0.91, -3.38], [5.95, -4.97], [-2.08, -1.58]],
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
                "init_x": 3.61,
                "init_y": 0.06,
                "init_a": 61.58,
                "velocity": 1.12,
                "goals": [[6.17, 2.16], [-1.69, 2.42], [-3.02, 2.47], [-5.78, 2.95]],
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
                "init_x": 3.67,
                "init_y": 1.06,
                "init_a": 61.58,
                "velocity": 1.12,
                "goals": [[6.17, 2.16], [5.67, 2.79], [3.68, -3.10], [6.85, -2.83]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.41, -4.56], [-5.76, 3.85], [6.54, 0.82], [4.37, -1.93]],
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
                "init_x": 0.35,
                "init_y": 1.25,
                "init_a": -72.79,
                "velocity": 0.80,
                "goals": [[-1.57, -3.14], [7.17, -5.00], [-0.87, 3.42], [-2.23, 5.98]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.97, -2.10], [-0.84, 1.73], [-3.25, 1.04], [-3.94, -1.66]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.83, -3.31], [-3.81, 4.53], [-5.86, -4.31], [-1.92, -3.95]],
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

def tests_adult_0_child_100_test_case_52_stopped_medium(tester):
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
                "goals": [[7.30, -0.08], [-3.57, 1.93], [1.87, 0.52], [5.51, -4.49]],
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
                "init_x": 8.28,
                "init_y": 0.13,
                "init_a": 58.20,
                "velocity": 0.83,
                "goals": [[7.30, -0.08], [3.34, -3.83], [6.97, -3.73], [-6.93, -0.86]],
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
                "init_x": -4.56,
                "init_y": -4.03,
                "init_a": -132.32,
                "velocity": 0.97,
                "goals": [[-4.56, -4.03], [5.41, -2.90], [-4.63, 3.70], [3.75, -4.05]],
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
                "init_x": -3.87,
                "init_y": -3.30,
                "init_a": -132.32,
                "velocity": 0.97,
                "goals": [[-4.56, -4.03], [-5.10, -4.80], [4.82, 5.31], [3.95, 1.17]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.39, 0.71], [1.07, -4.09], [-6.69, -3.59], [3.80, -3.08]],
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
                "init_x": -2.94,
                "init_y": -1.14,
                "init_a": -29.67,
                "velocity": 1.15,
                "goals": [[1.08, 5.17], [5.70, -1.28], [1.75, -0.60], [4.66, 2.73]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.40, -5.47], [-7.52, 2.62], [1.64, 2.18], [1.41, 0.17]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.88, -4.46], [4.21, -3.12], [-3.48, 5.62], [-2.83, -4.72]],
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

def tests_adult_0_child_100_test_case_53_stopped_medium(tester):
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
                "goals": [[-1.70, 4.83], [1.21, -1.70], [0.78, -0.86], [0.85, -1.10]],
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
                "goals": [[-1.70, 4.83], [3.93, -2.52], [-6.10, 2.08], [-6.03, -0.15]],
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
                "init_x": 0.75,
                "init_y": 0.98,
                "init_a": 22.73,
                "velocity": 0.92,
                "goals": [[0.75, 0.98], [2.65, -3.50], [7.36, 0.83], [-5.56, 3.10]],
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
                "init_x": 1.62,
                "init_y": 1.47,
                "init_a": 22.73,
                "velocity": 0.92,
                "goals": [[0.75, 0.98], [3.28, 0.22], [-3.76, -2.74], [-1.83, -4.34]],
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
                "init_x": 3.37,
                "init_y": 4.04,
                "init_a": -131.76,
                "velocity": 1.05,
                "goals": [[-1.15, -5.12], [1.40, -2.13], [-1.26, -4.76], [-0.65, 2.88]],
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
                "init_x": 0.62,
                "init_y": -0.05,
                "init_a": 16.07,
                "velocity": 1.13,
                "goals": [[-4.48, 4.09], [-1.35, 1.02], [6.36, 3.47], [-5.57, -0.97]],
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
                "init_x": -5.81,
                "init_y": -2.64,
                "init_a": 94.23,
                "velocity": 1.02,
                "goals": [[4.14, -2.26], [5.77, -3.85], [5.84, -3.26], [4.24, -5.13]],
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

def tests_adult_0_child_100_test_case_54_walking_medium(tester):
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
                "goals": [[3.98, 2.75], [-3.62, 4.63], [-5.91, -4.35], [-2.64, 4.80]],
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
                "init_x": -6.64,
                "init_y": 0.94,
                "init_a": 101.38,
                "velocity": 0.98,
                "goals": [[3.98, 2.75], [-0.56, -4.54], [6.67, 5.18], [-1.05, 3.24]],
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
                "init_x": -4.96,
                "init_y": -2.39,
                "init_a": 12.74,
                "velocity": 1.04,
                "goals": [[7.44, -4.34], [-1.68, 4.71], [-3.56, 3.04], [-0.87, 2.16]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.44, -4.34], [-0.85, -0.35], [7.61, -5.17], [-6.31, 0.15]],
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
                "init_x": -0.07,
                "init_y": 3.46,
                "init_a": -56.42,
                "velocity": 0.95,
                "goals": [[-2.62, 4.71], [2.32, 1.27], [2.76, -3.66], [-0.29, 0.13]],
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
                "init_x": 5.06,
                "init_y": 5.78,
                "init_a": -14.62,
                "velocity": 0.84,
                "goals": [[-3.58, 3.91], [1.22, 2.29], [2.08, 5.24], [-1.77, 5.65]],
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
                "init_x": -0.07,
                "init_y": -1.98,
                "init_a": -73.07,
                "velocity": 0.99,
                "goals": [[1.21, -0.99], [3.44, 4.18], [-3.93, 4.39], [-5.49, 0.36]],
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
                "init_x": 3.87,
                "init_y": -5.52,
                "init_a": 66.98,
                "velocity": 0.80,
                "goals": [[-2.75, -1.68], [-2.67, 1.21], [0.74, -4.06], [7.93, -0.29]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.25, -1.15], [-1.23, -5.89], [7.26, -5.09], [-4.08, -5.93]],
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

def tests_adult_0_child_100_test_case_55_walking_medium(tester):
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
                "goals": [[-1.34, -4.55], [-3.19, 4.51], [-2.52, -5.75], [-2.55, 2.22]],
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
                "init_x": 8.59,
                "init_y": -3.82,
                "init_a": -120.24,
                "velocity": 1.05,
                "goals": [[-1.34, -4.55], [2.37, -5.17], [2.92, 1.32], [4.14, -4.71]],
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
                "init_x": 0.59,
                "init_y": -4.57,
                "init_a": 45.41,
                "velocity": 1.09,
                "goals": [[1.47, -0.13], [4.34, 5.22], [0.20, -2.19], [-5.63, -5.26]],
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
                "init_x": 0.04,
                "init_y": -3.74,
                "init_a": 45.41,
                "velocity": 1.09,
                "goals": [[1.47, -0.13], [7.60, -3.27], [0.59, 1.07], [2.05, 3.62]],
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
                "init_x": -5.18,
                "init_y": 4.26,
                "init_a": 28.39,
                "velocity": 1.02,
                "goals": [[-6.21, 0.04], [4.28, 5.78], [3.63, 1.78], [-7.64, -4.34]],
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
                "init_x": 2.33,
                "init_y": -2.57,
                "init_a": -113.53,
                "velocity": 1.09,
                "goals": [[7.47, -0.66], [-3.74, 2.88], [-1.87, 4.39], [-1.67, -2.05]],
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
                "init_x": 0.49,
                "init_y": -4.05,
                "init_a": 162.45,
                "velocity": 0.89,
                "goals": [[-5.99, -4.82], [-6.91, 5.78], [-5.16, 3.15], [6.28, -3.28]],
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

def tests_adult_0_child_100_test_case_56_walking_medium(tester):
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
                "goals": [[-4.69, 0.44], [-4.52, -3.11], [-7.67, 1.82], [6.67, -4.41]],
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
                "init_x": -6.10,
                "init_y": 0.35,
                "init_a": -140.34,
                "velocity": 1.20,
                "goals": [[-4.69, 0.44], [0.72, -2.22], [6.23, -0.39], [0.74, 2.47]],
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
                "init_x": 5.17,
                "init_y": -1.24,
                "init_a": -143.67,
                "velocity": 0.83,
                "goals": [[2.94, -2.33], [4.60, -1.10], [-5.84, 3.90], [-4.63, -0.64]],
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
                "init_x": 4.23,
                "init_y": -1.59,
                "init_a": -143.67,
                "velocity": 0.83,
                "goals": [[2.94, -2.33], [-3.03, -1.81], [3.97, -3.31], [-4.08, 1.74]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.13, -0.44], [-0.60, -3.13], [-2.10, -2.73], [-4.91, -1.95]],
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
                "init_x": -3.66,
                "init_y": -4.57,
                "init_a": 106.14,
                "velocity": 1.08,
                "goals": [[0.23, -2.98], [0.69, 1.80], [-2.18, 0.04], [-7.38, -3.74]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.44, -1.66], [2.48, -5.62], [-4.42, -5.40], [7.43, -4.66]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.52, 1.95], [-2.72, 3.46], [-2.30, -0.97], [0.62, -0.14]],
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

def tests_adult_0_child_100_test_case_57_stopped_medium(tester):
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
                "goals": [[5.13, 2.94], [4.07, 1.28], [4.56, -1.14], [4.83, 2.16]],
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
                "init_x": 5.47,
                "init_y": 2.00,
                "init_a": -121.11,
                "velocity": 0.97,
                "goals": [[5.13, 2.94], [-1.34, 3.31], [-2.51, -1.04], [-6.14, 5.26]],
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
                "init_x": -1.51,
                "init_y": -1.09,
                "init_a": 156.54,
                "velocity": 1.15,
                "goals": [[-1.51, -1.09], [2.49, 1.19], [-6.31, 2.22], [-5.27, -1.29]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.51, -1.09], [-7.58, 1.75], [-6.66, -1.74], [1.14, 5.70]],
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
                "init_x": -2.62,
                "init_y": 2.11,
                "init_a": 48.72,
                "velocity": 0.94,
                "goals": [[0.75, -3.59], [-0.48, -5.21], [-0.91, -1.27], [-6.95, 3.00]],
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
                "init_x": 6.39,
                "init_y": -2.24,
                "init_a": 106.29,
                "velocity": 0.97,
                "goals": [[-1.12, 0.37], [1.92, -0.52], [6.31, 1.02], [1.79, -1.93]],
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
                "init_x": 6.44,
                "init_y": -0.88,
                "init_a": -107.32,
                "velocity": 0.97,
                "goals": [[-3.48, 5.36], [-3.37, 4.80], [4.79, -5.59], [4.47, 3.90]],
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
                "init_x": 7.65,
                "init_y": -0.54,
                "init_a": -62.86,
                "velocity": 0.91,
                "goals": [[4.32, -0.49], [-4.97, -5.15], [-6.50, 4.77], [7.89, 3.73]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.57, -1.98], [-2.64, 3.74], [-2.02, -3.53], [-0.54, 5.84]],
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

def tests_adult_0_child_100_test_case_58_walking_medium(tester):
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
                "goals": [[7.59, -4.42], [0.06, -0.79], [1.06, 0.07], [0.42, 5.49]],
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
                "init_x": -3.46,
                "init_y": -1.26,
                "init_a": -34.91,
                "velocity": 1.14,
                "goals": [[7.59, -4.42], [4.34, -5.72], [-0.09, 4.06], [-0.12, -5.11]],
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
                "init_x": 2.65,
                "init_y": -3.33,
                "init_a": 163.02,
                "velocity": 0.82,
                "goals": [[-3.20, -2.39], [-2.54, -3.17], [-2.86, -0.59], [-0.06, 2.53]],
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
                "init_x": 1.75,
                "init_y": -3.77,
                "init_a": 163.02,
                "velocity": 0.82,
                "goals": [[-3.20, -2.39], [0.13, 2.49], [3.01, 3.75], [-0.14, -3.91]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.59, 5.70], [5.89, -1.55], [7.29, -0.68], [2.40, 4.88]],
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
                "init_x": 0.76,
                "init_y": 3.08,
                "init_a": 0.03,
                "velocity": 0.82,
                "goals": [[2.48, -3.77], [2.41, 5.47], [-3.66, 1.82], [-2.17, 4.96]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.10, 3.83], [-4.12, -5.82], [5.61, 3.95], [-4.05, -5.25]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.40, 5.09], [7.85, 1.05], [-4.82, 2.19], [3.35, -3.76]],
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

def tests_adult_0_child_100_test_case_59_walking_medium(tester):
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
                "goals": [[-1.20, 1.27], [7.44, -5.36], [3.24, -3.47], [-0.55, -4.15]],
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
                "init_x": 0.28,
                "init_y": 1.05,
                "init_a": -32.70,
                "velocity": 1.10,
                "goals": [[-1.20, 1.27], [-6.39, -3.34], [5.11, -0.51], [-3.78, -4.08]],
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
                "init_x": -7.73,
                "init_y": -0.15,
                "init_a": -96.60,
                "velocity": 0.92,
                "goals": [[4.86, -0.21], [-7.44, 4.45], [-5.56, 4.41], [3.81, -2.71]],
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
                "init_x": -6.87,
                "init_y": 0.37,
                "init_a": -96.60,
                "velocity": 0.92,
                "goals": [[4.86, -0.21], [0.58, 3.05], [6.34, 3.43], [-3.81, 3.74]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.86, -2.58], [-6.36, -5.87], [5.37, 0.45], [1.32, 1.96]],
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
                "init_x": 7.71,
                "init_y": 3.33,
                "init_a": 157.25,
                "velocity": 0.82,
                "goals": [[-6.72, 5.69], [-2.07, 2.19], [-3.82, 3.55], [-5.94, -2.61]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.63, 1.86], [7.72, -4.31], [-7.15, 4.45], [0.45, 3.48]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[2.70, 5.85], [-2.65, -2.68], [-3.55, -4.81], [-4.57, 3.77]],
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

def tests_adult_0_child_100_test_case_60_walking_medium(tester):
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
                "goals": [[4.38, 5.93], [4.91, 5.51], [-4.90, -1.66], [-6.64, -1.93]],
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
                "init_x": 0.28,
                "init_y": 2.23,
                "init_a": 54.76,
                "velocity": 1.11,
                "goals": [[4.38, 5.93], [6.84, 4.26], [-7.76, 0.77], [5.55, 3.59]],
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
                "init_x": -7.61,
                "init_y": 5.59,
                "init_a": 47.70,
                "velocity": 1.07,
                "goals": [[-3.66, -3.74], [-2.52, -1.64], [7.40, -0.44], [2.10, 4.32]],
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
                "init_x": -8.60,
                "init_y": 5.71,
                "init_a": 47.70,
                "velocity": 1.07,
                "goals": [[-3.66, -3.74], [-7.69, 4.36], [4.58, 4.95], [-4.40, 1.72]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.57, -4.46], [-2.91, 3.61], [-1.96, 2.34], [7.76, 5.63]],
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
                "init_x": 6.07,
                "init_y": -5.15,
                "init_a": -178.76,
                "velocity": 0.98,
                "goals": [[6.53, 0.30], [1.01, -1.47], [-2.30, -5.66], [-5.20, -2.82]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.93, -4.43], [-5.42, 4.41], [0.92, 4.20], [-6.57, -2.12]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.08, -1.60], [-2.68, -5.12], [1.54, 1.97], [-2.95, -2.02]],
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

def tests_adult_0_child_100_test_case_61_stopped_medium(tester):
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
                "goals": [[0.99, 2.09], [-4.27, 2.24], [-0.35, -3.89], [-6.73, 4.66]],
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
                "init_x": 1.99,
                "init_y": 2.04,
                "init_a": -141.43,
                "velocity": 0.95,
                "goals": [[0.99, 2.09], [-5.38, 2.67], [0.01, -2.99], [5.26, -1.95]],
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
                "init_x": 1.07,
                "init_y": -3.92,
                "init_a": -110.00,
                "velocity": 0.90,
                "goals": [[1.07, -3.92], [1.44, -0.26], [-3.03, -5.65], [7.75, -4.64]],
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
                "init_x": 1.90,
                "init_y": -4.48,
                "init_a": -110.00,
                "velocity": 0.90,
                "goals": [[1.07, -3.92], [3.09, -2.64], [4.99, -4.60], [-3.95, 5.63]],
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
                "init_x": -6.25,
                "init_y": 4.61,
                "init_a": -60.70,
                "velocity": 1.10,
                "goals": [[5.56, 3.38], [4.12, 3.57], [-2.06, -2.72], [-6.22, -3.64]],
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
                "init_x": -1.56,
                "init_y": -5.40,
                "init_a": -170.98,
                "velocity": 1.11,
                "goals": [[4.71, -2.50], [1.17, 1.62], [3.35, 0.35], [5.93, -4.45]],
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
                "init_x": -7.50,
                "init_y": 2.54,
                "init_a": -89.36,
                "velocity": 1.13,
                "goals": [[-6.74, -0.84], [7.03, -1.04], [-0.63, 1.67], [-4.19, -0.10]],
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

def tests_adult_0_child_100_test_case_62_stopped_medium(tester):
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
                "goals": [[5.98, 4.30], [2.43, -1.63], [1.01, -2.50], [6.43, -1.70]],
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
                "init_x": 6.98,
                "init_y": 4.35,
                "init_a": -172.71,
                "velocity": 0.80,
                "goals": [[5.98, 4.30], [3.87, 0.76], [5.26, 2.50], [2.67, -0.27]],
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
                "init_x": -1.33,
                "init_y": 1.09,
                "init_a": -64.29,
                "velocity": 1.12,
                "goals": [[-1.33, 1.09], [-4.24, -2.57], [2.21, 4.47], [-0.55, 3.16]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.33, 1.09], [-4.26, 5.97], [1.77, -1.39], [6.34, 0.57]],
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
                "init_x": 5.50,
                "init_y": 0.96,
                "init_a": 90.77,
                "velocity": 1.13,
                "goals": [[-6.09, -0.56], [2.08, -4.71], [-3.98, -4.52], [-3.46, 4.77]],
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
                "init_x": 4.42,
                "init_y": -2.28,
                "init_a": -147.87,
                "velocity": 0.97,
                "goals": [[1.69, -0.46], [6.79, 1.17], [-7.88, 5.33], [0.35, -0.59]],
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
                "init_x": -4.90,
                "init_y": -3.48,
                "init_a": -67.97,
                "velocity": 1.17,
                "goals": [[-2.12, 1.55], [6.15, 5.61], [-7.88, -2.28], [-6.64, 2.33]],
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
                "init_x": -2.25,
                "init_y": -1.12,
                "init_a": 14.93,
                "velocity": 1.08,
                "goals": [[3.27, 4.54], [5.31, -3.15], [7.42, -2.36], [2.11, -3.97]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.15, 1.62], [7.39, 4.03], [-2.02, 3.11], [-6.02, 3.89]],
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

def tests_adult_0_child_100_test_case_63_walking_medium(tester):
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
                "goals": [[5.86, -3.97], [2.96, -5.89], [1.57, -1.38], [3.54, -0.54]],
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
                "init_x": -2.25,
                "init_y": 2.82,
                "init_a": -148.32,
                "velocity": 1.08,
                "goals": [[5.86, -3.97], [1.90, 4.83], [0.74, 0.44], [4.61, -1.88]],
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
                "init_x": -5.97,
                "init_y": -5.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goals": [[2.18, -1.61], [2.70, -5.97], [7.23, 0.70], [5.91, -0.28]],
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
                "init_x": -6.03,
                "init_y": -6.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goals": [[2.18, -1.61], [-4.72, -5.42], [-2.18, 1.91], [-0.86, -4.43]],
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
                "init_x": 2.53,
                "init_y": 3.99,
                "init_a": -28.58,
                "velocity": 1.19,
                "goals": [[4.54, 4.23], [0.02, 0.09], [0.80, -2.79], [-2.26, 1.06]],
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
                "init_x": -4.39,
                "init_y": 2.71,
                "init_a": -149.94,
                "velocity": 1.03,
                "goals": [[5.92, -2.47], [-0.80, 5.77], [3.56, 3.50], [-3.20, -0.21]],
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
                "init_x": -7.36,
                "init_y": 5.09,
                "init_a": -122.65,
                "velocity": 0.95,
                "goals": [[7.50, 1.97], [1.76, -5.85], [-3.61, 3.07], [-1.34, 3.46]],
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

def tests_adult_0_child_100_test_case_64_stopped_medium(tester):
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
                "goals": [[0.95, 2.92], [-7.90, 0.75], [-2.09, -3.72], [5.26, 5.74]],
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
                "init_x": 0.26,
                "init_y": 3.64,
                "init_a": -152.55,
                "velocity": 0.94,
                "goals": [[0.95, 2.92], [5.78, 4.61], [2.18, 5.01], [2.63, -1.02]],
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
                "init_x": 5.94,
                "init_y": 0.06,
                "init_a": -62.76,
                "velocity": 0.94,
                "goals": [[5.94, 0.06], [-7.11, 2.90], [-4.38, -5.11], [-6.22, 4.22]],
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
                "init_x": 6.91,
                "init_y": 0.29,
                "init_a": -62.76,
                "velocity": 0.94,
                "goals": [[5.94, 0.06], [-2.84, -1.16], [2.87, 3.44], [-5.81, 4.66]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.02, 3.59], [7.73, 4.04], [0.30, -1.91], [7.69, -4.35]],
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
                "init_x": -7.46,
                "init_y": 4.27,
                "init_a": -21.70,
                "velocity": 1.18,
                "goals": [[2.94, 5.50], [-6.24, 3.62], [5.85, 0.13], [-3.47, 5.82]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.92, -2.55], [4.03, -2.66], [-0.54, -5.96], [-7.82, -1.65]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.26, -1.95], [-2.30, 0.83], [3.29, 2.82], [-7.55, -4.81]],
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

def tests_adult_0_child_100_test_case_65_stopped_medium(tester):
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
                "goals": [[3.61, -4.95], [0.83, -5.08], [1.24, 0.33], [-6.56, 5.90]],
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
                "init_x": 2.64,
                "init_y": -5.16,
                "init_a": -6.29,
                "velocity": 0.99,
                "goals": [[3.61, -4.95], [-1.92, -2.12], [7.05, -0.13], [-1.24, -2.88]],
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
                "init_x": -0.85,
                "init_y": -1.73,
                "init_a": -0.49,
                "velocity": 0.82,
                "goals": [[-0.85, -1.73], [-5.98, 5.53], [5.64, 3.76], [-3.52, -4.60]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.85, -1.73], [6.04, -5.06], [3.15, -2.66], [4.37, -3.76]],
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
                "init_x": 5.80,
                "init_y": 2.56,
                "init_a": -26.85,
                "velocity": 1.05,
                "goals": [[-1.73, -5.06], [-7.76, 5.05], [2.34, 0.47], [0.63, 3.53]],
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
                "init_x": 3.53,
                "init_y": -4.42,
                "init_a": 131.50,
                "velocity": 1.16,
                "goals": [[-5.68, -0.30], [-1.69, 4.62], [-2.40, 4.96], [3.10, -5.18]],
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
                "init_x": 0.62,
                "init_y": 2.74,
                "init_a": 44.82,
                "velocity": 1.10,
                "goals": [[0.76, 5.05], [0.42, -2.04], [7.16, 2.37], [5.89, -3.62]],
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
                "init_x": -3.62,
                "init_y": -5.83,
                "init_a": -68.59,
                "velocity": 0.87,
                "goals": [[7.60, -5.23], [3.78, 1.92], [6.72, 4.67], [5.69, -2.76]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.71, 3.74], [-7.80, 1.79], [7.72, -5.61], [-0.24, -5.98]],
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

def tests_adult_0_child_100_test_case_66_walking_medium(tester):
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
                "goals": [[6.32, 2.26], [-6.85, -5.86], [3.53, -2.69], [6.09, -1.07]],
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
                "init_x": 2.21,
                "init_y": -5.72,
                "init_a": -57.89,
                "velocity": 0.85,
                "goals": [[6.32, 2.26], [-0.82, 2.38], [5.40, 4.81], [1.93, 2.35]],
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
                "init_x": -7.72,
                "init_y": -5.03,
                "init_a": -61.58,
                "velocity": 0.95,
                "goals": [[-3.30, -0.14], [3.20, 2.31], [1.83, -2.07], [-1.24, 0.13]],
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
                "init_x": -6.97,
                "init_y": -5.70,
                "init_a": -61.58,
                "velocity": 0.95,
                "goals": [[-3.30, -0.14], [-1.54, -2.51], [3.49, 0.52], [-7.23, 4.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.15, 3.97], [-5.77, -1.92], [7.17, 5.56], [0.82, -5.74]],
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
                "goals": [[-5.82, -2.46], [-3.10, 2.83], [-3.89, -4.84], [2.36, 4.27]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.64, -3.90], [3.49, 3.28], [-4.90, 2.12], [2.51, -4.08]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.54, -0.78], [2.24, -3.85], [-3.82, -5.17], [-7.54, 4.32]],
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

def tests_adult_0_child_100_test_case_67_stopped_medium(tester):
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
                "goals": [[3.58, -5.58], [-3.36, 1.52], [-0.52, 2.22], [0.01, -4.97]],
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
                "init_x": 3.40,
                "init_y": -4.60,
                "init_a": 151.44,
                "velocity": 1.15,
                "goals": [[3.58, -5.58], [-7.00, 3.97], [0.56, 1.76], [-7.97, -4.81]],
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
                "init_x": 6.06,
                "init_y": -4.08,
                "init_a": -98.20,
                "velocity": 1.20,
                "goals": [[6.06, -4.08], [6.95, -2.97], [-2.12, 2.09], [-1.67, 5.33]],
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
                "init_x": 6.67,
                "init_y": -4.87,
                "init_a": -98.20,
                "velocity": 1.20,
                "goals": [[6.06, -4.08], [6.53, -5.04], [2.48, -0.90], [4.18, 1.70]],
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
                "goals": [[-3.04, -2.77], [1.29, -3.95], [-2.72, 0.18], [-7.78, 0.36]],
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
                "init_x": 6.91,
                "init_y": -0.27,
                "init_a": 165.91,
                "velocity": 1.16,
                "goals": [[-3.49, 5.11], [4.14, 1.41], [4.85, 4.64], [-6.17, -2.13]],
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
                "init_x": 2.94,
                "init_y": 2.24,
                "init_a": -9.44,
                "velocity": 0.88,
                "goals": [[-4.85, 2.66], [1.47, -0.13], [-6.74, 3.21], [7.09, -3.16]],
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

def tests_adult_0_child_100_test_case_68_stopped_medium(tester):
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
                "goals": [[-1.49, -0.10], [1.09, 2.42], [-1.23, -3.31], [-2.22, 0.38]],
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
                "init_x": -0.79,
                "init_y": -0.82,
                "init_a": -39.32,
                "velocity": 1.01,
                "goals": [[-1.49, -0.10], [-7.72, 5.31], [-6.56, 1.04], [6.84, 4.87]],
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
                "init_x": -4.26,
                "init_y": 2.08,
                "init_a": -104.95,
                "velocity": 0.82,
                "goals": [[-4.26, 2.08], [3.40, 1.19], [6.35, -4.34], [6.78, -4.33]],
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
                "init_x": -5.26,
                "init_y": 2.07,
                "init_a": -104.95,
                "velocity": 0.82,
                "goals": [[-4.26, 2.08], [5.42, -4.31], [-3.45, -3.57], [-7.62, 4.53]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.74, -1.73], [2.66, -3.38], [-6.61, 5.48], [2.78, -1.23]],
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
                "init_x": -6.31,
                "init_y": -5.70,
                "init_a": 121.34,
                "velocity": 1.11,
                "goals": [[2.91, 4.38], [-1.61, 4.10], [-2.42, -2.94], [2.05, -1.94]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.58, 4.79], [-3.01, 5.07], [6.96, 0.16], [-3.62, -1.05]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.41, 2.67], [-0.74, 2.58], [6.35, 4.99], [-3.33, 4.62]],
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

def tests_adult_0_child_100_test_case_69_stopped_medium(tester):
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
                "goals": [[4.76, -5.57], [7.12, 4.18], [-5.04, 3.07], [7.59, 3.04]],
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
                "init_x": 3.86,
                "init_y": -5.99,
                "init_a": -169.40,
                "velocity": 0.82,
                "goals": [[4.76, -5.57], [6.92, -3.29], [-6.79, 2.32], [-4.73, -4.48]],
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
                "init_x": -3.47,
                "init_y": 5.59,
                "init_a": 19.05,
                "velocity": 0.91,
                "goals": [[-3.47, 5.59], [-6.93, -1.83], [-0.55, -2.26], [-0.31, -5.28]],
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
                "init_x": -2.53,
                "init_y": 5.93,
                "init_a": 19.05,
                "velocity": 0.91,
                "goals": [[-3.47, 5.59], [-4.49, -1.58], [7.63, 0.08], [-3.11, 3.55]],
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
                "init_x": -6.94,
                "init_y": 4.92,
                "init_a": 29.78,
                "velocity": 0.87,
                "goals": [[-0.14, -4.01], [5.26, 4.74], [-7.98, 2.14], [7.96, 1.19]],
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
                "init_x": 2.35,
                "init_y": -2.53,
                "init_a": 154.49,
                "velocity": 0.96,
                "goals": [[3.00, -2.26], [-2.61, 4.26], [-1.59, -1.12], [-5.88, -0.62]],
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
                "init_x": -4.31,
                "init_y": 1.90,
                "init_a": -11.53,
                "velocity": 1.20,
                "goals": [[-7.32, -3.19], [-1.48, -3.41], [7.42, -3.70], [-5.62, 4.22]],
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

def tests_adult_0_child_100_test_case_70_walking_medium(tester):
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
                "goals": [[-1.43, 4.61], [-7.95, -1.71], [0.49, -3.70], [0.40, -4.34]],
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
                "init_x": -7.03,
                "init_y": 1.90,
                "init_a": -118.43,
                "velocity": 1.01,
                "goals": [[-1.43, 4.61], [-6.55, -5.73], [-7.59, 5.91], [5.69, 3.11]],
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
                "goals": [[6.39, 4.01], [2.39, 2.57], [2.90, 2.42], [1.97, 1.64]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.39, 4.01], [2.15, -0.14], [-1.78, -4.85], [6.70, -2.34]],
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
                "init_x": -0.40,
                "init_y": -3.58,
                "init_a": -6.10,
                "velocity": 1.07,
                "goals": [[-2.32, 2.16], [-6.82, 4.59], [-1.59, -2.75], [-2.86, -1.06]],
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
                "init_x": 5.67,
                "init_y": -1.32,
                "init_a": -96.86,
                "velocity": 1.13,
                "goals": [[4.80, 1.90], [4.51, 0.16], [-5.87, 5.77], [-5.06, -3.97]],
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
                "init_x": 1.54,
                "init_y": 3.23,
                "init_a": -129.37,
                "velocity": 1.11,
                "goals": [[2.36, -4.09], [0.63, -5.75], [0.63, -2.94], [-2.99, 1.80]],
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
                "init_x": 2.13,
                "init_y": 1.65,
                "init_a": 152.00,
                "velocity": 0.86,
                "goals": [[3.01, -1.64], [0.59, -3.10], [-5.69, -4.14], [-4.98, 4.77]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[1.40, 1.41], [-5.79, 2.99], [3.59, -5.96], [-5.06, 0.92]],
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

def tests_adult_0_child_100_test_case_71_stopped_medium(tester):
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
                "goals": [[-6.42, -2.08], [7.99, 2.31], [5.88, -1.34], [-3.81, 0.46]],
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
                "init_x": -6.56,
                "init_y": -1.09,
                "init_a": 113.86,
                "velocity": 1.15,
                "goals": [[-6.42, -2.08], [4.69, 4.36], [-2.18, 0.02], [-1.10, -1.71]],
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
                "init_x": 5.10,
                "init_y": 3.40,
                "init_a": -32.45,
                "velocity": 0.85,
                "goals": [[5.10, 3.40], [-6.41, -2.67], [0.02, 0.68], [7.55, -4.24]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.10, 3.40], [-2.90, 4.80], [1.02, -2.18], [-3.37, 0.43]],
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
                "init_x": -1.34,
                "init_y": -3.02,
                "init_a": 143.97,
                "velocity": 0.90,
                "goals": [[5.00, 4.08], [-3.82, -2.43], [-3.28, 5.20], [-2.54, -2.41]],
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
                "init_x": 4.91,
                "init_y": 3.55,
                "init_a": 106.90,
                "velocity": 0.97,
                "goals": [[-1.46, -5.78], [7.10, -4.80], [3.23, 5.56], [-6.44, -3.11]],
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
                "init_x": 2.89,
                "init_y": 2.74,
                "init_a": -141.03,
                "velocity": 0.86,
                "goals": [[4.39, 1.89], [1.51, -1.75], [0.89, 3.75], [7.33, 3.70]],
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
                "init_x": -5.19,
                "init_y": 3.11,
                "init_a": 10.38,
                "velocity": 1.14,
                "goals": [[-5.77, -4.61], [3.51, 1.09], [5.29, 2.72], [3.59, -3.78]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.58, -3.18], [-4.75, 3.06], [1.25, 0.11], [-6.42, -5.50]],
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

def tests_adult_0_child_100_test_case_72_walking_medium(tester):
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
                "goals": [[-7.07, -5.05], [-5.65, 1.43], [2.77, -4.80], [4.84, 0.96]],
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
                "init_x": 6.52,
                "init_y": -0.71,
                "init_a": -148.95,
                "velocity": 1.15,
                "goals": [[-7.07, -5.05], [-1.22, 3.50], [6.42, 5.84], [6.09, -1.30]],
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
                "init_x": -7.19,
                "init_y": -2.77,
                "init_a": 93.35,
                "velocity": 0.82,
                "goals": [[1.89, -3.33], [-7.43, 5.87], [-3.12, 4.90], [-7.08, -2.79]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[1.89, -3.33], [4.09, 2.23], [3.90, 2.09], [2.95, -5.36]],
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
                "init_x": 1.73,
                "init_y": 1.94,
                "init_a": 138.40,
                "velocity": 0.87,
                "goals": [[5.54, -2.21], [-4.07, 2.30], [4.67, 2.82], [-4.36, -3.06]],
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
                "goals": [[2.74, 5.39], [4.48, 2.75], [1.56, 5.79], [2.66, -4.79]],
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
                "init_x": -3.56,
                "init_y": 3.81,
                "init_a": -153.11,
                "velocity": 0.91,
                "goals": [[-1.63, -2.04], [-3.46, -1.80], [7.12, -1.54], [-3.64, 1.85]],
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
                "init_x": -2.76,
                "init_y": 2.48,
                "init_a": -108.38,
                "velocity": 0.85,
                "goals": [[7.02, 1.04], [3.66, 1.01], [-0.88, -0.59], [-4.59, -3.03]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.82, 1.87], [-5.70, -0.53], [7.10, 3.98], [-2.90, -0.60]],
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

def tests_adult_0_child_100_test_case_73_walking_medium(tester):
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
                "goals": [[-2.56, -1.93], [-7.67, 4.55], [2.75, 4.83], [-4.13, -1.62]],
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
                "init_x": -2.52,
                "init_y": -2.34,
                "init_a": -138.70,
                "velocity": 0.97,
                "goals": [[-2.56, -1.93], [3.78, -4.29], [1.44, 5.99], [5.86, 3.81]],
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
                "init_x": -4.23,
                "init_y": -5.25,
                "init_a": 15.15,
                "velocity": 1.16,
                "goals": [[-7.16, 0.16], [6.08, 4.11], [-4.65, -0.91], [-0.97, -4.94]],
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
                "init_x": -5.08,
                "init_y": -4.72,
                "init_a": 15.15,
                "velocity": 1.16,
                "goals": [[-7.16, 0.16], [-2.52, 4.76], [-1.04, 5.13], [-0.80, -2.54]],
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
                "init_x": 0.26,
                "init_y": 5.90,
                "init_a": -52.58,
                "velocity": 1.03,
                "goals": [[1.08, 3.88], [4.81, 3.88], [7.25, -1.93], [2.10, -5.24]],
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
                "init_x": -2.13,
                "init_y": -1.13,
                "init_a": -16.95,
                "velocity": 0.99,
                "goals": [[-6.72, -4.09], [4.33, 5.67], [-6.75, 3.21], [1.70, 0.95]],
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
                "init_x": -2.13,
                "init_y": -2.73,
                "init_a": -125.38,
                "velocity": 1.08,
                "goals": [[6.91, 4.11], [-6.06, 3.09], [-1.83, 1.43], [5.59, 1.06]],
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

def tests_adult_0_child_100_test_case_74_walking_medium(tester):
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
                "goals": [[-6.21, 2.48], [4.99, 5.71], [-3.00, 4.01], [4.69, -1.30]],
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
                "init_x": 7.08,
                "init_y": 4.42,
                "init_a": 112.30,
                "velocity": 0.90,
                "goals": [[-6.21, 2.48], [-0.34, 4.73], [-2.50, 2.51], [2.86, 2.91]],
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
                "init_x": 0.70,
                "init_y": 5.00,
                "init_a": 59.38,
                "velocity": 0.83,
                "goals": [[2.09, 3.91], [-1.82, 0.74], [-6.37, -0.06], [-2.49, -0.58]],
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
                "init_x": 1.03,
                "init_y": 4.06,
                "init_a": 59.38,
                "velocity": 0.83,
                "goals": [[2.09, 3.91], [-4.39, -1.83], [3.23, -2.81], [5.62, -1.89]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.20, -2.26], [7.00, 5.39], [-6.22, -5.76], [-7.24, 2.63]],
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
                "init_x": -0.94,
                "init_y": 4.46,
                "init_a": -80.30,
                "velocity": 0.92,
                "goals": [[7.66, -1.08], [-3.06, 4.70], [-6.80, 4.15], [-4.73, -3.69]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[1.90, 2.82], [-6.65, 3.57], [-0.36, -0.62], [-1.58, -2.97]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.55, 4.44], [2.66, -3.05], [-3.22, -5.64], [-0.64, 1.58]],
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

def tests_adult_0_child_100_test_case_75_stopped_medium(tester):
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
                "goals": [[1.56, -2.16], [0.38, 3.86], [-1.91, -4.63], [3.41, -1.06]],
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
                "init_x": 0.87,
                "init_y": -1.43,
                "init_a": 168.66,
                "velocity": 0.84,
                "goals": [[1.56, -2.16], [-2.35, -4.38], [-7.72, 5.78], [6.47, 2.42]],
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
                "init_x": -5.66,
                "init_y": 1.49,
                "init_a": 20.31,
                "velocity": 1.08,
                "goals": [[-5.66, 1.49], [-0.34, 0.34], [-2.49, 0.21], [-3.41, -4.73]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.66, 1.49], [1.70, -4.15], [-7.57, -5.27], [-2.01, -5.37]],
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
                "init_x": -6.05,
                "init_y": 2.62,
                "init_a": 63.46,
                "velocity": 0.88,
                "goals": [[-4.47, -5.62], [-1.70, -0.03], [1.70, -5.74], [3.99, 4.75]],
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
                "init_x": 4.41,
                "init_y": 3.26,
                "init_a": -171.37,
                "velocity": 1.14,
                "goals": [[-6.55, 0.80], [-3.07, -5.45], [-3.09, -3.90], [6.55, -2.52]],
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
                "init_x": -6.50,
                "init_y": -1.97,
                "init_a": -13.49,
                "velocity": 1.07,
                "goals": [[3.56, -3.20], [-1.35, 0.74], [-0.58, -2.62], [1.34, -4.49]],
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
                "init_x": -5.24,
                "init_y": 2.69,
                "init_a": 113.86,
                "velocity": 0.84,
                "goals": [[3.58, 4.22], [-6.81, -5.32], [4.12, -3.11], [-7.06, -1.57]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.64, 1.01], [3.67, -1.78], [2.70, 1.53], [-0.66, -3.58]],
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

def tests_adult_0_child_100_test_case_76_stopped_medium(tester):
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
                "goals": [[5.08, -3.31], [-4.43, -1.86], [2.35, 3.27], [7.72, 1.74]],
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
                "init_x": 4.78,
                "init_y": -2.36,
                "init_a": -107.43,
                "velocity": 1.03,
                "goals": [[5.08, -3.31], [0.75, -2.65], [-1.34, -1.81], [0.31, 5.69]],
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
                "init_x": 4.12,
                "init_y": -2.17,
                "init_a": 49.74,
                "velocity": 1.12,
                "goals": [[4.12, -2.17], [6.11, -3.45], [7.10, 3.21], [-2.54, 2.31]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.12, -2.17], [2.68, 2.17], [6.39, -4.16], [2.44, -3.07]],
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
                "init_x": 1.91,
                "init_y": 5.47,
                "init_a": -147.17,
                "velocity": 0.99,
                "goals": [[1.40, -4.32], [2.22, -5.52], [-6.10, -3.15], [3.35, 1.02]],
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
                "init_x": 1.94,
                "init_y": -1.27,
                "init_a": 143.26,
                "velocity": 1.07,
                "goals": [[-0.12, -5.71], [-1.17, 2.33], [-1.87, 4.97], [5.48, -5.36]],
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
                "init_x": -7.35,
                "init_y": -2.49,
                "init_a": 117.15,
                "velocity": 0.81,
                "goals": [[-6.45, 1.16], [1.03, 5.57], [-4.46, -2.56], [7.58, 4.78]],
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
                "init_x": -5.61,
                "init_y": -2.48,
                "init_a": 70.00,
                "velocity": 1.04,
                "goals": [[-2.79, 5.21], [-5.23, -4.71], [3.02, 2.29], [-5.11, 5.00]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-7.96, -4.40], [-4.31, 0.82], [-1.09, -4.52], [2.15, 2.60]],
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

def tests_adult_0_child_100_test_case_77_walking_medium(tester):
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
                "goals": [[3.54, 3.83], [-1.48, 0.10], [-5.41, -4.19], [8.00, -1.66]],
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
                "init_x": -5.74,
                "init_y": -1.36,
                "init_a": 21.62,
                "velocity": 1.09,
                "goals": [[3.54, 3.83], [0.11, -1.30], [2.91, -0.89], [-0.99, -0.01]],
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
                "init_x": 4.38,
                "init_y": 2.72,
                "init_a": 41.10,
                "velocity": 0.98,
                "goals": [[-0.41, -4.71], [1.03, -0.78], [-6.61, 1.18], [-5.30, -2.86]],
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
                "init_x": 5.38,
                "init_y": 2.62,
                "init_a": 41.10,
                "velocity": 0.98,
                "goals": [[-0.41, -4.71], [-3.35, 2.44], [3.66, 4.70], [5.76, -2.52]],
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
                "init_x": -5.36,
                "init_y": 0.76,
                "init_a": 70.74,
                "velocity": 0.82,
                "goals": [[-7.83, -2.70], [-1.93, -2.87], [1.34, 5.58], [-2.07, -0.28]],
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
                "init_x": 4.64,
                "init_y": -1.38,
                "init_a": 131.93,
                "velocity": 1.09,
                "goals": [[2.86, -3.32], [1.10, -1.97], [-7.34, -5.24], [6.25, 5.57]],
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
                "init_x": -4.62,
                "init_y": 5.33,
                "init_a": 11.41,
                "velocity": 0.95,
                "goals": [[6.82, 2.03], [7.06, -2.88], [-5.08, -0.96], [-0.64, 1.75]],
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

def tests_adult_0_child_100_test_case_78_stopped_medium(tester):
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
                "goals": [[0.22, -1.46], [-4.94, 0.18], [6.63, 4.25], [-4.63, 1.41]],
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
                "init_x": 0.99,
                "init_y": -2.10,
                "init_a": 178.09,
                "velocity": 0.94,
                "goals": [[0.22, -1.46], [-2.66, -5.18], [7.69, 1.57], [7.51, 4.74]],
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
                "init_x": -2.86,
                "init_y": -0.75,
                "init_a": -5.96,
                "velocity": 1.13,
                "goals": [[-2.86, -0.75], [0.27, 3.30], [0.31, -3.52], [-3.74, 4.67]],
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
                "init_x": -3.84,
                "init_y": -0.57,
                "init_a": -5.96,
                "velocity": 1.13,
                "goals": [[-2.86, -0.75], [0.88, 3.14], [0.92, -3.90], [2.88, 0.22]],
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
                "init_x": 3.81,
                "init_y": 0.45,
                "init_a": 124.19,
                "velocity": 0.93,
                "goals": [[2.35, 4.29], [-1.91, -4.85], [-3.32, 1.31], [3.50, -1.04]],
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
                "init_x": -1.43,
                "init_y": -4.88,
                "init_a": -71.90,
                "velocity": 0.83,
                "goals": [[5.33, -1.40], [-0.54, -4.88], [-7.53, 4.40], [5.04, 1.68]],
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
                "init_x": 6.47,
                "init_y": -3.56,
                "init_a": -31.43,
                "velocity": 0.94,
                "goals": [[1.28, 2.97], [0.98, -5.04], [-6.57, -5.54], [-1.90, -1.72]],
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

def tests_adult_0_child_100_test_case_79_stopped_medium(tester):
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
                "goals": [[-7.54, -5.32], [2.50, -1.76], [5.83, -2.56], [1.33, -0.07]],
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
                "init_x": -8.44,
                "init_y": -4.88,
                "init_a": -114.62,
                "velocity": 0.92,
                "goals": [[-7.54, -5.32], [0.92, 1.15], [4.84, 1.57], [2.72, -5.86]],
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
                "init_x": 2.87,
                "init_y": -3.35,
                "init_a": 109.27,
                "velocity": 0.89,
                "goals": [[2.87, -3.35], [-0.13, 5.97], [2.54, 1.82], [0.20, 5.42]],
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
                "init_x": 3.87,
                "init_y": -3.25,
                "init_a": 109.27,
                "velocity": 0.89,
                "goals": [[2.87, -3.35], [-1.09, 1.97], [-7.46, 2.86], [7.67, -1.93]],
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
                "init_x": -2.31,
                "init_y": -1.97,
                "init_a": 123.27,
                "velocity": 1.05,
                "goals": [[-1.54, 5.84], [7.90, 3.48], [4.58, 0.69], [-6.75, -3.13]],
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
                "init_x": 0.22,
                "init_y": -2.80,
                "init_a": -88.28,
                "velocity": 0.92,
                "goals": [[4.17, -2.22], [-0.23, 4.02], [-5.15, 2.87], [-3.51, -2.81]],
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
                "init_x": -2.82,
                "init_y": 5.92,
                "init_a": 165.60,
                "velocity": 1.19,
                "goals": [[1.65, -5.65], [-0.86, -5.05], [5.57, -0.83], [-4.13, -4.68]],
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

def tests_adult_0_child_100_test_case_80_stopped_medium(tester):
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
                "goals": [[-6.45, -3.35], [4.59, -5.62], [0.62, 1.49], [4.75, 2.73]],
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
                "init_x": -7.11,
                "init_y": -4.10,
                "init_a": -144.16,
                "velocity": 0.82,
                "goals": [[-6.45, -3.35], [-3.29, -5.00], [2.41, -3.23], [-1.49, -1.12]],
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
                "init_x": 2.43,
                "init_y": 4.67,
                "init_a": -42.80,
                "velocity": 1.11,
                "goals": [[2.43, 4.67], [5.35, -4.78], [-1.31, -3.31], [-7.31, -5.53]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[2.43, 4.67], [6.74, -3.49], [-2.01, 2.20], [-1.96, -5.26]],
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
                "init_x": 0.82,
                "init_y": 4.40,
                "init_a": -82.79,
                "velocity": 0.92,
                "goals": [[3.44, 5.40], [-2.18, -4.01], [5.59, -5.86], [3.67, -5.08]],
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
                "init_x": -6.96,
                "init_y": 2.58,
                "init_a": -26.58,
                "velocity": 0.87,
                "goals": [[6.46, 2.05], [2.58, -1.30], [-5.60, 4.93], [4.70, 1.13]],
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
                "init_x": 0.70,
                "init_y": 1.83,
                "init_a": -81.47,
                "velocity": 1.15,
                "goals": [[-0.51, -1.76], [6.58, -0.52], [7.42, 5.99], [0.79, 1.31]],
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
                "init_x": 1.16,
                "init_y": 1.19,
                "init_a": -37.19,
                "velocity": 0.86,
                "goals": [[-5.04, 3.70], [-0.44, 2.09], [1.16, -1.53], [5.56, 0.67]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.32, -2.47], [-4.76, 2.09], [6.37, 0.21], [-6.51, -4.54]],
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

def tests_adult_0_child_100_test_case_81_stopped_medium(tester):
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
                "goals": [[-0.01, 0.38], [0.44, -5.47], [-4.41, 2.02], [-3.09, 2.35]],
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
                "init_x": 0.30,
                "init_y": 1.33,
                "init_a": 132.14,
                "velocity": 0.82,
                "goals": [[-0.01, 0.38], [-4.96, -0.71], [6.66, -0.97], [4.95, -0.38]],
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
                "init_x": 6.90,
                "init_y": -2.01,
                "init_a": 32.91,
                "velocity": 1.03,
                "goals": [[6.90, -2.01], [-6.99, -0.90], [-5.95, 5.61], [-1.47, 2.12]],
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
                "init_x": 6.39,
                "init_y": -2.87,
                "init_a": 32.91,
                "velocity": 1.03,
                "goals": [[6.90, -2.01], [2.93, 5.77], [6.72, 5.96], [-1.72, 4.70]],
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
                "init_x": 4.94,
                "init_y": -0.32,
                "init_a": 79.63,
                "velocity": 1.15,
                "goals": [[2.01, -4.61], [-2.66, 3.01], [-2.15, -2.53], [0.27, -2.89]],
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
                "init_x": -0.23,
                "init_y": -3.67,
                "init_a": -46.76,
                "velocity": 1.10,
                "goals": [[3.27, 5.92], [5.42, 1.11], [-1.33, 1.29], [-6.12, -1.99]],
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
                "init_x": -3.00,
                "init_y": -0.40,
                "init_a": -135.14,
                "velocity": 1.03,
                "goals": [[1.92, -4.35], [-7.64, 1.75], [1.21, -0.10], [4.21, 2.61]],
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

def tests_adult_0_child_100_test_case_82_walking_medium(tester):
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
                "goals": [[3.52, -4.79], [0.93, -1.81], [-1.25, 4.18], [1.99, 1.97]],
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
                "init_x": -5.36,
                "init_y": 5.25,
                "init_a": 158.47,
                "velocity": 0.95,
                "goals": [[3.52, -4.79], [-5.37, 1.19], [2.23, -4.11], [3.03, -2.31]],
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
                "init_x": 2.30,
                "init_y": -4.23,
                "init_a": 125.02,
                "velocity": 1.15,
                "goals": [[-6.63, -1.13], [-6.14, 2.76], [0.86, -5.59], [0.33, -5.22]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.63, -1.13], [-7.78, -3.27], [4.98, -5.01], [3.23, -4.58]],
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
                "init_x": -0.53,
                "init_y": 4.79,
                "init_a": 106.27,
                "velocity": 1.19,
                "goals": [[-7.03, -5.84], [-2.04, 0.66], [-0.85, -5.06], [-4.01, 2.96]],
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
                "init_x": 1.90,
                "init_y": 5.17,
                "init_a": 71.36,
                "velocity": 1.00,
                "goals": [[5.11, -2.87], [-6.04, -0.53], [-0.21, 0.49], [-5.97, -0.69]],
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
                "init_x": -0.52,
                "init_y": 0.27,
                "init_a": -18.49,
                "velocity": 0.96,
                "goals": [[7.43, -5.48], [-3.49, -2.78], [-2.93, 1.43], [-6.95, 2.20]],
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
                "init_x": 7.10,
                "init_y": 5.31,
                "init_a": -42.58,
                "velocity": 1.20,
                "goals": [[-2.11, 1.32], [-0.16, 5.34], [-5.25, -1.02], [-5.13, -0.60]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.43, 2.56], [1.59, 4.88], [-5.52, 3.56], [5.06, -1.54]],
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

def tests_adult_0_child_100_test_case_83_stopped_medium(tester):
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
                "goals": [[0.90, 3.69], [5.52, -5.94], [-2.08, 1.71], [-1.36, 2.59]],
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
                "init_x": 0.05,
                "init_y": 4.21,
                "init_a": 93.93,
                "velocity": 0.99,
                "goals": [[0.90, 3.69], [-3.08, -5.04], [-5.02, -4.46], [-5.52, -5.95]],
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
                "init_x": 7.38,
                "init_y": 1.35,
                "init_a": 11.15,
                "velocity": 1.18,
                "goals": [[7.38, 1.35], [4.46, -3.95], [-1.99, -3.50], [4.27, 3.78]],
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
                "init_x": 8.19,
                "init_y": 1.95,
                "init_a": 11.15,
                "velocity": 1.18,
                "goals": [[7.38, 1.35], [-0.73, 3.14], [-6.42, -5.98], [4.87, 3.35]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.84, 1.33], [-1.89, 2.50], [4.45, 5.30], [2.77, 0.28]],
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
                "init_x": -1.13,
                "init_y": -0.33,
                "init_a": 96.01,
                "velocity": 1.08,
                "goals": [[3.75, 4.82], [4.02, -2.94], [-5.37, 5.83], [4.48, -4.05]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.39, -0.30], [3.17, -1.64], [-0.13, -0.28], [-5.95, -4.50]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[2.01, -2.50], [-4.74, -4.87], [-5.10, 2.84], [0.67, -5.96]],
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

def tests_adult_0_child_100_test_case_84_walking_medium(tester):
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
                "goals": [[-5.76, -4.85], [6.26, -0.33], [0.94, -2.21], [4.14, -4.98]],
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
                "goals": [[-5.76, -4.85], [0.53, 2.06], [-2.84, 5.26], [1.48, 1.26]],
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
                "init_x": -7.70,
                "init_y": 0.75,
                "init_a": -40.36,
                "velocity": 0.84,
                "goals": [[-4.51, -5.16], [-7.90, 1.97], [-4.63, -1.20], [-2.31, 4.56]],
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
                "init_x": -8.04,
                "init_y": -0.19,
                "init_a": -40.36,
                "velocity": 0.84,
                "goals": [[-4.51, -5.16], [4.55, 0.91], [7.40, 2.43], [-6.56, -4.35]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.14, -0.38], [-3.02, -2.65], [-1.22, -0.31], [-1.37, 5.57]],
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
                "init_x": 5.43,
                "init_y": -1.02,
                "init_a": -141.47,
                "velocity": 0.87,
                "goals": [[6.90, 1.19], [-7.02, 3.52], [-6.12, -0.25], [-7.84, -4.20]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.48, -0.81], [2.85, 3.15], [2.17, -5.69], [-5.20, -2.62]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-4.01, -3.23], [3.05, -0.90], [-3.95, -1.43], [-7.74, -0.38]],
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

def tests_adult_0_child_100_test_case_85_stopped_medium(tester):
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
                "goals": [[-3.02, 2.61], [-7.23, -3.17], [-2.18, -3.19], [-0.76, 0.05]],
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
                "init_x": -3.70,
                "init_y": 3.34,
                "init_a": -47.10,
                "velocity": 0.97,
                "goals": [[-3.02, 2.61], [-7.53, 3.89], [-7.40, -5.08], [3.65, -4.31]],
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
                "init_x": -0.32,
                "init_y": -2.63,
                "init_a": 167.90,
                "velocity": 0.87,
                "goals": [[-0.32, -2.63], [-7.28, -4.19], [-6.42, 4.02], [-3.77, -5.79]],
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
                "init_x": -1.16,
                "init_y": -2.08,
                "init_a": 167.90,
                "velocity": 0.87,
                "goals": [[-0.32, -2.63], [1.16, -2.31], [-2.29, 5.98], [6.00, 5.26]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-7.51, 4.66], [-4.12, -1.94], [1.09, -2.42], [-1.64, -1.35]],
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
                "init_x": -1.30,
                "init_y": -4.00,
                "init_a": 157.21,
                "velocity": 1.19,
                "goals": [[-4.06, 5.03], [4.82, 4.59], [-4.83, 1.76], [1.37, -0.04]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[2.41, -2.97], [-6.34, -4.45], [5.02, 3.67], [-6.76, -1.00]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.59, 2.15], [-4.40, -5.08], [1.56, -4.79], [-2.75, -1.50]],
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

def tests_adult_0_child_100_test_case_86_walking_medium(tester):
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
                "goals": [[5.91, 3.44], [1.11, 5.24], [-7.05, -0.41], [-5.22, -1.04]],
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
                "init_x": 2.12,
                "init_y": -3.29,
                "init_a": -165.80,
                "velocity": 1.03,
                "goals": [[5.91, 3.44], [4.79, -3.22], [-6.06, 1.43], [1.31, 1.90]],
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
                "init_x": 2.69,
                "init_y": -4.53,
                "init_a": -24.70,
                "velocity": 0.82,
                "goals": [[-7.61, 4.56], [7.84, 3.67], [0.07, -4.91], [4.18, 1.19]],
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
                "init_x": 2.02,
                "init_y": -3.79,
                "init_a": -24.70,
                "velocity": 0.82,
                "goals": [[-7.61, 4.56], [-6.31, -1.14], [3.23, -1.64], [-6.56, 0.43]],
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
                "init_x": 4.11,
                "init_y": -3.70,
                "init_a": 69.15,
                "velocity": 0.83,
                "goals": [[-4.90, 1.72], [-4.00, -5.48], [-2.52, -1.63], [0.39, 5.09]],
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
                "init_x": 1.31,
                "init_y": 4.99,
                "init_a": -140.64,
                "velocity": 0.80,
                "goals": [[2.81, -0.67], [-2.32, -2.32], [-7.37, -4.80], [-0.33, 1.37]],
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
                "init_x": 7.49,
                "init_y": -3.22,
                "init_a": -15.36,
                "velocity": 0.98,
                "goals": [[5.80, -4.98], [-4.03, 2.80], [0.86, -3.64], [-5.11, -5.19]],
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

def tests_adult_0_child_100_test_case_87_stopped_medium(tester):
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
                "goals": [[-3.80, -5.85], [6.55, -3.81], [-5.54, 4.08], [3.71, -0.55]],
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
                "init_x": -4.68,
                "init_y": -5.37,
                "init_a": -131.38,
                "velocity": 0.94,
                "goals": [[-3.80, -5.85], [-6.74, -3.79], [-1.23, 0.96], [1.85, -0.43]],
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
                "goals": [[6.33, 4.53], [-4.89, -4.74], [1.65, 4.52], [0.04, -4.38]],
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
                "init_x": 5.41,
                "init_y": 4.12,
                "init_a": 32.49,
                "velocity": 1.06,
                "goals": [[6.33, 4.53], [3.24, 5.43], [4.52, 5.92], [2.20, -1.81]],
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
                "init_x": -0.36,
                "init_y": 4.54,
                "init_a": 64.20,
                "velocity": 1.03,
                "goals": [[-5.60, -0.41], [-0.50, -2.97], [-1.89, 4.94], [-5.32, 4.48]],
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
                "init_x": -2.97,
                "init_y": -1.95,
                "init_a": 19.91,
                "velocity": 1.16,
                "goals": [[-1.07, -3.22], [5.24, 2.28], [6.70, -1.30], [7.08, -1.55]],
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
                "init_x": -1.80,
                "init_y": 2.58,
                "init_a": -38.15,
                "velocity": 1.04,
                "goals": [[3.74, 4.37], [-5.09, 0.00], [7.37, 2.17], [-1.12, -2.73]],
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

def tests_adult_0_child_100_test_case_88_walking_medium(tester):
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
                "goals": [[2.02, -4.36], [-7.35, -4.95], [4.53, 1.32], [-2.22, -0.53]],
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
                "init_x": 5.14,
                "init_y": 4.63,
                "init_a": -52.22,
                "velocity": 1.14,
                "goals": [[2.02, -4.36], [2.77, -5.34], [-0.58, 3.68], [5.08, -3.47]],
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
                "init_x": 0.93,
                "init_y": 2.39,
                "init_a": -88.80,
                "velocity": 0.97,
                "goals": [[-6.64, -0.16], [-4.48, 2.33], [-1.83, -3.97], [-1.99, -3.76]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.64, -0.16], [-2.50, 1.85], [0.14, -2.21], [-6.10, 1.47]],
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
                "init_x": 4.32,
                "init_y": -2.58,
                "init_a": -126.36,
                "velocity": 0.88,
                "goals": [[-0.34, -0.49], [6.89, -3.51], [4.55, -0.92], [-0.29, 5.95]],
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
                "init_x": 0.59,
                "init_y": 0.46,
                "init_a": -80.05,
                "velocity": 1.08,
                "goals": [[-6.28, -0.19], [5.75, -2.05], [-6.40, 2.41], [-0.34, -5.80]],
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
                "init_x": 1.52,
                "init_y": -1.11,
                "init_a": 147.69,
                "velocity": 0.99,
                "goals": [[-1.04, 1.45], [-6.45, 0.21], [4.78, -1.23], [-2.24, 1.41]],
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
                "init_x": -6.08,
                "init_y": -0.56,
                "init_a": 135.74,
                "velocity": 1.04,
                "goals": [[-4.79, 5.44], [-2.88, -2.13], [2.81, 4.44], [6.90, -0.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.25, 5.48], [-7.51, 4.90], [-6.72, 3.50], [6.92, 5.51]],
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

def tests_adult_0_child_100_test_case_89_stopped_medium(tester):
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
                "goals": [[-1.11, 2.39], [-1.70, -0.85], [-5.34, -3.18], [-6.07, 3.50]],
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
                "init_x": -1.06,
                "init_y": 1.39,
                "init_a": 128.30,
                "velocity": 1.00,
                "goals": [[-1.11, 2.39], [-7.40, 3.60], [1.67, -3.67], [-1.68, 1.63]],
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
                "init_x": -4.99,
                "init_y": -0.89,
                "init_a": -171.32,
                "velocity": 0.93,
                "goals": [[-4.99, -0.89], [-5.59, -5.74], [7.67, 4.83], [-6.22, 5.31]],
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
                "init_x": -3.99,
                "init_y": -0.92,
                "init_a": -171.32,
                "velocity": 0.93,
                "goals": [[-4.99, -0.89], [-3.08, -5.30], [0.96, -3.75], [4.36, -5.31]],
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
                "init_x": 3.10,
                "init_y": 1.38,
                "init_a": -62.99,
                "velocity": 0.83,
                "goals": [[7.40, 5.93], [-2.24, 4.45], [-1.66, -5.57], [1.82, 4.95]],
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
                "init_x": -0.01,
                "init_y": -1.61,
                "init_a": -37.60,
                "velocity": 0.87,
                "goals": [[3.92, 0.83], [2.15, -0.01], [-0.70, -1.46], [-3.73, 5.43]],
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
                "init_x": -5.04,
                "init_y": 1.09,
                "init_a": 172.38,
                "velocity": 1.10,
                "goals": [[2.50, 4.84], [2.66, 1.48], [-6.61, -5.86], [-6.68, -1.03]],
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

def tests_adult_0_child_100_test_case_90_stopped_medium(tester):
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
                "goals": [[-0.93, 2.74], [-5.22, 0.57], [-4.93, 3.36], [-0.54, -2.40]],
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
                "init_x": -0.99,
                "init_y": 3.73,
                "init_a": 64.60,
                "velocity": 0.99,
                "goals": [[-0.93, 2.74], [4.67, 0.81], [3.67, 3.48], [-5.58, 2.95]],
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
                "init_x": -3.13,
                "init_y": -4.23,
                "init_a": 112.02,
                "velocity": 1.17,
                "goals": [[-3.13, -4.23], [5.21, -0.32], [6.34, 5.46], [4.97, -2.68]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.13, -4.23], [-5.36, -4.81], [1.03, 0.13], [5.62, -1.04]],
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
                "init_x": -3.05,
                "init_y": 5.39,
                "init_a": -4.71,
                "velocity": 1.16,
                "goals": [[2.37, 2.14], [2.85, -5.50], [2.04, -4.10], [-6.09, -1.69]],
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
                "init_x": 1.51,
                "init_y": 2.65,
                "init_a": 171.66,
                "velocity": 1.19,
                "goals": [[-5.78, -4.44], [6.86, -0.30], [-0.84, -5.75], [-1.40, -3.47]],
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
                "init_x": 3.54,
                "init_y": 5.12,
                "init_a": 27.87,
                "velocity": 1.12,
                "goals": [[-0.87, -4.13], [-1.80, 1.69], [4.95, 0.77], [1.26, -1.46]],
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
                "init_x": 4.08,
                "init_y": -1.20,
                "init_a": 108.61,
                "velocity": 0.96,
                "goals": [[4.47, -1.06], [-0.43, -0.04], [1.67, -5.45], [-7.43, 3.89]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.33, -5.26], [-1.24, -5.27], [-7.81, -4.02], [4.11, -1.95]],
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

def tests_adult_0_child_100_test_case_91_walking_medium(tester):
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
                "goals": [[-0.01, -0.15], [6.62, -3.49], [2.61, -1.95], [2.41, -2.48]],
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
                "init_x": -5.72,
                "init_y": 4.76,
                "init_a": 178.13,
                "velocity": 0.82,
                "goals": [[-0.01, -0.15], [3.72, -1.81], [0.06, 5.20], [3.79, -2.36]],
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
                "init_x": 4.61,
                "init_y": 4.42,
                "init_a": 133.09,
                "velocity": 1.18,
                "goals": [[6.72, -5.01], [-1.23, -5.92], [5.07, 5.89], [-4.07, -5.15]],
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
                "init_x": 5.56,
                "init_y": 4.74,
                "init_a": 133.09,
                "velocity": 1.18,
                "goals": [[6.72, -5.01], [-4.85, -2.94], [-1.75, -5.99], [-7.11, 0.38]],
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
                "init_x": -1.31,
                "init_y": 5.46,
                "init_a": 16.99,
                "velocity": 1.16,
                "goals": [[5.20, -5.13], [7.72, 0.04], [0.02, -2.51], [0.96, -1.60]],
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
                "init_x": 0.41,
                "init_y": 2.20,
                "init_a": -87.37,
                "velocity": 1.00,
                "goals": [[4.01, -5.30], [-7.70, 3.37], [-3.87, 1.11], [-1.56, -0.72]],
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
                "goals": [[-3.96, -5.81], [1.50, -5.72], [-5.76, -1.12], [4.18, 4.05]],
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

def tests_adult_0_child_100_test_case_92_stopped_medium(tester):
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
                "goals": [[5.47, -3.71], [4.45, -3.66], [2.37, -1.44], [1.64, 2.71]],
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
                "init_x": 4.71,
                "init_y": -4.37,
                "init_a": -69.69,
                "velocity": 0.98,
                "goals": [[5.47, -3.71], [-3.37, -5.91], [-2.49, -4.18], [-7.33, -2.03]],
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
                "init_x": -0.37,
                "init_y": 0.08,
                "init_a": 1.28,
                "velocity": 0.84,
                "goals": [[-0.37, 0.08], [3.51, 4.76], [-4.16, -2.07], [-3.21, -0.69]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.37, 0.08], [-5.13, 1.05], [1.27, -0.95], [5.12, -4.02]],
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
                "init_x": -4.59,
                "init_y": -0.39,
                "init_a": 90.21,
                "velocity": 1.16,
                "goals": [[-2.14, -4.77], [-1.62, -5.91], [-4.94, 5.81], [5.68, 0.37]],
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
                "init_x": -4.56,
                "init_y": 1.09,
                "init_a": 71.03,
                "velocity": 1.05,
                "goals": [[-4.70, -1.62], [5.30, 4.70], [4.55, -2.11], [3.97, 2.45]],
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
                "init_x": 0.48,
                "init_y": 3.41,
                "init_a": 76.50,
                "velocity": 0.94,
                "goals": [[2.90, -5.64], [-2.17, -2.87], [-3.77, -2.56], [-1.69, 1.61]],
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
                "init_x": -4.26,
                "init_y": 0.30,
                "init_a": 2.06,
                "velocity": 1.14,
                "goals": [[-7.29, 1.54], [1.10, -2.59], [0.24, 0.46], [5.65, -4.40]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.51, 2.65], [3.26, 0.25], [-3.65, -1.82], [6.84, -0.17]],
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

def tests_adult_0_child_100_test_case_93_stopped_medium(tester):
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
                "goals": [[7.19, 4.52], [2.59, -3.15], [-5.13, -5.64], [3.60, -4.30]],
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
                "init_x": 6.42,
                "init_y": 5.17,
                "init_a": 94.25,
                "velocity": 1.07,
                "goals": [[7.19, 4.52], [-5.60, -2.88], [6.86, 3.14], [-4.71, 5.39]],
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
                "init_x": 3.63,
                "init_y": -0.21,
                "init_a": -58.74,
                "velocity": 1.12,
                "goals": [[3.63, -0.21], [-7.18, -1.87], [-6.01, -2.73], [7.85, -5.21]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.63, -0.21], [-7.39, 5.29], [-5.71, 0.57], [-2.63, 1.90]],
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
                "init_x": 1.12,
                "init_y": 3.41,
                "init_a": -123.42,
                "velocity": 1.03,
                "goals": [[6.11, 2.16], [-5.03, 0.23], [-6.16, -0.31], [-4.74, -5.64]],
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
                "init_x": -6.17,
                "init_y": 0.19,
                "init_a": -115.45,
                "velocity": 1.12,
                "goals": [[1.08, 5.96], [-1.82, 1.86], [-2.20, 1.32], [0.35, 2.87]],
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
                "init_x": 0.25,
                "init_y": -5.08,
                "init_a": 45.43,
                "velocity": 0.88,
                "goals": [[4.40, 5.35], [-0.11, -0.11], [-2.01, -3.60], [-1.87, 2.03]],
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
                "init_x": 7.59,
                "init_y": 0.53,
                "init_a": 16.78,
                "velocity": 1.02,
                "goals": [[6.53, 0.29], [-6.36, 5.77], [-4.30, 2.84], [-7.19, -5.21]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.05, -1.26], [-2.15, -0.14], [-4.00, -2.04], [-5.83, -0.57]],
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

def tests_adult_0_child_100_test_case_94_stopped_medium(tester):
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
                "goals": [[2.89, 5.31], [-2.36, 5.85], [6.94, 4.59], [1.21, 4.33]],
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
                "init_x": 2.51,
                "init_y": 6.23,
                "init_a": -3.78,
                "velocity": 0.94,
                "goals": [[2.89, 5.31], [-5.41, -5.54], [4.93, -3.68], [0.58, 4.24]],
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
                "init_x": -1.03,
                "init_y": -4.62,
                "init_a": -91.41,
                "velocity": 1.02,
                "goals": [[-1.03, -4.62], [1.99, 0.78], [-3.21, -2.65], [1.02, 3.92]],
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
                "init_x": -1.34,
                "init_y": -5.57,
                "init_a": -91.41,
                "velocity": 1.02,
                "goals": [[-1.03, -4.62], [2.06, -5.56], [6.06, -2.03], [6.00, -3.80]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-2.50, -0.70], [-3.81, -1.26], [5.84, -3.93], [7.84, -2.61]],
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
                "init_x": 4.65,
                "init_y": -1.35,
                "init_a": -47.70,
                "velocity": 0.97,
                "goals": [[-0.98, 0.12], [-7.21, -4.99], [-2.51, 5.36], [6.97, 0.23]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.19, 3.93], [-3.66, -3.63], [2.72, 5.43], [1.87, -0.24]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-3.49, 2.80], [6.86, -4.60], [-7.18, 2.12], [-0.20, 3.52]],
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

def tests_adult_0_child_100_test_case_95_stopped_medium(tester):
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
                "goals": [[-6.54, -0.30], [-4.37, -5.87], [5.77, 5.39], [-2.03, -3.26]],
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
                "init_x": -5.74,
                "init_y": -0.90,
                "init_a": 117.02,
                "velocity": 0.80,
                "goals": [[-6.54, -0.30], [-5.15, 2.67], [-3.91, 0.55], [-0.39, 1.66]],
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
                "init_x": -6.81,
                "init_y": -0.97,
                "init_a": 177.66,
                "velocity": 1.08,
                "goals": [[-6.81, -0.97], [-1.09, 5.82], [-4.67, 3.84], [4.78, -2.38]],
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
                "init_x": -7.68,
                "init_y": -0.48,
                "init_a": 177.66,
                "velocity": 1.08,
                "goals": [[-6.81, -0.97], [6.51, -5.17], [-3.07, -4.73], [-7.83, -0.11]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.55, -4.29], [-5.85, -5.77], [1.59, 2.83], [7.63, -1.44]],
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
                "init_x": -0.46,
                "init_y": -2.46,
                "init_a": -1.37,
                "velocity": 0.86,
                "goals": [[7.35, -2.17], [0.15, 5.99], [-4.90, 1.88], [-1.22, -3.83]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.81, -1.33], [-4.39, -3.65], [2.68, -1.37], [6.22, 5.31]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.06, -5.11], [1.37, 5.35], [-6.25, -3.94], [1.58, -3.21]],
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

def tests_adult_0_child_100_test_case_96_stopped_medium(tester):
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
                "goals": [[1.16, 3.19], [-1.32, -5.41], [7.18, 2.77], [-7.67, 0.46]],
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
                "init_x": 0.16,
                "init_y": 3.15,
                "init_a": -13.09,
                "velocity": 1.08,
                "goals": [[1.16, 3.19], [-1.04, 4.75], [7.47, -4.87], [-1.60, -2.58]],
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
                "init_x": -5.08,
                "init_y": 0.38,
                "init_a": 39.15,
                "velocity": 1.09,
                "goals": [[-5.08, 0.38], [-7.53, 1.02], [7.75, -0.41], [-2.57, -3.91]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.08, 0.38], [-5.84, -3.67], [3.35, 2.41], [7.79, -1.99]],
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
                "init_x": 6.64,
                "init_y": -3.32,
                "init_a": 151.53,
                "velocity": 1.05,
                "goals": [[-7.73, 3.23], [7.52, 3.47], [0.48, 5.78], [-5.59, 0.51]],
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
                "init_x": 1.24,
                "init_y": -2.78,
                "init_a": 87.39,
                "velocity": 1.01,
                "goals": [[-7.51, 2.53], [-1.18, 0.93], [6.78, -5.23], [0.04, -3.64]],
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
                "init_x": 2.61,
                "init_y": 1.70,
                "init_a": -156.35,
                "velocity": 0.98,
                "goals": [[-2.04, -1.36], [5.44, 2.82], [-4.28, -5.73], [-0.16, 1.11]],
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
                "init_x": -6.06,
                "init_y": -0.40,
                "init_a": -73.50,
                "velocity": 0.95,
                "goals": [[6.30, 0.59], [-5.69, 5.38], [-7.62, 5.93], [5.01, 3.96]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[3.25, -5.64], [-7.17, -4.66], [-3.89, 4.51], [7.02, 1.67]],
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

def tests_adult_0_child_100_test_case_97_stopped_medium(tester):
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
                "goals": [[-3.96, -5.71], [0.73, 2.75], [1.57, -1.39], [3.52, 5.62]],
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
                "init_x": -4.92,
                "init_y": -5.98,
                "init_a": 135.41,
                "velocity": 1.14,
                "goals": [[-3.96, -5.71], [5.32, -0.94], [3.37, 0.88], [-0.82, -2.61]],
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
                "init_x": -7.00,
                "init_y": -3.76,
                "init_a": -8.34,
                "velocity": 0.81,
                "goals": [[-7.00, -3.76], [-2.92, -1.52], [-5.82, -5.92], [6.59, -0.37]],
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
                "init_x": -7.64,
                "init_y": -4.53,
                "init_a": -8.34,
                "velocity": 0.81,
                "goals": [[-7.00, -3.76], [-6.97, 1.11], [-4.36, -4.04], [3.58, -3.48]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.68, 2.36], [7.75, 5.24], [1.38, -3.76], [-5.96, -3.01]],
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
                "init_x": -2.89,
                "init_y": -3.42,
                "init_a": -142.97,
                "velocity": 0.81,
                "goals": [[3.83, -0.08], [-6.20, 3.82], [-5.88, -5.31], [0.27, 3.71]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.39, 3.59], [0.64, 3.67], [1.80, -2.74], [4.77, -0.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-0.54, 0.07], [5.51, 3.00], [-4.03, 1.61], [1.72, -5.78]],
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

def tests_adult_0_child_100_test_case_98_stopped_medium(tester):
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
                "goals": [[-3.25, -0.40], [-2.03, 5.76], [-0.41, -3.46], [7.09, 0.79]],
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
                "init_x": -2.85,
                "init_y": -1.31,
                "init_a": 89.69,
                "velocity": 0.96,
                "goals": [[-3.25, -0.40], [4.40, -4.88], [6.48, 3.67], [0.88, 2.88]],
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
                "init_x": 1.70,
                "init_y": -2.21,
                "init_a": -121.14,
                "velocity": 1.12,
                "goals": [[1.70, -2.21], [-5.32, 4.40], [-4.33, 5.41], [-4.10, 2.33]],
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
                "init_x": 2.19,
                "init_y": -1.34,
                "init_a": -121.14,
                "velocity": 1.12,
                "goals": [[1.70, -2.21], [6.21, -4.43], [-1.14, 2.14], [-0.19, -3.04]],
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
                "init_x": 3.55,
                "init_y": -2.58,
                "init_a": -105.13,
                "velocity": 0.83,
                "goals": [[-1.70, 5.79], [7.13, -4.92], [5.85, -4.40], [-3.48, 1.66]],
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
                "init_x": -0.43,
                "init_y": -0.68,
                "init_a": -90.57,
                "velocity": 0.92,
                "goals": [[2.65, 2.42], [6.09, -0.62], [1.22, -5.52], [2.83, 0.95]],
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
                "init_x": -6.41,
                "init_y": -0.93,
                "init_a": -177.47,
                "velocity": 0.96,
                "goals": [[-7.36, 1.83], [0.98, 1.16], [4.58, -1.96], [-6.43, -5.92]],
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

def tests_adult_0_child_100_test_case_99_stopped_medium(tester):
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
                "goals": [[0.77, 2.15], [1.04, -3.38], [3.99, -0.45], [7.78, -5.28]],
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
                "init_x": 1.52,
                "init_y": 1.48,
                "init_a": -109.88,
                "velocity": 0.91,
                "goals": [[0.77, 2.15], [-1.99, 2.44], [7.88, 0.49], [-7.30, -2.01]],
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
                "init_x": 7.24,
                "init_y": -5.77,
                "init_a": 94.01,
                "velocity": 0.99,
                "goals": [[7.24, -5.77], [5.86, -1.42], [-3.89, -3.77], [3.98, 0.58]],
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
                "init_x": 6.65,
                "init_y": -6.58,
                "init_a": 94.01,
                "velocity": 0.99,
                "goals": [[7.24, -5.77], [-4.78, -1.35], [6.96, -0.03], [5.85, -2.71]],
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
                "init_x": -4.35,
                "init_y": -5.99,
                "init_a": -43.20,
                "velocity": 0.94,
                "goals": [[-1.22, 2.65], [1.98, 1.41], [1.42, 5.08], [-0.04, 2.89]],
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
                "init_x": 1.32,
                "init_y": -0.58,
                "init_a": -131.71,
                "velocity": 1.19,
                "goals": [[4.83, -4.33], [2.97, 5.40], [-6.82, 4.84], [-4.23, -2.03]],
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
                "init_x": -1.35,
                "init_y": 0.48,
                "init_a": -56.40,
                "velocity": 1.13,
                "goals": [[1.13, 1.01], [-1.73, 1.61], [2.64, 2.46], [0.05, -1.66]],
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

def tests_adult_0_child_100_test_case_100_stopped_medium(tester):
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
                "goals": [[-0.41, -3.70], [-5.76, -1.76], [3.16, -4.25], [-5.31, 4.42]],
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
                "init_x": -1.04,
                "init_y": -2.93,
                "init_a": -13.51,
                "velocity": 1.16,
                "goals": [[-0.41, -3.70], [-0.86, -2.83], [1.04, 2.88], [5.31, -5.05]],
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
                "init_x": 5.70,
                "init_y": 0.93,
                "init_a": -126.29,
                "velocity": 0.96,
                "goals": [[5.70, 0.93], [-5.08, -2.86], [7.90, -1.81], [6.81, -2.93]],
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
                "init_x": 5.22,
                "init_y": 0.05,
                "init_a": -126.29,
                "velocity": 0.96,
                "goals": [[5.70, 0.93], [-2.24, -2.17], [3.96, -5.74], [-0.50, 1.36]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.21, 4.51], [-2.40, 1.59], [5.99, -5.75], [-1.05, -0.65]],
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
                "init_x": 3.14,
                "init_y": 3.78,
                "init_a": 161.95,
                "velocity": 1.07,
                "goals": [[7.95, -5.10], [5.00, 4.87], [-3.63, -0.82], [7.28, -1.03]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.01, 1.58], [-4.06, 1.33], [0.49, -5.92], [4.53, 5.48]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[0.60, -4.75], [-3.72, -0.58], [-1.62, 2.21], [7.26, -5.28]],
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
