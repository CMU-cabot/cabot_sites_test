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

def tests_adult_10_child_90_test_case_01_walking_high(tester):
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
                "goals": [[1.33, -4.88], [-1.13, 3.69], [-0.74, -3.57], [4.56, 5.91]],
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
                "goals": [[1.33, -4.88], [4.41, -4.48], [-5.33, -4.40], [-4.47, -1.92]],
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
                "init_x": 0.16,
                "init_y": -3.10,
                "init_a": 178.69,
                "velocity": 0.82,
                "goals": [[-7.55, 0.68], [2.96, -3.10], [-0.53, 4.17], [1.76, -3.59]],
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
                "init_x": 0.85,
                "init_y": -2.37,
                "init_a": 178.69,
                "velocity": 0.82,
                "goals": [[-7.55, 0.68], [7.78, -4.01], [-2.06, -1.93], [4.81, -1.56]],
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
                "goals": [[3.06, 1.47], [0.28, 4.00], [-3.38, -3.44], [3.28, -4.28]],
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
                "init_x": 3.20,
                "init_y": -5.14,
                "init_a": -25.90,
                "velocity": 0.84,
                "goals": [[-6.90, -2.24], [7.67, -3.61], [5.75, -0.08], [-4.55, 4.99]],
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
                "init_x": 4.94,
                "init_y": 5.53,
                "init_a": 35.17,
                "velocity": 0.83,
                "goals": [[-0.07, 0.32], [5.70, -0.92], [4.95, -1.61], [-1.30, -3.47]],
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
                "goals": [[4.23, -5.21], [0.62, -0.41], [-1.30, -3.58], [1.25, -4.24]],
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

def tests_adult_10_child_90_test_case_02_stopped_high(tester):
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
                "goals": [[-2.20, -5.37], [-5.59, 4.60], [2.32, -4.02], [4.30, 0.42]],
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
                "goals": [[-2.20, -5.37], [0.27, -0.93], [4.32, -4.79], [3.92, 5.30]],
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
                "init_x": 1.10,
                "init_y": 2.95,
                "init_a": 45.97,
                "velocity": 1.05,
                "goals": [[1.10, 2.95], [-5.82, 4.17], [5.24, 0.77], [4.97, -1.61]],
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
                "init_x": 1.08,
                "init_y": 3.95,
                "init_a": 45.97,
                "velocity": 1.05,
                "goals": [[1.10, 2.95], [3.91, -3.59], [-5.26, -1.91], [0.32, 2.76]],
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
                "goals": [[-1.03, 1.93], [3.02, -4.09], [-6.28, -4.14], [-0.94, 3.48]],
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
                "goals": [[-3.93, 2.51], [-6.78, -4.85], [3.25, 4.04], [-2.47, -2.72]],
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
                "init_x": -0.62,
                "init_y": -3.01,
                "init_a": 43.75,
                "velocity": 0.85,
                "goals": [[4.69, 4.01], [-5.63, 4.95], [-2.73, 2.73], [-1.23, 4.46]],
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
                "goals": [[5.16, 5.81], [2.05, -4.51], [1.50, 0.87], [5.42, -0.96]],
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

def tests_adult_10_child_90_test_case_03_walking_high(tester):
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
                "goals": [[-7.21, -5.96], [6.42, -0.87], [7.31, 3.64], [-6.53, -1.33]],
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
                "init_x": -8.40,
                "init_y": 3.89,
                "init_a": 85.34,
                "velocity": 0.98,
                "goals": [[-7.21, -5.96], [7.10, 3.41], [-1.47, 2.33], [-3.02, -3.37]],
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
                "goals": [[-1.61, 1.37], [-4.69, -1.88], [1.80, 5.80], [1.90, 1.63]],
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
                "init_x": 1.85,
                "init_y": 5.53,
                "init_a": -24.94,
                "velocity": 1.11,
                "goals": [[-1.61, 1.37], [-4.50, -0.93], [7.36, 4.29], [4.87, -5.94]],
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
                "init_x": -6.30,
                "init_y": -0.73,
                "init_a": -25.18,
                "velocity": 0.95,
                "goals": [[-2.94, 1.75], [1.04, -4.37], [-6.45, 3.65], [-0.68, -5.46]],
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
                "init_x": -7.31,
                "init_y": -2.03,
                "init_a": 113.87,
                "velocity": 0.90,
                "goals": [[-1.00, 5.09], [-6.69, 1.90], [5.04, -5.07], [2.66, 2.94]],
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

def tests_adult_10_child_90_test_case_04_walking_high(tester):
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
                "goals": [[3.93, -5.23], [-6.70, -0.82], [5.89, 0.09], [3.15, 3.30]],
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
                "init_x": 5.88,
                "init_y": 4.67,
                "init_a": -170.80,
                "velocity": 1.10,
                "goals": [[3.93, -5.23], [6.75, 0.82], [5.41, -3.74], [-1.01, 5.88]],
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
                "init_x": -0.29,
                "init_y": -5.48,
                "init_a": 178.16,
                "velocity": 0.92,
                "goals": [[-3.28, -5.61], [-0.09, 2.54], [-1.55, 0.34], [3.43, 3.56]],
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
                "goals": [[-3.28, -5.61], [4.73, -0.44], [2.36, -5.11], [2.67, 2.50]],
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
                "goals": [[-6.62, -3.02], [-4.12, -0.49], [-3.93, -5.24], [-6.58, 5.70]],
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
                "goals": [[-1.59, 5.37], [-5.77, 3.31], [-2.80, -4.97], [7.71, -4.76]],
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
                "init_x": -4.61,
                "init_y": 4.18,
                "init_a": 83.83,
                "velocity": 0.99,
                "goals": [[-3.91, 3.54], [6.64, -0.83], [-6.79, 4.20], [-0.99, -2.57]],
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
                "goals": [[5.78, 4.92], [0.65, 5.74], [1.81, -0.90], [-3.85, 1.71]],
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

def tests_adult_10_child_90_test_case_05_stopped_high(tester):
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
                "init_x": 2.05,
                "init_y": 2.86,
                "init_a": 23.79,
                "velocity": 0.85,
                "goals": [[2.05, 2.86], [2.29, 3.34], [-6.02, -5.13], [5.61, 5.29]],
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
                "goals": [[1.17, 0.49], [0.88, -0.84], [-0.82, -4.19], [-5.44, -0.38]],
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
                "init_x": 1.52,
                "init_y": 1.43,
                "init_a": -12.95,
                "velocity": 1.08,
                "goals": [[1.17, 0.49], [4.17, 2.19], [0.23, 0.08], [1.29, -0.88]],
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
                "init_x": -1.68,
                "init_y": -5.41,
                "init_a": 82.34,
                "velocity": 1.16,
                "goals": [[0.40, 0.79], [2.20, 3.14], [4.48, 1.66], [-7.01, 5.18]],
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
                "init_x": 0.94,
                "init_y": -3.31,
                "init_a": -128.96,
                "velocity": 1.01,
                "goals": [[7.95, 3.69], [-1.00, 5.32], [-2.42, 5.39], [-6.87, -0.39]],
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
                "init_x": 5.86,
                "init_y": -1.01,
                "init_a": 104.27,
                "velocity": 0.83,
                "goals": [[-3.89, -4.20], [-3.31, -0.98], [4.81, 0.90], [7.79, 5.01]],
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
                "init_x": 3.94,
                "init_y": -4.63,
                "init_a": 114.71,
                "velocity": 1.16,
                "goals": [[-4.39, 1.09], [2.41, 0.69], [-0.80, 1.51], [6.77, -4.92]],
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
                "init_x": 5.60,
                "init_y": 1.83,
                "init_a": 105.30,
                "velocity": 0.95,
                "goals": [[-2.42, 2.88], [7.47, 5.29], [-3.19, 4.07], [-1.85, 5.18]],
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

def tests_adult_10_child_90_test_case_06_stopped_high(tester):
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
                "goals": [[-7.86, 4.64], [-5.51, 3.46], [-5.45, 5.21], [4.72, -5.85]],
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
                "goals": [[-7.86, 4.64], [-7.46, 2.72], [3.90, 0.39], [2.62, 3.88]],
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
                "goals": [[5.29, -1.92], [6.47, 1.97], [1.40, -1.19], [3.92, 5.50]],
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
                "init_x": -0.30,
                "init_y": 0.50,
                "init_a": 130.30,
                "velocity": 0.85,
                "goals": [[-7.86, 0.86], [-0.40, -2.80], [-7.51, -3.53], [-6.20, 1.93]],
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
                "init_x": 1.82,
                "init_y": -1.05,
                "init_a": -12.64,
                "velocity": 0.98,
                "goals": [[3.97, 2.26], [6.35, -5.49], [0.52, 4.21], [-2.93, -0.36]],
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
                "goals": [[1.27, -5.58], [-6.12, 1.23], [-3.28, 4.42], [5.30, -2.42]],
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
                "init_x": 1.59,
                "init_y": 1.66,
                "init_a": -167.53,
                "velocity": 1.16,
                "goals": [[-3.61, 3.59], [2.22, -5.54], [4.98, 0.58], [6.33, 3.26]],
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

def tests_adult_10_child_90_test_case_07_stopped_high(tester):
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
                "goals": [[6.96, 4.07], [-2.14, 3.96], [-0.81, -5.87], [2.09, -3.72]],
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
                "init_x": 6.27,
                "init_y": 4.79,
                "init_a": 69.71,
                "velocity": 0.85,
                "goals": [[6.96, 4.07], [5.19, 3.00], [-7.56, -3.58], [7.88, 2.10]],
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
                "init_x": -6.10,
                "init_y": -3.72,
                "init_a": -3.82,
                "velocity": 1.18,
                "goals": [[-6.10, -3.72], [5.05, -2.08], [-1.29, -1.12], [-4.68, -4.69]],
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
                "init_x": -5.50,
                "init_y": -2.91,
                "init_a": -3.82,
                "velocity": 1.18,
                "goals": [[-6.10, -3.72], [4.40, -5.58], [-3.06, 1.64], [-2.70, -1.49]],
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
                "init_x": 5.29,
                "init_y": -2.36,
                "init_a": -63.89,
                "velocity": 1.00,
                "goals": [[2.70, -5.86], [-5.33, -1.73], [-4.12, -5.68], [6.43, 4.00]],
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
                "init_x": -4.74,
                "init_y": -5.15,
                "init_a": 69.68,
                "velocity": 0.91,
                "goals": [[-7.54, 2.22], [3.57, -5.83], [-4.32, 5.99], [-5.57, -1.05]],
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

def tests_adult_10_child_90_test_case_08_walking_high(tester):
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
                "goals": [[-4.33, 1.17], [5.62, -3.18], [-4.19, 4.92], [3.38, 4.63]],
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
                "goals": [[-4.33, 1.17], [-7.29, 3.23], [0.70, -2.72], [2.85, -3.19]],
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
                "init_x": -0.79,
                "init_y": -4.42,
                "init_a": -68.62,
                "velocity": 1.12,
                "goals": [[-3.67, -5.49], [-6.51, 1.06], [5.99, 2.50], [3.01, -4.10]],
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
                "init_x": -1.79,
                "init_y": -4.47,
                "init_a": -68.62,
                "velocity": 1.12,
                "goal_x": -3.67,
                "goal_y": -5.49,
                "n_actors": 9,
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
                "goals": [[-2.16, -0.09], [-1.77, 0.76], [7.60, -1.37], [5.79, -3.11]],
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
                "init_x": -0.12,
                "init_y": -1.92,
                "init_a": 79.02,
                "velocity": 0.88,
                "goals": [[6.02, -4.57], [-0.36, 3.19], [5.92, -2.60], [6.97, 4.79]],
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
                "init_x": 7.39,
                "init_y": -4.02,
                "init_a": 3.39,
                "velocity": 1.06,
                "goals": [[5.03, -0.86], [-3.28, -2.03], [7.14, -5.38], [5.70, 1.55]],
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
                "init_x": 3.42,
                "init_y": -1.79,
                "init_a": 64.83,
                "velocity": 1.20,
                "goals": [[4.49, -0.30], [-5.34, -0.24], [-3.37, -1.64], [4.99, 1.88]],
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
                "init_x": -2.49,
                "init_y": 4.70,
                "init_a": -51.15,
                "velocity": 1.09,
                "goals": [[7.74, 5.10], [-6.24, 3.86], [-6.55, -4.88], [7.42, -4.22]],
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

def tests_adult_10_child_90_test_case_09_stopped_high(tester):
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
                "goals": [[-3.95, 0.77], [2.67, -0.86], [-3.35, 0.87], [4.85, 1.90]],
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
                "init_x": -4.95,
                "init_y": 0.69,
                "init_a": 24.94,
                "velocity": 1.04,
                "goals": [[-3.95, 0.77], [-0.74, -1.19], [1.36, 0.48], [-6.44, -5.94]],
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
                "init_x": 3.40,
                "init_y": 4.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goals": [[3.40, 4.71], [-0.35, -0.40], [0.73, -3.23], [-6.61, 1.87]],
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
                "init_x": 3.46,
                "init_y": 3.71,
                "init_a": 41.58,
                "velocity": 1.02,
                "goals": [[3.40, 4.71], [-6.04, 1.80], [4.79, -1.73], [-2.71, -3.91]],
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
                "goals": [[-0.89, 3.23], [7.48, -0.10], [-5.38, -2.56], [-7.65, -3.85]],
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
                "goals": [[0.32, -3.93], [-7.45, -5.99], [2.61, -5.04], [4.66, -5.90]],
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

def tests_adult_10_child_90_test_case_10_walking_high(tester):
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
                "goals": [[7.33, 2.68], [3.13, -3.39], [5.08, 2.17], [-7.29, -3.13]],
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
                "init_x": -2.32,
                "init_y": 1.46,
                "init_a": 78.72,
                "velocity": 0.94,
                "goals": [[6.95, 2.73], [-5.97, 1.00], [-5.43, 4.98], [1.40, -1.21]],
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
                "init_x": -1.34,
                "init_y": 1.29,
                "init_a": 78.72,
                "velocity": 0.94,
                "goals": [[6.95, 2.73], [5.31, 1.16], [-7.97, -3.28], [1.31, 0.89]],
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
                "init_x": -7.96,
                "init_y": 1.52,
                "init_a": 104.30,
                "velocity": 0.86,
                "goals": [[1.44, 5.72], [-4.75, -2.27], [-4.55, 1.05], [1.86, -3.70]],
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
                "init_x": -3.63,
                "init_y": -5.88,
                "init_a": 121.01,
                "velocity": 0.85,
                "goals": [[-6.05, 4.07], [-5.59, -2.59], [-1.33, 0.68], [3.55, -2.45]],
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
                "init_x": 5.47,
                "init_y": 0.69,
                "init_a": -70.82,
                "velocity": 1.15,
                "goals": [[-2.04, -4.32], [-7.15, 5.16], [1.25, 0.63], [-1.99, -3.36]],
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
                "goals": [[4.23, -3.82], [-1.06, -4.96], [-7.76, -2.37], [2.35, 4.01]],
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

def tests_adult_10_child_90_test_case_11_walking_high(tester):
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
                "goals": [[3.12, 4.98], [1.25, 4.04], [6.11, -1.90], [-2.10, -0.32]],
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
                "goals": [[3.12, 4.98], [-1.50, 2.85], [-4.23, 3.06], [5.22, 3.70]],
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
                "init_x": 2.23,
                "init_y": -1.79,
                "init_a": -56.37,
                "velocity": 0.91,
                "goals": [[5.16, -3.74], [7.40, 3.18], [-5.22, 5.61], [-7.62, 1.26]],
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
                "init_x": 1.72,
                "init_y": -2.65,
                "init_a": -56.37,
                "velocity": 0.91,
                "goals": [[5.16, -3.74], [-3.73, -4.13], [-3.95, -5.21], [-3.96, 2.29]],
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
                "goals": [[-5.39, -3.91], [4.71, 0.99], [0.97, -0.16], [-0.06, 0.31]],
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
                "goals": [[-4.40, -3.83], [-7.18, -1.77], [-5.88, 5.22], [-7.42, -1.74]],
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
                "goals": [[7.17, 4.38], [-4.78, -4.93], [-7.05, 4.63], [2.95, 3.49]],
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

def tests_adult_10_child_90_test_case_12_walking_high(tester):
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
                "goals": [[2.00, 5.30], [-1.49, -3.17], [-0.02, 0.39], [5.71, 0.38]],
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
                "goals": [[2.00, 5.30], [5.15, -0.53], [-1.52, 2.58], [-5.01, -2.15]],
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
                "goals": [[0.82, 5.85], [-3.76, 3.82], [1.80, 2.17], [-4.05, 0.34]],
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
                "init_x": -2.55,
                "init_y": 3.79,
                "init_a": -122.78,
                "velocity": 0.95,
                "goals": [[0.82, 5.85], [-5.59, 1.31], [-4.36, 2.93], [-4.66, 2.45]],
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
                "init_x": -0.66,
                "init_y": 5.89,
                "init_a": 127.85,
                "velocity": 1.19,
                "goals": [[7.95, 3.92], [-7.37, -5.27], [4.00, 1.73], [-2.53, -0.54]],
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
                "goals": [[5.04, -0.76], [-4.13, 3.72], [6.83, -4.37], [0.70, 4.90]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.68,
                "init_y": -4.72,
                "init_a": 162.11,
                "velocity": 0.83,
                "goals": [[4.32, 3.00], [6.25, -5.72], [1.27, -5.33], [-4.68, -3.43]],
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

def tests_adult_10_child_90_test_case_13_stopped_high(tester):
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
                "goals": [[-5.18, -3.02], [4.28, 3.77], [-7.02, 0.09], [6.70, 2.14]],
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
                "goals": [[-5.18, -3.02], [4.55, -5.68], [0.07, 1.55], [-3.83, 0.47]],
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
                "goals": [[2.06, 1.40], [-7.75, 0.93], [0.51, -0.28], [-2.32, -2.41]],
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
                "init_x": 1.27,
                "init_y": 2.01,
                "init_a": 106.78,
                "velocity": 0.88,
                "goals": [[2.06, 1.40], [-3.04, 1.07], [-0.72, 5.42], [-5.43, -4.55]],
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
                "init_x": 6.06,
                "init_y": 5.55,
                "init_a": -17.15,
                "velocity": 1.03,
                "goals": [[-5.29, -3.55], [-3.66, -2.49], [2.21, -1.89], [4.54, -4.74]],
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
                "goals": [[6.06, 2.43], [-3.34, -0.49], [-4.23, 5.63], [7.95, 0.44]],
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
                "init_x": -5.44,
                "init_y": 0.03,
                "init_a": -80.05,
                "velocity": 0.87,
                "goals": [[-0.64, -2.50], [6.69, 5.24], [2.50, 4.89], [2.42, 5.02]],
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
                "init_x": -5.01,
                "init_y": 3.68,
                "init_a": 126.76,
                "velocity": 1.20,
                "goal_x": 4.85,
                "goal_y": 3.54,
                "n_actors": 9,
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
                "goals": [[-6.41, -2.51], [-7.83, -3.84], [-5.82, 4.17], [5.59, 5.78]],
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

def tests_adult_10_child_90_test_case_14_walking_high(tester):
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
                "goals": [[-3.91, -4.23], [-4.63, 5.79], [2.84, 2.26], [4.13, 1.67]],
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
                "goals": [[-3.91, -4.23], [-6.86, 1.87], [-0.43, -1.72], [-4.92, 1.96]],
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
                "init_x": 3.55,
                "init_y": 0.34,
                "init_a": 10.47,
                "velocity": 1.17,
                "goals": [[-0.96, 1.37], [3.34, -5.09], [3.15, 3.25], [-1.55, 3.30]],
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
                "init_x": 3.00,
                "init_y": -0.49,
                "init_a": 10.47,
                "velocity": 1.17,
                "goals": [[-0.96, 1.37], [-6.41, 2.51], [-6.75, 4.83], [-7.88, -3.03]],
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.82,
                "init_y": -3.44,
                "init_a": -147.27,
                "velocity": 0.91,
                "goals": [[-3.70, 1.42], [-7.43, 2.24], [5.73, -0.30], [2.98, -3.85]],
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
                "init_x": 7.41,
                "init_y": -0.93,
                "init_a": 25.66,
                "velocity": 0.88,
                "goals": [[5.62, -4.80], [1.07, 0.15], [2.79, -5.78], [7.94, 0.58]],
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

def tests_adult_10_child_90_test_case_15_walking_high(tester):
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
                "goals": [[5.44, 1.84], [-5.34, 4.85], [4.30, -4.22], [4.89, -5.87]],
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
                "goals": [[5.44, 1.84], [-7.43, -5.76], [-3.91, 0.30], [7.68, -5.34]],
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
                "goals": [[-4.50, -1.26], [5.25, 5.40], [-4.19, -1.09], [-1.85, 1.03]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.58,
                "init_y": -4.36,
                "init_a": -55.50,
                "velocity": 0.91,
                "goals": [[1.80, 1.34], [0.55, -1.03], [-2.30, 1.84], [-6.80, 4.51]],
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
                "init_x": 0.78,
                "init_y": 3.56,
                "init_a": 174.91,
                "velocity": 1.09,
                "goals": [[-0.84, 0.85], [3.63, 1.12], [-3.54, -2.97], [0.62, -0.73]],
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
                "init_x": 2.30,
                "init_y": 1.15,
                "init_a": -105.55,
                "velocity": 1.01,
                "goals": [[-1.51, 1.04], [-4.08, 4.11], [4.72, 5.50], [-3.51, 5.78]],
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
                "goals": [[3.89, 6.00], [-0.07, -4.37], [-5.04, -3.00], [-0.59, -3.15]],
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
                "init_x": -4.03,
                "init_y": 2.44,
                "init_a": -124.17,
                "velocity": 1.07,
                "goals": [[-5.48, -1.67], [-3.19, 0.96], [-3.84, -0.81], [-1.20, 5.53]],
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

def tests_adult_10_child_90_test_case_16_stopped_high(tester):
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
                "goals": [[-6.55, -5.05], [-3.01, -2.34], [-1.97, -5.73], [2.54, -0.90]],
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
                "goals": [[-6.55, -5.05], [-7.92, 2.82], [7.46, -0.24], [1.78, -4.04]],
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
                "goals": [[1.69, 0.82], [7.55, 2.04], [0.88, -3.01], [-5.77, 1.40]],
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
                "goals": [[2.77, 5.66], [0.29, 1.55], [-3.63, -2.58], [-6.29, -5.91]],
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
                "goals": [[2.95, -1.66], [-6.83, -4.86], [5.19, -0.53], [-0.01, 5.40]],
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
                "goals": [[-5.43, 0.96], [6.91, -4.30], [7.21, -4.94], [-4.34, -4.46]],
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
                "init_x": -3.30,
                "init_y": 0.16,
                "init_a": -27.20,
                "velocity": 1.11,
                "goals": [[-1.50, -3.07], [3.55, -3.26], [6.17, 5.26], [-1.41, 3.85]],
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
                "init_x": 7.14,
                "init_y": -5.54,
                "init_a": -177.69,
                "velocity": 1.04,
                "goals": [[0.46, -4.98], [-3.92, -2.04], [0.07, -1.67], [5.43, 4.97]],
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

def tests_adult_10_child_90_test_case_17_walking_high(tester):
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
                "goals": [[-6.97, -2.49], [0.77, 5.64], [7.59, 1.33], [3.85, -2.99]],
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
                "init_x": -3.30,
                "init_y": -1.21,
                "init_a": -145.87,
                "velocity": 1.13,
                "goals": [[-6.97, -2.49], [-0.82, -1.05], [2.35, 2.11], [6.14, 4.95]],
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
                "goals": [[-2.68, -5.19], [-2.86, 1.91], [-2.08, -2.71], [-2.61, 2.20]],
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
                "goals": [[-2.68, -5.19], [5.58, -4.39], [7.37, -4.31], [-7.42, 3.32]],
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
                "goals": [[5.68, -0.28], [-0.30, 1.26], [0.27, 4.20], [7.93, 2.72]],
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
                "goals": [[3.73, 2.84], [-3.33, 0.87], [4.77, 2.47], [-7.58, 3.12]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.20,
                "init_y": 2.57,
                "init_a": -114.47,
                "velocity": 0.97,
                "goals": [[-4.25, -3.98], [-4.36, 0.15], [6.40, -4.22], [-6.81, -5.26]],
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

def tests_adult_10_child_90_test_case_18_stopped_high(tester):
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
                "goals": [[0.31, -0.60], [2.73, 3.13], [3.26, -0.61], [-1.21, 3.59]],
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
                "init_x": 0.95,
                "init_y": 0.17,
                "init_a": -84.34,
                "velocity": 1.01,
                "goals": [[0.31, -0.60], [-1.93, 4.65], [4.71, 4.59], [-2.88, -5.74]],
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
                "goals": [[3.35, 1.81], [-0.83, 0.77], [-5.08, -5.74], [2.41, 5.64]],
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
                "init_x": 2.36,
                "init_y": 1.96,
                "init_a": -69.12,
                "velocity": 0.81,
                "goals": [[3.35, 1.81], [-5.33, -1.28], [4.98, -1.33], [-1.75, 0.55]],
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.39,
                "init_y": 4.85,
                "init_a": 70.57,
                "velocity": 0.82,
                "goals": [[-5.02, 0.17], [2.22, 2.13], [-1.26, 3.65], [6.92, -2.47]],
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
                "init_x": 5.93,
                "init_y": 1.89,
                "init_a": 133.26,
                "velocity": 0.91,
                "goals": [[2.02, 4.88], [-4.03, 3.66], [-7.43, 4.52], [4.44, 5.74]],
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

def tests_adult_10_child_90_test_case_19_walking_high(tester):
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
                "goals": [[5.86, -5.17], [7.10, 2.94], [3.58, -5.90], [4.25, -5.31]],
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
                "init_x": -0.58,
                "init_y": 0.45,
                "init_a": 139.11,
                "velocity": 0.99,
                "goals": [[5.86, -5.17], [4.86, 0.42], [5.16, 0.58], [-5.44, -1.58]],
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
                "init_x": -0.55,
                "init_y": 1.76,
                "init_a": 115.59,
                "velocity": 0.98,
                "goals": [[1.77, 2.95], [-6.55, 4.49], [4.85, -0.49], [-3.49, -5.05]],
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
                "goals": [[1.77, 2.95], [4.13, -4.32], [-6.84, 0.24], [-4.93, -1.63]],
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.27,
                "init_y": 2.25,
                "init_a": -73.45,
                "velocity": 1.03,
                "goals": [[-1.16, -3.00], [-1.32, -1.52], [-0.59, -4.01], [0.19, -3.29]],
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
                "init_x": -1.20,
                "init_y": 0.12,
                "init_a": -48.05,
                "velocity": 1.10,
                "goals": [[7.29, 4.25], [7.29, -3.32], [-4.95, -5.07], [-6.77, 4.78]],
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

def tests_adult_10_child_90_test_case_20_stopped_high(tester):
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
                "goals": [[-1.01, -0.02], [-4.19, 0.45], [-1.46, -0.20], [5.97, 3.87]],
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
                "goals": [[-1.01, -0.02], [-2.53, -2.68], [-2.70, -2.20], [-2.45, 5.41]],
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
                "goals": [[1.23, -1.65], [4.98, -5.41], [-5.87, -5.09], [3.13, -1.52]],
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
                "init_x": 7.56,
                "init_y": 1.95,
                "init_a": 133.43,
                "velocity": 0.98,
                "goals": [[5.90, -1.13], [-2.38, 5.44], [-2.49, 1.78], [5.25, -1.47]],
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
                "init_x": -3.53,
                "init_y": 3.64,
                "init_a": 54.42,
                "velocity": 0.88,
                "goals": [[4.04, 3.23], [3.48, 1.60], [-5.43, -0.96], [5.86, 4.75]],
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
                "init_x": 7.96,
                "init_y": 2.02,
                "init_a": -169.07,
                "velocity": 1.16,
                "goals": [[0.74, 4.24], [-5.45, -5.04], [4.95, -0.58], [-7.86, -0.92]],
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
                "init_x": -2.20,
                "init_y": -5.99,
                "init_a": -58.94,
                "velocity": 0.90,
                "goals": [[-4.91, 1.40], [0.85, -1.16], [2.42, 3.28], [6.55, 3.60]],
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

def tests_adult_10_child_90_test_case_21_stopped_high(tester):
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
                "init_x": 4.30,
                "init_y": 5.63,
                "init_a": 38.92,
                "velocity": 0.90,
                "goal_x": 4.30,
                "goal_y": 5.63,
                "n_actors": 8,
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
                "goals": [[4.30, 5.63], [6.23, -1.43], [-0.06, -4.82], [-7.56, 3.44]],
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
                "init_x": 6.83,
                "init_y": -5.95,
                "init_a": 44.68,
                "velocity": 0.92,
                "goals": [[6.83, -5.95], [-7.12, 3.08], [-6.02, 3.09], [-6.51, -1.24]],
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
                "init_x": 7.19,
                "init_y": -5.01,
                "init_a": 44.68,
                "velocity": 0.92,
                "goals": [[6.83, -5.95], [-3.65, 1.66], [-0.79, -2.95], [-3.76, 4.69]],
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
                "init_x": 1.64,
                "init_y": -2.02,
                "init_a": 37.98,
                "velocity": 1.08,
                "goals": [[-7.17, 3.33], [7.15, -4.26], [4.90, -1.68], [2.15, 5.09]],
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
                "init_x": -4.96,
                "init_y": 1.86,
                "init_a": 1.30,
                "velocity": 1.15,
                "goals": [[-0.53, 1.73], [-5.37, 2.07], [-7.58, 5.58], [3.50, -4.80]],
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
                "goals": [[-5.95, 4.01], [2.51, -1.23], [-4.64, -1.24], [-4.47, 0.88]],
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
                "goals": [[-3.26, -5.17], [-1.26, 3.93], [4.33, 1.03], [-4.99, 4.52]],
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

def tests_adult_10_child_90_test_case_22_walking_high(tester):
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
                "goals": [[-0.12, -5.67], [-3.66, 0.33], [2.33, -0.09], [4.92, 0.61]],
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
                "init_x": 1.10,
                "init_y": 2.08,
                "init_a": -62.86,
                "velocity": 1.02,
                "goals": [[-0.12, -5.67], [1.31, -1.60], [3.10, -2.17], [3.18, -1.19]],
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
                "goals": [[-1.87, 1.12], [7.33, 2.74], [0.95, 3.41], [-4.16, 0.13]],
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
                "goals": [[3.01, -0.47], [-0.71, -2.84], [1.66, 2.95], [2.65, -0.74]],
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
                "init_x": -3.78,
                "init_y": -1.27,
                "init_a": -128.07,
                "velocity": 0.88,
                "goals": [[3.60, 1.88], [2.22, -1.27], [0.03, -2.97], [2.85, -4.18]],
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
                "init_x": -0.28,
                "init_y": 3.41,
                "init_a": -123.21,
                "velocity": 1.03,
                "goals": [[6.26, -3.32], [-3.05, -0.64], [4.81, -3.74], [3.71, 2.03]],
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

def tests_adult_10_child_90_test_case_23_walking_high(tester):
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
                "goals": [[-2.58, 4.91], [6.83, 4.26], [-7.43, -1.70], [-7.96, -1.11]],
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
                "goals": [[-2.58, 4.91], [4.90, -1.54], [-4.41, 2.78], [0.58, -2.89]],
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
                "init_x": 0.05,
                "init_y": 5.36,
                "init_a": -21.93,
                "velocity": 1.07,
                "goals": [[4.46, 0.65], [-7.69, -2.36], [-2.35, 4.81], [2.42, -4.67]],
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
                "init_x": 0.63,
                "init_y": 4.55,
                "init_a": -21.93,
                "velocity": 1.07,
                "goals": [[4.46, 0.65], [-2.51, 1.73], [6.84, 4.05], [0.51, -5.93]],
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
                "goals": [[6.19, -3.25], [-2.56, 1.02], [4.04, -1.32], [-3.20, -3.21]],
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
                "init_x": 4.09,
                "init_y": 3.93,
                "init_a": 95.03,
                "velocity": 1.03,
                "goal_x": -6.87,
                "goal_y": -3.96,
                "n_actors": 7,
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
                "goals": [[1.23, -5.29], [-3.00, 3.22], [-2.26, -5.79], [5.40, 3.63]],
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

def tests_adult_10_child_90_test_case_24_stopped_high(tester):
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
                "init_x": -7.24,
                "init_y": 1.09,
                "init_a": 144.32,
                "velocity": 1.15,
                "goals": [[-7.24, 1.09], [7.87, -5.91], [3.47, -2.67], [-0.73, -0.43]],
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
                "init_x": -6.92,
                "init_y": 2.04,
                "init_a": 144.32,
                "velocity": 1.15,
                "goals": [[-7.24, 1.09], [6.78, -5.85], [2.43, -4.34], [3.11, 5.80]],
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
                "init_x": 3.44,
                "init_y": -2.45,
                "init_a": 127.26,
                "velocity": 0.87,
                "goal_x": 3.44,
                "goal_y": -2.45,
                "n_actors": 7,
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
                "goals": [[3.44, -2.45], [-2.52, -1.05], [1.85, 5.11], [-4.78, 0.99]],
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
                "goals": [[1.02, 3.12], [2.25, -2.12], [1.40, 5.52], [-6.22, -0.15]],
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
                "init_x": -0.55,
                "init_y": 1.61,
                "init_a": 119.66,
                "velocity": 0.95,
                "goals": [[1.18, 1.43], [-2.32, -3.50], [5.80, -0.73], [-2.16, 2.55]],
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
                "init_x": 2.36,
                "init_y": -2.20,
                "init_a": 147.26,
                "velocity": 0.81,
                "goals": [[-1.24, -3.52], [7.32, -2.43], [6.08, -5.45], [1.27, -3.23]],
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

def tests_adult_10_child_90_test_case_25_stopped_high(tester):
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
                "goals": [[1.98, 0.61], [0.37, -2.13], [-0.00, -5.02], [-6.63, -2.92]],
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
                "goals": [[1.98, 0.61], [-3.64, 4.45], [0.89, -2.13], [7.78, 0.06]],
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
                "init_x": -0.45,
                "init_y": -2.13,
                "init_a": -100.62,
                "velocity": 0.84,
                "goals": [[-0.45, -2.13], [-4.27, -4.08], [3.82, -5.74], [-6.48, -2.88]],
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
                "goals": [[-4.95, -2.61], [3.52, -3.09], [0.30, -0.78], [-3.30, -5.16]],
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
                "init_x": -4.26,
                "init_y": -5.25,
                "init_a": 99.04,
                "velocity": 0.84,
                "goals": [[-7.89, -4.06], [-6.42, 2.73], [7.54, 1.75], [-7.02, -5.14]],
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
                "init_x": 7.23,
                "init_y": -1.59,
                "init_a": 46.51,
                "velocity": 0.89,
                "goals": [[5.63, -3.67], [-1.86, 4.80], [-2.04, 2.91], [3.73, -3.87]],
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
                "goals": [[1.13, -2.94], [-1.22, 3.47], [-5.22, -4.96], [0.07, 3.70]],
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
                "goals": [[-3.59, -5.29], [3.42, -1.44], [-6.42, -0.23], [-2.07, -5.89]],
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

def tests_adult_10_child_90_test_case_26_stopped_high(tester):
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
                "init_x": 3.65,
                "init_y": 0.78,
                "init_a": -127.80,
                "velocity": 1.12,
                "goal_x": 3.65,
                "goal_y": 0.78,
                "n_actors": 9,
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
                "goals": [[3.65, 0.78], [6.58, 2.17], [5.87, 0.65], [7.40, -3.95]],
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
                "init_x": 5.42,
                "init_y": 1.51,
                "init_a": 108.02,
                "velocity": 1.20,
                "goals": [[5.42, 1.51], [6.27, 3.34], [7.08, 2.96], [-7.26, -2.58]],
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
                "init_x": 4.65,
                "init_y": 0.87,
                "init_a": 108.02,
                "velocity": 1.20,
                "goals": [[5.42, 1.51], [7.82, -0.50], [0.63, -5.61], [-1.94, -0.21]],
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
                "init_x": -1.38,
                "init_y": -4.17,
                "init_a": 108.82,
                "velocity": 1.01,
                "goals": [[6.90, 1.37], [-1.43, 1.49], [7.84, 0.05], [4.28, 3.37]],
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
                "init_x": -0.02,
                "init_y": -0.17,
                "init_a": 43.03,
                "velocity": 0.95,
                "goals": [[5.11, -3.62], [7.22, -5.20], [7.16, 2.27], [1.50, 4.61]],
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
                "init_x": 4.54,
                "init_y": 4.74,
                "init_a": -41.76,
                "velocity": 0.82,
                "goals": [[0.69, -4.67], [6.11, -5.24], [7.61, -5.13], [-7.15, 2.06]],
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
                "goals": [[0.40, 0.88], [-1.07, -2.59], [2.41, 2.96], [-4.87, 2.13]],
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
                "goals": [[5.92, 2.13], [-0.11, -1.83], [-2.02, -4.70], [-6.44, -2.36]],
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

def tests_adult_10_child_90_test_case_27_walking_high(tester):
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
                "init_x": -4.65,
                "init_y": 5.95,
                "init_a": -174.00,
                "velocity": 0.93,
                "goal_x": 2.09,
                "goal_y": -0.92,
                "n_actors": 8,
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
                "goals": [[2.09, -0.92], [-3.94, -5.69], [-2.74, -5.39], [-4.96, 1.55]],
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
                "init_x": -1.36,
                "init_y": 5.49,
                "init_a": 64.23,
                "velocity": 1.17,
                "goals": [[-2.03, -4.57], [3.06, 2.14], [-7.61, -5.44], [5.70, -2.42]],
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
                "init_x": -1.62,
                "init_y": 4.53,
                "init_a": 64.23,
                "velocity": 1.17,
                "goals": [[-2.03, -4.57], [-0.28, 2.48], [-1.41, 5.24], [-5.71, -5.39]],
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
                "init_x": -3.09,
                "init_y": -4.75,
                "init_a": -166.89,
                "velocity": 0.80,
                "goals": [[-6.11, 2.08], [-0.89, -3.01], [1.48, -1.48], [3.45, -1.20]],
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
                "init_x": -2.54,
                "init_y": -2.62,
                "init_a": 121.59,
                "velocity": 0.97,
                "goals": [[-3.08, 0.68], [-0.37, 0.97], [-5.54, -2.56], [-3.72, -4.96]],
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
                "init_x": 4.86,
                "init_y": 1.46,
                "init_a": -61.77,
                "velocity": 0.96,
                "goals": [[-4.06, 3.71], [3.59, 2.79], [-4.04, -1.81], [1.38, -5.49]],
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
                "init_x": -3.98,
                "init_y": 3.77,
                "init_a": -85.03,
                "velocity": 0.94,
                "goals": [[3.70, 0.17], [-7.56, 4.25], [-4.94, 2.81], [-2.06, 1.64]],
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

def tests_adult_10_child_90_test_case_28_walking_high(tester):
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
                "goals": [[7.97, -3.35], [-0.22, -4.16], [-2.16, 0.49], [-4.20, 1.79]],
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
                "init_x": 4.93,
                "init_y": 2.82,
                "init_a": -131.64,
                "velocity": 1.17,
                "goals": [[7.97, -3.35], [2.78, 3.33], [-5.17, -2.85], [-5.26, 4.28]],
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
                "init_x": -0.31,
                "init_y": -3.52,
                "init_a": 0.96,
                "velocity": 0.84,
                "goals": [[7.13, 3.79], [2.93, 5.05], [-5.48, -1.51], [0.67, 3.61]],
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
                "init_x": 0.23,
                "init_y": -2.68,
                "init_a": 0.96,
                "velocity": 0.84,
                "goals": [[7.13, 3.79], [-5.10, 4.71], [-5.23, -2.90], [0.15, -4.00]],
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
                "goals": [[5.91, 0.30], [-2.17, -4.40], [1.56, 3.14], [-5.90, -2.46]],
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
                "init_x": 4.61,
                "init_y": 4.57,
                "init_a": -31.19,
                "velocity": 1.02,
                "goals": [[6.86, -1.43], [-3.13, -4.80], [-6.38, -5.22], [-1.35, -1.12]],
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
                "init_x": 0.95,
                "init_y": 1.66,
                "init_a": 52.24,
                "velocity": 1.02,
                "goals": [[-4.69, 2.62], [5.38, -2.54], [-0.22, 0.93], [-6.07, -0.14]],
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

def tests_adult_10_child_90_test_case_29_walking_high(tester):
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
                "goals": [[-4.89, 0.56], [3.55, -0.65], [5.58, 3.41], [-4.70, 4.26]],
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
                "init_x": -0.79,
                "init_y": -3.87,
                "init_a": 107.23,
                "velocity": 0.97,
                "goals": [[-4.89, 0.56], [3.47, -3.24], [-3.50, 1.14], [3.15, 0.57]],
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
                "init_x": -7.30,
                "init_y": -0.17,
                "init_a": -176.10,
                "velocity": 0.99,
                "goal_x": 2.76,
                "goal_y": 2.61,
                "n_actors": 8,
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
                "goals": [[2.76, 2.61], [-1.14, -5.84], [7.34, 2.63], [-3.00, -5.08]],
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
                "init_x": 3.36,
                "init_y": 5.71,
                "init_a": 121.57,
                "velocity": 0.99,
                "goals": [[-4.09, 2.16], [-6.65, -2.98], [-6.39, 0.28], [1.07, -3.49]],
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
                "init_x": -7.25,
                "init_y": -3.25,
                "init_a": -91.75,
                "velocity": 0.82,
                "goals": [[-1.68, -1.92], [7.45, 3.77], [-4.16, 3.57], [-4.81, 4.18]],
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
                "init_x": 0.55,
                "init_y": -5.72,
                "init_a": 39.05,
                "velocity": 0.93,
                "goals": [[-5.65, -0.63], [-5.28, -4.93], [3.90, 0.54], [6.09, 0.71]],
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
                "init_x": 0.17,
                "init_y": -1.93,
                "init_a": -136.42,
                "velocity": 0.99,
                "goals": [[-0.85, 3.41], [6.97, -5.55], [1.55, -0.66], [7.90, -2.60]],
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

def tests_adult_10_child_90_test_case_30_walking_high(tester):
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
                "goals": [[-5.84, -5.51], [-3.64, 4.25], [4.97, -5.02], [-6.90, 3.06]],
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
                "goals": [[-5.84, -5.51], [-1.96, 5.75], [3.95, -0.65], [5.67, 5.40]],
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
                "goals": [[-3.80, 0.36], [6.23, -5.88], [-3.56, 1.05], [-6.97, 4.99]],
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
                "init_x": -3.81,
                "init_y": 5.17,
                "init_a": -109.88,
                "velocity": 0.83,
                "goals": [[-3.80, 0.36], [-2.27, 3.33], [1.80, 2.68], [-6.46, 5.81]],
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
                "init_x": 0.03,
                "init_y": -2.63,
                "init_a": -62.33,
                "velocity": 1.10,
                "goals": [[-1.92, -2.70], [2.01, 1.06], [-3.56, 2.87], [-2.26, 2.97]],
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
                "goals": [[4.33, 1.13], [4.69, 1.11], [-4.45, -4.37], [5.89, 3.66]],
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

def tests_adult_10_child_90_test_case_31_stopped_high(tester):
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
                "goals": [[7.04, -2.91], [-1.22, -0.89], [-2.49, -3.22], [-1.35, 3.67]],
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
                "goals": [[1.59, -2.71], [-5.09, -1.26], [-5.88, 2.33], [-5.54, -2.79]],
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
                "goals": [[1.59, -2.71], [-2.45, -4.08], [-1.58, 5.55], [6.11, 2.69]],
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
                "goals": [[3.23, 2.89], [-3.87, 3.62], [1.35, -2.72], [7.10, 0.84]],
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
                "goals": [[7.42, -5.70], [-1.49, 5.21], [7.06, 4.41], [5.89, 1.03]],
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
                "init_x": -7.13,
                "init_y": -3.15,
                "init_a": 52.77,
                "velocity": 0.84,
                "goals": [[-6.95, 1.01], [6.43, -2.67], [2.97, 2.94], [5.21, -0.75]],
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

def tests_adult_10_child_90_test_case_32_walking_high(tester):
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
                "goals": [[-0.98, 0.29], [5.09, -1.41], [-1.19, -1.12], [-2.43, -3.19]],
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
                "goals": [[-0.98, 0.29], [4.87, 0.05], [-0.03, 5.21], [-7.78, 5.93]],
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
                "init_x": -4.91,
                "init_y": -1.13,
                "init_a": 121.66,
                "velocity": 0.99,
                "goals": [[5.31, -0.65], [-1.91, -1.76], [0.20, -4.62], [3.94, -3.04]],
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
                "goals": [[5.31, -0.65], [-2.42, 3.24], [3.70, -2.27], [-4.48, 3.57]],
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
                "goals": [[-1.27, 5.80], [4.24, -2.40], [3.17, -3.74], [-3.42, -3.34]],
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
                "goals": [[-5.68, -4.75], [5.28, 5.46], [0.55, -1.80], [1.96, 1.20]],
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
                "goals": [[-5.75, 0.95], [-7.15, -2.04], [2.06, 2.04], [-0.76, 4.63]],
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
                "init_x": -5.23,
                "init_y": 5.04,
                "init_a": -124.46,
                "velocity": 1.08,
                "goals": [[-0.39, 4.01], [-5.00, -4.43], [7.93, -4.06], [2.27, 5.33]],
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

def tests_adult_10_child_90_test_case_33_stopped_high(tester):
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
                "goals": [[6.00, 5.47], [0.66, 0.25], [2.45, 2.82], [0.14, 0.51]],
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
                "goals": [[5.78, 4.94], [-2.11, -0.28], [7.55, 0.84], [-3.67, 2.06]],
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
                "init_x": 5.41,
                "init_y": 4.02,
                "init_a": -171.83,
                "velocity": 0.87,
                "goals": [[5.78, 4.94], [7.69, 2.13], [4.12, -5.74], [3.23, 3.86]],
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
                "goals": [[6.19, 4.16], [7.54, 4.29], [1.78, 2.62], [2.29, 5.58]],
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
                "goals": [[5.30, -5.56], [5.51, 1.08], [3.95, -3.88], [0.97, -2.57]],
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
                "goals": [[-5.18, 5.63], [2.54, -2.73], [2.85, 5.99], [3.70, 1.12]],
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

def tests_adult_10_child_90_test_case_34_walking_high(tester):
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
                "goals": [[-5.18, -5.55], [6.04, -5.71], [-4.81, 5.36], [5.40, 1.36]],
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
                "init_x": -4.20,
                "init_y": 0.80,
                "init_a": -177.86,
                "velocity": 0.96,
                "goals": [[-5.18, -5.55], [3.39, -4.37], [-1.19, 2.08], [2.04, -0.77]],
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
                "init_x": 2.93,
                "init_y": 0.83,
                "init_a": 172.32,
                "velocity": 0.81,
                "goals": [[-4.45, 4.16], [7.02, 5.58], [-2.48, -2.09], [-3.01, -0.33]],
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
                "init_x": 3.50,
                "init_y": 1.65,
                "init_a": 172.32,
                "velocity": 0.81,
                "goals": [[-4.45, 4.16], [0.14, -2.45], [4.76, -3.13], [7.76, 0.64]],
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
                "init_x": -6.75,
                "init_y": 1.31,
                "init_a": 14.10,
                "velocity": 0.98,
                "goal_x": -7.44,
                "goal_y": 5.58,
                "n_actors": 9,
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
                "goals": [[6.34, 4.21], [-4.96, -1.11], [-4.56, -1.10], [2.58, 5.18]],
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
                "init_x": 2.99,
                "init_y": -5.51,
                "init_a": 147.11,
                "velocity": 1.08,
                "goals": [[-0.71, -1.26], [3.42, -3.99], [3.61, 0.14], [-0.67, 2.15]],
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
                "init_x": 1.41,
                "init_y": -1.12,
                "init_a": 24.71,
                "velocity": 0.98,
                "goals": [[6.40, -4.51], [5.68, -0.88], [4.04, -5.00], [-6.24, 4.23]],
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
                "init_x": -7.43,
                "init_y": 3.14,
                "init_a": -174.29,
                "velocity": 0.91,
                "goals": [[-1.91, -1.61], [-1.85, 2.49], [-4.62, 4.71], [-6.67, -2.41]],
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

def tests_adult_10_child_90_test_case_35_walking_high(tester):
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
                "goals": [[1.46, -2.66], [-4.56, -5.27], [-7.67, -4.05], [0.68, 1.71]],
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
                "init_x": -5.56,
                "init_y": 3.78,
                "init_a": 56.70,
                "velocity": 1.00,
                "goals": [[1.46, -2.66], [-0.38, 4.48], [-4.04, -3.23], [-7.42, 0.35]],
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
                "init_x": -6.95,
                "init_y": -0.73,
                "init_a": 3.93,
                "velocity": 1.01,
                "goals": [[5.96, -5.59], [1.24, -3.95], [4.54, -0.05], [-2.81, -3.63]],
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
                "goals": [[5.96, -5.59], [7.93, 3.99], [0.42, -2.94], [-5.56, 3.61]],
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
                "init_x": -3.22,
                "init_y": -4.94,
                "init_a": 53.95,
                "velocity": 1.09,
                "goals": [[-3.83, 2.14], [5.52, -3.86], [3.52, -5.12], [1.07, -0.90]],
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
                "init_x": 4.36,
                "init_y": -2.37,
                "init_a": -67.30,
                "velocity": 1.02,
                "goal_x": -1.21,
                "goal_y": 2.63,
                "n_actors": 8,
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
                "goals": [[-5.35, 1.00], [-0.55, -4.10], [7.13, 0.96], [1.77, -5.73]],
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
                "init_x": 6.46,
                "init_y": -3.23,
                "init_a": 77.23,
                "velocity": 0.86,
                "goals": [[-6.34, 2.17], [-1.82, -4.11], [-7.28, 5.55], [-5.90, -3.04]],
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

def tests_adult_10_child_90_test_case_36_walking_high(tester):
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
                "goals": [[0.54, -3.91], [4.00, -1.27], [3.19, -1.21], [7.71, 2.27]],
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
                "init_x": -0.54,
                "init_y": 2.51,
                "init_a": 110.55,
                "velocity": 0.89,
                "goals": [[0.54, -3.91], [-0.32, 4.66], [-0.26, 5.23], [2.06, 3.81]],
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
                "init_x": -0.07,
                "init_y": -1.82,
                "init_a": 36.02,
                "velocity": 1.18,
                "goals": [[-4.56, 4.52], [0.74, -2.61], [-6.72, 5.77], [-0.92, -1.76]],
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
                "init_x": -1.07,
                "init_y": -1.83,
                "init_a": 36.02,
                "velocity": 1.18,
                "goals": [[-4.56, 4.52], [7.44, 5.73], [-0.07, 0.05], [-4.06, 4.58]],
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.36,
                "init_y": 1.62,
                "init_a": -100.53,
                "velocity": 1.14,
                "goals": [[5.23, -0.57], [3.58, -5.41], [-3.41, 1.91], [0.61, -5.21]],
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
                "init_x": -7.25,
                "init_y": -5.69,
                "init_a": 29.69,
                "velocity": 0.88,
                "goals": [[6.38, 1.11], [-6.01, -1.88], [2.21, 4.73], [-1.93, -0.39]],
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

def tests_adult_10_child_90_test_case_37_stopped_high(tester):
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
                "goals": [[-5.02, 0.90], [-3.42, 1.95], [4.80, -3.44], [3.66, 2.89]],
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
                "init_x": -5.46,
                "init_y": 0.00,
                "init_a": 102.07,
                "velocity": 1.19,
                "goals": [[-5.02, 0.90], [5.31, -2.40], [7.89, -3.84], [7.03, 1.01]],
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
                "init_x": -7.28,
                "init_y": 2.94,
                "init_a": 59.29,
                "velocity": 0.97,
                "goals": [[-7.28, 2.94], [0.54, -2.34], [3.64, -3.33], [-5.33, 3.02]],
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
                "init_x": -6.35,
                "init_y": 2.58,
                "init_a": 59.29,
                "velocity": 0.97,
                "goals": [[-7.28, 2.94], [-2.73, -1.31], [-0.84, 2.04], [-1.97, 3.72]],
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
                "goals": [[-2.91, 4.58], [-0.77, -1.74], [5.89, -4.54], [-2.26, -0.61]],
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
                "goals": [[-7.76, -1.67], [-1.72, 5.76], [2.42, -3.86], [1.90, -5.28]],
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

def tests_adult_10_child_90_test_case_38_walking_high(tester):
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
                "init_x": -2.19,
                "init_y": -2.55,
                "init_a": 3.03,
                "velocity": 1.14,
                "goals": [[1.07, 1.67], [-5.88, -1.31], [-4.83, 2.59], [3.43, 2.44]],
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
                "init_x": -1.82,
                "init_y": -3.48,
                "init_a": 3.03,
                "velocity": 1.14,
                "goals": [[1.07, 1.67], [-3.33, -3.23], [-2.91, 3.10], [-3.36, 4.18]],
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
                "init_x": -2.18,
                "init_y": -4.12,
                "init_a": -120.71,
                "velocity": 0.85,
                "goals": [[6.44, -4.33], [-4.84, -4.85], [0.12, 5.25], [-3.54, 3.70]],
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
                "init_x": -1.21,
                "init_y": -4.36,
                "init_a": -120.71,
                "velocity": 0.85,
                "goals": [[6.44, -4.33], [-1.75, 1.43], [-5.87, -3.56], [6.37, 1.29]],
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
                "init_x": 2.64,
                "init_y": -0.80,
                "init_a": 171.31,
                "velocity": 0.89,
                "goals": [[7.38, 3.56], [-5.56, 0.73], [-5.60, 3.54], [-3.83, -2.35]],
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
                "init_x": -6.45,
                "init_y": 2.72,
                "init_a": 68.59,
                "velocity": 1.11,
                "goal_x": 3.60,
                "goal_y": -1.38,
                "n_actors": 7,
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
                "goals": [[2.81, -4.52], [-7.57, -0.68], [5.27, -5.90], [4.33, -0.28]],
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

def tests_adult_10_child_90_test_case_39_stopped_high(tester):
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
                "goals": [[1.38, -2.33], [-4.24, 2.25], [7.84, -1.88], [-5.09, 2.32]],
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
                "init_x": 1.15,
                "init_y": -1.36,
                "init_a": 0.15,
                "velocity": 1.07,
                "goals": [[1.38, -2.33], [5.38, 4.09], [5.99, 2.50], [3.55, 2.61]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.82,
                "init_y": -3.83,
                "init_a": -161.85,
                "velocity": 1.12,
                "goals": [[5.80, -4.06], [0.46, -2.66], [-2.50, -1.56], [-5.46, -4.00]],
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
                "init_x": -5.39,
                "init_y": -1.09,
                "init_a": 97.81,
                "velocity": 1.15,
                "goals": [[-4.19, -4.10], [5.74, -3.47], [7.56, -5.01], [-5.72, 1.80]],
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
                "init_x": 2.84,
                "init_y": 1.30,
                "init_a": 153.54,
                "velocity": 0.90,
                "goals": [[0.87, 1.56], [-6.47, -3.79], [-7.54, -0.67], [5.17, -3.19]],
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
                "init_x": 6.97,
                "init_y": 3.81,
                "init_a": 49.81,
                "velocity": 1.20,
                "goals": [[-5.73, -1.01], [-6.50, -0.42], [-1.03, 2.90], [-5.93, 4.36]],
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

def tests_adult_10_child_90_test_case_40_walking_high(tester):
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
                "goals": [[4.02, -2.08], [-3.91, 4.57], [-6.69, 4.73], [-1.69, -4.85]],
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
                "init_x": 1.48,
                "init_y": 0.33,
                "init_a": 150.93,
                "velocity": 1.08,
                "goals": [[4.02, -2.08], [1.77, 4.73], [4.26, -5.76], [3.72, 0.00]],
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
                "init_x": 0.03,
                "init_y": 5.34,
                "init_a": -120.42,
                "velocity": 1.10,
                "goals": [[1.54, -0.24], [1.41, 0.53], [-0.04, 2.92], [3.05, -4.82]],
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
                "goals": [[-3.55, -0.33], [2.55, 5.62], [4.87, -4.51], [-6.17, 5.19]],
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
                "goals": [[-0.57, -3.39], [4.62, -3.43], [5.83, 3.75], [-4.30, -0.06]],
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
                "init_x": -6.77,
                "init_y": 2.56,
                "init_a": 40.74,
                "velocity": 0.90,
                "goals": [[1.33, -2.03], [0.42, -2.38], [-2.15, -1.33], [-3.49, 1.65]],
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
                "init_x": -0.47,
                "init_y": -0.40,
                "init_a": -149.70,
                "velocity": 0.92,
                "goals": [[6.65, 4.79], [-6.75, 5.76], [2.77, -5.31], [2.91, -1.92]],
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

def tests_adult_10_child_90_test_case_41_stopped_high(tester):
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
                "goals": [[-1.93, 2.39], [-2.36, -1.49], [4.84, 0.62], [-2.38, -4.69]],
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
                "init_x": -1.72,
                "init_y": 1.41,
                "init_a": 59.77,
                "velocity": 0.91,
                "goals": [[-1.93, 2.39], [-4.48, 4.49], [-2.82, -0.33], [-3.67, -5.30]],
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
                "init_x": 4.94,
                "init_y": 5.24,
                "init_a": 70.36,
                "velocity": 0.90,
                "goals": [[4.94, 5.24], [3.15, 5.20], [-3.31, 1.02], [-1.92, -2.48]],
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
                "init_x": 5.91,
                "init_y": 5.49,
                "init_a": 70.36,
                "velocity": 0.90,
                "goals": [[4.94, 5.24], [1.25, 1.53], [-6.78, 3.96], [-0.66, -4.16]],
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.43,
                "init_y": -3.08,
                "init_a": -131.12,
                "velocity": 1.13,
                "goals": [[3.58, 2.53], [-7.73, -5.51], [3.72, -1.79], [2.05, 0.65]],
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
                "init_x": -1.41,
                "init_y": -0.24,
                "init_a": -65.68,
                "velocity": 1.18,
                "goals": [[-5.10, 3.83], [-7.44, -1.58], [-0.83, -4.45], [1.07, -3.84]],
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

def tests_adult_10_child_90_test_case_42_walking_high(tester):
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
                "goals": [[2.37, 5.22], [1.61, -5.42], [0.75, 5.20], [-0.86, 2.01]],
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
                "init_x": -3.48,
                "init_y": -2.71,
                "init_a": -52.09,
                "velocity": 0.92,
                "goals": [[2.37, 5.22], [-7.33, 0.10], [-7.32, -5.77], [-5.96, -0.12]],
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
                "init_x": -6.76,
                "init_y": 1.80,
                "init_a": -27.05,
                "velocity": 1.08,
                "goals": [[7.65, 3.40], [5.24, 3.00], [6.95, 2.42], [-6.84, -3.50]],
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
                "init_x": -6.38,
                "init_y": 2.73,
                "init_a": -27.05,
                "velocity": 1.08,
                "goals": [[7.65, 3.40], [-1.21, -0.95], [-6.59, -0.15], [4.10, 5.22]],
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
                "init_x": 5.24,
                "init_y": 1.73,
                "init_a": 137.96,
                "velocity": 0.97,
                "goals": [[-7.48, -1.91], [5.72, -0.53], [-1.96, 1.81], [6.83, -5.45]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.68,
                "init_y": -5.24,
                "init_a": 11.42,
                "velocity": 1.14,
                "goals": [[-5.03, 2.17], [-3.84, -0.25], [-1.96, -2.90], [4.12, 0.52]],
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
                "init_x": 2.76,
                "init_y": 0.58,
                "init_a": 88.37,
                "velocity": 1.09,
                "goals": [[4.50, 4.39], [2.74, 3.58], [-0.48, -4.80], [-0.30, 4.59]],
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
                "init_x": 5.57,
                "init_y": -0.62,
                "init_a": -134.04,
                "velocity": 1.05,
                "goals": [[-4.23, 5.54], [-0.85, 4.58], [-1.04, -3.20], [-2.78, 0.07]],
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

def tests_adult_10_child_90_test_case_43_walking_high(tester):
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
                "init_x": 1.71,
                "init_y": -3.94,
                "init_a": -23.77,
                "velocity": 1.05,
                "goals": [[2.63, 3.63], [7.08, -3.87], [2.93, -4.30], [-0.64, -1.40]],
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
                "init_x": 1.86,
                "init_y": -4.93,
                "init_a": -23.77,
                "velocity": 1.05,
                "goals": [[2.63, 3.63], [-0.44, 3.17], [6.89, -1.07], [7.59, 3.42]],
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
                "init_x": 7.77,
                "init_y": -5.60,
                "init_a": -159.53,
                "velocity": 0.97,
                "goals": [[-2.60, 5.89], [1.33, -0.12], [-2.30, 4.81], [1.22, 1.78]],
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
                "init_x": 6.97,
                "init_y": -6.21,
                "init_a": -159.53,
                "velocity": 0.97,
                "goals": [[-2.60, 5.89], [-1.23, 0.69], [4.02, -3.12], [3.92, -4.15]],
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
                "init_x": 3.87,
                "init_y": -0.38,
                "init_a": -55.49,
                "velocity": 1.02,
                "goals": [[3.92, -3.53], [3.50, -2.45], [1.33, 2.01], [6.67, -0.24]],
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
                "init_x": -2.24,
                "init_y": -0.94,
                "init_a": 33.02,
                "velocity": 0.82,
                "goal_x": 6.94,
                "goal_y": -0.46,
                "n_actors": 9,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.18,
                "init_y": 4.20,
                "init_a": 93.86,
                "velocity": 1.16,
                "goals": [[3.92, 2.85], [2.73, 0.68], [0.27, -1.63], [-3.22, -2.36]],
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
                "init_x": 2.02,
                "init_y": -1.93,
                "init_a": -2.01,
                "velocity": 1.18,
                "goals": [[-4.09, 4.76], [7.00, 2.10], [-6.24, -0.05], [-3.64, 4.04]],
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
                "init_x": 3.73,
                "init_y": 4.24,
                "init_a": -179.33,
                "velocity": 0.88,
                "goals": [[7.26, -1.91], [-0.60, -3.17], [-3.79, 5.68], [-6.48, 4.95]],
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

def tests_adult_10_child_90_test_case_44_walking_high(tester):
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
                "init_x": -1.97,
                "init_y": 5.85,
                "init_a": -117.48,
                "velocity": 1.11,
                "goals": [[0.38, -5.16], [-7.67, 4.44], [-3.97, -5.75], [5.23, 2.31]],
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
                "init_x": -1.20,
                "init_y": 6.48,
                "init_a": -117.48,
                "velocity": 1.11,
                "goals": [[0.38, -5.16], [0.47, 2.14], [2.41, 3.14], [6.42, -1.78]],
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
                "init_x": 2.27,
                "init_y": -1.60,
                "init_a": -86.40,
                "velocity": 0.89,
                "goal_x": 2.43,
                "goal_y": -0.14,
                "n_actors": 9,
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
                "goals": [[2.43, -0.14], [-4.75, 3.86], [-0.83, 2.45], [-2.05, -4.00]],
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
                "init_x": -4.62,
                "init_y": 4.54,
                "init_a": 159.95,
                "velocity": 0.95,
                "goals": [[6.97, -5.90], [0.95, -1.05], [-3.85, 2.12], [3.56, 0.48]],
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
                "init_x": -1.24,
                "init_y": -3.33,
                "init_a": -176.35,
                "velocity": 0.84,
                "goals": [[5.89, -2.08], [-7.02, -3.00], [-1.03, 0.68], [3.68, -3.96]],
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
                "init_x": -5.36,
                "init_y": 1.26,
                "init_a": -100.91,
                "velocity": 1.04,
                "goals": [[2.90, -3.99], [-6.83, 2.89], [7.18, 2.62], [0.96, -3.49]],
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
                "init_x": -0.38,
                "init_y": 2.96,
                "init_a": -124.13,
                "velocity": 0.82,
                "goals": [[-0.79, -2.99], [-0.76, 3.46], [3.17, -0.76], [7.21, -4.60]],
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
                "init_x": 1.23,
                "init_y": 4.31,
                "init_a": 108.14,
                "velocity": 0.96,
                "goals": [[5.88, 1.40], [-3.25, -2.53], [2.59, -4.12], [-6.75, 5.13]],
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

def tests_adult_10_child_90_test_case_45_walking_high(tester):
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
                "init_x": -3.31,
                "init_y": 1.60,
                "init_a": -133.51,
                "velocity": 1.17,
                "goal_x": 6.75,
                "goal_y": 0.40,
                "n_actors": 8,
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
                "goals": [[6.75, 0.40], [2.43, -0.90], [-6.78, -2.92], [-1.75, 4.91]],
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
                "goals": [[1.43, 1.81], [5.00, -0.82], [3.49, -5.67], [6.33, 1.70]],
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
                "goals": [[1.43, 1.81], [-0.55, -4.96], [2.43, 2.65], [-4.73, 2.10]],
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
                "init_x": 3.93,
                "init_y": 0.15,
                "init_a": 83.14,
                "velocity": 1.01,
                "goals": [[-1.29, 2.63], [-7.07, -0.93], [-7.20, 5.82], [-5.55, 1.75]],
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
                "init_x": 4.92,
                "init_y": -2.49,
                "init_a": -178.31,
                "velocity": 1.08,
                "goals": [[6.35, -5.18], [4.18, 1.30], [-6.37, -4.12], [-5.07, 1.10]],
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
                "init_x": 6.83,
                "init_y": -1.06,
                "init_a": -59.37,
                "velocity": 1.18,
                "goals": [[7.97, 2.06], [-3.22, 1.66], [0.28, 4.14], [2.12, -4.96]],
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
                "init_x": 6.96,
                "init_y": -0.63,
                "init_a": 13.29,
                "velocity": 1.00,
                "goals": [[-2.68, 5.16], [-5.64, 4.37], [-3.02, 1.91], [4.63, -4.81]],
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

def tests_adult_10_child_90_test_case_46_walking_high(tester):
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
                "goals": [[-5.50, 3.69], [-6.43, 3.90], [6.37, -0.89], [4.22, 5.43]],
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
                "init_x": 6.33,
                "init_y": 0.08,
                "init_a": 19.67,
                "velocity": 1.01,
                "goals": [[-5.50, 3.69], [7.38, -2.23], [-2.96, -1.30], [4.87, 1.84]],
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
                "goals": [[-3.89, -3.62], [1.84, -2.68], [1.20, 5.93], [-6.01, -1.86]],
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
                "goals": [[7.28, -3.23], [5.89, -1.26], [7.53, 5.50], [3.66, -2.05]],
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
                "goals": [[7.74, 2.67], [-5.07, 4.17], [7.69, 1.48], [-0.14, 0.56]],
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
                "goals": [[-1.70, -4.43], [4.91, -0.75], [-1.42, -2.43], [5.62, -5.90]],
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
                "goals": [[4.64, -1.43], [5.52, 0.30], [4.31, -0.94], [2.92, 0.23]],
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

def tests_adult_10_child_90_test_case_47_stopped_high(tester):
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
                "goals": [[-0.67, 0.17], [2.23, 0.15], [-1.04, -3.43], [-6.79, -4.98]],
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
                "init_x": -1.62,
                "init_y": 0.48,
                "init_a": 144.19,
                "velocity": 0.96,
                "goals": [[-0.67, 0.17], [6.06, 3.67], [-5.60, 2.62], [-4.81, -2.06]],
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
                "init_x": -7.45,
                "init_y": 5.92,
                "init_a": -61.07,
                "velocity": 1.16,
                "goals": [[-7.45, 5.92], [2.75, 5.62], [7.09, 2.75], [0.03, -1.76]],
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
                "init_x": -6.77,
                "init_y": 5.18,
                "init_a": -61.07,
                "velocity": 1.16,
                "goals": [[-7.45, 5.92], [0.48, 1.69], [2.09, 5.14], [-3.42, -1.29]],
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
                "init_x": 2.42,
                "init_y": 3.37,
                "init_a": 38.20,
                "velocity": 1.03,
                "goals": [[7.93, 5.30], [-5.63, 1.60], [-0.17, -3.09], [7.82, 4.39]],
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
                "init_x": 0.78,
                "init_y": 4.46,
                "init_a": -88.04,
                "velocity": 1.05,
                "goal_x": -4.74,
                "goal_y": 0.90,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.87,
                "init_y": 2.12,
                "init_a": 61.55,
                "velocity": 1.10,
                "goals": [[-0.35, -0.15], [-1.82, 4.89], [0.85, 4.08], [-4.69, -5.47]],
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
                "init_x": 7.43,
                "init_y": 0.06,
                "init_a": 77.21,
                "velocity": 1.20,
                "goals": [[-0.80, -5.75], [-3.19, -2.41], [1.23, 3.32], [-1.26, -2.09]],
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

def tests_adult_10_child_90_test_case_48_stopped_high(tester):
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
                "goals": [[7.78, 4.30], [-6.68, -0.96], [4.43, 3.34], [-4.44, 5.01]],
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
                "init_x": 6.88,
                "init_y": 3.87,
                "init_a": -171.53,
                "velocity": 1.10,
                "goals": [[7.78, 4.30], [3.99, 3.59], [1.95, -5.15], [7.61, 2.91]],
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
                "init_x": 3.34,
                "init_y": 4.74,
                "init_a": 171.74,
                "velocity": 0.89,
                "goal_x": 3.34,
                "goal_y": 4.74,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.87,
                "init_y": 3.89,
                "init_a": 171.74,
                "velocity": 0.89,
                "goals": [[3.34, 4.74], [-7.52, -0.83], [2.21, 3.40], [1.58, -5.67]],
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
                "init_x": 1.70,
                "init_y": 5.04,
                "init_a": 93.06,
                "velocity": 0.87,
                "goals": [[-7.19, -5.23], [6.84, 4.62], [0.99, 5.80], [-6.58, -4.39]],
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
                "init_x": -4.47,
                "init_y": 0.04,
                "init_a": -59.45,
                "velocity": 1.09,
                "goals": [[-3.99, -5.37], [-2.96, -1.23], [1.33, 2.82], [6.96, -5.15]],
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
                "goals": [[7.42, -2.57], [1.31, -0.85], [-6.02, 5.95], [-1.83, -5.71]],
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
                "goals": [[-3.87, -5.18], [-2.30, 3.40], [4.20, -1.86], [1.49, 2.70]],
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

def tests_adult_10_child_90_test_case_49_walking_high(tester):
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
                "init_x": 6.07,
                "init_y": -1.15,
                "init_a": -56.41,
                "velocity": 0.89,
                "goal_x": 4.09,
                "goal_y": 3.57,
                "n_actors": 8,
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
                "goals": [[4.09, 3.57], [-1.63, -1.84], [6.73, 1.32], [1.16, 5.72]],
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
                "init_x": 3.19,
                "init_y": 4.24,
                "init_a": 34.19,
                "velocity": 1.07,
                "goals": [[4.49, 1.10], [0.01, -0.36], [7.75, 2.50], [-3.54, 1.46]],
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
                "init_x": 2.30,
                "init_y": 3.79,
                "init_a": 34.19,
                "velocity": 1.07,
                "goals": [[4.49, 1.10], [2.14, -4.60], [1.00, 5.12], [-0.91, -2.89]],
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
                "init_x": 0.47,
                "init_y": 5.92,
                "init_a": -29.06,
                "velocity": 1.05,
                "goals": [[-5.51, 4.39], [-7.33, -5.04], [1.84, 5.88], [-4.73, 5.79]],
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
                "init_x": -7.71,
                "init_y": -3.31,
                "init_a": -104.12,
                "velocity": 0.80,
                "goals": [[-0.45, 3.79], [-1.42, -4.65], [2.22, -1.78], [-6.50, 5.90]],
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
                "init_x": 1.94,
                "init_y": -1.27,
                "init_a": 27.10,
                "velocity": 1.09,
                "goals": [[-7.70, 3.21], [-7.73, 1.92], [3.18, 0.30], [-4.97, 2.53]],
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
                "init_x": 5.13,
                "init_y": -0.55,
                "init_a": -22.34,
                "velocity": 0.88,
                "goals": [[-4.68, -4.74], [-4.93, 2.72], [-2.56, 3.51], [1.66, -2.33]],
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

def tests_adult_10_child_90_test_case_50_stopped_high(tester):
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
                "init_x": 5.01,
                "init_y": -5.23,
                "init_a": -86.53,
                "velocity": 0.93,
                "goals": [[5.01, -5.23], [5.94, 5.24], [-2.32, -5.06], [-0.43, 0.48]],
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
                "init_x": 5.60,
                "init_y": -4.42,
                "init_a": -86.53,
                "velocity": 0.93,
                "goals": [[5.01, -5.23], [-2.25, 5.92], [-6.72, 5.45], [-5.15, -0.72]],
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
                "init_x": 1.32,
                "init_y": 1.84,
                "init_a": 137.64,
                "velocity": 1.01,
                "goal_x": 1.32,
                "goal_y": 1.84,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.39,
                "init_y": 2.22,
                "init_a": 137.64,
                "velocity": 1.01,
                "goals": [[1.32, 1.84], [8.00, 3.15], [-4.43, 4.70], [-5.12, 4.98]],
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
                "init_x": 1.83,
                "init_y": -4.83,
                "init_a": -0.31,
                "velocity": 1.05,
                "goals": [[7.51, 1.54], [5.28, -1.12], [1.85, 3.24], [7.50, -3.09]],
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
                "init_x": 6.20,
                "init_y": -5.91,
                "init_a": 50.73,
                "velocity": 1.00,
                "goals": [[-6.69, 1.33], [-3.41, -5.69], [-5.26, -1.39], [4.71, 3.06]],
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
                "init_x": -7.43,
                "init_y": 5.77,
                "init_a": 165.08,
                "velocity": 1.14,
                "goals": [[-5.72, 3.98], [1.60, -2.24], [-1.63, 0.38], [4.03, 5.23]],
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
                "init_x": 5.20,
                "init_y": 0.75,
                "init_a": 128.89,
                "velocity": 1.06,
                "goals": [[-4.81, 1.98], [5.93, -5.03], [2.05, 0.33], [-6.70, -0.72]],
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

def tests_adult_10_child_90_test_case_51_walking_high(tester):
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
                "goals": [[-3.53, 3.45], [-7.39, -2.90], [-3.80, -2.14], [-1.43, 2.17]],
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
                "init_x": -5.30,
                "init_y": -2.34,
                "init_a": -32.01,
                "velocity": 0.85,
                "goals": [[-3.53, 3.45], [2.62, -2.29], [-2.11, 1.11], [-1.46, 5.75]],
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
                "init_x": 3.61,
                "init_y": 0.06,
                "init_a": 61.58,
                "velocity": 1.12,
                "goals": [[6.17, 2.16], [5.54, -4.59], [4.49, 4.49], [7.17, -1.70]],
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
                "init_x": 3.67,
                "init_y": 1.06,
                "init_a": 61.58,
                "velocity": 1.12,
                "goals": [[6.17, 2.16], [5.51, 0.85], [-7.59, 2.20], [-0.48, 2.93]],
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
                "init_x": 7.83,
                "init_y": -5.77,
                "init_a": -84.96,
                "velocity": 1.12,
                "goal_x": -4.41,
                "goal_y": -4.56,
                "n_actors": 8,
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
                "goals": [[-1.57, -3.14], [4.48, 4.56], [1.49, -0.51], [-1.07, 0.11]],
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
                "init_x": -2.33,
                "init_y": 4.33,
                "init_a": -174.49,
                "velocity": 1.19,
                "goals": [[-0.97, -2.10], [-6.87, 0.20], [1.03, 0.75], [-2.17, -3.11]],
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
                "init_x": 2.12,
                "init_y": -0.91,
                "init_a": -39.51,
                "velocity": 0.88,
                "goals": [[-3.83, -3.31], [4.27, -0.48], [-4.62, -5.38], [6.02, -2.59]],
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

def tests_adult_10_child_90_test_case_52_stopped_high(tester):
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
                "goals": [[7.30, -0.08], [6.59, 0.66], [6.54, 5.93], [-1.72, 3.11]],
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
                "goals": [[-4.56, -4.03], [-1.54, -4.97], [-2.81, 0.20], [7.73, -2.41]],
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
                "init_x": -3.87,
                "init_y": -3.30,
                "init_a": -132.32,
                "velocity": 0.97,
                "goals": [[-4.56, -4.03], [7.99, 3.84], [-7.27, -1.49], [-2.46, 3.72]],
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
                "init_x": -2.72,
                "init_y": -1.52,
                "init_a": 85.48,
                "velocity": 0.86,
                "goals": [[-2.39, 0.71], [7.36, 4.80], [5.53, 4.80], [2.11, -2.38]],
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
                "goals": [[1.08, 5.17], [0.68, -1.49], [1.61, -4.59], [7.99, 3.63]],
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
                "init_x": 7.87,
                "init_y": 2.75,
                "init_a": -89.74,
                "velocity": 1.11,
                "goals": [[-2.40, -5.47], [2.49, -1.81], [4.19, -1.69], [3.49, 2.16]],
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
                "init_x": -3.13,
                "init_y": -3.98,
                "init_a": 10.11,
                "velocity": 0.99,
                "goals": [[5.88, -4.46], [-5.68, -3.66], [-5.19, 3.35], [-1.49, 2.41]],
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

def tests_adult_10_child_90_test_case_53_stopped_high(tester):
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
                "goals": [[-1.70, 4.83], [-4.18, -0.05], [-2.81, 3.94], [-5.27, 5.08]],
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
                "goals": [[-1.70, 4.83], [-7.56, 5.68], [4.66, 0.56], [5.21, -2.85]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.62,
                "init_y": 1.47,
                "init_a": 22.73,
                "velocity": 0.92,
                "goals": [[0.75, 0.98], [-3.71, -2.00], [-4.50, 5.13], [-5.18, 1.70]],
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
                "init_x": 3.37,
                "init_y": 4.04,
                "init_a": -131.76,
                "velocity": 1.05,
                "goals": [[-1.15, -5.12], [5.02, 0.03], [2.60, -3.15], [6.97, -1.14]],
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
                "init_x": 0.62,
                "init_y": -0.05,
                "init_a": 16.07,
                "velocity": 1.13,
                "goals": [[-4.48, 4.09], [-4.31, -4.71], [-1.03, -2.04], [-4.11, -5.47]],
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
                "init_x": -5.81,
                "init_y": -2.64,
                "init_a": 94.23,
                "velocity": 1.02,
                "goals": [[4.14, -2.26], [-6.28, 1.55], [7.41, 0.53], [5.08, -4.88]],
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

def tests_adult_10_child_90_test_case_54_walking_high(tester):
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
                "goals": [[3.98, 2.75], [0.95, 3.77], [-3.73, -0.17], [-4.15, 1.76]],
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
                "init_x": -6.64,
                "init_y": 0.94,
                "init_a": 101.38,
                "velocity": 0.98,
                "goals": [[3.98, 2.75], [6.32, -5.91], [-0.62, -3.12], [2.43, -4.23]],
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
                "goals": [[7.44, -4.34], [6.21, 5.99], [-7.28, -5.95], [7.65, -0.14]],
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
                "init_x": -0.07,
                "init_y": 3.46,
                "init_a": -56.42,
                "velocity": 0.95,
                "goals": [[-2.62, 4.71], [-1.78, -3.06], [5.92, -1.80], [1.47, 1.76]],
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
                "goals": [[-3.58, 3.91], [-5.60, -5.48], [-6.55, 1.11], [-7.38, 4.93]],
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
                "goals": [[1.21, -0.99], [-3.59, -1.04], [-7.36, 2.23], [-5.41, 0.75]],
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
                "goals": [[-2.75, -1.68], [-1.93, -4.54], [0.28, -4.09], [2.08, -1.73]],
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
                "init_x": -3.42,
                "init_y": 5.56,
                "init_a": -132.23,
                "velocity": 1.14,
                "goals": [[4.25, -1.15], [-3.31, 5.91], [-7.42, -3.65], [-2.98, -0.87]],
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

def tests_adult_10_child_90_test_case_55_walking_high(tester):
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
                "goals": [[-1.34, -4.55], [-7.31, 4.29], [-6.41, 1.18], [-3.68, 1.59]],
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
                "goals": [[-1.34, -4.55], [-6.07, 1.14], [3.12, -5.15], [5.13, -1.86]],
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
                "init_x": 0.59,
                "init_y": -4.57,
                "init_a": 45.41,
                "velocity": 1.09,
                "goals": [[1.47, -0.13], [-7.22, -3.39], [-0.49, 0.64], [-1.52, 5.66]],
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
                "init_x": 0.04,
                "init_y": -3.74,
                "init_a": 45.41,
                "velocity": 1.09,
                "goals": [[1.47, -0.13], [-3.94, 4.26], [4.58, -5.70], [-2.76, -4.29]],
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
                "init_x": -5.18,
                "init_y": 4.26,
                "init_a": 28.39,
                "velocity": 1.02,
                "goals": [[-6.21, 0.04], [-6.45, 0.91], [5.33, -0.94], [-3.47, -0.95]],
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
                "init_x": 2.33,
                "init_y": -2.57,
                "init_a": -113.53,
                "velocity": 1.09,
                "goal_x": 7.47,
                "goal_y": -0.66,
                "n_actors": 7,
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
                "goals": [[-5.99, -4.82], [7.57, 5.51], [-7.98, -4.59], [-1.33, -2.78]],
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

def tests_adult_10_child_90_test_case_56_stopped_high(tester):
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
                "goals": [[-0.82, -4.08], [-0.67, -5.54], [5.81, 3.01], [-4.12, 3.16]],
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
                "goals": [[6.34, -1.27], [1.46, -2.44], [7.93, 2.60], [-4.50, -0.64]],
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
                "goals": [[6.34, -1.27], [-2.04, 3.19], [-7.77, 3.51], [-6.43, -1.83]],
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
                "goals": [[-4.30, -3.27], [6.40, 2.36], [-7.63, -0.48], [0.87, 4.21]],
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
                "init_x": 1.96,
                "init_y": 1.96,
                "init_a": -15.06,
                "velocity": 1.13,
                "goals": [[1.87, 0.87], [7.63, 4.36], [-6.73, 2.74], [-6.28, 2.48]],
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
                "init_x": 4.27,
                "init_y": -2.52,
                "init_a": 34.89,
                "velocity": 1.00,
                "goals": [[-0.33, 0.68], [-5.07, 3.88], [6.85, 0.50], [1.56, -1.39]],
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

def tests_adult_10_child_90_test_case_57_walking_high(tester):
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
                "goals": [[-3.00, 1.72], [7.26, 0.09], [-3.79, 4.40], [-7.39, -2.44]],
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
                "goals": [[-3.00, 1.72], [-3.82, 4.22], [-0.98, 1.06], [3.53, -3.82]],
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
                "init_x": 4.14,
                "init_y": -2.97,
                "init_a": -81.96,
                "velocity": 1.11,
                "goal_x": 2.34,
                "goal_y": 2.19,
                "n_actors": 8,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.95,
                "init_y": -2.38,
                "init_a": -81.96,
                "velocity": 1.11,
                "goals": [[2.34, 2.19], [0.99, 2.10], [-7.19, -4.86], [1.72, 1.22]],
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
                "init_x": 0.91,
                "init_y": 1.21,
                "init_a": 92.39,
                "velocity": 1.11,
                "goals": [[-7.01, -1.92], [1.60, -2.46], [5.45, -3.65], [-4.25, 4.88]],
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
                "init_x": -5.93,
                "init_y": 4.20,
                "init_a": 144.29,
                "velocity": 1.07,
                "goals": [[-6.05, -3.53], [-1.59, -2.27], [1.96, -0.73], [-1.03, 0.79]],
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
                "init_x": 2.33,
                "init_y": -2.44,
                "init_a": 64.95,
                "velocity": 1.04,
                "goals": [[3.76, 4.39], [-0.30, -5.42], [-1.61, -0.32], [-4.23, 0.32]],
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
                "init_x": -0.67,
                "init_y": 2.07,
                "init_a": 167.06,
                "velocity": 1.19,
                "goals": [[1.82, 2.68], [5.12, 5.33], [6.80, 4.67], [3.23, -5.63]],
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

def tests_adult_10_child_90_test_case_58_walking_high(tester):
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
                "goals": [[7.59, -4.42], [4.32, -5.25], [-1.51, 4.08], [-0.81, 2.01]],
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
                "goals": [[7.59, -4.42], [3.88, 4.87], [-7.51, 3.66], [2.67, -3.60]],
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
                "goals": [[-3.20, -2.39], [2.99, 4.53], [3.30, 0.56], [-7.60, -4.34]],
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
                "goals": [[-3.20, -2.39], [2.50, -5.57], [-7.11, 4.66], [-3.24, 0.93]],
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
                "init_x": -3.81,
                "init_y": -1.27,
                "init_a": 168.39,
                "velocity": 0.91,
                "goals": [[4.59, 5.70], [0.97, -5.74], [-1.73, -3.73], [-7.45, 0.67]],
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
                "init_x": 0.76,
                "init_y": 3.08,
                "init_a": 0.03,
                "velocity": 0.82,
                "goals": [[2.48, -3.77], [3.65, -5.87], [4.56, -4.50], [4.24, 3.64]],
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
                "init_x": -7.48,
                "init_y": -4.83,
                "init_a": -0.69,
                "velocity": 1.17,
                "goals": [[7.10, 3.83], [6.55, -5.28], [-3.71, -1.73], [1.84, 5.26]],
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

def tests_adult_10_child_90_test_case_59_stopped_high(tester):
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
                "goals": [[-2.03, 2.97], [0.56, -1.04], [3.85, -2.85], [-2.95, 1.96]],
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
                "goals": [[-7.88, 2.24], [0.89, -1.47], [-4.58, 4.63], [4.14, -5.34]],
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
                "goals": [[-7.88, 2.24], [0.38, -1.51], [-7.74, 4.05], [0.82, -3.08]],
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
                "init_x": 7.93,
                "init_y": -1.30,
                "init_a": -85.66,
                "velocity": 0.84,
                "goals": [[2.91, 3.14], [-2.01, 1.09], [1.05, -1.29], [-0.36, -4.88]],
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
                "goals": [[-1.26, 5.49], [-3.87, -1.60], [-0.34, -5.46], [0.65, 1.16]],
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
                "init_x": 5.43,
                "init_y": 3.73,
                "init_a": 2.90,
                "velocity": 1.12,
                "goals": [[-0.54, -0.27], [-7.30, 4.74], [-6.36, 5.99], [-3.55, -5.83]],
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
                "init_x": -5.56,
                "init_y": -3.81,
                "init_a": 148.06,
                "velocity": 0.86,
                "goals": [[-1.79, -3.44], [7.86, 5.27], [-0.84, -4.71], [-7.08, -3.00]],
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
                "init_x": -7.29,
                "init_y": 5.16,
                "init_a": 104.51,
                "velocity": 1.16,
                "goals": [[2.05, -3.99], [-4.46, 0.80], [-4.18, 3.29], [-5.67, -0.96]],
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

def tests_adult_10_child_90_test_case_60_stopped_high(tester):
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
                "goals": [[-3.35, 3.57], [0.33, 0.57], [-3.06, -0.65], [7.35, 0.96]],
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
                "init_x": -2.87,
                "init_y": 4.45,
                "init_a": -20.12,
                "velocity": 1.11,
                "goals": [[-3.35, 3.57], [6.00, -4.49], [-7.26, -3.33], [5.62, 5.97]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.16,
                "init_y": -0.35,
                "init_a": 37.99,
                "velocity": 0.81,
                "goals": [[0.28, -1.34], [-3.13, 1.18], [-6.41, -0.24], [-2.81, -1.28]],
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
                "init_x": -7.98,
                "init_y": -5.17,
                "init_a": 34.84,
                "velocity": 1.18,
                "goals": [[7.51, 3.84], [-5.20, -1.43], [5.11, -4.44], [2.04, 0.92]],
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
                "init_x": 0.02,
                "init_y": 2.79,
                "init_a": 140.78,
                "velocity": 0.84,
                "goals": [[-0.27, 5.15], [-1.90, -4.99], [2.10, 3.14], [-1.68, -2.67]],
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
                "init_x": 0.91,
                "init_y": -2.06,
                "init_a": 52.26,
                "velocity": 1.19,
                "goals": [[7.85, -1.87], [7.51, 4.52], [-1.89, -0.66], [-5.51, 2.81]],
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
                "init_x": -4.36,
                "init_y": -2.75,
                "init_a": -143.58,
                "velocity": 1.18,
                "goals": [[0.64, -5.88], [5.77, -5.51], [1.64, -4.49], [3.08, -4.75]],
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
                "init_x": 5.37,
                "init_y": 5.20,
                "init_a": -29.40,
                "velocity": 1.19,
                "goals": [[-7.03, -4.57], [-3.12, 1.96], [0.76, -5.19], [4.65, 4.93]],
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

def tests_adult_10_child_90_test_case_61_stopped_high(tester):
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
                "goals": [[0.99, 2.09], [-6.79, -3.57], [-5.44, -0.16], [-7.47, -0.38]],
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
                "goals": [[0.99, 2.09], [2.92, -2.38], [-6.32, -4.29], [5.63, -5.28]],
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
                "init_x": 1.07,
                "init_y": -3.92,
                "init_a": -110.00,
                "velocity": 0.90,
                "goals": [[1.07, -3.92], [-0.24, -0.22], [6.72, -0.57], [7.51, 4.23]],
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
                "init_x": 1.90,
                "init_y": -4.48,
                "init_a": -110.00,
                "velocity": 0.90,
                "goals": [[1.07, -3.92], [-7.17, -2.90], [6.20, -1.47], [3.84, 4.23]],
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
                "goals": [[5.56, 3.38], [5.37, -3.67], [2.88, -4.92], [1.00, 2.23]],
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
                "goals": [[4.71, -2.50], [-0.37, 5.47], [0.79, -3.69], [-3.73, -3.14]],
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

def tests_adult_10_child_90_test_case_62_walking_high(tester):
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
                "init_x": -4.56,
                "init_y": -4.27,
                "init_a": 112.02,
                "velocity": 0.86,
                "goals": [[-7.59, 2.13], [-0.00, 5.39], [-4.28, 5.88], [-2.39, -1.44]],
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
                "init_x": -4.52,
                "init_y": -3.27,
                "init_a": 112.02,
                "velocity": 0.86,
                "goals": [[-7.59, 2.13], [0.08, 0.80], [5.64, -0.75], [1.66, -2.00]],
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
                "init_x": -3.93,
                "init_y": 5.96,
                "init_a": -119.34,
                "velocity": 0.83,
                "goals": [[2.85, -3.95], [2.96, 3.85], [-3.34, 0.31], [-2.06, -3.27]],
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
                "init_x": -3.35,
                "init_y": 6.77,
                "init_a": -119.34,
                "velocity": 0.83,
                "goals": [[2.85, -3.95], [-2.89, -1.77], [2.46, 0.72], [-3.40, -1.69]],
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
                "init_x": -3.44,
                "init_y": -1.00,
                "init_a": 6.99,
                "velocity": 1.03,
                "goals": [[6.05, -3.90], [-7.14, 5.65], [6.63, -0.84], [-5.00, 1.25]],
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
                "goals": [[-2.34, -3.16], [-6.35, 5.10], [-6.83, -4.37], [-5.56, 2.01]],
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
                "init_x": -3.63,
                "init_y": 3.32,
                "init_a": -145.86,
                "velocity": 1.14,
                "goals": [[7.91, -1.49], [3.24, -2.00], [4.89, -1.18], [-6.05, -2.05]],
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
                "init_x": 6.36,
                "init_y": 0.84,
                "init_a": 131.06,
                "velocity": 1.08,
                "goals": [[5.61, 4.37], [7.78, -2.31], [-5.13, 4.32], [-5.29, 3.78]],
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

def tests_adult_10_child_90_test_case_63_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.25,
                "init_y": 2.82,
                "init_a": -148.32,
                "velocity": 1.08,
                "goals": [[5.86, -3.97], [-0.78, -1.24], [6.64, 2.59], [-7.37, 2.43]],
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
                "init_x": -5.97,
                "init_y": -5.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goals": [[2.18, -1.61], [-6.28, 4.87], [-7.43, -4.97], [6.51, -0.67]],
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
                "init_x": -6.03,
                "init_y": -6.65,
                "init_a": 159.23,
                "velocity": 0.81,
                "goals": [[2.18, -1.61], [3.57, -1.08], [0.29, 4.84], [-7.46, 1.60]],
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
                "init_x": 2.53,
                "init_y": 3.99,
                "init_a": -28.58,
                "velocity": 1.19,
                "goals": [[4.54, 4.23], [6.73, -5.03], [-6.02, 5.24], [7.67, 4.49]],
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
                "init_x": -4.39,
                "init_y": 2.71,
                "init_a": -149.94,
                "velocity": 1.03,
                "goals": [[5.92, -2.47], [-2.12, 0.37], [-1.79, -2.39], [5.19, 3.08]],
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
                "goals": [[7.50, 1.97], [1.16, -1.97], [-6.61, 0.95], [-0.69, -0.62]],
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

def tests_adult_10_child_90_test_case_64_walking_high(tester):
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
                "goals": [[6.25, -5.10], [-0.93, -3.32], [0.95, 2.97], [-3.54, 1.40]],
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
                "init_x": -0.15,
                "init_y": 4.66,
                "init_a": 100.23,
                "velocity": 0.82,
                "goals": [[6.25, -5.10], [3.21, -1.70], [-3.68, -1.50], [-7.61, 5.39]],
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
                "init_x": -0.02,
                "init_y": -4.35,
                "init_a": -116.00,
                "velocity": 0.93,
                "goals": [[-6.82, -1.31], [-7.24, -4.23], [-5.58, 0.49], [-7.08, -5.14]],
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
                "goals": [[-6.82, -1.31], [-7.96, 2.76], [-0.55, -2.01], [-1.35, 5.43]],
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
                "init_x": 3.48,
                "init_y": -3.61,
                "init_a": -147.54,
                "velocity": 1.16,
                "goals": [[3.11, 5.94], [-3.97, -2.85], [1.13, 5.54], [-3.27, -2.10]],
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
                "goals": [[1.58, -0.89], [7.04, 1.84], [-4.96, -1.35], [-2.45, -1.92]],
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
                "init_x": 4.24,
                "init_y": -4.07,
                "init_a": 27.79,
                "velocity": 0.92,
                "goals": [[5.09, -3.04], [4.78, 5.15], [-2.30, -2.68], [7.54, 0.14]],
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

def tests_adult_10_child_90_test_case_65_walking_high(tester):
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
                "goals": [[6.29, 2.71], [-5.75, 5.00], [-7.35, -3.24], [-2.74, 3.35]],
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
                "init_x": 5.60,
                "init_y": 5.03,
                "init_a": -173.85,
                "velocity": 1.17,
                "goals": [[6.29, 2.71], [-4.08, -4.87], [1.34, -4.50], [-6.37, 5.87]],
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
                "init_x": 2.28,
                "init_y": 2.28,
                "init_a": -134.52,
                "velocity": 1.01,
                "goals": [[-6.16, -4.07], [6.32, 2.10], [3.24, 2.37], [-1.31, 3.99]],
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
                "init_x": 1.47,
                "init_y": 2.87,
                "init_a": -134.52,
                "velocity": 1.01,
                "goals": [[-6.16, -4.07], [7.48, -3.24], [2.66, 4.72], [-7.37, -5.08]],
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
                "init_x": 1.69,
                "init_y": 0.32,
                "init_a": -124.69,
                "velocity": 0.95,
                "goals": [[-1.30, -1.65], [4.97, 5.36], [-6.22, 3.70], [7.54, -2.66]],
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
                "goals": [[-4.13, 2.82], [-3.82, 1.69], [-5.42, -3.28], [-2.69, 0.01]],
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
                "init_x": 0.64,
                "init_y": -4.65,
                "init_a": 134.13,
                "velocity": 1.19,
                "goals": [[-7.18, -0.40], [5.64, -2.68], [5.95, 5.42], [3.79, -1.10]],
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

def tests_adult_10_child_90_test_case_66_walking_high(tester):
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
                "goals": [[6.32, 2.26], [2.32, -0.30], [-3.13, 5.38], [6.07, 0.18]],
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
                "init_x": 2.21,
                "init_y": -5.72,
                "init_a": -57.89,
                "velocity": 0.85,
                "goals": [[6.32, 2.26], [-4.48, 5.94], [-3.55, 0.31], [1.32, -0.13]],
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
                "init_x": -7.72,
                "init_y": -5.03,
                "init_a": -61.58,
                "velocity": 0.95,
                "goals": [[-3.30, -0.14], [0.54, 0.42], [6.99, -5.86], [-4.00, 5.22]],
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
                "init_x": -6.97,
                "init_y": -5.70,
                "init_a": -61.58,
                "velocity": 0.95,
                "goals": [[-3.30, -0.14], [3.11, 3.55], [-4.34, -4.01], [6.53, 5.56]],
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
                "init_x": 1.51,
                "init_y": 3.82,
                "init_a": -71.78,
                "velocity": 0.84,
                "goals": [[5.15, 3.97], [5.62, 0.31], [3.10, 3.03], [7.18, -1.71]],
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
                "init_x": -4.72,
                "init_y": -1.73,
                "init_a": -102.43,
                "velocity": 1.06,
                "goals": [[-5.82, -2.46], [0.34, -4.62], [5.31, 4.39], [4.72, -3.80]],
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
                "init_x": 1.03,
                "init_y": 4.61,
                "init_a": 77.00,
                "velocity": 1.00,
                "goals": [[-6.64, -3.90], [-4.62, 0.79], [2.68, -0.51], [5.47, 0.08]],
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

def tests_adult_10_child_90_test_case_67_stopped_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.40,
                "init_y": -4.60,
                "init_a": 151.44,
                "velocity": 1.15,
                "goals": [[3.58, -5.58], [-0.69, -4.38], [3.87, 5.50], [6.71, 2.06]],
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
                "init_x": 6.06,
                "init_y": -4.08,
                "init_a": -98.20,
                "velocity": 1.20,
                "goals": [[6.06, -4.08], [3.89, 2.18], [-3.47, -4.46], [6.78, 5.32]],
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
                "init_x": 6.67,
                "init_y": -4.87,
                "init_a": -98.20,
                "velocity": 1.20,
                "goals": [[6.06, -4.08], [3.11, 0.73], [-3.19, -4.96], [0.82, -2.55]],
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
                "init_x": -2.14,
                "init_y": -0.24,
                "init_a": 16.58,
                "velocity": 1.18,
                "goals": [[-3.04, -2.77], [1.77, -2.22], [3.23, 1.10], [-0.34, -1.59]],
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
                "goals": [[-3.49, 5.11], [-2.13, -0.56], [-1.55, 4.83], [-3.69, 3.81]],
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
                "goals": [[-4.85, 2.66], [2.09, -0.70], [-2.51, 0.20], [4.19, 5.63]],
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

def tests_adult_10_child_90_test_case_68_walking_high(tester):
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
                "goals": [[-2.19, -1.06], [7.73, -0.74], [-0.06, 5.59], [4.99, -1.46]],
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
                "init_x": 0.33,
                "init_y": 3.60,
                "init_a": 22.54,
                "velocity": 0.99,
                "goals": [[-2.19, -1.06], [-5.87, -1.23], [1.88, 3.91], [3.76, 0.35]],
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
                "goals": [[6.64, -5.39], [-1.30, -1.12], [-1.81, -0.72], [4.31, 4.22]],
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
                "init_x": -5.49,
                "init_y": -3.25,
                "init_a": 132.75,
                "velocity": 1.04,
                "goals": [[6.64, -5.39], [0.62, -2.85], [-7.83, -1.75], [7.03, 1.24]],
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
                "init_x": 4.64,
                "init_y": 0.83,
                "init_a": 101.87,
                "velocity": 0.85,
                "goals": [[-6.89, -2.20], [6.15, 0.64], [-6.97, 2.13], [-3.13, -4.62]],
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
                "init_x": 4.34,
                "init_y": -4.02,
                "init_a": 52.76,
                "velocity": 1.18,
                "goals": [[-5.82, 2.39], [4.23, -3.31], [7.52, -3.80], [3.93, -5.67]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.15,
                "init_y": -4.80,
                "init_a": 177.34,
                "velocity": 0.96,
                "goals": [[5.18, -0.20], [-0.79, 0.48], [-5.29, -1.61], [-4.56, -0.07]],
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
                "init_x": -3.40,
                "init_y": 0.32,
                "init_a": 128.88,
                "velocity": 0.83,
                "goals": [[-0.02, 0.98], [4.70, 5.54], [3.43, -0.71], [3.59, 1.06]],
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

def tests_adult_10_child_90_test_case_69_walking_high(tester):
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
                "goals": [[-6.91, 1.91], [0.62, 2.98], [5.88, 5.42], [4.39, 1.29]],
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
                "init_x": 5.55,
                "init_y": 1.38,
                "init_a": 56.35,
                "velocity": 0.89,
                "goals": [[-6.91, 1.91], [4.94, 4.00], [4.75, 3.56], [-2.24, -1.20]],
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
                "init_x": 7.66,
                "init_y": 2.94,
                "init_a": 1.31,
                "velocity": 1.16,
                "goals": [[-4.05, -1.06], [-6.97, -5.91], [-3.68, -3.66], [-4.71, 2.92]],
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
                "init_x": 6.75,
                "init_y": 2.53,
                "init_a": 1.31,
                "velocity": 1.16,
                "goals": [[-4.05, -1.06], [-6.61, -2.93], [2.22, -4.12], [6.61, 5.47]],
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
                "init_x": -6.46,
                "init_y": 1.28,
                "init_a": 106.46,
                "velocity": 1.12,
                "goals": [[4.34, 5.84], [-2.77, 3.39], [-1.58, 2.27], [4.20, -1.96]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.75,
                "init_y": -1.45,
                "init_a": -30.41,
                "velocity": 0.96,
                "goals": [[5.81, -1.29], [-4.85, 3.71], [1.79, -0.33], [-4.00, -3.22]],
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

def tests_adult_10_child_90_test_case_70_walking_high(tester):
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
                "goals": [[-1.43, 4.61], [-6.89, 5.16], [-4.66, -3.71], [1.96, 3.96]],
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
                "init_x": -7.03,
                "init_y": 1.90,
                "init_a": -118.43,
                "velocity": 1.01,
                "goals": [[-1.43, 4.61], [-0.66, 4.40], [-5.57, -1.85], [2.36, 3.29]],
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
                "init_x": 0.97,
                "init_y": -4.68,
                "init_a": -118.41,
                "velocity": 1.03,
                "goal_x": 6.39,
                "goal_y": 4.01,
                "n_actors": 9,
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
                "goals": [[6.39, 4.01], [0.72, 0.06], [6.61, 2.37], [-4.70, 3.48]],
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
                "init_x": -0.40,
                "init_y": -3.58,
                "init_a": -6.10,
                "velocity": 1.07,
                "goals": [[-2.32, 2.16], [-5.58, -4.34], [-2.99, 3.67], [3.21, -3.38]],
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
                "init_x": 5.67,
                "init_y": -1.32,
                "init_a": -96.86,
                "velocity": 1.13,
                "goals": [[4.80, 1.90], [2.66, 0.74], [2.59, 0.44], [-3.28, 0.89]],
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
                "init_x": 1.54,
                "init_y": 3.23,
                "init_a": -129.37,
                "velocity": 1.11,
                "goals": [[2.36, -4.09], [6.49, 2.66], [1.66, -0.13], [0.52, -3.90]],
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
                "init_x": 2.13,
                "init_y": 1.65,
                "init_a": 152.00,
                "velocity": 0.86,
                "goals": [[3.01, -1.64], [-2.36, 5.09], [7.94, 3.93], [-0.25, 2.81]],
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
                "init_x": 3.45,
                "init_y": -0.87,
                "init_a": -140.80,
                "velocity": 0.83,
                "goals": [[1.40, 1.41], [-0.28, -0.12], [-3.34, 4.17], [-4.04, 1.89]],
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

def tests_adult_10_child_90_test_case_71_stopped_high(tester):
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
                "goals": [[-6.42, -2.08], [1.54, -1.86], [-7.92, 0.60], [-1.57, 1.61]],
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
                "init_x": -6.56,
                "init_y": -1.09,
                "init_a": 113.86,
                "velocity": 1.15,
                "goals": [[-6.42, -2.08], [-2.05, -4.92], [-7.55, -0.26], [-3.34, 2.93]],
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
                "goals": [[5.10, 3.40], [4.65, 0.09], [2.98, 2.08], [7.18, 4.82]],
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
                "goals": [[5.10, 3.40], [-2.55, -5.69], [-6.25, 3.89], [-0.04, -5.66]],
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
                "goals": [[5.00, 4.08], [-7.21, -1.39], [-5.47, -4.84], [-5.66, 2.14]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.89,
                "init_y": 2.74,
                "init_a": -141.03,
                "velocity": 0.86,
                "goals": [[4.39, 1.89], [-4.44, -3.81], [5.59, 0.73], [-1.79, 5.73]],
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
                "init_x": -5.19,
                "init_y": 3.11,
                "init_a": 10.38,
                "velocity": 1.14,
                "goals": [[-5.77, -4.61], [5.38, -2.04], [-0.92, 5.43], [-6.51, -5.08]],
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
                "goals": [[3.58, -3.18], [2.53, 3.56], [4.95, 0.52], [2.74, 5.31]],
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

def tests_adult_10_child_90_test_case_72_stopped_high(tester):
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
                "goals": [[-3.50, -2.26], [-6.23, -1.51], [1.94, 3.69], [6.22, -2.31]],
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
                "init_x": -2.60,
                "init_y": -1.81,
                "init_a": 130.02,
                "velocity": 0.86,
                "goals": [[-3.50, -2.26], [-1.28, 2.61], [3.54, 5.09], [4.21, 0.98]],
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
                "init_x": -6.75,
                "init_y": -3.64,
                "init_a": 140.74,
                "velocity": 1.00,
                "goals": [[-6.75, -3.64], [2.66, 2.81], [-6.49, 1.21], [-2.73, 2.25]],
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
                "init_x": -7.75,
                "init_y": -3.70,
                "init_a": 140.74,
                "velocity": 1.00,
                "goals": [[-6.75, -3.64], [1.87, -5.82], [7.40, -2.98], [4.65, 4.41]],
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
                "goals": [[1.75, -0.05], [6.67, 0.62], [-5.10, 1.40], [-0.58, -3.72]],
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
                "init_x": -0.98,
                "init_y": 2.55,
                "init_a": -69.61,
                "velocity": 1.20,
                "goals": [[6.73, 0.50], [5.43, -4.01], [2.62, 4.65], [2.21, 2.49]],
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
                "init_x": -0.89,
                "init_y": 4.37,
                "init_a": 67.29,
                "velocity": 0.81,
                "goals": [[-6.96, 2.72], [-4.52, 5.34], [-6.04, 0.78], [-5.35, 5.62]],
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
                "init_x": 4.50,
                "init_y": -4.21,
                "init_a": 26.27,
                "velocity": 0.97,
                "goals": [[1.53, -0.48], [-6.91, -2.45], [2.19, 5.06], [4.15, -2.65]],
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

def tests_adult_10_child_90_test_case_73_stopped_high(tester):
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
                "init_x": 5.90,
                "init_y": -2.77,
                "init_a": -116.60,
                "velocity": 0.82,
                "goals": [[5.90, -2.77], [-3.86, -4.40], [6.13, 4.09], [-7.97, 3.96]],
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
                "init_x": 6.90,
                "init_y": -2.68,
                "init_a": -116.60,
                "velocity": 0.82,
                "goals": [[5.90, -2.77], [-3.39, 4.45], [1.34, -1.73], [4.16, -2.85]],
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
                "goals": [[7.95, -3.46], [-1.54, -4.54], [7.45, -0.99], [5.99, -5.19]],
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
                "goals": [[7.95, -3.46], [1.46, -2.05], [3.21, -5.20], [-3.87, -5.92]],
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
                "init_x": -0.55,
                "init_y": 0.92,
                "init_a": -103.91,
                "velocity": 1.17,
                "goals": [[-5.19, 5.35], [5.78, 5.70], [5.94, 0.26], [-0.29, 4.80]],
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
                "init_x": -7.95,
                "init_y": 4.34,
                "init_a": -102.42,
                "velocity": 1.15,
                "goal_x": -2.42,
                "goal_y": 4.76,
                "n_actors": 8,
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
                "goals": [[-1.60, -0.72], [3.06, -1.57], [-7.30, -3.72], [-5.34, -4.71]],
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
                "init_x": 1.71,
                "init_y": 2.52,
                "init_a": -136.41,
                "velocity": 0.82,
                "goals": [[4.19, 1.06], [1.35, 0.30], [-6.13, -5.60], [2.42, 2.51]],
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

def tests_adult_10_child_90_test_case_74_walking_high(tester):
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
                "goals": [[-6.21, 2.48], [7.56, -3.55], [-2.43, 4.15], [7.60, -0.21]],
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
                "init_x": 7.08,
                "init_y": 4.42,
                "init_a": 112.30,
                "velocity": 0.90,
                "goals": [[-6.21, 2.48], [2.16, 3.08], [5.37, -2.98], [2.52, -3.33]],
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
                "goals": [[2.09, 3.91], [-3.76, 1.18], [4.44, 4.70], [1.04, -1.07]],
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
                "goals": [[2.09, 3.91], [6.36, -5.11], [-1.05, 5.74], [6.60, 2.54]],
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.94,
                "init_y": 4.46,
                "init_a": -80.30,
                "velocity": 0.92,
                "goals": [[7.66, -1.08], [7.10, 4.55], [5.83, 5.40], [-2.49, 5.47]],
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
                "goals": [[1.90, 2.82], [-0.11, -2.06], [6.49, 0.41], [-5.37, 2.33]],
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
                "init_x": -6.84,
                "init_y": 2.69,
                "init_a": -165.69,
                "velocity": 0.89,
                "goals": [[4.55, 4.44], [3.69, -1.60], [-1.15, 3.45], [0.65, -5.50]],
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

def tests_adult_10_child_90_test_case_75_walking_high(tester):
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
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.00,
                "init_y": 2.66,
                "init_a": 72.15,
                "velocity": 0.99,
                "goals": [[4.86, 1.57], [-5.85, -3.22], [1.60, -4.13], [-0.82, 3.04]],
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
                "init_x": -4.80,
                "init_y": 2.01,
                "init_a": 65.93,
                "velocity": 1.04,
                "goals": [[-4.17, 2.05], [-7.23, 0.22], [7.21, -1.14], [1.58, 3.26]],
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
                "init_x": -5.04,
                "init_y": 2.98,
                "init_a": 65.93,
                "velocity": 1.04,
                "goals": [[-4.17, 2.05], [6.24, 3.43], [0.77, -5.75], [7.64, 4.44]],
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
                "init_x": -2.49,
                "init_y": 0.45,
                "init_a": 25.33,
                "velocity": 1.13,
                "goals": [[0.54, -2.40], [2.10, 0.83], [-6.54, -4.22], [-4.94, -2.00]],
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
                "init_x": 3.87,
                "init_y": -3.46,
                "init_a": -44.09,
                "velocity": 0.91,
                "goals": [[-0.80, 0.52], [4.89, -3.62], [6.72, -4.89], [6.09, -4.33]],
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
                "goals": [[5.52, 3.58], [1.57, 4.54], [4.66, 5.49], [2.02, -4.19]],
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

def tests_adult_10_child_90_test_case_76_walking_high(tester):
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
                "init_x": 2.17,
                "init_y": 0.01,
                "init_a": -44.48,
                "velocity": 0.92,
                "goals": [[3.16, -0.18], [-3.18, -1.20], [3.91, 2.38], [0.89, -5.63]],
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
                "goals": [[3.87, -1.18], [5.83, 4.84], [-6.49, 1.52], [-4.11, 4.63]],
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
                "goals": [[3.87, -1.18], [-5.43, -1.52], [-4.21, 0.04], [-7.25, -0.39]],
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
                "init_x": -6.46,
                "init_y": 3.18,
                "init_a": -21.39,
                "velocity": 1.03,
                "goals": [[0.42, 1.82], [-7.61, 3.14], [-3.64, -2.81], [-0.83, -5.20]],
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
                "init_x": 2.56,
                "init_y": -1.81,
                "init_a": -45.64,
                "velocity": 1.05,
                "goals": [[1.25, -5.61], [2.47, 2.45], [0.24, -3.46], [-2.15, 0.97]],
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
                "goals": [[-6.27, -3.68], [-3.72, 4.38], [5.66, -0.29], [2.11, -0.66]],
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

def tests_adult_10_child_90_test_case_77_stopped_high(tester):
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
                "goals": [[0.53, 3.14], [6.25, -1.94], [5.44, 4.34], [4.67, -1.47]],
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
                "goals": [[0.53, 3.14], [4.08, 1.74], [-4.17, -3.99], [1.19, -0.37]],
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
                "goals": [[2.35, -0.77], [-6.24, 1.20], [-3.54, 5.72], [2.18, -3.53]],
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
                "goals": [[3.45, -1.11], [0.78, -0.31], [1.58, -2.06], [-0.59, -3.72]],
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
                "init_x": -4.44,
                "init_y": 0.87,
                "init_a": -68.27,
                "velocity": 0.96,
                "goals": [[-1.49, 3.88], [5.63, 1.22], [5.29, -2.64], [5.70, 5.78]],
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
                "init_x": -0.91,
                "init_y": -0.23,
                "init_a": 1.76,
                "velocity": 0.95,
                "goals": [[-3.93, -4.08], [-5.70, -4.09], [2.53, -5.47], [4.95, 3.89]],
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
                "goals": [[4.76, -1.54], [-6.26, 3.24], [-4.31, -5.85], [4.48, -4.67]],
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
                "init_x": -1.30,
                "init_y": 5.90,
                "init_a": 133.55,
                "velocity": 1.00,
                "goals": [[1.78, 2.47], [-2.25, 4.79], [-1.06, -2.14], [-3.96, 4.14]],
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

def tests_adult_10_child_90_test_case_78_stopped_high(tester):
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
                "goals": [[0.22, -1.46], [-4.99, 4.79], [-4.71, -3.66], [1.79, -3.98]],
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
                "init_x": 0.99,
                "init_y": -2.10,
                "init_a": 178.09,
                "velocity": 0.94,
                "goals": [[0.22, -1.46], [4.79, -5.60], [7.35, 1.23], [-6.30, 2.08]],
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
                "init_x": -2.86,
                "init_y": -0.75,
                "init_a": -5.96,
                "velocity": 1.13,
                "goals": [[-2.86, -0.75], [3.87, 2.30], [-2.95, 3.69], [-1.15, -0.29]],
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
                "goals": [[2.35, 4.29], [-3.42, 0.00], [1.48, 1.35], [-6.66, 1.89]],
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
                "goals": [[5.33, -1.40], [4.44, -0.59], [-2.38, -1.06], [2.93, 4.26]],
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
                "init_x": 6.47,
                "init_y": -3.56,
                "init_a": -31.43,
                "velocity": 0.94,
                "goals": [[1.28, 2.97], [7.10, 5.84], [-6.20, 3.95], [3.76, 1.48]],
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

def tests_adult_10_child_90_test_case_79_stopped_high(tester):
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
                "init_x": -7.54,
                "init_y": -5.32,
                "init_a": -114.62,
                "velocity": 0.92,
                "goal_x": -7.54,
                "goal_y": -5.32,
                "n_actors": 7,
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
                "goals": [[-7.54, -5.32], [5.98, 1.67], [3.44, -1.35], [-2.30, -4.27]],
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
                "init_x": 2.87,
                "init_y": -3.35,
                "init_a": 109.27,
                "velocity": 0.89,
                "goals": [[2.87, -3.35], [5.53, -4.17], [-3.28, 2.02], [5.31, 0.69]],
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
                "init_x": 3.87,
                "init_y": -3.25,
                "init_a": 109.27,
                "velocity": 0.89,
                "goals": [[2.87, -3.35], [-2.28, 0.50], [1.74, -4.39], [-2.57, 4.12]],
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
                "init_x": -2.31,
                "init_y": -1.97,
                "init_a": 123.27,
                "velocity": 1.05,
                "goals": [[-1.54, 5.84], [-4.93, -5.97], [6.39, -4.03], [7.28, -5.48]],
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
                "init_x": 0.22,
                "init_y": -2.80,
                "init_a": -88.28,
                "velocity": 0.92,
                "goals": [[4.17, -2.22], [3.30, -5.44], [2.58, 2.23], [2.90, -2.34]],
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
                "init_x": -2.82,
                "init_y": 5.92,
                "init_a": 165.60,
                "velocity": 1.19,
                "goals": [[1.65, -5.65], [-7.95, -2.92], [-2.30, -4.25], [-3.56, -0.58]],
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

def tests_adult_10_child_90_test_case_80_walking_high(tester):
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
                "init_x": 3.16,
                "init_y": 2.70,
                "init_a": 56.49,
                "velocity": 0.97,
                "goals": [[-6.75, -1.59], [-7.94, 2.98], [0.00, 3.63], [5.45, -4.45]],
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
                "init_x": 3.40,
                "init_y": 3.67,
                "init_a": 56.49,
                "velocity": 0.97,
                "goals": [[-6.75, -1.59], [7.58, 3.17], [-1.77, -3.55], [-4.21, 4.13]],
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
                "init_x": -2.52,
                "init_y": -3.54,
                "init_a": 127.59,
                "velocity": 0.84,
                "goals": [[-7.11, -2.60], [0.61, 2.13], [-1.66, 2.17], [-4.21, -5.24]],
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
                "goals": [[-2.97, -1.98], [7.61, -4.77], [4.13, 2.12], [-3.47, 5.08]],
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
                "init_x": 0.39,
                "init_y": -3.57,
                "init_a": -20.36,
                "velocity": 0.98,
                "goals": [[-3.33, 1.95], [-3.90, 3.02], [-6.59, 2.77], [7.72, -1.25]],
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
                "init_x": 1.13,
                "init_y": 1.02,
                "init_a": -72.97,
                "velocity": 1.03,
                "goals": [[-0.12, 1.97], [-3.57, 4.98], [7.71, 0.42], [-3.33, 4.68]],
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

def tests_adult_10_child_90_test_case_81_stopped_high(tester):
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
                "goals": [[-0.01, 0.38], [1.08, 2.48], [-4.63, -5.65], [4.36, -2.90]],
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
                "init_x": 0.30,
                "init_y": 1.33,
                "init_a": 132.14,
                "velocity": 0.82,
                "goals": [[-0.01, 0.38], [5.77, 4.06], [4.72, -1.72], [-7.27, 0.14]],
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
                "goals": [[6.90, -2.01], [-4.50, 0.54], [7.63, 3.09], [2.80, 1.62]],
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
                "goals": [[6.90, -2.01], [-5.74, 4.71], [5.39, -2.41], [1.70, -3.76]],
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
                "init_x": 4.94,
                "init_y": -0.32,
                "init_a": 79.63,
                "velocity": 1.15,
                "goals": [[2.01, -4.61], [4.51, 4.67], [-6.86, -4.95], [0.64, 4.53]],
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
                "init_x": -0.23,
                "init_y": -3.67,
                "init_a": -46.76,
                "velocity": 1.10,
                "goals": [[3.27, 5.92], [-7.00, 0.15], [5.92, -2.90], [5.20, 3.14]],
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

def tests_adult_10_child_90_test_case_82_walking_high(tester):
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
                "goals": [[3.52, -4.79], [-4.30, -2.03], [5.01, -6.00], [7.39, -4.11]],
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
                "init_x": -5.36,
                "init_y": 5.25,
                "init_a": 158.47,
                "velocity": 0.95,
                "goals": [[3.52, -4.79], [1.46, 3.81], [-4.62, -3.84], [7.26, -5.58]],
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
                "goals": [[-6.63, -1.13], [-0.25, 3.83], [-3.84, -2.89], [-0.13, -2.06]],
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
                "goals": [[-7.03, -5.84], [-7.56, -1.17], [3.22, 5.23], [5.43, 1.00]],
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
                "goals": [[5.11, -2.87], [1.00, 5.42], [6.72, -1.12], [-7.51, -5.96]],
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
                "init_x": -0.52,
                "init_y": 0.27,
                "init_a": -18.49,
                "velocity": 0.96,
                "goals": [[7.43, -5.48], [-3.45, -3.07], [-4.76, -0.68], [0.10, 0.63]],
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
                "init_x": 7.10,
                "init_y": 5.31,
                "init_a": -42.58,
                "velocity": 1.20,
                "goals": [[-2.11, 1.32], [-7.80, 4.57], [-0.94, 1.25], [-4.65, 4.43]],
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
                "init_x": 5.44,
                "init_y": -3.03,
                "init_a": -44.47,
                "velocity": 1.02,
                "goals": [[7.43, 2.56], [-5.40, 4.41], [-4.83, 4.77], [-0.14, -1.73]],
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

def tests_adult_10_child_90_test_case_83_walking_high(tester):
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
                "goals": [[7.21, 2.86], [-3.90, -1.76], [2.32, 0.99], [-5.98, 0.84]],
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
                "init_x": 4.42,
                "init_y": -5.53,
                "init_a": 15.18,
                "velocity": 1.18,
                "goals": [[7.21, 2.86], [-4.42, 2.04], [-1.93, -2.15], [7.22, 2.30]],
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
                "init_x": -6.14,
                "init_y": -2.00,
                "init_a": 134.89,
                "velocity": 1.10,
                "goals": [[3.62, -5.16], [6.37, -2.71], [1.37, -3.33], [-7.25, 2.78]],
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
                "init_x": -6.90,
                "init_y": -2.65,
                "init_a": 134.89,
                "velocity": 1.10,
                "goals": [[3.62, -5.16], [-2.46, -1.61], [5.68, 2.05], [-4.18, 4.76]],
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
                "goals": [[4.58, -4.29], [-0.64, 0.95], [1.91, 3.51], [-7.03, 4.23]],
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
                "init_x": 5.52,
                "init_y": 0.26,
                "init_a": 163.92,
                "velocity": 1.11,
                "goal_x": -1.51,
                "goal_y": 4.88,
                "n_actors": 7,
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
                "goals": [[7.27, -3.39], [1.93, 1.35], [-5.79, -5.52], [-1.05, 3.32]],
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

def tests_adult_10_child_90_test_case_84_walking_high(tester):
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
                "goals": [[-5.76, -4.85], [6.17, -3.26], [3.36, 5.87], [5.70, 4.51]],
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
                "goals": [[-5.76, -4.85], [-6.75, -1.12], [-3.60, -2.38], [-6.09, -3.19]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -8.04,
                "init_y": -0.19,
                "init_a": -40.36,
                "velocity": 0.84,
                "goals": [[-4.51, -5.16], [3.45, -5.18], [-0.95, -4.92], [1.74, 3.47]],
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
                "init_x": -4.87,
                "init_y": 3.19,
                "init_a": -53.55,
                "velocity": 1.04,
                "goals": [[-2.14, -0.38], [5.28, -0.42], [1.50, 6.00], [3.13, -1.83]],
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
                "goals": [[6.90, 1.19], [1.25, -5.88], [-6.42, -2.17], [-0.79, 3.46]],
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
                "init_x": -7.49,
                "init_y": 1.54,
                "init_a": -117.03,
                "velocity": 1.07,
                "goals": [[0.48, -0.81], [-4.11, 2.74], [-1.14, 0.90], [-0.82, 4.70]],
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
                "init_x": -1.38,
                "init_y": 3.85,
                "init_a": -135.10,
                "velocity": 0.88,
                "goals": [[-4.01, -3.23], [4.89, -5.18], [-4.51, -2.67], [5.44, -2.28]],
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

def tests_adult_10_child_90_test_case_85_stopped_high(tester):
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
                "goals": [[-3.02, 2.61], [5.28, -5.57], [-2.84, 0.95], [-7.75, 3.60]],
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
                "init_x": -3.70,
                "init_y": 3.34,
                "init_a": -47.10,
                "velocity": 0.97,
                "goals": [[-3.02, 2.61], [2.62, -1.12], [3.27, 5.83], [0.67, -5.18]],
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
                "init_x": -0.32,
                "init_y": -2.63,
                "init_a": 167.90,
                "velocity": 0.87,
                "goals": [[-0.32, -2.63], [-4.01, 0.27], [1.69, 3.55], [-6.84, -3.25]],
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
                "init_x": -1.16,
                "init_y": -2.08,
                "init_a": 167.90,
                "velocity": 0.87,
                "goals": [[-0.32, -2.63], [7.78, 3.97], [-6.01, -1.35], [-1.77, 1.97]],
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
                "init_x": 3.11,
                "init_y": -2.19,
                "init_a": 110.59,
                "velocity": 1.06,
                "goals": [[-7.51, 4.66], [0.91, 0.05], [4.73, 2.53], [7.52, 2.69]],
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
                "init_x": -1.30,
                "init_y": -4.00,
                "init_a": 157.21,
                "velocity": 1.19,
                "goals": [[-4.06, 5.03], [-3.96, 1.18], [6.81, 2.51], [-4.08, -0.24]],
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
                "init_x": -3.48,
                "init_y": 2.99,
                "init_a": 107.28,
                "velocity": 0.93,
                "goal_x": 2.41,
                "goal_y": -2.97,
                "n_actors": 8,
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
                "goals": [[6.59, 2.15], [2.96, 5.61], [-4.76, -1.21], [7.32, -5.12]],
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

def tests_adult_10_child_90_test_case_86_stopped_high(tester):
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
                "goals": [[4.14, -3.34], [6.22, -0.70], [0.58, 4.11], [-5.43, 5.19]],
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
                "init_x": 4.15,
                "init_y": -2.34,
                "init_a": -44.50,
                "velocity": 0.91,
                "goals": [[4.14, -3.34], [-0.36, -5.03], [-6.81, 4.82], [-5.85, 3.65]],
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
                "init_x": 0.69,
                "init_y": -2.12,
                "init_a": -138.46,
                "velocity": 1.05,
                "goals": [[0.69, -2.12], [-7.80, 4.09], [-3.55, 5.70], [3.23, -5.90]],
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
                "init_x": 0.96,
                "init_y": -3.09,
                "init_a": -138.46,
                "velocity": 1.05,
                "goal_x": 0.69,
                "goal_y": -2.12,
                "n_actors": 8,
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
                "goals": [[-0.77, -4.85], [-3.25, 3.74], [-2.17, -1.45], [-6.24, 1.54]],
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
                "goals": [[3.36, -5.42], [5.19, -5.23], [-3.40, 4.79], [-3.03, 0.05]],
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
                "init_x": -0.56,
                "init_y": 1.66,
                "init_a": -109.24,
                "velocity": 0.80,
                "goals": [[-2.36, 3.39], [7.05, -1.50], [3.62, 3.27], [4.15, -0.17]],
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
                "init_x": 6.50,
                "init_y": 4.98,
                "init_a": 81.01,
                "velocity": 1.16,
                "goals": [[0.39, -3.17], [-6.71, -0.10], [4.94, 2.14], [-3.52, -5.94]],
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

def tests_adult_10_child_90_test_case_87_walking_high(tester):
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
                "goals": [[5.78, 0.95], [-2.54, -5.56], [0.18, 4.60], [0.21, 5.56]],
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
                "goals": [[5.78, 0.95], [2.84, -3.30], [-6.36, 2.26], [4.11, -3.43]],
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
                "init_x": 2.26,
                "init_y": 5.03,
                "init_a": 23.30,
                "velocity": 0.92,
                "goals": [[5.24, -3.04], [-2.03, -1.90], [-6.24, 4.76], [-7.35, 3.92]],
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
                "init_x": 1.26,
                "init_y": 5.11,
                "init_a": 23.30,
                "velocity": 0.92,
                "goals": [[5.24, -3.04], [-5.10, -0.78], [-4.35, 4.03], [-4.44, -5.38]],
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
                "goals": [[4.63, 5.51], [-7.44, 0.93], [-4.14, -3.25], [3.20, 4.46]],
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
                "goals": [[-4.66, -0.15], [-2.40, -2.97], [-6.05, 0.49], [-5.44, -4.70]],
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
                "init_x": 4.41,
                "init_y": -2.16,
                "init_a": -86.81,
                "velocity": 1.14,
                "goals": [[1.29, 4.51], [4.09, 3.21], [1.94, -2.52], [5.62, -3.65]],
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

def tests_adult_10_child_90_test_case_88_stopped_high(tester):
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
                "goals": [[1.67, -1.74], [-7.09, 3.52], [-5.62, -2.51], [1.77, 2.00]],
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
                "init_x": 2.30,
                "init_y": -0.97,
                "init_a": -63.27,
                "velocity": 0.82,
                "goals": [[1.67, -1.74], [-6.30, 2.43], [6.07, 2.25], [0.23, 1.52]],
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
                "init_x": 5.91,
                "init_y": -4.60,
                "init_a": 71.80,
                "velocity": 1.07,
                "goals": [[5.91, -4.60], [-1.23, -0.12], [0.46, 4.96], [-1.82, -0.37]],
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
                "init_x": 4.93,
                "init_y": -4.80,
                "init_a": 71.80,
                "velocity": 1.07,
                "goals": [[5.91, -4.60], [-1.56, -5.30], [-4.19, 4.92], [-3.88, 5.92]],
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
                "init_x": 6.15,
                "init_y": 2.28,
                "init_a": 149.14,
                "velocity": 0.80,
                "goals": [[-3.71, 0.39], [-2.19, -3.45], [0.16, 5.14], [0.88, -3.93]],
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
                "init_x": 1.24,
                "init_y": 5.62,
                "init_a": -36.05,
                "velocity": 1.10,
                "goals": [[-7.02, -1.96], [-0.97, 0.16], [-6.26, 3.25], [-5.84, 3.45]],
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

def tests_adult_10_child_90_test_case_89_walking_high(tester):
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
                "goals": [[0.14, -2.21], [-3.86, 0.43], [-7.09, 2.26], [0.84, -3.66]],
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
                "init_x": -3.31,
                "init_y": 1.78,
                "init_a": -82.96,
                "velocity": 0.89,
                "goals": [[0.14, -2.21], [6.57, 5.24], [7.93, -2.58], [-7.41, 3.77]],
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
                "init_x": -0.61,
                "init_y": 4.91,
                "init_a": 108.11,
                "velocity": 0.96,
                "goals": [[4.90, 5.22], [2.74, -1.88], [4.60, 3.45], [-7.10, 0.99]],
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
                "init_x": -0.68,
                "init_y": 3.91,
                "init_a": 108.11,
                "velocity": 0.96,
                "goals": [[4.90, 5.22], [7.32, 5.06], [-2.42, 5.58], [0.91, -2.45]],
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
                "init_x": -1.24,
                "init_y": 2.69,
                "init_a": -31.82,
                "velocity": 1.11,
                "goals": [[-5.11, -2.95], [4.78, 5.82], [-7.82, -0.61], [2.64, -2.79]],
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
                "init_x": -1.58,
                "init_y": -4.70,
                "init_a": 28.71,
                "velocity": 0.83,
                "goals": [[-0.55, -3.10], [-0.42, 2.73], [-1.75, 3.37], [0.38, 3.26]],
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
                "init_x": 1.16,
                "init_y": 1.79,
                "init_a": 38.43,
                "velocity": 1.15,
                "goals": [[-5.70, -5.64], [4.82, 3.83], [7.13, 2.68], [-1.80, -4.13]],
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
                "init_x": 6.39,
                "init_y": -0.27,
                "init_a": -98.85,
                "velocity": 0.89,
                "goal_x": -2.89,
                "goal_y": -1.10,
                "n_actors": 9,
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
                "goals": [[-3.55, 2.27], [-2.79, -5.38], [5.59, 4.02], [3.40, -4.15]],
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

def tests_adult_10_child_90_test_case_90_stopped_high(tester):
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
                "goals": [[-0.93, 2.74], [0.04, 1.45], [-0.92, 5.62], [-3.39, -5.55]],
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
                "goals": [[-3.13, -4.23], [-0.65, -4.04], [-5.20, 5.09], [-4.16, 5.90]],
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
                "goals": [[-3.13, -4.23], [5.49, -5.77], [-0.17, 4.51], [7.98, 5.73]],
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
                "goals": [[2.37, 2.14], [-2.19, 5.18], [-1.09, 2.32], [-1.34, 4.45]],
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
                "goals": [[-5.78, -4.44], [2.08, -0.46], [-7.18, -0.27], [-5.40, -2.06]],
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
                "init_x": 3.54,
                "init_y": 5.12,
                "init_a": 27.87,
                "velocity": 1.12,
                "goals": [[-0.87, -4.13], [5.70, -5.95], [-3.13, -0.96], [2.56, -1.72]],
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
                "init_x": 4.08,
                "init_y": -1.20,
                "init_a": 108.61,
                "velocity": 0.96,
                "goals": [[4.47, -1.06], [-3.46, 4.91], [0.19, -5.16], [4.55, 4.42]],
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
                "init_x": -6.10,
                "init_y": -1.78,
                "init_a": 109.96,
                "velocity": 0.99,
                "goals": [[-6.33, -5.26], [2.93, 2.17], [6.99, -3.52], [4.95, 3.82]],
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

def tests_adult_10_child_90_test_case_91_stopped_high(tester):
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
                "init_x": -6.96,
                "init_y": 4.39,
                "init_a": -13.42,
                "velocity": 0.93,
                "goals": [[-6.96, 4.39], [6.06, 2.29], [-3.20, -1.39], [-0.66, 4.92]],
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
                "init_x": -5.96,
                "init_y": 4.50,
                "init_a": -13.42,
                "velocity": 0.93,
                "goals": [[-6.96, 4.39], [1.34, 1.35], [-5.47, 3.95], [2.17, -1.90]],
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
                "init_x": -2.51,
                "init_y": 4.82,
                "init_a": -48.16,
                "velocity": 1.02,
                "goals": [[-2.51, 4.82], [7.43, 0.52], [3.39, 1.28], [-1.39, 4.66]],
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
                "goals": [[-2.51, 4.82], [7.39, -2.54], [1.95, -0.89], [-3.52, 2.22]],
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
                "init_x": 1.75,
                "init_y": -2.91,
                "init_a": -108.30,
                "velocity": 1.20,
                "goals": [[-0.91, -0.02], [-4.41, -0.52], [7.37, -2.15], [-3.96, 0.42]],
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
                "goals": [[-6.31, 5.32], [-3.62, 2.31], [-6.03, 1.91], [-7.24, 5.00]],
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

def tests_adult_10_child_90_test_case_92_walking_high(tester):
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
                "goals": [[-3.22, -5.19], [-2.50, 1.36], [-0.39, -2.62], [7.89, -5.87]],
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
                "init_x": 2.40,
                "init_y": 4.48,
                "init_a": 131.30,
                "velocity": 1.07,
                "goals": [[-3.22, -5.19], [-4.67, 2.73], [7.39, -5.82], [-1.78, -5.93]],
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
                "init_x": -0.70,
                "init_y": -0.09,
                "init_a": -53.62,
                "velocity": 0.85,
                "goals": [[-3.26, -1.48], [2.37, -2.71], [6.17, -5.59], [4.91, -4.96]],
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
                "init_x": -0.76,
                "init_y": 0.91,
                "init_a": -53.62,
                "velocity": 0.85,
                "goals": [[-3.26, -1.48], [-0.43, -2.66], [-6.78, -1.76], [5.49, -2.78]],
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
                "init_x": 3.96,
                "init_y": -3.02,
                "init_a": -117.92,
                "velocity": 1.02,
                "goals": [[4.20, -2.79], [-2.27, 3.19], [-1.36, -3.36], [5.84, -4.95]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.76,
                "init_y": 3.67,
                "init_a": 6.84,
                "velocity": 0.89,
                "goals": [[0.04, -4.50], [2.92, -4.08], [-3.66, 3.33], [7.74, -3.37]],
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
                "init_x": -1.32,
                "init_y": -4.07,
                "init_a": 174.76,
                "velocity": 1.08,
                "goals": [[-2.29, 4.25], [-1.62, 2.24], [-2.89, -4.79], [2.17, 1.06]],
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

def tests_adult_10_child_90_test_case_93_walking_high(tester):
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
                "goals": [[-1.77, 3.82], [-2.74, -5.82], [5.45, -5.95], [-3.45, 4.65]],
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
                "init_x": 5.51,
                "init_y": 4.97,
                "init_a": 26.10,
                "velocity": 0.99,
                "goals": [[-1.77, 3.82], [0.54, -2.52], [6.80, -5.88], [-6.69, 0.23]],
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
                "init_x": -3.97,
                "init_y": -3.55,
                "init_a": -139.66,
                "velocity": 0.89,
                "goals": [[-4.21, -5.96], [2.95, -0.32], [-4.37, -1.45], [5.59, -4.24]],
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
                "goals": [[6.38, -4.82], [-7.64, 5.13], [4.63, 1.55], [4.52, 1.80]],
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
                "goals": [[1.99, -4.98], [-6.47, 5.92], [-5.80, -4.29], [-6.32, -1.71]],
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
                "init_x": -3.46,
                "init_y": 4.62,
                "init_a": 112.00,
                "velocity": 1.13,
                "goals": [[-0.18, -4.11], [3.66, 1.59], [4.63, 1.24], [2.49, -5.53]],
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

def tests_adult_10_child_90_test_case_94_stopped_high(tester):
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
                "goals": [[2.89, 5.31], [-7.41, -1.15], [-7.98, -4.92], [-5.95, -1.05]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.03,
                "init_y": -4.62,
                "init_a": -91.41,
                "velocity": 1.02,
                "goals": [[-1.03, -4.62], [-2.31, 0.19], [7.08, -5.21], [0.68, 0.60]],
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
                "init_x": -1.34,
                "init_y": -5.57,
                "init_a": -91.41,
                "velocity": 1.02,
                "goals": [[-1.03, -4.62], [0.43, 0.91], [6.92, -5.90], [-6.97, -4.83]],
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
                "init_x": -1.90,
                "init_y": 5.02,
                "init_a": 75.35,
                "velocity": 1.19,
                "goals": [[-2.50, -0.70], [3.86, 1.53], [1.98, -1.57], [5.88, -0.14]],
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
                "init_x": 4.65,
                "init_y": -1.35,
                "init_a": -47.70,
                "velocity": 0.97,
                "goals": [[-0.98, 0.12], [6.85, -3.36], [-6.60, 1.32], [6.64, -5.76]],
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
                "goals": [[3.19, 3.93], [-5.35, 5.72], [-1.96, -4.23], [1.05, 4.68]],
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
                "goals": [[-3.49, 2.80], [5.76, -5.96], [3.01, -5.67], [-0.27, -0.68]],
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

def tests_adult_10_child_90_test_case_95_walking_high(tester):
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
                "goals": [[7.02, -3.76], [-1.81, 2.32], [6.53, 3.18], [-1.84, -1.69]],
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
                "init_x": -5.12,
                "init_y": -2.74,
                "init_a": 157.73,
                "velocity": 0.85,
                "goals": [[7.02, -3.76], [-3.45, 1.66], [6.69, -2.82], [6.13, 1.64]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.42,
                "init_y": -3.51,
                "init_a": 137.27,
                "velocity": 1.08,
                "goals": [[-0.39, -4.50], [-4.17, -1.30], [-7.35, -1.56], [-3.55, 0.19]],
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
                "init_x": 7.22,
                "init_y": -0.54,
                "init_a": -83.15,
                "velocity": 1.08,
                "goals": [[1.65, 4.25], [7.92, -5.66], [-0.81, 2.57], [-1.53, -2.81]],
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
                "init_x": 4.95,
                "init_y": 0.18,
                "init_a": 89.82,
                "velocity": 1.20,
                "goals": [[6.00, -3.35], [-4.92, -4.52], [6.06, 3.56], [1.67, -1.70]],
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
                "init_x": -6.69,
                "init_y": -3.58,
                "init_a": 55.47,
                "velocity": 0.89,
                "goals": [[-3.91, -5.08], [-7.78, -3.06], [-4.37, 1.71], [6.29, 5.90]],
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
                "init_x": 0.19,
                "init_y": 4.61,
                "init_a": -169.14,
                "velocity": 1.19,
                "goals": [[5.96, 0.82], [-0.11, 2.33], [7.99, 3.38], [-7.49, 0.10]],
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
                "init_x": -2.03,
                "init_y": -0.89,
                "init_a": -137.35,
                "velocity": 1.00,
                "goals": [[3.57, 5.44], [3.79, 1.02], [2.95, 4.45], [6.19, 0.05]],
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

def tests_adult_10_child_90_test_case_96_walking_high(tester):
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
                "init_x": 7.04,
                "init_y": -3.74,
                "init_a": 30.13,
                "velocity": 0.89,
                "goals": [[-2.82, 3.08], [-3.70, 5.37], [-6.15, -2.50], [-1.56, 2.68]],
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
                "init_x": 6.07,
                "init_y": -3.47,
                "init_a": 30.13,
                "velocity": 0.89,
                "goals": [[-2.82, 3.08], [-0.99, -1.94], [-1.10, 1.30], [-1.80, 1.76]],
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
                "goals": [[2.84, -0.13], [3.65, 3.73], [7.63, -1.17], [-2.50, 0.38]],
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
                "init_x": 0.67,
                "init_y": 3.32,
                "init_a": 113.88,
                "velocity": 0.89,
                "goals": [[4.87, -2.61], [0.03, 1.86], [4.18, 2.11], [-2.00, 3.81]],
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
                "init_x": 3.25,
                "init_y": 4.21,
                "init_a": 26.08,
                "velocity": 1.09,
                "goals": [[-2.18, -1.43], [2.89, 0.80], [-4.83, -1.11], [-5.90, 2.72]],
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
                "init_x": -5.78,
                "init_y": -2.80,
                "init_a": -176.42,
                "velocity": 1.13,
                "goals": [[-7.31, -2.48], [1.20, -5.12], [4.53, -3.40], [2.35, -0.18]],
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
                "init_x": -5.07,
                "init_y": -0.90,
                "init_a": 106.82,
                "velocity": 0.99,
                "goals": [[7.27, 0.53], [3.79, 3.40], [1.50, 4.08], [-2.01, 2.26]],
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
                "init_x": -1.49,
                "init_y": 0.03,
                "init_a": 13.65,
                "velocity": 0.91,
                "goals": [[4.14, -5.00], [-7.87, -4.62], [0.29, -2.85], [2.85, 4.73]],
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

def tests_adult_10_child_90_test_case_97_walking_high(tester):
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
                "goals": [[2.60, -2.87], [-6.89, -1.28], [7.75, -0.62], [5.18, 5.62]],
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
                "goals": [[2.60, -2.87], [-4.84, 1.28], [0.11, 3.32], [0.72, 1.63]],
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
                "goals": [[-2.32, -4.17], [-0.14, 3.06], [3.05, 2.03], [-1.90, -3.72]],
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
                "goals": [[-2.32, -4.17], [1.66, -5.08], [-5.70, 0.50], [2.67, -2.95]],
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
                "goals": [[-0.17, -4.25], [-2.40, -3.47], [-3.92, -0.51], [-1.40, 2.75]],
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
                "init_x": 4.82,
                "init_y": 1.14,
                "init_a": 82.00,
                "velocity": 0.89,
                "goals": [[-6.57, 5.79], [7.34, 0.28], [2.52, -2.91], [-1.48, 0.72]],
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
                "goals": [[4.26, -5.22], [-5.50, 4.97], [-1.68, -0.20], [-2.88, -5.81]],
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

def tests_adult_10_child_90_test_case_98_walking_high(tester):
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
                "goals": [[-5.49, -0.88], [2.61, -2.34], [7.11, -0.41], [-1.59, 1.34]],
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
                "init_x": 2.62,
                "init_y": 6.23,
                "init_a": 58.44,
                "velocity": 1.05,
                "goals": [[-5.49, -0.88], [6.37, 3.62], [-0.85, -3.30], [4.05, -1.27]],
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
                "init_x": -1.00,
                "init_y": -4.89,
                "init_a": -51.17,
                "velocity": 0.92,
                "goals": [[-6.17, -2.00], [0.25, -4.67], [7.87, 4.43], [-1.14, 2.76]],
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
                "init_x": -0.68,
                "init_y": -5.83,
                "init_a": -51.17,
                "velocity": 0.92,
                "goals": [[-6.17, -2.00], [3.63, -5.47], [0.23, 5.72], [2.03, -5.64]],
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
                "init_x": -3.96,
                "init_y": -2.65,
                "init_a": -69.77,
                "velocity": 0.81,
                "goals": [[-0.54, -2.42], [3.46, -1.91], [-2.31, -3.41], [2.93, 1.60]],
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
                "init_x": -3.94,
                "init_y": -3.58,
                "init_a": -179.63,
                "velocity": 1.17,
                "goals": [[-1.00, 2.93], [-3.37, -2.36], [7.51, 5.84], [2.29, -5.61]],
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
                "init_x": -3.94,
                "init_y": -2.81,
                "init_a": 128.03,
                "velocity": 0.82,
                "goals": [[-0.35, -3.38], [7.61, 1.68], [4.64, -5.07], [-7.25, -1.24]],
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
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.63,
                "init_y": 3.92,
                "init_a": -113.88,
                "velocity": 1.07,
                "goals": [[4.80, 4.32], [7.76, 3.66], [3.01, -4.69], [5.27, -4.61]],
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

def tests_adult_10_child_90_test_case_99_stopped_high(tester):
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
                "goals": [[0.77, 2.15], [-3.27, -0.90], [2.49, 0.87], [-2.32, 2.35]],
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
                "init_y": 1.48,
                "init_a": -109.88,
                "velocity": 0.91,
                "goal_x": 0.77,
                "goal_y": 2.15,
                "n_actors": 7,
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
                "goals": [[7.24, -5.77], [-5.38, 4.64], [3.66, 1.41], [5.25, -4.76]],
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
                "init_x": 6.65,
                "init_y": -6.58,
                "init_a": 94.01,
                "velocity": 0.99,
                "goals": [[7.24, -5.77], [-7.38, -5.29], [6.43, -4.80], [0.43, -2.54]],
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
                "init_x": -4.35,
                "init_y": -5.99,
                "init_a": -43.20,
                "velocity": 0.94,
                "goals": [[-1.22, 2.65], [-6.46, -0.56], [4.01, 0.34], [0.86, 0.43]],
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
                "init_x": 1.32,
                "init_y": -0.58,
                "init_a": -131.71,
                "velocity": 1.19,
                "goals": [[4.83, -4.33], [-6.85, 1.72], [4.96, -0.64], [-2.38, -5.64]],
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
                "goals": [[1.13, 1.01], [7.90, 4.48], [-4.97, -4.66], [2.08, -4.57]],
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

def tests_adult_10_child_90_test_case_100_stopped_high(tester):
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
                "goals": [[-0.41, -3.70], [7.71, 3.58], [0.38, 3.90], [-2.48, 2.96]],
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
                "init_x": -1.04,
                "init_y": -2.93,
                "init_a": -13.51,
                "velocity": 1.16,
                "goals": [[-0.41, -3.70], [7.79, -2.15], [7.45, -0.27], [-7.82, 3.51]],
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
                "goals": [[5.70, 0.93], [-3.46, 2.01], [-2.66, 1.33], [-3.46, -3.61]],
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
                "init_x": 5.22,
                "init_y": 0.05,
                "init_a": -126.29,
                "velocity": 0.96,
                "goals": [[5.70, 0.93], [-2.61, -1.30], [-1.64, -4.40], [7.07, -3.80]],
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
                "init_x": -3.19,
                "init_y": -5.62,
                "init_a": 116.97,
                "velocity": 0.92,
                "goals": [[6.21, 4.51], [-1.24, -4.48], [4.89, -2.74], [1.40, 0.91]],
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
                "init_x": 3.14,
                "init_y": 3.78,
                "init_a": 161.95,
                "velocity": 1.07,
                "goals": [[7.95, -5.10], [-1.97, 2.50], [0.21, 2.02], [-6.80, -4.34]],
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
                "init_x": 5.09,
                "init_y": 2.93,
                "init_a": 84.73,
                "velocity": 1.13,
                "goal_x": -5.01,
                "goal_y": 1.58,
                "n_actors": 8,
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
                "goals": [[0.60, -4.75], [-4.23, 2.54], [-3.72, -3.01], [-4.81, -2.52]],
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
