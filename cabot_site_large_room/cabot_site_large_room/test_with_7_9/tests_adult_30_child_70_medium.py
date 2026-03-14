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

def tests_adult_30_child_70_test_case_01_walking_medium(tester):
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
                "goals": [[1.33, -4.88], [4.95, 5.13], [-1.75, -4.26], [-0.99, 5.44]],
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
                "init_x": 7.39,
                "init_y": -4.30,
                "init_a": -80.12,
                "velocity": 0.85,
                "goals": [[1.33, -4.88], [-6.20, 0.30], [3.91, -5.63], [-1.30, 2.47]],
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
                "goals": [[2.39, -5.04], [5.16, 5.94], [-2.94, 4.48], [1.76, -4.89]],
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
                "init_x": 5.72,
                "init_y": 3.87,
                "init_a": 157.68,
                "velocity": 1.06,
                "goals": [[3.06, 1.47], [-3.75, 0.70], [7.80, 1.98], [-2.81, 3.54]],
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
                "goals": [[-0.07, 0.32], [-3.11, -2.67], [-3.51, -1.92], [-0.83, 2.73]],
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
                "init_x": 4.83,
                "init_y": -3.85,
                "init_a": -164.71,
                "velocity": 0.84,
                "goals": [[4.23, -5.21], [1.25, 5.53], [2.83, 0.45], [7.56, 2.03]],
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

def tests_adult_30_child_70_test_case_02_stopped_medium(tester):
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
                "goals": [[-2.20, -5.37], [6.06, 4.32], [3.20, 3.62], [-7.27, -2.87]],
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
                "init_x": -3.03,
                "init_y": -5.92,
                "init_a": 54.41,
                "velocity": 1.18,
                "goals": [[-2.20, -5.37], [0.26, 2.33], [3.50, 0.34], [6.85, -4.86]],
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
                "goals": [[1.10, 2.95], [6.90, -1.73], [-0.56, 0.20], [3.76, -0.56]],
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
                "init_x": -5.57,
                "init_y": 1.70,
                "init_a": -149.25,
                "velocity": 1.17,
                "goals": [[-1.03, 1.93], [-7.99, 3.18], [3.81, 3.94], [-5.43, 1.43]],
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
                "init_x": -2.35,
                "init_y": -1.49,
                "init_a": 152.80,
                "velocity": 0.99,
                "goals": [[-3.93, 2.51], [7.58, 4.29], [1.61, -0.43], [2.00, -1.43]],
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
                "goals": [[5.16, 5.81], [-1.87, -3.31], [0.08, -4.58], [-5.73, -5.82]],
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

def tests_adult_30_child_70_test_case_03_stopped_medium(tester):
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
                "goals": [[-5.67, 3.62], [-1.86, 2.93], [-4.47, 0.21], [-4.90, 4.52]],
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
                "init_x": -4.82,
                "init_y": 3.09,
                "init_a": 170.56,
                "velocity": 0.94,
                "goals": [[-5.67, 3.62], [-5.97, 0.01], [2.49, 3.90], [1.51, -1.64]],
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
                "goals": [[4.60, 0.13], [-2.30, 4.78], [-7.04, -1.30], [-3.90, 5.70]],
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
                "init_x": -0.99,
                "init_y": 4.32,
                "init_a": 98.92,
                "velocity": 1.19,
                "goals": [[-3.83, 4.86], [4.19, 1.92], [5.84, 2.43], [-6.79, 4.92]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.82,
                "init_y": -1.69,
                "init_a": 83.20,
                "velocity": 0.95,
                "goals": [[1.67, 2.40], [5.68, -2.85], [-7.01, -5.07], [3.90, -0.78]],
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

def tests_adult_30_child_70_test_case_04_walking_medium(tester):
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
                "goals": [[3.93, -5.23], [2.16, 5.85], [-3.87, -1.88], [3.29, -0.95]],
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
                "goals": [[-3.28, -5.61], [-2.75, 3.16], [-6.68, -3.74], [-6.81, -1.37]],
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
                "init_x": -1.27,
                "init_y": -5.32,
                "init_a": 178.16,
                "velocity": 0.92,
                "goals": [[-3.28, -5.61], [-3.24, -0.54], [-1.74, -0.85], [6.18, -3.14]],
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
                "init_x": 4.85,
                "init_y": 0.64,
                "init_a": -141.28,
                "velocity": 0.81,
                "goals": [[-6.62, -3.02], [-4.53, -4.14], [7.35, -3.94], [6.65, -4.22]],
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
                "init_x": 5.34,
                "init_y": 4.91,
                "init_a": 43.26,
                "velocity": 0.81,
                "goals": [[-1.59, 5.37], [7.56, 5.35], [-7.32, 0.02], [-3.71, -1.83]],
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
                "goals": [[5.78, 4.92], [4.68, -3.05], [-7.27, -0.24], [-1.34, -0.91]],
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

def tests_adult_30_child_70_test_case_05_walking_medium(tester):
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
                "goals": [[-0.81, -2.77], [-2.01, -0.05], [-7.96, 1.23], [-7.19, -2.41]],
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
                "goals": [[-2.10, 1.87], [0.31, 1.82], [1.61, -2.85], [-5.49, 5.30]],
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
                "goals": [[2.12, 4.59], [-7.32, -2.86], [-2.11, -1.59], [-7.10, 3.79]],
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
                "goals": [[-2.98, 5.35], [0.15, -4.78], [3.22, -2.47], [-6.39, -0.07]],
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
                "goals": [[-0.44, 1.54], [-7.62, -4.95], [-0.58, 5.83], [5.14, 2.93]],
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
                "goals": [[0.77, -2.60], [-1.42, -2.87], [1.08, 2.60], [0.94, -3.56]],
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

def tests_adult_30_child_70_test_case_06_stopped_medium(tester):
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
                "goals": [[-7.86, 4.64], [-7.35, -5.38], [3.90, 4.99], [1.93, 1.11]],
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
                "init_x": -8.51,
                "init_y": 3.87,
                "init_a": 9.91,
                "velocity": 0.98,
                "goals": [[-7.86, 4.64], [-4.91, 2.61], [-2.99, -3.89], [0.40, 0.42]],
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
                "goals": [[5.29, -1.92], [2.40, 4.45], [6.10, 4.00], [7.18, 1.06]],
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
                "goals": [[3.97, 2.26], [3.91, 4.40], [1.80, -1.03], [0.60, -5.98]],
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
                "init_x": -7.47,
                "init_y": -3.98,
                "init_a": -140.24,
                "velocity": 1.15,
                "goals": [[1.27, -5.58], [-3.82, -1.73], [6.30, -1.77], [4.69, 4.00]],
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

def tests_adult_30_child_70_test_case_07_walking_medium(tester):
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
                "goals": [[5.00, 4.61], [-2.83, -3.45], [4.50, 4.48], [-1.49, -1.62]],
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
                "goals": [[4.51, -1.38], [-4.23, -1.63], [4.36, 3.90], [-5.20, -3.96]],
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
                "goals": [[-5.39, 1.51], [7.40, 4.24], [-6.54, -0.81], [-2.46, 0.06]],
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
                "goals": [[-4.87, -0.78], [-5.18, 0.54], [-2.39, 5.58], [-7.93, 5.90]],
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
                "goals": [[-6.89, 1.07], [2.29, 3.44], [6.71, -5.26], [-1.15, -2.89]],
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
                "goals": [[1.77, -2.67], [7.81, 3.20], [5.87, -3.97], [-2.97, -5.58]],
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

def tests_adult_30_child_70_test_case_08_walking_medium(tester):
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
                "goals": [[-4.33, 1.17], [5.44, -1.67], [0.07, -5.41], [-2.86, 0.45]],
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
                "init_x": -0.80,
                "init_y": -0.57,
                "init_a": 159.35,
                "velocity": 1.06,
                "goals": [[-4.33, 1.17], [-7.41, -1.60], [-5.47, -2.65], [-5.00, 3.89]],
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
                "goals": [[-3.67, -5.49], [2.69, 0.08], [-2.16, -1.71], [5.29, 2.22]],
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
                "init_x": 7.26,
                "init_y": 1.75,
                "init_a": 14.41,
                "velocity": 0.88,
                "goals": [[-2.16, -0.09], [0.28, 4.82], [-0.25, -2.93], [6.60, -4.68]],
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
                "goals": [[5.03, -0.86], [4.63, -5.85], [-4.84, -4.79], [2.97, 2.34]],
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
                "goals": [[7.74, 5.10], [3.45, -2.80], [7.59, 3.65], [-3.59, -4.78]],
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

def tests_adult_30_child_70_test_case_09_walking_medium(tester):
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
                "goals": [[7.80, -5.23], [-5.34, -0.45], [4.08, -2.04], [7.06, -0.01]],
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
                "init_x": 1.08,
                "init_y": -1.33,
                "init_a": 84.23,
                "velocity": 1.03,
                "goals": [[7.80, -5.23], [3.22, -4.45], [-7.12, 5.17], [-6.97, 4.94]],
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
                "goals": [[-6.60, -5.68], [6.86, 5.65], [-4.61, -1.59], [-6.86, -0.40]],
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
                "goals": [[-0.68, 0.10], [1.31, -5.45], [-3.42, 4.86], [7.46, -0.65]],
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
                "init_x": 7.74,
                "init_y": 5.33,
                "init_a": -174.77,
                "velocity": 0.99,
                "goals": [[-0.67, 2.02], [7.11, 5.94], [3.64, 0.96], [-7.24, -2.81]],
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

def tests_adult_30_child_70_test_case_10_stopped_medium(tester):
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
                "goals": [[5.20, -4.57], [4.97, -0.07], [-5.01, 1.39], [7.38, -4.60]],
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
                "goals": [[-5.86, 5.62], [6.37, -1.17], [3.87, 5.92], [0.68, 3.88]],
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
                "goals": [[-5.86, 5.62], [5.60, -1.25], [4.54, -3.34], [-0.61, 1.77]],
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
                "goals": [[2.28, 0.07], [0.59, -0.35], [1.38, 5.93], [7.77, -1.19]],
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
                "goals": [[-0.85, -2.97], [7.03, 2.21], [2.85, 4.33], [2.79, 4.01]],
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

def tests_adult_30_child_70_test_case_11_walking_medium(tester):
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
                "goals": [[3.12, 4.98], [7.69, 5.58], [-4.60, -5.58], [-4.94, -4.28]],
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
                "goals": [[3.12, 4.98], [7.07, 0.41], [5.79, 4.80], [-2.62, -5.08]],
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
                "goals": [[5.16, -3.74], [-3.37, 4.12], [0.43, 2.66], [5.78, 0.76]],
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
                "goals": [[3.05, -0.44], [-2.80, 2.18], [6.65, 4.88], [-7.05, -3.34]],
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
                "goals": [[7.17, 4.38], [-5.86, 4.46], [-2.26, 3.18], [-3.55, 5.07]],
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

def tests_adult_30_child_70_test_case_12_walking_medium(tester):
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
                "goals": [[2.00, 5.30], [-2.31, 3.24], [7.66, 4.69], [-5.71, 2.24]],
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
                "goals": [[2.00, 5.30], [1.03, -1.56], [3.10, 4.11], [-0.70, 5.30]],
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
                "goals": [[0.82, 5.85], [1.27, 0.54], [4.95, -1.76], [6.91, -5.41]],
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
                "goals": [[7.95, 3.92], [-1.64, 3.23], [-4.30, -1.87], [4.50, -4.41]],
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
                "goals": [[5.04, -0.76], [-7.83, 0.02], [-5.20, -3.96], [7.59, 3.33]],
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

def tests_adult_30_child_70_test_case_13_stopped_medium(tester):
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
                "goals": [[-5.18, -3.02], [2.40, 2.46], [-1.25, 4.65], [4.16, -3.51]],
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
                "goals": [[-5.18, -3.02], [1.40, 3.08], [-0.03, -4.79], [6.19, -2.59]],
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
                "goals": [[2.06, 1.40], [-2.98, 2.06], [4.79, 0.61], [-0.26, 0.76]],
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
                "goals": [[-5.29, -3.55], [3.72, -5.78], [-5.14, 0.58], [-3.94, 1.32]],
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
                "goals": [[6.06, 2.43], [-7.68, 0.67], [-4.93, 3.66], [-4.93, 3.74]],
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
                "goals": [[4.85, 3.54], [-5.38, 2.87], [2.54, -3.34], [5.89, 1.68]],
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

def tests_adult_30_child_70_test_case_14_stopped_medium(tester):
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
                "goals": [[-3.73, -3.35], [2.56, -0.05], [4.20, -5.54], [0.18, -2.31]],
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
                "goals": [[-6.95, 1.56], [6.24, -4.47], [7.12, -1.41], [5.08, 0.27]],
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
                "goals": [[-6.95, 1.56], [-3.02, 3.80], [-7.34, 0.43], [-7.96, -3.14]],
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
                "goals": [[-0.21, -3.28], [4.47, -1.73], [6.50, -2.63], [1.43, -0.37]],
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
                "goals": [[-1.23, -4.95], [2.17, -1.73], [3.22, 4.70], [-2.43, -2.59]],
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
                "goals": [[6.14, -4.00], [4.97, 0.71], [1.05, -0.48], [-7.81, 3.11]],
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

def tests_adult_30_child_70_test_case_15_walking_medium(tester):
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
                "goals": [[5.44, 1.84], [2.87, 1.14], [-3.05, -2.85], [-0.77, 2.73]],
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
                "goals": [[5.44, 1.84], [-5.80, 2.32], [-2.14, -5.18], [4.51, -3.73]],
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
                "goals": [[-4.50, -1.26], [0.05, 4.09], [5.63, 4.54], [6.12, -1.19]],
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
                "goals": [[-4.50, -1.26], [-7.93, -3.51], [-6.90, -1.40], [-6.58, 0.52]],
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
                "goals": [[-1.51, 1.04], [-3.10, 0.92], [4.47, 4.26], [4.28, 2.75]],
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
                "goals": [[3.89, 6.00], [-4.57, -5.42], [0.89, 5.31], [0.33, 0.42]],
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

def tests_adult_30_child_70_test_case_16_stopped_medium(tester):
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
                "goals": [[-6.55, -5.05], [-0.01, 4.06], [-4.11, 5.74], [-0.60, -2.66]],
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
                "goals": [[-6.55, -5.05], [1.30, -5.36], [-5.04, -3.13], [7.79, -4.05]],
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
                "goals": [[1.69, 0.82], [-0.07, 5.95], [-7.86, -2.51], [4.26, 0.02]],
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
                "goals": [[1.69, 0.82], [-7.88, -0.06], [7.69, 5.69], [-0.71, 4.96]],
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
                "goals": [[2.95, -1.66], [-5.36, -1.65], [3.94, 0.49], [5.40, 2.81]],
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
                "goals": [[-5.43, 0.96], [-1.09, 1.26], [1.82, 0.84], [-4.98, -3.54]],
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

def tests_adult_30_child_70_test_case_17_walking_medium(tester):
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
                "goals": [[-6.97, -2.49], [5.98, 1.98], [3.21, 2.90], [1.28, 2.37]],
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
                "goals": [[-2.68, -5.19], [5.02, -3.60], [-5.45, -2.70], [3.43, -5.52]],
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
                "goals": [[-2.68, -5.19], [5.86, 5.63], [5.43, -2.13], [-0.33, -2.09]],
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
                "goals": [[3.73, 2.84], [-2.70, 5.03], [0.95, -3.99], [1.95, 4.83]],
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
                "goals": [[-0.72, 3.98], [7.69, -0.14], [5.60, -0.81], [-5.06, 1.76]],
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

def tests_adult_30_child_70_test_case_18_walking_medium(tester):
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
                "goals": [[-2.54, -1.40], [-1.60, 0.59], [4.50, 5.30], [4.31, 2.22]],
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
                "init_x": -4.92,
                "init_y": -2.81,
                "init_a": -162.69,
                "velocity": 1.07,
                "goals": [[-2.54, -1.40], [1.57, -1.64], [-2.58, 3.63], [-5.66, 3.03]],
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
                "goals": [[4.80, -0.27], [-5.24, -4.92], [-1.91, -3.90], [-6.59, 1.41]],
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
                "init_x": -1.26,
                "init_y": -2.69,
                "init_a": 179.74,
                "velocity": 0.89,
                "goals": [[7.37, -5.06], [-0.49, -0.29], [1.68, -0.61], [-6.89, 5.62]],
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

def tests_adult_30_child_70_test_case_19_stopped_medium(tester):
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
                "goals": [[3.29, -3.00], [4.11, -0.75], [4.62, 4.09], [-7.23, 5.56]],
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
                "goals": [[-1.04, -5.18], [-2.63, -0.05], [-2.03, 5.04], [5.51, -2.26]],
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
                "goals": [[-1.04, -5.18], [5.71, -5.59], [-4.77, -1.17], [3.06, -3.55]],
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
                "goals": [[6.62, 1.26], [6.91, 2.93], [7.59, -1.04], [-6.81, -5.78]],
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

def tests_adult_30_child_70_test_case_20_walking_medium(tester):
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
                "goals": [[-0.91, -2.09], [-5.29, 1.54], [-5.55, 1.47], [-6.70, -4.89]],
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
                "init_x": -1.75,
                "init_y": 0.10,
                "init_a": 20.01,
                "velocity": 1.19,
                "goals": [[-0.91, -2.09], [4.08, 0.47], [0.72, -3.49], [6.09, 4.15]],
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
                "init_x": 6.30,
                "init_y": -1.36,
                "init_a": 114.84,
                "velocity": 0.83,
                "goals": [[7.52, 4.59], [-4.27, 2.27], [4.38, -2.47], [5.74, 2.84]],
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
                "goals": [[3.08, -4.86], [5.55, 1.16], [-0.87, 0.91], [-2.24, 5.52]],
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
                "init_x": 4.20,
                "init_y": 5.90,
                "init_a": 89.15,
                "velocity": 0.96,
                "goals": [[3.43, -2.21], [5.61, -0.81], [3.19, 5.84], [-5.47, 1.86]],
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

def tests_adult_30_child_70_test_case_21_stopped_medium(tester):
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
                "goals": [[4.30, 5.63], [2.51, 5.93], [2.45, -5.40], [7.70, -3.18]],
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
                "goals": [[6.83, -5.95], [6.04, 1.38], [-3.63, -2.77], [3.38, -5.64]],
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
                "goals": [[-0.53, 1.73], [-1.77, 4.12], [3.33, -1.84], [1.20, -2.91]],
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
                "goals": [[-5.95, 4.01], [7.21, -1.95], [-7.05, 3.13], [3.25, 2.05]],
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
                "goals": [[-3.26, -5.17], [0.88, 4.48], [2.52, 1.49], [-6.93, 0.60]],
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

def tests_adult_30_child_70_test_case_22_walking_medium(tester):
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
                "goals": [[-0.12, -5.67], [-0.73, -4.65], [-4.11, -2.37], [3.01, 0.78]],
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
                "goals": [[-1.87, 1.12], [-1.16, 4.51], [3.67, -2.38], [-1.82, 2.52]],
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
                "goals": [[3.01, -0.47], [-5.97, 3.62], [-1.05, -4.24], [7.52, 3.38]],
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
                "goals": [[6.26, -3.32], [5.38, 1.43], [0.29, 4.79], [-6.22, -0.59]],
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

def tests_adult_30_child_70_test_case_23_walking_medium(tester):
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
                "goals": [[-2.58, 4.91], [7.46, -1.71], [6.20, 3.02], [2.44, -0.80]],
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
                "goals": [[-2.58, 4.91], [6.14, 2.90], [5.19, 5.08], [-3.74, 0.54]],
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
                "goals": [[4.46, 0.65], [-0.88, 2.50], [-5.49, -2.49], [0.35, 0.41]],
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
                "goals": [[-6.87, -3.96], [1.51, 2.31], [-7.14, -4.22], [2.91, -1.55]],
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

def tests_adult_30_child_70_test_case_24_walking_medium(tester):
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
                "goals": [[3.12, 0.02], [-2.84, 5.92], [-3.32, -0.26], [-3.70, 0.65]],
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
                "goals": [[-1.54, -1.08], [4.42, -5.80], [2.11, -5.69], [4.60, 2.81]],
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
                "goals": [[6.34, 4.38], [2.92, 5.68], [-6.53, 5.94], [-1.51, -2.31]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.49,
                "init_y": -1.87,
                "init_a": -14.55,
                "velocity": 0.91,
                "goals": [[-1.22, -4.48], [-5.20, 2.23], [-2.61, 3.63], [4.41, 4.37]],
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
                "goals": [[0.34, 0.72], [4.11, 5.55], [-4.98, -0.31], [6.74, -5.21]],
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
                "goals": [[7.66, 1.79], [-1.32, 2.01], [-0.86, -1.73], [0.35, -5.37]],
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

def tests_adult_30_child_70_test_case_25_stopped_medium(tester):
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
                "goals": [[1.98, 0.61], [-7.45, -1.61], [7.31, -4.59], [-0.56, 0.91]],
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
                "goals": [[1.98, 0.61], [6.75, 1.90], [-7.75, -5.68], [2.69, -4.31]],
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
                "goals": [[-4.95, -2.61], [1.10, -1.20], [-5.44, -1.38], [-2.33, -5.21]],
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
                "goals": [[5.63, -3.67], [-5.58, 4.33], [7.42, -0.58], [2.74, 0.45]],
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
                "goals": [[1.13, -2.94], [-1.81, 0.60], [-7.38, -3.51], [0.73, -4.29]],
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
                "goals": [[-3.59, -5.29], [-3.90, -1.60], [-5.33, 1.69], [-3.32, -4.53]],
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

def tests_adult_30_child_70_test_case_26_stopped_medium(tester):
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
                "goals": [[3.65, 0.78], [-7.32, -1.05], [-2.46, 5.10], [-2.83, -4.28]],
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
                "init_x": 3.62,
                "init_y": -0.22,
                "init_a": -127.80,
                "velocity": 1.12,
                "goals": [[3.65, 0.78], [4.95, 5.27], [4.80, 0.11], [-6.69, 1.64]],
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
                "goals": [[6.90, 1.37], [-5.99, -2.05], [6.71, -4.72], [-0.70, 3.97]],
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
                "goals": [[0.69, -4.67], [-5.85, 4.35], [-2.04, 0.05], [5.21, -2.34]],
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
                "init_x": 2.31,
                "init_y": 1.46,
                "init_a": -126.73,
                "velocity": 0.81,
                "goals": [[0.40, 0.88], [4.86, 3.18], [2.67, -4.60], [-0.55, -0.43]],
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
                "init_x": -0.05,
                "init_y": 4.31,
                "init_a": 163.06,
                "velocity": 0.93,
                "goals": [[5.92, 2.13], [-5.58, -0.05], [2.08, -4.90], [4.57, -0.04]],
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

def tests_adult_30_child_70_test_case_27_stopped_medium(tester):
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
                "goals": [[4.96, -5.82], [3.79, 2.56], [-0.70, 5.36], [7.68, -0.36]],
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
                "goals": [[-2.25, -3.99], [-6.76, 0.36], [-6.51, -2.51], [-4.68, -1.97]],
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
                "init_x": 2.83,
                "init_y": 5.30,
                "init_a": 129.63,
                "velocity": 1.06,
                "goals": [[0.21, 0.12], [3.05, -2.54], [7.83, 4.84], [-2.85, -2.02]],
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
                "goals": [[-3.35, 3.10], [7.21, -0.95], [7.70, 4.72], [3.73, 5.12]],
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
                "init_x": -0.36,
                "init_y": -3.13,
                "init_a": 55.77,
                "velocity": 1.02,
                "goals": [[-0.03, -4.83], [2.57, 1.11], [4.96, -4.52], [7.66, -1.25]],
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
                "init_x": 0.49,
                "init_y": 5.94,
                "init_a": 6.01,
                "velocity": 1.20,
                "goals": [[0.24, 0.48], [-7.06, 0.08], [-1.38, 1.77], [7.97, 2.15]],
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

def tests_adult_30_child_70_test_case_28_stopped_medium(tester):
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
                "goals": [[-6.29, -4.87], [-7.02, 2.69], [6.44, 1.49], [2.87, -3.52]],
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
                "init_x": -6.65,
                "init_y": -3.93,
                "init_a": 148.49,
                "velocity": 0.90,
                "goals": [[-6.29, -4.87], [4.22, -1.56], [-0.63, 0.26], [-3.35, 3.71]],
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
                "init_x": 5.85,
                "init_y": 0.21,
                "init_a": -17.76,
                "velocity": 1.06,
                "goals": [[5.85, 0.21], [2.59, -1.28], [-3.41, 2.05], [7.19, -1.89]],
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
                "goals": [[1.09, 5.72], [1.38, -1.99], [2.02, -2.00], [1.22, -1.64]],
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
                "goals": [[4.21, 1.28], [-6.81, -5.58], [3.28, 3.60], [2.04, -2.53]],
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
                "goals": [[-0.23, 0.89], [4.75, -1.97], [3.95, -2.34], [0.36, 3.99]],
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

def tests_adult_30_child_70_test_case_29_stopped_medium(tester):
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
                "goals": [[-7.23, 0.12], [-5.16, 5.93], [5.48, -5.45], [7.37, 3.46]],
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
                "goals": [[-1.94, -4.89], [-2.78, 0.77], [-3.52, 4.89], [6.86, -2.63]],
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
                "init_x": 1.53,
                "init_y": -1.32,
                "init_a": -158.25,
                "velocity": 1.07,
                "goals": [[3.86, -1.31], [-5.79, 5.93], [-5.40, 3.86], [-3.13, -0.24]],
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
                "init_x": 1.19,
                "init_y": -2.66,
                "init_a": 167.75,
                "velocity": 0.84,
                "goals": [[-2.81, -4.86], [6.49, -0.11], [-6.73, 5.24], [-1.43, 3.04]],
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
                "init_x": -3.91,
                "init_y": -5.30,
                "init_a": -81.55,
                "velocity": 1.06,
                "goals": [[-1.04, -1.19], [-6.86, 3.95], [-3.97, -1.64], [2.28, 0.13]],
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

def tests_adult_30_child_70_test_case_30_walking_medium(tester):
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
                "goals": [[-5.84, -5.51], [-1.94, 1.93], [-5.79, 2.71], [2.49, -4.65]],
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
                "init_x": -4.15,
                "init_y": -1.90,
                "init_a": -14.21,
                "velocity": 1.10,
                "goals": [[-5.84, -5.51], [-4.95, -2.00], [-1.17, -1.58], [-0.37, 1.14]],
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
                "init_x": -2.92,
                "init_y": 4.71,
                "init_a": -109.88,
                "velocity": 0.83,
                "goals": [[-3.80, 0.36], [-7.17, -2.16], [4.07, 5.35], [-2.50, -3.48]],
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
                "goals": [[2.42, 4.06], [6.58, 1.99], [4.01, 3.10], [1.39, 0.19]],
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

def tests_adult_30_child_70_test_case_31_stopped_medium(tester):
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
                "goals": [[7.04, -2.91], [6.12, -1.04], [0.94, 5.71], [-7.24, 1.30]],
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
                "goals": [[1.59, -2.71], [6.41, 3.47], [-6.71, 5.33], [4.25, -1.35]],
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
                "goals": [[1.59, -2.71], [-5.68, 1.76], [-2.24, 4.74], [-1.42, 2.59]],
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
                "goals": [[3.23, 2.89], [2.07, -2.37], [-1.63, -4.16], [-1.30, -0.07]],
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

def tests_adult_30_child_70_test_case_32_walking_medium(tester):
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
                "goals": [[-0.98, 0.29], [3.73, -1.17], [5.15, -0.66], [4.70, 4.61]],
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
                "init_x": 1.52,
                "init_y": -1.70,
                "init_a": 76.89,
                "velocity": 1.18,
                "goals": [[-0.98, 0.29], [3.56, 4.75], [4.10, 3.88], [6.01, -4.94]],
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
                "goals": [[5.31, -0.65], [-5.38, 3.95], [7.95, -3.87], [1.62, -3.35]],
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
                "init_x": 2.20,
                "init_y": 4.35,
                "init_a": 128.58,
                "velocity": 0.83,
                "goals": [[-1.27, 5.80], [6.30, 0.79], [2.67, 4.11], [4.45, 5.81]],
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
                "init_x": 1.48,
                "init_y": -4.41,
                "init_a": 59.41,
                "velocity": 1.12,
                "goals": [[-5.68, -4.75], [-0.35, 4.86], [-7.11, -0.61], [-4.10, 1.30]],
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
                "goals": [[-5.75, 0.95], [0.15, 5.42], [-4.82, 0.71], [-1.45, -0.95]],
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

def tests_adult_30_child_70_test_case_33_stopped_medium(tester):
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
                "goals": [[5.78, 4.94], [-3.84, 4.09], [2.17, 4.47], [5.35, -0.42]],
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
                "goals": [[6.19, 4.16], [0.57, -0.69], [-2.35, 2.09], [-7.89, -2.86]],
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
                "goals": [[5.30, -5.56], [-1.49, 0.50], [5.72, -2.58], [6.21, -1.86]],
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
                "goals": [[-5.18, 5.63], [-5.30, 5.80], [7.47, -1.95], [-6.86, -2.52]],
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

def tests_adult_30_child_70_test_case_34_stopped_medium(tester):
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
                "goals": [[7.29, 0.59], [-1.40, 3.84], [6.27, 0.95], [-6.86, -2.56]],
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
                "init_x": 8.22,
                "init_y": 0.24,
                "init_a": 37.87,
                "velocity": 0.93,
                "goals": [[7.29, 0.59], [-2.03, -2.59], [3.52, 3.57], [4.57, -5.86]],
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
                "init_x": -0.07,
                "init_y": 0.15,
                "init_a": 51.97,
                "velocity": 0.85,
                "goals": [[-0.07, 0.15], [2.85, 4.66], [2.04, -5.86], [-6.82, -5.87]],
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
                "goals": [[5.80, -1.24], [3.72, -0.99], [-7.81, -4.17], [-1.38, -0.67]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.74,
                "init_y": -4.19,
                "init_a": 137.42,
                "velocity": 1.20,
                "goals": [[0.79, -2.01], [0.27, 4.71], [0.34, 2.64], [-3.82, -2.18]],
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

def tests_adult_30_child_70_test_case_35_walking_medium(tester):
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
                "goals": [[1.46, -2.66], [-2.00, 2.50], [7.88, -4.59], [7.88, 3.79]],
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
                "goals": [[5.96, -5.59], [-7.07, -4.25], [-0.71, -4.77], [-7.34, 1.18]],
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
                "init_x": -6.03,
                "init_y": -1.11,
                "init_a": 3.93,
                "velocity": 1.01,
                "goals": [[5.96, -5.59], [-6.77, -3.30], [-3.25, 1.35], [7.40, 2.35]],
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
                "goals": [[-1.21, 2.63], [-6.90, -5.42], [-2.13, 4.58], [5.94, -4.80]],
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
                "goals": [[-6.34, 2.17], [-1.51, 5.17], [-0.03, 4.31], [-2.36, 1.74]],
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

def tests_adult_30_child_70_test_case_36_stopped_medium(tester):
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
                "goals": [[-3.70, -3.49], [4.17, -1.08], [-5.27, 3.64], [-7.30, 1.29]],
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
                "goals": [[-4.13, 5.07], [3.47, 0.97], [4.28, -2.86], [-7.40, 0.39]],
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
                "goals": [[-4.13, 5.07], [0.91, 3.56], [7.54, -4.91], [2.98, -3.38]],
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
                "goals": [[-0.60, 3.76], [-2.01, -0.71], [-0.04, 0.75], [-1.20, 5.94]],
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

def tests_adult_30_child_70_test_case_37_stopped_medium(tester):
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
                "goals": [[-5.02, 0.90], [-2.27, -2.87], [-0.82, -3.45], [-1.72, 0.78]],
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
                "goals": [[-7.28, 2.94], [4.73, -1.76], [0.99, -2.66], [4.96, 1.47]],
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
                "goals": [[-6.22, -1.79], [4.00, -5.22], [-0.65, 0.67], [4.49, 1.14]],
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
                "goals": [[-2.91, 4.58], [3.80, -3.60], [4.94, -0.49], [-3.26, 0.23]],
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

def tests_adult_30_child_70_test_case_38_stopped_medium(tester):
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
                "goals": [[4.49, 2.40], [5.83, 0.40], [-5.58, -0.80], [-1.48, -5.36]],
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
                "goals": [[5.17, 1.61], [5.54, 2.31], [-6.23, 1.64], [-2.86, -3.32]],
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
                "goals": [[2.19, -4.41], [3.76, -2.72], [-1.42, 2.27], [-3.66, -5.95]],
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
                "goals": [[1.65, 3.36], [1.73, 0.80], [-0.48, 1.27], [-5.52, 2.87]],
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
                "goals": [[-1.23, -5.42], [1.79, -1.92], [2.20, 4.07], [1.92, 0.22]],
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
                "goals": [[6.65, -3.74], [-1.67, -2.31], [-0.51, -3.70], [6.92, 5.39]],
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

def tests_adult_30_child_70_test_case_39_walking_medium(tester):
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
                "goals": [[-1.97, 5.75], [-7.68, 0.81], [-3.21, -1.62], [-0.19, 1.12]],
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
                "init_x": -5.21,
                "init_y": -2.70,
                "init_a": -116.94,
                "velocity": 0.91,
                "goals": [[-1.97, 5.75], [-3.70, 5.33], [4.79, 3.63], [-1.10, 5.45]],
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
                "goals": [[-0.69, 0.71], [3.69, 4.38], [6.57, 4.97], [-2.78, 2.72]],
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
                "init_x": 7.93,
                "init_y": 3.39,
                "init_a": 70.42,
                "velocity": 0.94,
                "goals": [[5.51, 0.12], [6.23, 3.09], [-5.72, -2.56], [-0.59, 0.02]],
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
                "goals": [[-0.11, -0.10], [-6.17, 3.19], [1.29, -5.38], [7.48, -4.28]],
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

def tests_adult_30_child_70_test_case_40_walking_medium(tester):
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
                "goals": [[4.02, -2.08], [6.34, 3.60], [-1.91, 0.19], [-7.22, 0.54]],
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
                "goals": [[1.54, -0.24], [-1.91, 3.99], [2.49, -4.96], [-2.06, 2.69]],
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
                "goals": [[-3.55, -0.33], [6.53, -5.35], [-4.08, -5.55], [2.01, 2.28]],
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
                "init_y": 5.94,
                "init_a": -36.94,
                "velocity": 0.98,
                "goals": [[-0.57, -3.39], [7.10, -3.85], [6.30, -4.35], [7.96, 2.04]],
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
                "goals": [[6.65, 4.79], [-3.04, 1.23], [-1.26, -2.00], [4.22, 4.08]],
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

def tests_adult_30_child_70_test_case_41_stopped_medium(tester):
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
                "goals": [[-1.93, 2.39], [3.03, -0.71], [-4.47, -0.93], [0.91, 2.14]],
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
                "goals": [[4.94, 5.24], [6.88, -3.67], [1.65, -3.60], [-0.31, 3.78]],
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
                "init_x": 4.66,
                "init_y": -3.23,
                "init_a": 128.60,
                "velocity": 1.03,
                "goals": [[1.37, -0.70], [6.19, -5.85], [7.49, 5.36], [2.41, -3.85]],
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
                "init_x": 7.43,
                "init_y": -3.08,
                "init_a": -131.12,
                "velocity": 1.13,
                "goals": [[3.58, 2.53], [6.06, 0.03], [5.31, 1.30], [6.05, 0.43]],
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

def tests_adult_30_child_70_test_case_42_stopped_medium(tester):
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
                "goals": [[-6.12, 1.34], [-5.53, 0.56], [3.00, -2.98], [-6.71, 3.77]],
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
                "goals": [[5.35, 1.82], [0.15, -3.72], [7.71, 0.93], [2.88, 3.70]],
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
                "goals": [[1.85, -0.85], [1.74, 4.37], [3.00, -4.70], [-3.00, -3.08]],
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
                "init_x": 3.18,
                "init_y": -2.71,
                "init_a": -90.97,
                "velocity": 0.81,
                "goals": [[-6.03, -2.63], [0.93, 0.65], [-5.33, 1.96], [1.66, 2.57]],
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
                "goals": [[0.93, -2.93], [-0.73, 5.67], [-1.69, 0.60], [-5.68, -0.77]],
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
                "init_x": 7.78,
                "init_y": 1.87,
                "init_a": -25.37,
                "velocity": 0.99,
                "goals": [[-0.77, 2.38], [-0.69, 1.74], [-3.45, 1.05], [5.01, -3.12]],
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

def tests_adult_30_child_70_test_case_43_stopped_medium(tester):
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
                "goals": [[6.02, 3.19], [7.85, 4.19], [2.44, -5.15], [-1.44, 3.34]],
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
                "goals": [[0.74, 5.47], [-3.47, 3.58], [-3.81, -2.65], [-6.48, 1.08]],
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
                "goals": [[4.02, 1.16], [1.24, 2.16], [6.55, -4.06], [2.94, 1.10]],
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
                "goals": [[4.59, 0.20], [-4.70, 2.38], [3.05, 1.74], [-5.80, 5.92]],
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
                "goals": [[7.41, -0.38], [7.92, 3.22], [-1.63, -3.68], [6.45, 2.22]],
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
                "goals": [[-7.31, -3.40], [2.42, 2.80], [-5.91, 0.43], [-1.10, -0.08]],
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

def tests_adult_30_child_70_test_case_44_stopped_medium(tester):
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
                "goals": [[-2.76, -1.11], [5.50, 2.81], [4.97, -0.66], [-2.40, 3.02]],
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
                "goals": [[7.80, 3.42], [-1.00, -1.43], [-4.33, -2.81], [7.22, -3.51]],
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
                "goals": [[7.80, 3.42], [-0.95, 5.70], [-1.77, 5.10], [6.27, -0.68]],
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
                "goals": [[-3.59, -0.02], [-0.70, -1.84], [6.31, 5.30], [-1.09, -5.72]],
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
                "goals": [[-7.51, 0.54], [1.22, 2.05], [-0.06, 5.07], [-5.79, -1.36]],
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

def tests_adult_30_child_70_test_case_45_walking_medium(tester):
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
                "goals": [[6.75, 0.40], [-3.45, 5.11], [2.80, -4.21], [1.84, -2.61]],
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
                "init_x": -3.58,
                "init_y": 0.63,
                "init_a": -133.51,
                "velocity": 1.17,
                "goals": [[6.75, 0.40], [1.33, -3.27], [-3.55, -2.63], [4.14, -3.12]],
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
                "init_x": 5.91,
                "init_y": 2.01,
                "init_a": -61.08,
                "velocity": 1.00,
                "goals": [[1.43, 1.81], [-3.90, 0.63], [-0.08, 4.67], [7.12, 0.34]],
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
                "init_x": 6.32,
                "init_y": 1.10,
                "init_a": -61.08,
                "velocity": 1.00,
                "goals": [[1.43, 1.81], [5.80, 5.49], [0.67, 1.63], [5.45, -5.47]],
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
                "goals": [[6.35, -5.18], [-6.39, -3.63], [-2.66, 3.53], [-6.98, -2.66]],
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

def tests_adult_30_child_70_test_case_46_walking_medium(tester):
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
                "goals": [[-3.89, -3.62], [1.50, -3.88], [-2.19, 4.18], [-3.72, 1.69]],
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
                "goals": [[7.28, -3.23], [0.28, -5.87], [3.60, -5.32], [6.55, -0.51]],
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
                "goals": [[7.74, 2.67], [-3.44, -5.45], [2.01, -2.45], [-6.63, 1.70]],
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
                "goals": [[-1.70, -4.43], [-0.11, -4.10], [5.79, -3.45], [6.73, 5.95]],
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
                "goals": [[4.64, -1.43], [-2.37, 5.53], [0.62, -2.82], [-0.47, -4.34]],
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

def tests_adult_30_child_70_test_case_47_stopped_medium(tester):
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
                "goals": [[-0.67, 0.17], [-3.07, 4.09], [6.84, -3.48], [1.01, 5.61]],
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
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.45,
                "init_y": 5.92,
                "init_a": -61.07,
                "velocity": 1.16,
                "goals": [[-7.45, 5.92], [-0.18, -4.70], [-6.24, -5.45], [-2.72, -3.09]],
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
                "goals": [[7.93, 5.30], [-7.10, -3.95], [-2.29, -2.30], [-7.33, 4.98]],
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
                "init_x": 0.78,
                "init_y": 4.46,
                "init_a": -88.04,
                "velocity": 1.05,
                "goals": [[-4.74, 0.90], [-6.33, -5.95], [4.36, -0.46], [-7.78, -0.88]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.43,
                "init_y": 0.06,
                "init_a": 77.21,
                "velocity": 1.20,
                "goals": [[-0.80, -5.75], [-0.29, -1.53], [-1.85, -0.47], [7.30, -1.95]],
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

def tests_adult_30_child_70_test_case_48_stopped_medium(tester):
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
                "goals": [[7.78, 4.30], [7.59, 4.80], [-5.32, -3.07], [-2.56, -5.64]],
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
                "goals": [[3.34, 4.74], [-4.94, 3.28], [2.68, -4.67], [-2.07, 1.04]],
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
                "goals": [[-3.99, -5.37], [-1.47, -0.26], [7.70, -3.36], [6.70, 3.85]],
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
                "init_x": -4.36,
                "init_y": 3.63,
                "init_a": -52.24,
                "velocity": 1.09,
                "goals": [[7.42, -2.57], [-4.51, -0.10], [-1.72, 0.74], [6.46, 5.60]],
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
                "init_x": 5.19,
                "init_y": -0.21,
                "init_a": 93.32,
                "velocity": 0.88,
                "goals": [[-3.87, -5.18], [2.15, -5.51], [-2.20, 4.33], [-3.94, -5.27]],
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

def tests_adult_30_child_70_test_case_49_stopped_medium(tester):
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
                "goals": [[5.73, 1.50], [6.47, -2.73], [2.40, -0.37], [3.02, 3.75]],
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
                "goals": [[2.23, -5.14], [-7.84, -4.58], [1.03, -5.30], [-2.11, 3.55]],
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
                "init_x": -0.53,
                "init_y": 0.82,
                "init_a": -11.78,
                "velocity": 0.83,
                "goals": [[-1.01, -1.26], [-4.96, -5.80], [0.05, -0.19], [3.64, -2.94]],
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
                "init_x": 0.04,
                "init_y": -3.67,
                "init_a": -121.43,
                "velocity": 0.91,
                "goals": [[6.62, 3.52], [2.60, -2.92], [4.77, -3.44], [-2.09, -5.40]],
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

def tests_adult_30_child_70_test_case_50_walking_medium(tester):
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
                "goals": [[0.27, -0.82], [-4.36, -0.31], [-3.92, -0.15], [-3.56, 2.87]],
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
                "goals": [[0.27, -0.82], [6.73, -5.06], [-5.35, -1.48], [5.34, 4.98]],
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
                "goals": [[3.12, 1.80], [-5.70, -1.12], [1.06, 3.13], [-7.94, -0.86]],
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
                "goals": [[3.12, 1.80], [-5.62, 0.92], [-1.90, 5.21], [6.85, 4.24]],
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

def tests_adult_30_child_70_test_case_51_stopped_medium(tester):
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
                "goals": [[-2.15, -3.09], [5.06, 0.41], [7.66, 4.20], [5.37, 3.99]],
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
                "init_x": -2.15,
                "init_y": -2.09,
                "init_a": 39.50,
                "velocity": 0.99,
                "goals": [[-2.15, -3.09], [-4.22, -0.79], [1.51, -0.23], [-2.96, -1.60]],
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
                "goals": [[-0.59, 2.08], [7.98, 0.57], [2.71, 3.10], [5.97, 3.59]],
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
                "init_x": -6.38,
                "init_y": -0.15,
                "init_a": 86.54,
                "velocity": 0.86,
                "goals": [[0.37, -5.05], [-7.14, 3.14], [7.78, 1.45], [-3.33, -4.53]],
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
                "init_x": -2.70,
                "init_y": -2.19,
                "init_a": 53.44,
                "velocity": 0.80,
                "goals": [[-5.93, -5.48], [-7.13, 3.43], [1.96, 2.35], [6.24, 1.73]],
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

def tests_adult_30_child_70_test_case_52_walking_medium(tester):
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
                "goals": [[1.47, 1.12], [-1.99, 5.10], [4.14, 3.10], [-7.53, -1.47]],
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
                "init_x": -3.49,
                "init_y": -4.09,
                "init_a": 21.73,
                "velocity": 1.06,
                "goals": [[1.47, 1.12], [-3.06, 0.74], [2.96, 2.53], [-2.22, 2.89]],
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
                "init_x": 7.23,
                "init_y": -1.20,
                "init_a": -163.11,
                "velocity": 1.12,
                "goals": [[0.64, -4.35], [-6.28, 2.66], [-0.93, -1.03], [-0.37, -0.95]],
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
                "init_x": 0.99,
                "init_y": -1.32,
                "init_a": 12.62,
                "velocity": 0.87,
                "goals": [[1.66, -3.46], [-1.23, -0.51], [7.74, -2.63], [0.49, -3.38]],
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
                "goals": [[-7.25, -4.53], [-3.77, 4.57], [-5.18, 5.70], [-2.96, 4.04]],
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

def tests_adult_30_child_70_test_case_53_walking_medium(tester):
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
                "goals": [[-0.08, 3.28], [5.40, -0.44], [-1.19, 0.08], [-3.68, 3.39]],
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
                "init_x": 8.11,
                "init_y": -1.97,
                "init_a": -170.03,
                "velocity": 0.96,
                "goals": [[-0.08, 3.28], [0.89, 3.26], [5.15, -0.78], [6.46, 4.73]],
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
                "init_x": -1.88,
                "init_y": -4.47,
                "init_a": 91.15,
                "velocity": 1.04,
                "goals": [[6.39, -3.55], [-2.34, 5.16], [2.17, 0.11], [5.66, -3.52]],
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
                "goals": [[-2.28, -1.42], [-3.38, -4.64], [0.93, 2.74], [6.78, 0.65]],
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

def tests_adult_30_child_70_test_case_54_walking_medium(tester):
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
                "goals": [[3.98, 2.75], [7.19, -2.39], [2.21, -3.94], [-3.24, 3.18]],
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
                "goals": [[7.44, -4.34], [6.81, -4.38], [-7.19, 3.85], [6.69, -4.54]],
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
                "goals": [[-2.62, 4.71], [-2.74, -2.87], [-4.98, 3.82], [-4.72, 5.77]],
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
                "goals": [[-3.58, 3.91], [6.75, 1.60], [3.16, 2.12], [3.71, -1.38]],
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
                "goals": [[1.21, -0.99], [1.07, -3.32], [-5.58, -0.51], [-7.50, 1.10]],
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
                "goals": [[-2.75, -1.68], [3.09, -2.72], [-2.62, -2.17], [1.44, 1.65]],
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

def tests_adult_30_child_70_test_case_55_walking_medium(tester):
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
                "goals": [[-1.34, -4.55], [3.76, 2.96], [-6.83, -1.91], [-1.29, 4.48]],
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
                "goals": [[-1.34, -4.55], [-5.90, -3.69], [0.15, -4.23], [3.37, -5.33]],
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
                "goals": [[7.47, -0.66], [6.65, -4.35], [2.57, -0.86], [-4.35, 4.03]],
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
                "goals": [[-5.99, -4.82], [2.11, 0.98], [0.45, -1.72], [5.12, 3.39]],
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

def tests_adult_30_child_70_test_case_56_stopped_medium(tester):
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
                "goals": [[6.34, -1.27], [-6.02, -0.41], [7.75, 5.32], [3.79, -2.23]],
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
                "init_x": 6.62,
                "init_y": -0.31,
                "init_a": -88.45,
                "velocity": 0.90,
                "goals": [[6.34, -1.27], [0.44, -4.01], [-7.26, -5.58], [-4.31, -4.32]],
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
                "init_x": -4.78,
                "init_y": -1.42,
                "init_a": 134.78,
                "velocity": 1.04,
                "goals": [[-4.30, -3.27], [-4.93, -2.32], [5.84, 1.85], [-6.00, -3.99]],
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
                "goals": [[-0.33, 0.68], [7.85, 5.69], [-3.60, 2.29], [-1.31, 0.33]],
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

def tests_adult_30_child_70_test_case_57_stopped_medium(tester):
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
                "goals": [[5.13, 2.94], [-7.91, 2.64], [-1.62, 0.91], [-3.44, 1.12]],
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
                "goals": [[5.13, 2.94], [3.64, 2.92], [-1.45, 0.54], [6.24, 0.30]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.62,
                "init_y": 2.11,
                "init_a": 48.72,
                "velocity": 0.94,
                "goals": [[0.75, -3.59], [1.61, 5.89], [7.88, 0.42], [-4.98, -0.72]],
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
                "goals": [[-1.12, 0.37], [-4.25, 0.42], [3.11, 2.81], [0.96, 5.13]],
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
                "goals": [[-3.48, 5.36], [4.74, -5.19], [-1.58, -5.56], [3.32, 4.87]],
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
                "goals": [[4.32, -0.49], [-4.60, -1.72], [6.34, -3.85], [-2.53, 4.63]],
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

def tests_adult_30_child_70_test_case_58_walking_medium(tester):
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
                "goals": [[7.59, -4.42], [-0.82, -5.73], [7.14, 4.63], [4.21, 3.53]],
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
                "goals": [[7.59, -4.42], [-1.67, 2.80], [4.01, -5.09], [-2.42, 5.41]],
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
                "goals": [[-3.20, -2.39], [3.21, 4.52], [-2.98, -0.72], [1.13, 2.59]],
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
                "goals": [[-3.20, -2.39], [0.81, 3.66], [5.34, -0.89], [-2.66, 2.17]],
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
                "goals": [[2.48, -3.77], [-7.22, 5.51], [2.35, 2.66], [-4.97, 3.96]],
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

def tests_adult_30_child_70_test_case_59_stopped_medium(tester):
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
                "goals": [[-2.03, 2.97], [-5.90, -1.27], [7.22, -1.03], [7.13, 4.38]],
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
                "init_x": -2.50,
                "init_y": 2.09,
                "init_a": 120.09,
                "velocity": 1.04,
                "goals": [[-2.03, 2.97], [-6.27, -5.87], [3.35, 3.11], [-0.50, -1.15]],
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
                "init_x": -7.88,
                "init_y": 2.24,
                "init_a": 29.74,
                "velocity": 0.95,
                "goals": [[-7.88, 2.24], [-1.21, 4.83], [-3.24, 1.64], [-2.46, -5.11]],
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
                "init_x": -8.30,
                "init_y": 1.33,
                "init_a": 29.74,
                "velocity": 0.95,
                "goals": [[-7.88, 2.24], [2.75, 0.84], [-4.86, 2.30], [0.65, 5.40]],
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
                "goals": [[-1.26, 5.49], [-2.87, 1.60], [-3.36, -5.80], [-7.10, -1.89]],
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
                "goals": [[2.05, -3.99], [-1.83, -3.55], [6.39, 4.98], [-1.11, -2.17]],
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

def tests_adult_30_child_70_test_case_60_walking_medium(tester):
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
                "goals": [[4.38, 5.93], [-6.76, 0.13], [-1.07, 1.18], [3.40, 5.73]],
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
                "goals": [[-3.66, -3.74], [5.95, 2.93], [-4.39, -1.70], [1.64, -5.31]],
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
                "goals": [[-5.57, -4.46], [1.05, -0.86], [-6.77, -4.37], [-1.99, 4.68]],
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
                "goals": [[6.53, 0.30], [-4.79, -4.70], [2.52, -1.77], [7.72, -4.40]],
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
                "goals": [[3.93, -4.43], [-6.23, -3.18], [-6.53, -0.26], [-1.14, -4.86]],
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

def tests_adult_30_child_70_test_case_61_stopped_medium(tester):
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
                "goals": [[0.99, 2.09], [-1.67, 1.31], [-3.31, -1.97], [3.88, -2.35]],
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
                "goals": [[0.99, 2.09], [-0.14, -4.75], [-0.91, -2.33], [7.91, -5.25]],
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
                "goals": [[1.07, -3.92], [0.17, 4.93], [-6.46, -4.73], [7.52, -0.52]],
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
                "goals": [[5.56, 3.38], [5.86, 3.17], [-2.01, -1.09], [1.13, 5.64]],
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

def tests_adult_30_child_70_test_case_62_stopped_medium(tester):
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
                "goals": [[5.98, 4.30], [0.40, -5.59], [-0.54, 1.75], [-6.83, -3.45]],
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
                "goals": [[-1.33, 1.09], [-1.97, 0.76], [0.43, -0.67], [3.36, -3.70]],
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
                "goals": [[-1.33, 1.09], [-6.16, 3.04], [2.77, -5.13], [5.24, -5.92]],
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
                "goals": [[-6.09, -0.56], [2.51, -1.75], [-0.50, 0.92], [1.48, -2.04]],
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
                "goals": [[1.69, -0.46], [-6.95, -0.97], [-6.40, -5.12], [-3.87, -0.82]],
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
                "goals": [[3.27, 4.54], [7.59, -0.70], [-6.07, 2.61], [-1.34, -3.86]],
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

def tests_adult_30_child_70_test_case_63_stopped_medium(tester):
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
                "goals": [[7.87, -2.53], [-2.75, -4.88], [4.30, -3.93], [3.94, 4.53]],
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
                "goals": [[1.47, -3.00], [0.90, 0.03], [-6.25, -2.12], [-7.35, 0.02]],
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
                "init_x": 1.01,
                "init_y": -2.10,
                "init_a": 168.65,
                "velocity": 0.89,
                "goals": [[1.47, -3.00], [3.71, -5.75], [1.51, -1.16], [1.81, -0.05]],
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
                "init_x": 5.36,
                "init_y": -4.22,
                "init_a": 74.54,
                "velocity": 1.07,
                "goals": [[-6.73, 0.89], [7.92, 4.79], [3.62, 3.25], [1.80, 3.02]],
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

def tests_adult_30_child_70_test_case_64_walking_medium(tester):
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
                "goals": [[6.25, -5.10], [7.48, 0.10], [-6.05, -3.24], [-1.53, -0.62]],
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
                "goals": [[-6.82, -1.31], [1.92, 2.08], [-4.51, -1.74], [6.23, 1.99]],
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
                "init_x": -0.73,
                "init_y": -3.64,
                "init_a": -116.00,
                "velocity": 0.93,
                "goals": [[-6.82, -1.31], [-7.37, -3.20], [-4.62, -1.46], [-7.89, 2.31]],
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
                "goals": [[-4.24, -0.40], [-4.61, -2.14], [0.48, -5.96], [-6.54, 1.86]],
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
                "goals": [[5.09, -3.04], [5.82, -2.36], [3.41, -1.06], [0.90, -3.48]],
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

def tests_adult_30_child_70_test_case_65_stopped_medium(tester):
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
                "goals": [[3.61, -4.95], [4.48, -1.27], [-0.72, -5.70], [2.07, 1.44]],
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
                "goals": [[3.61, -4.95], [-7.82, -0.65], [2.25, -3.68], [-3.71, -4.94]],
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
                "goals": [[-0.85, -1.73], [5.89, -0.68], [-1.37, 5.43], [6.30, -3.13]],
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
                "goals": [[-1.73, -5.06], [-4.03, 0.39], [3.80, -0.85], [4.12, -5.45]],
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
                "goals": [[-5.68, -0.30], [1.50, 0.91], [-7.71, 3.64], [4.43, 5.24]],
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
                "goals": [[0.76, 5.05], [6.15, 0.72], [7.53, -3.51], [-1.00, -4.05]],
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

def tests_adult_30_child_70_test_case_66_stopped_medium(tester):
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
                "goals": [[-6.27, -5.88], [-1.86, 3.68], [2.91, -4.06], [-1.19, 3.53]],
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
                "goals": [[-1.27, -0.65], [-4.65, -4.81], [-2.80, -4.98], [6.02, -0.85]],
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
                "init_x": -0.28,
                "init_y": -0.50,
                "init_a": 52.54,
                "velocity": 1.02,
                "goals": [[-1.27, -0.65], [7.34, 2.30], [4.79, -0.12], [-6.64, 1.34]],
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
                "init_x": 0.42,
                "init_y": -4.77,
                "init_a": -108.03,
                "velocity": 0.98,
                "goals": [[-7.09, -5.41], [7.92, 1.68], [-0.76, -5.07], [4.65, -4.36]],
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

def tests_adult_30_child_70_test_case_67_stopped_medium(tester):
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
                "goals": [[6.06, -4.08], [-1.13, -0.63], [-3.15, 5.01], [6.17, 3.46]],
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
                "goals": [[-3.04, -2.77], [7.30, 0.41], [-6.77, -4.42], [4.80, 2.77]],
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
                "goals": [[-3.49, 5.11], [0.27, 4.20], [-1.29, -0.46], [1.79, 0.15]],
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
                "goals": [[-4.85, 2.66], [-2.31, -3.56], [-0.43, 4.56], [6.71, 1.02]],
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

def tests_adult_30_child_70_test_case_68_stopped_medium(tester):
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
                "goals": [[-1.49, -0.10], [1.66, -5.79], [6.27, 2.63], [-4.49, 4.54]],
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
                "goals": [[-4.26, 2.08], [-7.28, 4.14], [3.33, 4.86], [-7.10, 3.96]],
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
                "goals": [[7.74, -1.73], [-7.60, 5.62], [-1.01, -3.76], [-3.88, -3.56]],
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
                "goals": [[-2.58, 4.79], [-7.05, 3.12], [2.59, 2.33], [3.02, 2.51]],
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
                "goals": [[-5.41, 2.67], [-1.17, 5.65], [4.07, -0.28], [-2.45, -1.72]],
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

def tests_adult_30_child_70_test_case_69_stopped_medium(tester):
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
                "goals": [[4.76, -5.57], [-4.64, 5.98], [5.32, -5.51], [2.17, -0.81]],
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
                "goals": [[-3.47, 5.59], [-6.94, -2.77], [-1.29, 5.15], [3.85, 1.23]],
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
                "goals": [[-0.14, -4.01], [-3.62, -0.04], [2.29, 4.08], [7.87, -4.72]],
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
                "goals": [[-7.32, -3.19], [-6.35, -1.14], [-3.12, -4.38], [1.11, -4.31]],
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

def tests_adult_30_child_70_test_case_70_stopped_medium(tester):
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
                "goals": [[-5.48, 3.20], [3.45, -5.70], [-4.26, 3.99], [-5.98, 4.48]],
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
                "goals": [[-5.16, -5.71], [-0.60, 0.24], [7.68, -2.67], [-7.09, 4.10]],
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
                "init_x": -6.16,
                "init_y": -5.72,
                "init_a": -47.27,
                "velocity": 1.07,
                "goals": [[-5.16, -5.71], [1.49, 2.97], [-6.00, -0.98], [4.31, -4.56]],
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
                "goals": [[-3.56, 5.83], [-1.58, 4.09], [-2.30, 0.81], [2.34, -1.40]],
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

def tests_adult_30_child_70_test_case_71_stopped_medium(tester):
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
                "goals": [[-6.42, -2.08], [5.98, -2.69], [4.03, 3.99], [2.72, -1.83]],
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
                "goals": [[5.10, 3.40], [1.39, -2.86], [0.92, 5.18], [5.70, 0.41]],
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
                "goals": [[5.10, 3.40], [2.78, 5.55], [-7.63, 4.96], [1.98, 5.44]],
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
                "goals": [[5.00, 4.08], [-5.50, 5.90], [5.17, -3.28], [6.40, -2.90]],
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
                "goals": [[-5.77, -4.61], [-3.70, -2.56], [7.08, -3.29], [-3.22, 5.17]],
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
                "goals": [[3.58, -3.18], [-5.16, -1.74], [5.80, 5.30], [-1.37, 1.73]],
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

def tests_adult_30_child_70_test_case_72_walking_medium(tester):
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
                "goals": [[-7.07, -5.05], [7.66, -0.86], [-4.41, 3.14], [5.77, -2.35]],
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
                "goals": [[1.89, -3.33], [-1.80, -5.06], [1.18, 2.84], [-5.93, -0.21]],
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
                "goals": [[5.54, -2.21], [-1.57, -0.36], [-6.61, 4.26], [2.57, 1.03]],
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
                "goals": [[-1.63, -2.04], [-2.69, -0.98], [6.84, -1.11], [-4.19, -0.92]],
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
                "goals": [[7.02, 1.04], [7.61, 2.10], [3.34, 0.23], [-6.79, 5.38]],
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
                "goals": [[-4.82, 1.87], [-3.48, -5.28], [0.15, -5.26], [-7.87, 1.35]],
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

def tests_adult_30_child_70_test_case_73_stopped_medium(tester):
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
                "goals": [[5.90, -2.77], [-2.31, -2.68], [-2.40, 2.11], [5.79, -3.47]],
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
                "init_x": 7.95,
                "init_y": -3.46,
                "init_a": 157.32,
                "velocity": 0.82,
                "goals": [[7.95, -3.46], [3.09, 1.75], [0.45, -2.38], [6.58, 2.03]],
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
                "init_x": 8.16,
                "init_y": -4.44,
                "init_a": 157.32,
                "velocity": 0.82,
                "goals": [[7.95, -3.46], [-0.79, 3.21], [1.60, 1.89], [6.15, -0.41]],
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
                "goals": [[-2.42, 4.76], [-4.06, -5.00], [-5.79, 3.13], [-4.20, 0.21]],
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
                "init_x": -6.54,
                "init_y": 5.24,
                "init_a": -112.55,
                "velocity": 1.03,
                "goals": [[-1.60, -0.72], [-0.59, 4.14], [4.08, 4.39], [-3.90, 2.72]],
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

def tests_adult_30_child_70_test_case_74_walking_medium(tester):
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
                "goals": [[-6.21, 2.48], [-6.14, 1.83], [-2.44, -0.34], [-6.90, 5.11]],
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
                "goals": [[2.09, 3.91], [7.84, -2.07], [0.37, 3.54], [-0.33, -4.89]],
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
                "goals": [[7.20, -2.26], [-2.90, 0.56], [-1.87, -2.81], [3.62, 2.37]],
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
                "goals": [[7.66, -1.08], [-0.72, 4.08], [4.07, -1.51], [5.93, -4.09]],
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
                "goals": [[1.90, 2.82], [-4.06, 0.75], [-6.99, -5.72], [5.91, -1.67]],
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

def tests_adult_30_child_70_test_case_75_walking_medium(tester):
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
                "goals": [[4.86, 1.57], [-3.11, 5.35], [-7.77, -3.51], [1.93, -4.92]],
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
                "goals": [[-4.17, 2.05], [-5.76, -4.89], [-1.18, 0.69], [2.63, -4.81]],
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
                "goals": [[-0.80, 0.52], [7.62, 1.33], [-3.99, 0.43], [-4.77, -0.27]],
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
                "init_x": -4.58,
                "init_y": 0.81,
                "init_a": -0.59,
                "velocity": 0.95,
                "goals": [[5.52, 3.58], [-2.27, -4.83], [-6.51, -0.92], [0.52, 0.57]],
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

def tests_adult_30_child_70_test_case_76_walking_medium(tester):
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
                "goals": [[3.87, -1.18], [-4.44, -4.45], [2.41, -1.36], [6.94, -3.10]],
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
                "goals": [[3.87, -1.18], [2.75, 2.13], [-5.18, -5.70], [-5.02, -5.56]],
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
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.56,
                "init_y": -1.81,
                "init_a": -45.64,
                "velocity": 1.05,
                "goals": [[1.25, -5.61], [3.61, 3.42], [0.79, 0.38], [4.82, 5.85]],
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
                "init_x": -0.31,
                "init_y": 4.43,
                "init_a": 179.07,
                "velocity": 1.10,
                "goals": [[-6.27, -3.68], [-1.68, 3.53], [-3.64, 5.34], [-2.80, 0.59]],
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

def tests_adult_30_child_70_test_case_77_stopped_medium(tester):
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
                "goals": [[0.53, 3.14], [5.08, 3.11], [3.95, 0.66], [-0.95, 1.42]],
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
                "init_x": 0.49,
                "init_y": 4.14,
                "init_a": -1.21,
                "velocity": 0.87,
                "goals": [[0.53, 3.14], [-2.89, -3.20], [2.25, -4.63], [-7.42, 5.07]],
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
                "init_x": 2.35,
                "init_y": -0.77,
                "init_a": 26.77,
                "velocity": 0.94,
                "goals": [[2.35, -0.77], [7.69, 2.17], [0.58, 5.01], [-4.89, 2.84]],
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
                "init_x": 2.92,
                "init_y": 0.05,
                "init_a": 26.77,
                "velocity": 0.94,
                "goals": [[2.35, -0.77], [-1.15, -3.51], [2.41, 4.57], [2.51, 0.02]],
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
                "goals": [[-3.93, -4.08], [-3.32, 4.37], [5.68, -5.16], [-3.41, 4.38]],
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
                "init_x": -4.15,
                "init_y": 0.64,
                "init_a": -66.29,
                "velocity": 1.02,
                "goals": [[4.76, -1.54], [-5.12, 2.30], [-4.43, 0.82], [-7.83, -0.67]],
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

def tests_adult_30_child_70_test_case_78_stopped_medium(tester):
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
                "goals": [[0.22, -1.46], [-0.68, 0.75], [7.65, -1.96], [-4.77, -1.95]],
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
                "goals": [[-2.86, -0.75], [1.76, 4.63], [4.22, 3.78], [-5.24, -1.66]],
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
                "goals": [[2.35, 4.29], [1.96, -2.38], [-6.17, 1.09], [-5.84, -3.78]],
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
                "goals": [[5.33, -1.40], [4.21, 5.37], [-4.67, -4.33], [-3.33, -5.89]],
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

def tests_adult_30_child_70_test_case_79_walking_medium(tester):
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
                "goals": [[-3.51, -2.41], [3.44, 4.21], [-1.41, 2.43], [0.64, 2.93]],
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
                "init_x": -1.41,
                "init_y": 0.98,
                "init_a": -22.80,
                "velocity": 1.08,
                "goals": [[-3.51, -2.41], [1.22, 0.98], [3.81, -3.39], [4.44, 4.40]],
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
                "init_x": -2.61,
                "init_y": -4.98,
                "init_a": 87.89,
                "velocity": 0.89,
                "goals": [[-1.48, 0.97], [3.71, 4.08], [6.75, 4.31], [6.87, -5.93]],
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
                "goals": [[-4.68, 5.18], [-0.91, 5.18], [-0.62, -4.85], [4.28, 0.96]],
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
                "init_x": -6.07,
                "init_y": 0.80,
                "init_a": -131.31,
                "velocity": 0.94,
                "goals": [[4.55, 3.13], [-2.17, 4.40], [-3.43, 2.77], [-4.20, -1.24]],
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
                "goals": [[4.62, 1.01], [5.76, 1.67], [-1.34, 5.80], [7.26, 2.17]],
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

def tests_adult_30_child_70_test_case_80_stopped_medium(tester):
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
                "goals": [[-6.45, -3.35], [-6.49, 3.99], [-3.63, -2.42], [-4.29, 4.01]],
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
                "goals": [[2.43, 4.67], [7.00, 5.83], [1.65, 2.10], [-2.39, -2.34]],
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
                "goals": [[2.43, 4.67], [-7.04, -3.30], [4.12, -3.51], [-5.70, -2.27]],
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
                "goals": [[6.46, 2.05], [5.35, -0.79], [2.58, -3.19], [4.34, 3.00]],
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
                "goals": [[-5.04, 3.70], [4.22, 0.40], [-3.92, 0.21], [-1.88, 0.10]],
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
                "goals": [[-3.32, -2.47], [6.84, -3.81], [5.82, -1.89], [-0.55, 4.50]],
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

def tests_adult_30_child_70_test_case_81_stopped_medium(tester):
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
                "goals": [[-0.01, 0.38], [-7.41, -3.23], [2.76, -0.13], [2.29, -2.45]],
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
                "goals": [[6.90, -2.01], [6.61, 1.83], [-2.94, 3.93], [3.39, -4.50]],
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
                "goals": [[6.90, -2.01], [6.01, 3.77], [6.54, -1.34], [1.34, 5.24]],
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
                "goals": [[1.92, -4.35], [5.58, 4.19], [8.00, 1.74], [4.04, -0.44]],
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

def tests_adult_30_child_70_test_case_82_walking_medium(tester):
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
                "goals": [[3.52, -4.79], [-4.19, 1.07], [-4.29, 1.06], [-6.14, 4.35]],
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
                "goals": [[-6.63, -1.13], [-4.26, -2.95], [-7.81, -2.59], [7.01, 1.50]],
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
                "goals": [[-6.63, -1.13], [-7.40, -3.60], [1.24, -3.32], [-0.84, 4.89]],
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
                "goals": [[-7.03, -5.84], [4.46, 2.26], [-1.72, 3.60], [6.19, 3.00]],
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
                "goals": [[5.11, -2.87], [-1.84, 5.65], [7.88, 5.08], [4.10, 0.72]],
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
                "goals": [[-2.11, 1.32], [4.51, -0.22], [-1.48, 1.59], [-2.24, -4.04]],
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

def tests_adult_30_child_70_test_case_83_walking_medium(tester):
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
                "goals": [[7.21, 2.86], [2.45, -3.09], [7.05, -3.82], [-4.72, 1.62]],
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
                "goals": [[3.62, -5.16], [-4.03, -0.92], [-3.48, 3.28], [-7.15, 1.68]],
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
                "init_x": -5.37,
                "init_y": 2.13,
                "init_a": -155.57,
                "velocity": 0.88,
                "goals": [[4.58, -4.29], [0.99, 3.98], [3.42, -2.37], [4.60, -1.17]],
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
                "init_x": 5.52,
                "init_y": 0.26,
                "init_a": 163.92,
                "velocity": 1.11,
                "goals": [[-1.51, 4.88], [2.03, -0.99], [-0.35, 2.12], [6.12, 4.77]],
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

def tests_adult_30_child_70_test_case_84_walking_medium(tester):
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
                "goals": [[-5.76, -4.85], [-0.63, -2.97], [-7.99, -5.23], [2.55, -5.10]],
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
                "goals": [[-5.76, -4.85], [1.84, 0.34], [-5.42, -3.64], [-0.70, -3.38]],
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
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.87,
                "init_y": 3.19,
                "init_a": -53.55,
                "velocity": 1.04,
                "goals": [[-2.14, -0.38], [6.29, -4.70], [3.72, -2.42], [4.18, 3.13]],
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
                "goals": [[6.90, 1.19], [-6.10, 1.31], [-2.61, 0.76], [7.93, -0.58]],
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
                "goals": [[-4.01, -3.23], [-2.88, 0.63], [-1.08, 3.09], [1.35, -3.11]],
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

def tests_adult_30_child_70_test_case_85_walking_medium(tester):
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
                "goals": [[-3.10, -5.21], [2.90, 4.58], [-5.44, -2.10], [-5.87, 4.86]],
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
                "init_x": -4.45,
                "init_y": -3.63,
                "init_a": 63.92,
                "velocity": 0.81,
                "goals": [[-2.52, -2.69], [4.59, 5.38], [-7.63, -0.85], [7.04, -5.71]],
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
                "goals": [[4.55, -3.90], [-1.70, 1.36], [6.51, -0.51], [-2.24, -0.66]],
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
                "init_x": -6.64,
                "init_y": 5.97,
                "init_a": -69.34,
                "velocity": 0.95,
                "goals": [[-7.63, -0.95], [-6.90, 2.54], [0.43, -4.83], [-3.64, -3.06]],
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
                "init_x": 3.73,
                "init_y": 2.84,
                "init_a": -155.75,
                "velocity": 0.86,
                "goals": [[-2.30, 2.64], [1.27, 3.44], [5.68, -3.81], [6.09, -4.27]],
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

def tests_adult_30_child_70_test_case_86_walking_medium(tester):
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
                "goals": [[5.91, 3.44], [-5.11, -4.26], [5.76, -4.00], [-2.54, 1.96]],
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
                "goals": [[5.91, 3.44], [-4.42, 0.56], [7.85, -5.28], [0.66, 3.62]],
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
                "goals": [[2.81, -0.67], [-0.26, -0.82], [-6.65, 2.38], [3.97, -5.76]],
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
                "goals": [[5.80, -4.98], [-1.14, -4.72], [7.43, -4.26], [-5.02, 5.19]],
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

def tests_adult_30_child_70_test_case_87_walking_medium(tester):
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
                "goals": [[5.78, 0.95], [7.67, 3.98], [-6.70, -5.97], [-2.53, 5.77]],
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
                "init_x": -8.00,
                "init_y": -1.91,
                "init_a": -3.83,
                "velocity": 1.08,
                "goals": [[5.78, 0.95], [5.40, -0.52], [-6.93, 5.69], [5.14, 5.46]],
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
                "goals": [[5.24, -3.04], [3.70, -5.26], [4.18, 2.37], [7.12, -4.14]],
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
                "init_x": -0.07,
                "init_y": 5.51,
                "init_a": 143.54,
                "velocity": 0.99,
                "goals": [[4.63, 5.51], [-3.20, -5.10], [-7.50, -0.04], [-3.65, 2.25]],
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
                "init_x": -6.58,
                "init_y": 3.11,
                "init_a": -42.04,
                "velocity": 0.83,
                "goals": [[-4.66, -0.15], [6.81, -2.78], [-4.98, 3.69], [5.60, -5.42]],
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

def tests_adult_30_child_70_test_case_88_walking_medium(tester):
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
                "goals": [[2.02, -4.36], [7.63, -4.53], [2.44, 5.33], [7.60, -1.86]],
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
                "goals": [[2.02, -4.36], [5.36, -1.97], [5.24, -1.23], [6.82, 5.23]],
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
                "goals": [[-0.34, -0.49], [3.83, -4.25], [-6.84, -5.49], [-4.14, -4.66]],
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
                "goals": [[-6.28, -0.19], [-1.93, 1.37], [5.96, 2.51], [2.11, 1.14]],
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
                "goals": [[-4.79, 5.44], [-7.65, -4.66], [-5.02, -4.32], [-5.65, -3.65]],
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
                "goals": [[-6.25, 5.48], [-3.53, -5.41], [2.95, 5.34], [-1.44, -2.28]],
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

def tests_adult_30_child_70_test_case_89_stopped_medium(tester):
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
                "goals": [[-1.11, 2.39], [4.46, -3.96], [-5.01, 5.12], [-7.35, 0.93]],
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
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.99,
                "init_y": -0.92,
                "init_a": -171.32,
                "velocity": 0.93,
                "goals": [[-4.99, -0.89], [-0.60, 4.36], [-0.20, 1.77], [-5.37, 0.03]],
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
                "goals": [[3.92, 0.83], [-2.51, -5.03], [5.84, 0.31], [3.51, 2.37]],
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
                "goals": [[2.50, 4.84], [1.93, 3.31], [-7.99, 4.53], [-3.64, 3.23]],
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

def tests_adult_30_child_70_test_case_90_stopped_medium(tester):
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
                "goals": [[-0.93, 2.74], [-2.09, -0.97], [-3.31, 1.48], [3.33, -5.95]],
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
                "goals": [[-3.13, -4.23], [4.06, 1.35], [-6.36, 4.78], [-2.60, 0.68]],
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
                "goals": [[-3.13, -4.23], [-4.99, -5.92], [-1.62, -2.58], [2.89, 3.87]],
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
                "goals": [[2.37, 2.14], [3.40, -3.16], [0.12, 4.95], [0.75, 5.63]],
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
                "goals": [[-5.78, -4.44], [4.10, -1.10], [-6.16, -2.25], [6.36, -1.04]],
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
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.10,
                "init_y": -1.78,
                "init_a": 109.96,
                "velocity": 0.99,
                "goals": [[-6.33, -5.26], [-2.03, 1.65], [-0.95, -2.09], [-2.23, -5.08]],
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

def tests_adult_30_child_70_test_case_91_stopped_medium(tester):
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
                "goals": [[-2.51, 4.82], [5.77, 1.80], [3.42, 2.61], [-7.81, 0.41]],
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
                "init_x": -3.51,
                "init_y": 4.69,
                "init_a": -48.16,
                "velocity": 1.02,
                "goals": [[-2.51, 4.82], [-1.13, 2.42], [3.02, 0.37], [-3.80, -0.52]],
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
                "goals": [[-6.31, 5.32], [-7.33, -0.17], [7.57, 0.37], [-7.16, -5.82]],
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
                "init_x": 2.58,
                "init_y": -4.65,
                "init_a": 113.75,
                "velocity": 0.99,
                "goals": [[4.20, 0.75], [-2.29, -4.59], [-6.67, 0.31], [5.45, -1.15]],
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

def tests_adult_30_child_70_test_case_92_stopped_medium(tester):
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
                "goals": [[-0.37, 0.08], [-7.07, 2.21], [-0.34, 3.91], [3.82, -5.79]],
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
                "goals": [[-0.37, 0.08], [7.69, -4.29], [3.32, 5.48], [-3.29, -5.79]],
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
                "goals": [[-2.14, -4.77], [-8.00, -1.23], [-3.48, -4.06], [4.87, 0.68]],
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
                "goals": [[-4.70, -1.62], [-3.34, -4.30], [1.72, -0.50], [5.05, 5.61]],
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
                "goals": [[2.90, -5.64], [-5.29, 3.39], [6.44, 1.54], [-3.23, -5.28]],
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
                "goals": [[3.51, 2.65], [5.28, -4.81], [6.89, -3.88], [-7.87, -3.88]],
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

def tests_adult_30_child_70_test_case_93_walking_medium(tester):
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
                "goals": [[-1.77, 3.82], [1.74, 3.33], [6.08, -2.84], [0.16, -5.27]],
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
                "goals": [[-4.21, -5.96], [-2.70, -0.26], [6.91, -5.88], [4.72, -5.30]],
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
                "goals": [[6.38, -4.82], [-5.11, 0.51], [-2.50, 3.02], [-5.99, -2.71]],
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
                "init_x": -3.41,
                "init_y": 2.45,
                "init_a": -71.39,
                "velocity": 1.02,
                "goals": [[1.99, -4.98], [-0.37, 2.61], [4.90, 4.89], [-4.08, -5.64]],
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

def tests_adult_30_child_70_test_case_94_stopped_medium(tester):
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
                "goals": [[2.89, 5.31], [-1.48, -1.25], [0.03, -1.54], [-0.70, 2.39]],
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
                "goals": [[2.89, 5.31], [1.54, -0.36], [7.50, -2.04], [1.08, 4.83]],
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
                "goals": [[-0.98, 0.12], [1.99, 5.61], [-7.25, 5.78], [3.20, 2.99]],
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
                "goals": [[3.19, 3.93], [2.66, 3.25], [-5.90, 4.14], [2.56, 4.42]],
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
                "goals": [[-3.49, 2.80], [-5.45, -3.29], [1.40, -4.33], [4.21, -1.47]],
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

def tests_adult_30_child_70_test_case_95_stopped_medium(tester):
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
                "goals": [[-6.81, -0.97], [0.32, -0.01], [-2.20, -2.05], [-4.78, -5.31]],
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
                "goals": [[-6.81, -0.97], [1.30, -3.28], [-7.66, -4.44], [-2.70, 2.06]],
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
                "goals": [[0.55, -4.29], [-0.21, 4.51], [7.74, 0.86], [2.44, -0.86]],
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
                "goals": [[7.35, -2.17], [0.40, 3.27], [0.01, -3.73], [7.46, 4.22]],
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
                "goals": [[-5.06, -5.11], [-4.71, 1.31], [-0.58, -1.92], [3.45, -4.91]],
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

def tests_adult_30_child_70_test_case_96_stopped_medium(tester):
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
                "goals": [[1.16, 3.19], [4.38, 2.34], [1.58, 2.02], [5.18, -1.24]],
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
                "goals": [[1.16, 3.19], [-3.52, 1.65], [-1.61, 2.97], [4.95, -2.45]],
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
                "goals": [[-7.73, 3.23], [3.38, 4.11], [4.66, -5.12], [5.14, -2.28]],
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
                "goals": [[-7.51, 2.53], [-1.27, -3.03], [0.85, -1.95], [-3.53, -0.77]],
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
                "goals": [[6.30, 0.59], [3.48, 2.66], [2.28, -3.09], [1.89, -0.70]],
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
                "goals": [[3.25, -5.64], [6.07, 4.56], [7.23, 4.33], [2.65, -2.29]],
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

def tests_adult_30_child_70_test_case_97_walking_medium(tester):
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
                "goals": [[2.60, -2.87], [4.12, 4.71], [5.22, -1.51], [4.46, -5.81]],
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
                "init_x": 7.33,
                "init_y": 1.60,
                "init_a": -160.74,
                "velocity": 0.94,
                "goals": [[2.60, -2.87], [-7.75, 0.95], [6.46, 1.54], [-4.57, -0.69]],
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
                "init_x": -2.91,
                "init_y": -0.96,
                "init_a": 75.76,
                "velocity": 0.97,
                "goals": [[-2.32, -4.17], [5.26, -0.90], [6.51, 1.57], [-4.07, 4.43]],
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
                "init_x": -2.15,
                "init_y": -0.30,
                "init_a": 75.76,
                "velocity": 0.97,
                "goals": [[-2.32, -4.17], [0.55, 3.03], [3.02, -3.69], [-1.15, -3.93]],
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
                "init_x": -0.39,
                "init_y": -0.57,
                "init_a": -133.27,
                "velocity": 0.99,
                "goals": [[-0.17, -4.25], [-2.78, 1.37], [4.12, -4.88], [-4.62, -0.69]],
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

def tests_adult_30_child_70_test_case_98_stopped_medium(tester):
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
                "goals": [[-3.25, -0.40], [4.03, 4.45], [0.79, -1.31], [1.69, -3.66]],
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
                "goals": [[1.70, -2.21], [-5.64, -4.63], [4.85, 3.77], [7.86, -1.55]],
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
                "goals": [[1.70, -2.21], [7.48, -3.83], [2.46, -5.57], [7.93, 1.12]],
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
                "goals": [[-1.70, 5.79], [-7.95, 0.56], [-7.15, -0.23], [-6.56, 2.07]],
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

def tests_adult_30_child_70_test_case_99_walking_medium(tester):
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
                "goals": [[-1.67, -5.92], [-1.94, 3.37], [-5.01, -4.32], [-3.27, 2.83]],
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
                "init_x": -0.95,
                "init_y": -3.18,
                "init_a": 91.23,
                "velocity": 0.81,
                "goals": [[-1.67, -5.92], [-3.65, 2.85], [-7.40, 3.18], [-6.93, -1.76]],
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
                "goals": [[-4.45, -2.68], [-3.05, -1.58], [6.52, -0.40], [5.42, -3.60]],
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
                "init_x": 1.33,
                "init_y": 1.47,
                "init_a": 171.72,
                "velocity": 0.87,
                "goals": [[-3.38, -1.67], [-1.90, 5.56], [-4.11, -1.96], [-0.35, -5.57]],
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
                "goals": [[-0.99, -3.43], [-2.69, -4.65], [-0.82, -4.04], [6.44, -0.81]],
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

def tests_adult_30_child_70_test_case_100_stopped_medium(tester):
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
                "goals": [[-0.41, -3.70], [-6.73, -2.28], [-5.86, 5.07], [-2.11, -2.57]],
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
                "goals": [[5.70, 0.93], [6.55, 3.30], [-7.42, -0.26], [2.18, 0.38]],
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
                "goals": [[6.21, 4.51], [6.64, 4.49], [2.42, -3.90], [0.84, -3.95]],
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
                "goals": [[-5.01, 1.58], [4.21, -2.91], [-1.57, -0.53], [-0.05, -5.05]],
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
                "goals": [[0.60, -4.75], [4.26, 0.42], [-6.55, -2.94], [0.59, 0.24]],
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
