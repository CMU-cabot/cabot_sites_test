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

def tests_adult_70_child_30_test_case_01_stopped_high(tester):
    # Pairs Moving: False
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 20
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.80,
                "init_y": -6.54,
                "init_a": 144.90,
                "velocity": 1.11,
                "goals": [[5.80, -6.54], [-1.14, 4.29], [2.69, 5.24], [0.85, 1.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.97,
                "init_y": -5.99,
                "init_a": 144.90,
                "velocity": 1.11,
                "goals": [[5.80, -6.54], [-1.47, 3.90], [-5.23, 6.33], [0.33, 1.18]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.01,
                "init_y": 2.38,
                "init_a": -35.67,
                "velocity": 1.03,
                "goal_x": -2.01,
                "goal_y": 2.38,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.79,
                "init_y": 1.76,
                "init_a": -35.67,
                "velocity": 1.03,
                "goals": [[-2.01, 2.38], [0.75, 1.94], [-6.83, 3.24], [-5.11, -6.88]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.19,
                "init_y": 3.79,
                "init_a": 75.06,
                "velocity": 1.17,
                "goal_x": -7.19,
                "goal_y": 3.79,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_pair2_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.41,
                "init_y": 2.82,
                "init_a": 75.06,
                "velocity": 1.17,
                "goal_x": -7.19,
                "goal_y": 3.79,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.49,
                "init_y": 3.79,
                "init_a": -112.48,
                "velocity": 1.00,
                "goals": [[6.32, 6.82], [5.57, -1.67], [-0.57, 4.53], [0.78, 4.20]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.70,
                "init_y": 6.04,
                "init_a": 49.07,
                "velocity": 0.96,
                "goal_x": -5.93,
                "goal_y": 5.80,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.94,
                "init_y": 1.49,
                "init_a": -152.90,
                "velocity": 1.02,
                "goal_x": -1.73,
                "goal_y": -3.09,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.34,
                "init_y": -7.43,
                "init_a": 134.43,
                "velocity": 1.04,
                "goal_x": -7.91,
                "goal_y": 1.27,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.26,
                "init_y": -4.79,
                "init_a": 154.56,
                "velocity": 1.14,
                "goal_x": 0.62,
                "goal_y": 7.81,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single5',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.51,
                "init_y": -7.38,
                "init_a": -46.68,
                "velocity": 0.87,
                "goal_x": 7.14,
                "goal_y": -6.16,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single6_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.35,
                "init_y": 5.81,
                "init_a": -165.16,
                "velocity": 0.84,
                "goals": [[0.30, -2.25], [6.57, -5.27], [-4.95, -1.36], [-5.98, -1.24]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single7_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.66,
                "init_y": 2.73,
                "init_a": 122.94,
                "velocity": 0.82,
                "goals": [[-5.94, -6.06], [-4.82, -7.59], [5.24, 6.52], [5.99, 1.52]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single8',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.40,
                "init_y": 2.40,
                "init_a": -84.72,
                "velocity": 1.13,
                "goal_x": -6.46,
                "goal_y": 6.40,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single9',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.41,
                "init_y": -6.14,
                "init_a": -107.50,
                "velocity": 1.02,
                "goal_x": 6.33,
                "goal_y": -6.27,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single10',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.78,
                "init_y": 0.46,
                "init_a": -8.96,
                "velocity": 1.11,
                "goal_x": -3.97,
                "goal_y": 6.42,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single11',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.72,
                "init_y": -1.75,
                "init_a": 0.87,
                "velocity": 1.02,
                "goal_x": 3.30,
                "goal_y": 5.07,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single12',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.54,
                "init_y": -5.98,
                "init_a": -113.74,
                "velocity": 1.11,
                "goal_x": 4.55,
                "goal_y": -3.94,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single13',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.13,
                "init_y": 5.90,
                "init_a": -157.42,
                "velocity": 0.85,
                "goal_x": -6.47,
                "goal_y": 5.85,
                "n_actors": 20,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)

def tests_adult_70_child_30_test_case_02_walking_high(tester):
    # Pairs Moving: True
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 25
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 0.00,
                "init_y": 4.44,
                "init_a": 23.06,
                "velocity": 1.15,
                "goal_x": 6.16,
                "goal_y": 1.57,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.00,
                "init_y": 4.45,
                "init_a": 23.06,
                "velocity": 1.15,
                "goals": [[6.16, 1.57], [-3.99, 7.48], [-4.07, -4.32], [-1.90, -3.29]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.72,
                "init_y": 0.20,
                "init_a": 177.69,
                "velocity": 0.82,
                "goals": [[4.38, 6.04], [-3.56, -2.72], [-4.90, -1.40], [-0.91, 4.06]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.74,
                "init_y": 0.37,
                "init_a": 177.69,
                "velocity": 0.82,
                "goal_x": 4.38,
                "goal_y": 6.04,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_pair2_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.76,
                "init_y": -5.20,
                "init_a": -37.15,
                "velocity": 0.95,
                "goal_x": 0.24,
                "goal_y": 1.19,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_pair2_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -8.55,
                "init_y": -4.60,
                "init_a": -37.15,
                "velocity": 0.95,
                "goal_x": 0.24,
                "goal_y": 1.19,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.33,
                "init_y": 0.62,
                "init_a": -174.80,
                "velocity": 0.94,
                "goals": [[7.77, 5.56], [-2.25, 2.52], [4.75, 4.06], [-6.08, 7.75]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.75,
                "init_y": 1.54,
                "init_a": 178.38,
                "velocity": 1.11,
                "goal_x": -2.78,
                "goal_y": -3.81,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.85,
                "init_y": -0.94,
                "init_a": -165.89,
                "velocity": 0.92,
                "goal_x": -1.77,
                "goal_y": -0.38,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.98,
                "init_y": -6.67,
                "init_a": -132.79,
                "velocity": 0.87,
                "goal_x": -7.23,
                "goal_y": 2.53,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.21,
                "init_y": -5.66,
                "init_a": -96.25,
                "velocity": 1.02,
                "goal_x": -2.61,
                "goal_y": -3.68,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single5_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.49,
                "init_y": -6.95,
                "init_a": 3.78,
                "velocity": 1.12,
                "goals": [[-3.00, -2.87], [2.84, -1.66], [-6.38, 3.39], [-5.22, -1.49]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single6',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.85,
                "init_y": -7.99,
                "init_a": 44.99,
                "velocity": 1.02,
                "goal_x": 3.85,
                "goal_y": 3.41,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single7',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.54,
                "init_y": 3.59,
                "init_a": 149.78,
                "velocity": 1.17,
                "goal_x": 6.19,
                "goal_y": -0.62,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single8_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.58,
                "init_y": 1.86,
                "init_a": -1.33,
                "velocity": 0.96,
                "goals": [[-0.66, -1.80], [-6.47, -1.00], [5.80, 0.52], [3.23, 2.95]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single9',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.27,
                "init_y": 3.83,
                "init_a": 115.72,
                "velocity": 0.94,
                "goal_x": -0.42,
                "goal_y": 5.25,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single10',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.38,
                "init_y": 7.99,
                "init_a": 143.32,
                "velocity": 1.08,
                "goal_x": -2.08,
                "goal_y": -3.25,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single11_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.28,
                "init_y": -4.50,
                "init_a": 115.11,
                "velocity": 1.04,
                "goals": [[8.00, 3.23], [-4.21, 4.50], [-1.58, -5.89], [2.33, -3.85]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single12',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.02,
                "init_y": -1.38,
                "init_a": -113.06,
                "velocity": 0.87,
                "goal_x": -1.71,
                "goal_y": 6.33,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single13',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.91,
                "init_y": 1.03,
                "init_a": -53.04,
                "velocity": 0.86,
                "goal_x": 4.89,
                "goal_y": -4.30,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single14_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.97,
                "init_y": -5.29,
                "init_a": 100.82,
                "velocity": 0.94,
                "goals": [[-7.83, 3.25], [-1.93, -3.53], [-6.45, 7.91], [-4.35, -2.72]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single15',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.50,
                "init_y": 5.93,
                "init_a": 83.00,
                "velocity": 1.20,
                "goal_x": 3.54,
                "goal_y": -2.19,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single16',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.86,
                "init_y": -5.75,
                "init_a": 77.65,
                "velocity": 0.82,
                "goal_x": -2.86,
                "goal_y": 5.74,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single17',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.67,
                "init_y": 6.94,
                "init_a": 48.78,
                "velocity": 1.06,
                "goal_x": 1.34,
                "goal_y": 1.43,
                "n_actors": 25,
            },
        },
        {
            "name": 'actor_single18',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.16,
                "init_y": 5.30,
                "init_a": -128.41,
                "velocity": 1.20,
                "goal_x": -3.11,
                "goal_y": -2.12,
                "n_actors": 25,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)
