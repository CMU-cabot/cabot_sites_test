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
    
    n_actors = 24
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.26,
                "init_y": -4.46,
                "init_a": -133.08,
                "velocity": 0.99,
                "goals": [[2.02, 1.94], [1.42, -7.66], [0.93, -4.90], [4.17, -1.00]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.53,
                "init_y": -5.43,
                "init_a": -133.08,
                "velocity": 0.99,
                "goals": [[2.02, 1.94], [3.09, -1.10], [4.59, 6.13], [4.39, -5.26]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.03,
                "init_y": -7.23,
                "init_a": -98.83,
                "velocity": 1.12,
                "goals": [[-1.26, 1.04], [3.83, -4.55], [4.32, -2.40], [-2.65, 7.81]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.48,
                "init_y": -8.13,
                "init_a": -98.83,
                "velocity": 1.12,
                "goals": [[-1.26, 1.04], [-1.16, 1.41], [-3.88, 7.31], [-7.29, -1.55]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.11,
                "init_y": -5.40,
                "init_a": -12.16,
                "velocity": 1.06,
                "goals": [[6.96, -3.50], [4.96, 6.15], [7.95, -4.83], [3.33, 2.99]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.92,
                "init_y": -5.99,
                "init_a": -12.16,
                "velocity": 1.06,
                "goal_x": 6.96,
                "goal_y": -3.50,
                "n_actors": 24,
            },
        },
        {
            "name": 'actor_pair3_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.58,
                "init_y": 0.39,
                "init_a": 28.31,
                "velocity": 1.17,
                "goal_x": -0.96,
                "goal_y": -3.37,
                "n_actors": 24,
            },
        },
        {
            "name": 'actor_pair3_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.87,
                "init_y": -0.56,
                "init_a": 28.31,
                "velocity": 1.17,
                "goals": [[-0.96, -3.37], [-0.47, 6.72], [2.23, 3.83], [-0.39, 6.81]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair4_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.38,
                "init_y": 3.50,
                "init_a": 52.18,
                "velocity": 0.96,
                "goals": [[7.03, 1.75], [7.48, -6.00], [-6.80, -4.38], [4.44, 0.40]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair4_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.82,
                "init_y": 4.33,
                "init_a": 52.18,
                "velocity": 0.96,
                "goals": [[7.03, 1.75], [2.15, -3.74], [2.80, -4.58], [1.86, 5.58]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.33,
                "init_y": 7.27,
                "init_a": -149.36,
                "velocity": 1.12,
                "goals": [[0.78, 4.76], [7.65, -7.32], [6.91, -7.12], [-6.83, 5.32]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.74,
                "init_y": -6.66,
                "init_a": -26.24,
                "velocity": 0.99,
                "goals": [[-0.12, 5.41], [7.83, -2.14], [-7.50, -3.53], [-6.55, 3.20]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.01,
                "init_y": -5.12,
                "init_a": 90.43,
                "velocity": 1.13,
                "goals": [[7.17, -6.53], [-6.46, 4.55], [3.89, -0.26], [-0.89, -2.64]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.24,
                "init_y": -7.84,
                "init_a": 104.95,
                "velocity": 0.89,
                "goals": [[6.68, 2.14], [-5.50, 0.27], [-6.12, 7.95], [-6.00, 2.28]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.44,
                "init_y": -5.49,
                "init_a": 140.56,
                "velocity": 0.96,
                "goals": [[7.10, -7.57], [-2.19, 2.39], [-5.72, -6.16], [-5.18, 1.19]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single5_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.11,
                "init_y": -7.36,
                "init_a": -81.09,
                "velocity": 1.18,
                "goals": [[-2.62, -7.26], [5.47, 5.76], [-7.54, -1.14], [1.45, -4.34]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single6_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.99,
                "init_y": 3.36,
                "init_a": 79.04,
                "velocity": 1.13,
                "goals": [[-6.13, 0.19], [0.03, -7.83], [5.47, 4.47], [-6.72, 5.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single7_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.24,
                "init_y": -1.23,
                "init_a": 35.14,
                "velocity": 0.84,
                "goals": [[5.86, 5.22], [3.52, -7.77], [2.75, 0.41], [-2.83, 3.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single8_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.41,
                "init_y": -3.16,
                "init_a": -168.73,
                "velocity": 1.03,
                "goals": [[7.27, 7.44], [2.97, -6.72], [-0.94, -1.92], [3.54, 3.44]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single9_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.46,
                "init_y": 7.46,
                "init_a": 73.39,
                "velocity": 0.92,
                "goals": [[4.10, 3.79], [0.04, -1.33], [-4.94, -5.20], [-0.40, 5.93]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single10_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.69,
                "init_y": 4.83,
                "init_a": 123.20,
                "velocity": 1.17,
                "goals": [[-1.58, 6.47], [-6.07, -4.76], [6.81, 3.91], [-4.12, 6.42]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single11_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.65,
                "init_y": 3.51,
                "init_a": 49.80,
                "velocity": 0.93,
                "goals": [[0.58, -1.93], [-3.61, 0.74], [3.57, 5.67], [4.89, 3.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single12_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.33,
                "init_y": 1.90,
                "init_a": 58.59,
                "velocity": 1.13,
                "goals": [[-4.38, -6.75], [-3.74, -4.76], [-3.24, -1.41], [-2.42, 2.32]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single13',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.23,
                "init_y": 5.30,
                "init_a": -96.14,
                "velocity": 0.87,
                "goal_x": 4.97,
                "goal_y": -5.15,
                "n_actors": 24,
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
    
    n_actors = 24
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.20,
                "init_y": -7.16,
                "init_a": 54.41,
                "velocity": 1.18,
                "goals": [[-2.20, -7.16], [-1.35, -4.67], [1.48, -0.50], [-7.04, 3.86]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.03,
                "init_y": -7.71,
                "init_a": 54.41,
                "velocity": 1.18,
                "goals": [[-2.20, -7.16], [-1.88, 7.91], [-7.57, -5.47], [3.63, -0.72]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.10,
                "init_y": 3.93,
                "init_a": 45.97,
                "velocity": 1.05,
                "goals": [[1.10, 3.93], [0.79, 5.12], [-1.94, -1.74], [-7.82, 0.47]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.08,
                "init_y": 4.93,
                "init_a": 45.97,
                "velocity": 1.05,
                "goals": [[1.10, 3.93], [3.32, -0.81], [-0.29, -0.61], [5.52, -5.61]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.57,
                "init_y": 2.27,
                "init_a": -149.25,
                "velocity": 1.17,
                "goals": [[-5.57, 2.27], [1.23, 6.08], [2.61, -3.37], [7.63, 6.83]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.49,
                "init_y": 2.66,
                "init_a": -149.25,
                "velocity": 1.17,
                "goals": [[-5.57, 2.27], [0.39, 0.93], [-7.11, 1.70], [-0.22, 2.13]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.58,
                "init_y": -2.35,
                "init_a": -44.72,
                "velocity": 1.17,
                "goal_x": -0.45,
                "goal_y": -3.93,
                "n_actors": 24,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.34,
                "init_y": -0.62,
                "init_a": -90.31,
                "velocity": 1.05,
                "goals": [[-5.83, 4.69], [0.45, -2.85], [3.00, 6.18], [-7.12, 1.85]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.35,
                "init_y": -5.39,
                "init_a": -101.54,
                "velocity": 1.18,
                "goal_x": -3.41,
                "goal_y": 4.82,
                "n_actors": 24,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.53,
                "init_y": -2.97,
                "init_a": 67.92,
                "velocity": 0.81,
                "goals": [[-2.71, 5.16], [0.31, 4.85], [7.66, -4.22], [-0.27, 2.88]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.74,
                "init_y": 3.63,
                "init_a": -50.92,
                "velocity": 1.18,
                "goals": [[1.93, 6.65], [0.36, -5.00], [-0.25, 7.27], [-0.07, 4.94]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single5_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.95,
                "init_y": 0.55,
                "init_a": -52.86,
                "velocity": 1.00,
                "goals": [[4.10, -3.66], [-3.89, 6.25], [2.94, 6.72], [-0.47, -0.83]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single6_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.78,
                "init_y": 4.79,
                "init_a": -50.02,
                "velocity": 1.11,
                "goals": [[-6.69, 6.20], [-2.26, 4.37], [2.06, 4.25], [-6.77, 5.47]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single7_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.98,
                "init_y": -0.84,
                "init_a": -46.38,
                "velocity": 1.13,
                "goals": [[-2.82, -1.92], [2.25, 7.86], [7.42, 7.79], [2.00, 5.06]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single8_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.40,
                "init_y": -5.00,
                "init_a": -145.56,
                "velocity": 0.88,
                "goals": [[2.81, -6.97], [-7.03, -0.76], [1.18, 6.62], [-6.38, -3.66]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single9_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.22,
                "init_y": -4.82,
                "init_a": -97.82,
                "velocity": 0.82,
                "goals": [[4.88, 6.27], [-2.34, 6.33], [5.10, 5.26], [0.89, -0.06]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single10_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.67,
                "init_y": -6.74,
                "init_a": -150.23,
                "velocity": 1.10,
                "goals": [[-2.40, 1.47], [-4.44, -1.40], [-3.89, -7.60], [-5.18, 2.66]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single11_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.75,
                "init_y": 5.91,
                "init_a": 65.27,
                "velocity": 0.81,
                "goals": [[4.88, -1.30], [1.16, 7.44], [3.95, 6.92], [7.11, -0.64]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single12_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.95,
                "init_y": 5.62,
                "init_a": 92.42,
                "velocity": 1.06,
                "goals": [[-5.89, -5.93], [2.26, -1.63], [4.56, 3.73], [4.74, 6.94]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single13',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.02,
                "init_y": 6.60,
                "init_a": -157.14,
                "velocity": 0.83,
                "goal_x": -4.15,
                "goal_y": 6.14,
                "n_actors": 24,
            },
        },
        {
            "name": 'actor_single14_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.43,
                "init_y": -1.16,
                "init_a": -14.85,
                "velocity": 0.86,
                "goals": [[-6.64, 0.91], [-6.12, 3.67], [-4.81, 2.89], [-5.39, -0.08]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single15_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.56,
                "init_y": 5.44,
                "init_a": -60.82,
                "velocity": 1.16,
                "goals": [[6.08, -2.24], [-6.01, -0.71], [4.33, -2.37], [-4.24, -4.34]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single16_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.95,
                "init_y": 7.20,
                "init_a": -15.22,
                "velocity": 1.10,
                "goals": [[-6.25, 3.69], [5.86, 5.56], [3.34, -5.92], [7.20, -3.37]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single17_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.85,
                "init_y": 0.50,
                "init_a": -169.94,
                "velocity": 1.04,
                "goals": [[-6.73, 5.85], [-7.27, -0.94], [6.90, -6.42], [-4.79, -4.25]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)
