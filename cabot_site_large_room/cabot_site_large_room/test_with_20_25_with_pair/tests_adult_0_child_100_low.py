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
                "goals": [[5.80, -6.54], [0.59, 3.76], [-2.75, -1.00], [3.47, -0.40]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[5.80, -6.54], [-3.20, 1.71], [-3.63, -0.80], [-4.53, -5.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.01,
                "init_y": 2.38,
                "init_a": -35.67,
                "velocity": 1.03,
                "goals": [[-2.01, 2.38], [-5.96, -4.73], [6.15, 7.87], [-4.82, -0.80]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
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
                "goals": [[-2.01, 2.38], [6.68, -6.03], [-5.79, 3.59], [-0.57, -6.44]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.19,
                "init_y": 3.79,
                "init_a": 75.06,
                "velocity": 1.17,
                "goals": [[-7.19, 3.79], [6.67, -4.22], [-0.32, 5.19], [7.16, 3.70]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.41,
                "init_y": 2.82,
                "init_a": 75.06,
                "velocity": 1.17,
                "goals": [[-7.19, 3.79], [3.18, 1.43], [-5.11, -3.85], [1.34, 0.49]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
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
                "goals": [[6.32, 6.82], [-7.23, -3.93], [6.60, 1.32], [4.70, 7.42]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.70,
                "init_y": 6.04,
                "init_a": 49.07,
                "velocity": 0.96,
                "goals": [[-5.93, 5.80], [-6.04, -0.29], [-6.90, -3.07], [1.66, -1.39]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.94,
                "init_y": 1.49,
                "init_a": -152.90,
                "velocity": 1.02,
                "goals": [[-1.73, -3.09], [3.72, -7.30], [-4.66, 7.29], [6.24, -6.30]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.34,
                "init_y": -7.43,
                "init_a": 134.43,
                "velocity": 1.04,
                "goals": [[-7.91, 1.27], [-7.99, -1.23], [-0.50, -5.11], [1.32, 4.61]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.26,
                "init_y": -4.79,
                "init_a": 154.56,
                "velocity": 1.14,
                "goals": [[0.62, 7.81], [-3.05, -3.58], [4.80, -0.80], [-7.57, -7.84]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single5_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.51,
                "init_y": -7.38,
                "init_a": -46.68,
                "velocity": 0.87,
                "goals": [[7.14, -6.16], [-6.99, -1.27], [-3.35, -5.69], [-3.94, -2.86]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
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
                "goals": [[0.30, -2.25], [-4.14, 2.18], [2.83, -0.98], [-6.05, 7.08]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-5.94, -6.06], [0.43, 3.89], [-7.57, -4.16], [-4.60, 0.69]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single8_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -0.40,
                "init_y": 2.40,
                "init_a": -84.72,
                "velocity": 1.13,
                "goals": [[-6.46, 6.40], [-3.04, 1.92], [-1.64, -6.16], [-5.42, -2.91]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single9_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.41,
                "init_y": -6.14,
                "init_a": -107.50,
                "velocity": 1.02,
                "goals": [[6.33, -6.27], [-7.90, -5.15], [-6.35, -4.70], [2.35, -1.10]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single10_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.78,
                "init_y": 0.46,
                "init_a": -8.96,
                "velocity": 1.11,
                "goals": [[-3.97, 6.42], [-0.79, 2.68], [1.87, -3.90], [-5.44, -2.14]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single11_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.72,
                "init_y": -1.75,
                "init_a": 0.87,
                "velocity": 1.02,
                "goals": [[3.30, 5.07], [6.70, -0.54], [-0.19, 3.14], [0.56, 1.10]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single12_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.54,
                "init_y": -5.98,
                "init_a": -113.74,
                "velocity": 1.11,
                "goals": [[4.55, -3.94], [1.27, 7.62], [-6.39, 5.40], [-0.57, 5.07]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single13_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.13,
                "init_y": 5.90,
                "init_a": -157.42,
                "velocity": 0.85,
                "goals": [[-6.47, 5.85], [-4.37, -4.53], [-0.47, -4.54], [-4.03, -2.05]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
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
    
    n_actors = 25
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 0.00,
                "init_y": 4.44,
                "init_a": 23.06,
                "velocity": 1.15,
                "goals": [[6.16, 1.57], [7.51, -4.07], [7.17, -2.76], [-6.81, -2.71]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
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
                "goals": [[6.16, 1.57], [7.60, -0.77], [4.82, 6.69], [2.79, 6.03]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[4.38, 6.04], [-0.53, -2.12], [2.18, 1.00], [0.98, -4.63]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -2.74,
                "init_y": 0.37,
                "init_a": 177.69,
                "velocity": 0.82,
                "goals": [[4.38, 6.04], [-5.33, -7.67], [2.48, -0.25], [6.23, -7.26]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -7.76,
                "init_y": -5.20,
                "init_a": -37.15,
                "velocity": 0.95,
                "goals": [[0.24, 1.19], [-4.88, -4.46], [-7.18, -5.14], [-3.21, 3.17]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -8.55,
                "init_y": -4.60,
                "init_a": -37.15,
                "velocity": 0.95,
                "goals": [[0.24, 1.19], [7.08, 1.32], [5.40, -7.64], [-0.75, 7.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
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
                "goals": [[7.77, 5.56], [-2.86, -2.46], [-0.75, -1.41], [4.13, -6.81]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.75,
                "init_y": 1.54,
                "init_a": 178.38,
                "velocity": 1.11,
                "goals": [[-2.78, -3.81], [7.70, -1.55], [1.73, 6.71], [-1.17, 6.58]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.85,
                "init_y": -0.94,
                "init_a": -165.89,
                "velocity": 0.92,
                "goals": [[-1.77, -0.38], [-5.19, 7.62], [-6.59, -4.85], [-3.63, 5.55]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.98,
                "init_y": -6.67,
                "init_a": -132.79,
                "velocity": 0.87,
                "goals": [[-7.23, 2.53], [3.70, 7.79], [2.98, 0.54], [5.85, 7.10]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single4_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.21,
                "init_y": -5.66,
                "init_a": -96.25,
                "velocity": 1.02,
                "goals": [[-2.61, -3.68], [0.35, -3.88], [1.01, -2.39], [3.82, -6.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
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
                "goals": [[-3.00, -2.87], [2.47, 1.27], [6.42, 6.46], [-7.64, 6.80]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single6_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.85,
                "init_y": -7.99,
                "init_a": 44.99,
                "velocity": 1.02,
                "goals": [[3.85, 3.41], [3.24, -0.15], [0.48, 3.02], [-6.20, -0.53]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single7_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -6.54,
                "init_y": 3.59,
                "init_a": 149.78,
                "velocity": 1.17,
                "goals": [[6.19, -0.62], [7.37, 7.67], [4.93, 0.06], [5.76, 7.91]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
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
                "goals": [[-0.66, -1.80], [0.09, -6.64], [1.35, -3.90], [-4.56, -6.05]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single9_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 1.27,
                "init_y": 3.83,
                "init_a": 115.72,
                "velocity": 0.94,
                "goals": [[-0.42, 5.25], [-3.46, -7.89], [5.98, 6.68], [3.79, -3.24]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single10_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 2.38,
                "init_y": 7.99,
                "init_a": 143.32,
                "velocity": 1.08,
                "goals": [[-2.08, -3.25], [6.03, -3.34], [4.38, 0.31], [6.88, -3.79]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
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
                "goals": [[8.00, 3.23], [1.85, 2.12], [5.88, -5.34], [-7.12, 3.96]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single12_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.02,
                "init_y": -1.38,
                "init_a": -113.06,
                "velocity": 0.87,
                "goals": [[-1.71, 6.33], [0.48, 4.06], [6.16, -2.97], [-2.25, -4.98]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single13_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 3.91,
                "init_y": 1.03,
                "init_a": -53.04,
                "velocity": 0.86,
                "goals": [[4.89, -4.30], [-1.44, 3.77], [3.92, -0.69], [-5.62, -1.14]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
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
                "goals": [[-7.83, 3.25], [5.34, -1.91], [-0.04, 0.71], [4.97, 0.39]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single15_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 7.50,
                "init_y": 5.93,
                "init_a": 83.00,
                "velocity": 1.20,
                "goals": [[3.54, -2.19], [6.45, -2.46], [5.85, -7.69], [-3.57, -1.10]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single16_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -1.86,
                "init_y": -5.75,
                "init_a": 77.65,
                "velocity": 0.82,
                "goals": [[-2.86, 5.74], [-2.38, 2.81], [-6.93, -0.74], [-2.62, 3.03]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single17_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.67,
                "init_y": 6.94,
                "init_a": 48.78,
                "velocity": 1.06,
                "goals": [[1.34, 1.43], [-5.29, -2.28], [7.24, 3.66], [3.46, -6.88]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single18_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 4.16,
                "init_y": 5.30,
                "init_a": -128.41,
                "velocity": 1.20,
                "goals": [[-3.11, -2.12], [-3.59, -3.75], [3.03, 0.42], [7.92, 7.89]],
                "change_interval_min": 7,
                "change_interval_max": 9,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)
