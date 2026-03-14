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

def tests_adult_20_child_80_test_case_01_stopped_high(tester):
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
                "goals": [[5.80, -6.54], [3.59, 3.78], [1.71, 2.90], [-2.67, -2.48]],
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
                "goals": [[5.80, -6.54], [-0.70, -1.52], [4.92, -0.34], [-4.43, -4.10]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.01, 2.38], [-1.98, -3.75], [-6.14, 1.08], [1.17, -4.52]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.01, 2.38], [1.12, -6.14], [7.53, -7.62], [3.82, -7.19]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-7.19, 3.79], [7.13, -0.61], [-7.47, 1.15], [-6.24, 0.45]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-7.19, 3.79], [5.05, 2.46], [-2.76, 6.78], [6.06, -1.11]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[6.32, 6.82], [-5.10, 7.21], [4.37, 0.41], [-0.24, 2.87]],
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
            "name": 'actor_single3_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -5.34,
                "init_y": -7.43,
                "init_a": 134.43,
                "velocity": 1.04,
                "goals": [[-7.91, 1.27], [-7.98, -7.01], [7.32, -0.91], [6.87, -3.18]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[0.62, 7.81], [7.85, 5.20], [-3.41, 7.57], [1.08, -0.49]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[7.14, -6.16], [1.37, 7.70], [-2.43, 0.88], [2.99, 5.43]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single6',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.35,
                "init_y": 5.81,
                "init_a": -165.16,
                "velocity": 0.84,
                "goal_x": 0.30,
                "goal_y": -2.25,
                "n_actors": 20,
            },
        },
        {
            "name": 'actor_single7',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.66,
                "init_y": 2.73,
                "init_a": 122.94,
                "velocity": 0.82,
                "goal_x": -5.94,
                "goal_y": -6.06,
                "n_actors": 20,
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
                "goals": [[-6.46, 6.40], [-2.48, 3.82], [-4.86, 2.10], [-1.65, -5.01]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[6.33, -6.27], [0.18, -3.29], [4.01, -7.30], [-4.57, -2.81]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-3.97, 6.42], [3.88, 1.59], [-1.60, -6.06], [4.44, 0.32]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[3.30, 5.07], [-2.36, -1.90], [-6.83, 6.34], [-2.39, -4.87]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[4.55, -3.94], [-7.21, 6.35], [6.89, -4.60], [4.40, -2.91]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-6.47, 5.85], [-1.22, 4.34], [5.63, 6.61], [3.36, 5.74]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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

def tests_adult_20_child_80_test_case_02_walking_high(tester):
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
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.00,
                "init_y": 4.45,
                "init_a": 23.06,
                "velocity": 1.15,
                "goal_x": 6.16,
                "goal_y": 1.57,
                "n_actors": 25,
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
                "goals": [[4.38, 6.04], [7.05, -6.21], [-6.65, 1.38], [5.25, -5.82]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[4.38, 6.04], [0.81, 7.82], [7.80, 7.79], [2.90, -3.83]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[0.24, 1.19], [5.66, 0.87], [-0.22, 4.13], [2.86, -2.60]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[0.24, 1.19], [5.65, 4.08], [6.37, 0.64], [6.08, -3.80]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[7.77, 5.56], [6.21, -1.07], [5.01, -0.34], [7.00, 7.29]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.78, -3.81], [-1.52, 2.58], [7.18, 6.49], [2.65, 7.48]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-1.77, -0.38], [3.23, 7.65], [1.93, -5.85], [-2.68, -3.65]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-7.23, 2.53], [-7.72, 7.78], [-6.04, 4.24], [0.64, 4.42]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.61, -3.68], [-1.76, -2.68], [-4.56, 3.49], [5.77, -7.26]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-3.00, -2.87], [-2.77, -7.89], [6.84, -5.91], [5.13, 4.38]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[3.85, 3.41], [1.56, -0.23], [-1.21, -4.61], [5.42, 7.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
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
                "goals": [[-0.66, -1.80], [0.07, -4.79], [-6.30, -5.49], [-3.03, -7.49]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-0.42, 5.25], [-3.09, -2.47], [4.25, -2.87], [7.36, -3.75]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.08, -3.25], [7.19, 5.53], [7.06, 2.55], [1.73, -1.72]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[8.00, 3.23], [1.42, 7.74], [4.25, -7.03], [7.96, -4.42]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-1.71, 6.33], [-7.86, -5.24], [5.20, -4.99], [5.63, 4.06]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[4.89, -4.30], [4.87, -6.73], [-6.73, -3.34], [3.54, 0.45]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single14',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.97,
                "init_y": -5.29,
                "init_a": 100.82,
                "velocity": 0.94,
                "goal_x": -7.83,
                "goal_y": 3.25,
                "n_actors": 25,
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
                "goals": [[3.54, -2.19], [-2.36, 7.31], [-4.71, -6.89], [-2.07, -7.82]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-2.86, 5.74], [-2.95, 3.01], [-1.19, -7.00], [2.27, 2.53]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[1.34, 1.43], [1.74, -7.69], [-7.61, -7.00], [-3.62, 4.76]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 25,
                "random_seed": 100,
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
