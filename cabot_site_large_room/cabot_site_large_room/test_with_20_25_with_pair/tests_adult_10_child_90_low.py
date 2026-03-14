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

def tests_adult_10_child_90_test_case_01_walking_low(tester):
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
                "goals": [[2.02, 1.94], [-6.44, 7.60], [-0.67, -0.45], [0.00, -6.31]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[2.02, 1.94], [-6.69, -0.41], [7.27, -1.81], [-1.80, 6.82]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-1.26, 1.04], [4.72, -3.35], [-0.97, -6.24], [3.02, -1.29]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-1.26, 1.04], [-4.71, 1.97], [-2.43, 1.85], [-5.00, -3.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[6.96, -3.50], [5.34, -5.56], [7.72, 3.13], [-4.47, 7.16]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-0.96, -3.37], [-5.70, 7.13], [0.23, -0.05], [-6.12, -7.31]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[7.03, 1.75], [2.51, 0.88], [4.37, 6.47], [1.11, -0.07]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[7.03, 1.75], [-7.93, -2.04], [3.22, -0.03], [7.66, -6.98]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[0.78, 4.76], [4.38, 2.54], [-1.11, -3.66], [-3.54, -2.57]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-0.12, 5.41], [0.46, 1.24], [-0.98, -2.06], [-4.10, -6.94]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[7.17, -6.53], [-7.33, -2.58], [-7.91, 7.32], [7.36, 4.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[6.68, 2.14], [5.30, 6.66], [2.47, 5.12], [7.84, -4.51]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[7.10, -7.57], [-5.32, -3.97], [-7.97, 7.08], [-0.04, 6.29]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-2.62, -7.26], [6.19, 2.29], [3.56, 5.89], [-0.09, 2.01]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-6.13, 0.19], [5.51, -2.06], [-7.40, -0.08], [5.46, -1.63]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[5.86, 5.22], [4.57, 2.12], [6.39, 2.15], [5.72, 0.65]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[7.27, 7.44], [-3.18, 5.76], [-2.89, -5.64], [-4.80, -0.51]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[4.10, 3.79], [4.22, 3.91], [-5.86, 3.37], [-0.87, -4.99]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-1.58, 6.47], [3.81, -5.63], [7.38, 0.54], [5.53, -2.16]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[0.58, -1.93], [1.74, -1.14], [2.06, 5.23], [-1.81, 5.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-4.38, -6.75], [3.24, -1.68], [-0.56, 4.44], [2.00, -6.65]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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

def tests_adult_10_child_90_test_case_02_stopped_low(tester):
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
                "goals": [[-2.20, -7.16], [1.97, -0.61], [-5.81, 5.10], [1.35, 7.82]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-2.20, -7.16], [-4.94, -5.20], [1.90, 4.80], [-0.24, -7.87]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[1.10, 3.93], [-7.56, 6.95], [-2.91, 6.67], [-0.23, 6.08]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[1.10, 3.93], [4.66, 2.48], [7.63, 6.02], [-3.73, 1.22]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-5.57, 2.27], [-5.10, 2.98], [1.42, 4.01], [7.62, -4.90]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-5.57, 2.27], [-1.10, -3.23], [-4.76, -0.39], [-1.39, 7.02]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-5.83, 4.69], [3.14, 6.33], [-6.37, -3.99], [-5.56, 7.31]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-2.71, 5.16], [4.05, -0.50], [6.52, -2.45], [-4.69, -4.45]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[1.93, 6.65], [-0.59, 2.89], [5.24, 4.87], [7.17, -3.16]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[4.10, -3.66], [-3.01, 2.71], [-1.20, 7.61], [6.08, 7.03]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-6.69, 6.20], [7.77, -2.17], [-1.70, 7.96], [-7.01, -1.32]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-2.82, -1.92], [7.94, -3.75], [4.84, 0.80], [5.08, 3.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[2.81, -6.97], [-1.76, 0.96], [5.11, 6.76], [-3.27, 6.54]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[4.88, 6.27], [4.94, 0.49], [1.81, -0.99], [6.21, 0.02]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-2.40, 1.47], [-2.46, -0.18], [-4.13, 0.58], [2.67, 1.95]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[4.88, -1.30], [6.88, -4.71], [-4.49, -5.47], [6.37, 4.72]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-5.89, -5.93], [-4.57, 2.63], [-3.07, -1.91], [2.22, 4.88]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-6.64, 0.91], [0.72, 5.40], [2.92, 5.69], [-6.11, 2.35]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[6.08, -2.24], [4.69, 4.07], [5.13, -5.19], [0.93, -3.52]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-6.25, 3.69], [4.97, -5.40], [-0.39, 6.09], [-6.12, -5.50]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
                "goals": [[-6.73, 5.85], [3.45, -3.32], [-5.20, -1.73], [4.04, 6.36]],
                "change_interval_min": 7,
                "change_interval_max": 9,
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
