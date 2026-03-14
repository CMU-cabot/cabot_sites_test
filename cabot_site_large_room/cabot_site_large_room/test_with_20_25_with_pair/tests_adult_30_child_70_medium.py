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
    
    n_actors = 24
    
    _setup_actors(tester, actors=[
        {
            "name": 'actor_pair0_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.26,
                "init_y": -4.46,
                "init_a": -133.08,
                "velocity": 0.99,
                "goal_x": 2.02,
                "goal_y": 1.94,
                "n_actors": 24,
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
                "goals": [[2.02, 1.94], [-2.37, 5.91], [-5.21, 6.51], [-0.33, 2.90]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.26, 1.04], [7.57, -6.83], [-7.39, 5.32], [1.33, 4.49]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.26, 1.04], [7.19, -4.46], [3.60, -7.45], [0.67, -3.81]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[6.96, -3.50], [-0.87, 4.74], [4.40, -6.79], [-2.79, -5.08]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "init_x": 4.92,
                "init_y": -5.99,
                "init_a": -12.16,
                "velocity": 1.06,
                "goals": [[6.96, -3.50], [2.24, -1.61], [-5.29, 7.89], [4.53, -7.95]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair3_0_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 5.58,
                "init_y": 0.39,
                "init_a": 28.31,
                "velocity": 1.17,
                "goals": [[-0.96, -3.37], [6.61, 2.41], [2.22, 0.01], [-7.64, 7.30]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair3_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.87,
                "init_y": -0.56,
                "init_a": 28.31,
                "velocity": 1.17,
                "goal_x": -0.96,
                "goal_y": -3.37,
                "n_actors": 24,
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
                "goals": [[7.03, 1.75], [-0.19, -0.03], [4.35, 3.07], [3.55, 4.89]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.03, 1.75], [-7.16, -7.23], [-1.51, -5.63], [-1.58, -6.48]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "init_x": -1.33,
                "init_y": 7.27,
                "init_a": -149.36,
                "velocity": 1.12,
                "goal_x": 0.78,
                "goal_y": 4.76,
                "n_actors": 24,
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
                "goals": [[-0.12, 5.41], [-5.94, 4.46], [7.56, -5.25], [7.56, -6.82]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[7.17, -6.53], [4.91, -6.26], [0.48, -1.45], [-3.71, 5.04]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 5.24,
                "init_y": -7.84,
                "init_a": 104.95,
                "velocity": 0.89,
                "goal_x": 6.68,
                "goal_y": 2.14,
                "n_actors": 24,
            },
        },
        {
            "name": 'actor_single4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.44,
                "init_y": -5.49,
                "init_a": 140.56,
                "velocity": 0.96,
                "goal_x": 7.10,
                "goal_y": -7.57,
                "n_actors": 24,
            },
        },
        {
            "name": 'actor_single5',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -1.11,
                "init_y": -7.36,
                "init_a": -81.09,
                "velocity": 1.18,
                "goal_x": -2.62,
                "goal_y": -7.26,
                "n_actors": 24,
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
                "goals": [[-6.13, 0.19], [5.26, -5.05], [-7.27, 4.30], [-3.54, 7.72]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[5.86, 5.22], [-4.31, 5.32], [-0.89, -1.64], [2.97, -4.55]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single8',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.41,
                "init_y": -3.16,
                "init_a": -168.73,
                "velocity": 1.03,
                "goal_x": 7.27,
                "goal_y": 7.44,
                "n_actors": 24,
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
                "goals": [[4.10, 3.79], [-3.72, 0.01], [0.92, -5.43], [0.92, 0.45]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-1.58, 6.47], [-0.79, 6.21], [7.41, 5.21], [6.70, -2.88]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single11',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.65,
                "init_y": 3.51,
                "init_a": 49.80,
                "velocity": 0.93,
                "goal_x": 0.58,
                "goal_y": -1.93,
                "n_actors": 24,
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
                "goals": [[-4.38, -6.75], [4.28, 4.48], [6.88, -5.93], [-4.52, -4.17]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single13_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.23,
                "init_y": 5.30,
                "init_a": -96.14,
                "velocity": 0.87,
                "goals": [[4.97, -5.15], [-6.78, -6.31], [6.98, -4.06], [2.13, -2.56]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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

def tests_adult_30_child_70_test_case_02_stopped_medium(tester):
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
                "goals": [[-2.20, -7.16], [-6.23, 1.79], [-1.36, -4.28], [5.07, -0.56]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -3.03,
                "init_y": -7.71,
                "init_a": 54.41,
                "velocity": 1.18,
                "goal_x": -2.20,
                "goal_y": -7.16,
                "n_actors": 24,
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
                "goals": [[1.10, 3.93], [0.87, 5.42], [5.35, -7.12], [2.84, 0.32]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[1.10, 3.93], [7.99, 3.23], [-7.66, -7.64], [-7.43, 3.61]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair2_0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -5.57,
                "init_y": 2.27,
                "init_a": -149.25,
                "velocity": 1.17,
                "goal_x": -5.57,
                "goal_y": 2.27,
                "n_actors": 24,
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
                "goals": [[-5.57, 2.27], [3.66, -6.84], [-1.70, -0.86], [-2.24, -3.42]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "init_x": 2.58,
                "init_y": -2.35,
                "init_a": -44.72,
                "velocity": 1.17,
                "goals": [[-0.45, -3.93], [-2.32, -6.19], [-0.19, 2.15], [-2.26, -0.87]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "init_x": 3.34,
                "init_y": -0.62,
                "init_a": -90.31,
                "velocity": 1.05,
                "goals": [[-5.83, 4.69], [-1.01, -1.28], [-6.83, 0.57], [-2.24, -2.58]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "init_x": 5.35,
                "init_y": -5.39,
                "init_a": -101.54,
                "velocity": 1.18,
                "goals": [[-3.41, 4.82], [1.36, 5.84], [-1.91, -4.30], [1.07, -2.37]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 2.53,
                "init_y": -2.97,
                "init_a": 67.92,
                "velocity": 0.81,
                "goal_x": -2.71,
                "goal_y": 5.16,
                "n_actors": 24,
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
                "goals": [[1.93, 6.65], [5.93, 4.49], [-0.91, -0.26], [-3.89, 1.54]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.10, -3.66], [-5.12, 2.28], [1.92, 5.35], [-4.24, -4.53]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-6.69, 6.20], [-7.86, 6.40], [1.56, -7.15], [3.66, -6.74]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single7',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 6.98,
                "init_y": -0.84,
                "init_a": -46.38,
                "velocity": 1.13,
                "goal_x": -2.82,
                "goal_y": -1.92,
                "n_actors": 24,
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
                "goals": [[2.81, -6.97], [-1.00, -6.03], [-6.47, 2.37], [0.30, -1.30]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single9',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 7.22,
                "init_y": -4.82,
                "init_a": -97.82,
                "velocity": 0.82,
                "goal_x": 4.88,
                "goal_y": 6.27,
                "n_actors": 24,
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
                "goals": [[-2.40, 1.47], [-6.67, -4.70], [-7.94, -7.60], [5.17, 5.55]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[4.88, -1.30], [-3.91, -1.96], [1.19, 6.21], [3.38, -0.75]],
                "change_interval_min": 3,
                "change_interval_max": 5,
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
                "goals": [[-5.89, -5.93], [-0.01, -6.62], [-3.96, -4.45], [5.60, 1.08]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single13_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.02,
                "init_y": 6.60,
                "init_a": -157.14,
                "velocity": 0.83,
                "goals": [[-4.15, 6.14], [-4.37, -6.70], [-4.63, 2.10], [7.43, 7.07]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single14',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -0.43,
                "init_y": -1.16,
                "init_a": -14.85,
                "velocity": 0.86,
                "goal_x": -6.64,
                "goal_y": 0.91,
                "n_actors": 24,
            },
        },
        {
            "name": 'actor_single15',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -7.56,
                "init_y": 5.44,
                "init_a": -60.82,
                "velocity": 1.16,
                "goal_x": 6.08,
                "goal_y": -2.24,
                "n_actors": 24,
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
                "goals": [[-6.25, 3.69], [2.12, 4.05], [-5.35, -3.58], [-5.38, 1.87]],
                "change_interval_min": 3,
                "change_interval_max": 5,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single17',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -6.85,
                "init_y": 0.50,
                "init_a": -169.94,
                "velocity": 1.04,
                "goal_x": -6.73,
                "goal_y": 5.85,
                "n_actors": 24,
            },
        },
    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)
