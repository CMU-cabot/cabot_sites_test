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

def tests_adult_50_child_50_test_case_01_stopped_high(tester):
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
                "goals": [[5.80, -6.54], [-2.49, 7.26], [-5.26, 5.49], [-5.90, 0.66]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair0_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 4.97,
                "init_y": -5.99,
                "init_a": 144.90,
                "velocity": 1.11,
                "goal_x": 5.80,
                "goal_y": -6.54,
                "n_actors": 20,
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
                "goals": [[-2.01, 2.38], [-3.62, -6.98], [3.35, -3.41], [-5.72, -5.18]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.79,
                "init_y": 1.76,
                "init_a": -35.67,
                "velocity": 1.03,
                "goal_x": -2.01,
                "goal_y": 2.38,
                "n_actors": 20,
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
                "goals": [[6.32, 6.82], [-3.27, 7.00], [1.78, 7.29], [-7.75, -6.27]],
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
            "name": 'actor_single2_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -3.94,
                "init_y": 1.49,
                "init_a": -152.90,
                "velocity": 1.02,
                "goals": [[-1.73, -3.09], [-0.58, -6.25], [3.91, 0.89], [-1.70, 4.10]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[-7.91, 1.27], [-7.65, -0.42], [-5.24, 6.07], [-0.58, -3.14]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
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
            "name": 'actor_single5_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": -4.51,
                "init_y": -7.38,
                "init_a": -46.68,
                "velocity": 0.87,
                "goals": [[7.14, -6.16], [6.04, -7.99], [-0.85, 6.45], [7.32, 7.62]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[0.30, -2.25], [6.81, -0.80], [-2.71, -7.78], [-2.34, -2.25]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
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
                "goals": [[-6.46, 6.40], [-2.36, 3.81], [-7.06, 6.79], [5.14, 7.72]],
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
                "goals": [[6.33, -6.27], [-3.84, -6.34], [2.94, 3.71], [1.86, -0.30]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 20,
                "random_seed": 100,
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
            "name": 'actor_single13_child',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "radius": 0.25,
                "init_x": 6.13,
                "init_y": 5.90,
                "init_a": -157.42,
                "velocity": 0.85,
                "goals": [[-6.47, 5.85], [-0.34, 1.05], [0.17, -6.50], [3.35, 6.11]],
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

def tests_adult_50_child_50_test_case_02_stopped_high(tester):
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
                "goals": [[-2.20, -7.16], [-5.98, 6.34], [-4.14, -6.47], [5.88, -3.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[1.10, 3.93], [-0.21, 3.85], [2.35, 4.06], [2.75, 6.54]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_pair1_1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 1.08,
                "init_y": 4.93,
                "init_a": 45.97,
                "velocity": 1.05,
                "goal_x": 1.10,
                "goal_y": 3.93,
                "n_actors": 24,
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
                "goals": [[-5.57, 2.27], [-3.10, -4.35], [2.77, 7.87], [-0.96, -3.69]],
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
                "init_x": 2.58,
                "init_y": -2.35,
                "init_a": -44.72,
                "velocity": 1.17,
                "goals": [[-0.45, -3.93], [3.62, 5.67], [4.10, 3.37], [-3.31, -5.60]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": 3.34,
                "init_y": -0.62,
                "init_a": -90.31,
                "velocity": 1.05,
                "goal_x": -5.83,
                "goal_y": 4.69,
                "n_actors": 24,
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
                "goals": [[-3.41, 4.82], [-2.65, 7.99], [-6.20, 7.98], [-5.49, 2.90]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[1.93, 6.65], [3.03, -4.35], [-3.33, 4.12], [-2.59, -5.46]],
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
                "goals": [[4.10, -3.66], [0.29, 4.40], [-7.77, -4.19], [7.73, -4.90]],
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
                "goals": [[-6.69, 6.20], [-0.08, -6.41], [-7.46, -3.73], [2.62, -4.03]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
                "goals": [[2.81, -6.97], [-2.39, 4.00], [5.88, -6.22], [2.67, -7.05]],
                "change_interval_min": 1,
                "change_interval_max": 2,
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
            "name": 'actor_single10',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -2.67,
                "init_y": -6.74,
                "init_a": -150.23,
                "velocity": 1.10,
                "goal_x": -2.40,
                "goal_y": 1.47,
                "n_actors": 24,
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
                "goals": [[4.88, -1.30], [-4.45, 2.75], [-6.12, -1.30], [-2.00, 0.23]],
                "change_interval_min": 1,
                "change_interval_max": 2,
                "change_probability": 1.0,
                "velocity_range": [0.8, 1.2],
                "n_actors": 24,
                "random_seed": 100,
            },
        },
        {
            "name": 'actor_single12',
            "module": "pedestrian.walk_sfm",
            "params": {
                "radius": 0.25,
                "init_x": -4.95,
                "init_y": 5.62,
                "init_a": 92.42,
                "velocity": 1.06,
                "goal_x": -5.89,
                "goal_y": -5.93,
                "n_actors": 24,
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
                "goals": [[-6.25, 3.69], [-2.35, -2.50], [-2.72, -1.71], [5.13, 7.30]],
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
                "goals": [[-6.73, 5.85], [6.48, -1.02], [-6.31, 2.88], [6.12, 0.43]],
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
