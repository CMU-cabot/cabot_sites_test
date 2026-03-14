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


# Converted test case using walk_sfm
def test_sfm_case1_move_towards_a_pedestrian(tester):
    # 1.1 Frontal Approace
    tester.check_collision()
    tester.reset_position()
    
    n_actors = 6
    
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": 10.0,
                "init_y": -0.5,
                "init_a": 180.0,
                "velocity": 0.5,
                "goal_x": -10.0,
                "goal_y": -0.5,
                "n_actors": n_actors,
            },
        },
        {
            "name": 'actor1',
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": 4.5,
                "init_y": -1.0,
                "init_a": 180.0,
                "velocity": 0.0,
                "goal_x": 4.5,
                "goal_y": -1.0,
                "n_actors": n_actors,
            },
        },
        {
            "name": 'actor2',
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": 6.0,
                "init_y": 6.0,
                "init_a": -90.0,
                "velocity": 0.95,
                "goal_x": 6.0,
                "goal_y": -6.0,
                "n_actors": n_actors,
            },
        },
        {
            "name": 'actor3',
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": 9.0,
                "init_y": 1.0,
                "init_a": 90.0,
                "velocity": 0.0,
                "goal_x": 9.0,
                "goal_y": 1.0,
                "n_actors": n_actors,
            },
        },
        {
            "name": 'actor4',
            "module": "pedestrian.walk_sfm",
            "params": {
                "init_x": 10.0,
                "init_y": 1.8,
                "init_a": 180.0,
                "velocity": 0.4,
                "goal_x": -10.0,
                "goal_y": 1.8,
                "n_actors": n_actors,
            },
        },
        {
            "name": 'child_actor0',
            "module": "pedestrian.walk_sfm_child",
            "params": {
                "init_x": 0.0,
                "init_y": 5.0,
                "init_a": -90.0,
                "velocity": 1.0,
                "goals": [[0.0, -5.0], [5.0, 0.0], [-5.0, 0.0]],
                "change_interval_min": 3.0,
                "change_interval_max": 8.0,
                "change_probability": 1.0,
                "velocity_range": [0.3, 1.4],
                "n_actors": n_actors,
            },
        },
    ])
    
    # Keep the same metrics as reference
    _add_metric_condition_lt(tester, "total_time", 30)
    _add_metric_condition_lt(tester, "robot_path_length", 20.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)
