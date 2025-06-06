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
    tester.config['init_x'] = 0.0
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
            "maximum_distance_to_people",
            "robot_on_person_collision_count",
            "person_on_robot_collision_count"
        ],
        # robot_radius=0.45  # default value
    )

    tester.set_people_detection_range(
        min_range=0.29, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        max_range=7.07, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
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
    # tester.wait_localization_started()
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
    """
    Adds a metric condition to the tester where the metric value must be less than the specified threshold.
    :param tester: The tester object that handles the metric conditions.
    :param metric_name: The name of the metric to evaluate.
    :param success_threshold: The threshold value that the metric must be less than to meet the condition.
    """
    condition = f"{success_threshold} > value"
    tester.add_metric_condition({"name": metric_name, "condition": condition})


def _add_metric_condition_gt(tester, metric_name, success_threshold):
    """
    Adds a metric condition to the tester where the metric value must be greater than the specified threshold.
    :param tester: The tester object that handles the metric conditions.
    :param metric_name: The name of the metric to evaluate.
    :param success_threshold: The threshold value that the metric must be greater than to meet the condition.
    """
    condition = f"{success_threshold} < value"
    tester.add_metric_condition({"name": metric_name, "condition": condition})


def _setup_actors(tester, actors):
    tester.setup_actors(actors=actors)
    time.sleep(5)


def test_category1_case1_move_towards_a_pedestrian(tester):
    # 1.1 Frontal Approace
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 10.0,
                "init_y": -0.5,
                "init_a": 180.0,
                "velocity": 0.5,
                "decel_distance": 1.5,
                "pause_distance": 1.0,
                "stop_time": 3.0
            },
        },
    ])
    _add_metric_condition_lt(tester, "total_time", 26) # 26 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 12.6) # 12.6 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case1_move_towards_and_pass_left(tester):
    # 1.1 Frontal Approach
    tester.check_collision()
    tester.reset_position()

    actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 10.0,
                "init_y": 0.0,
                "init_a": 175.0,
                "velocity": 0.75,
                "goal_x": -10.0,
                "goal_y": 0.0,
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 23) # 23 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "time_not_moving", 3.7) # 3.7 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case1_move_towards_and_pass_right(tester):
    # 1.1 Frontal Approach
    tester.check_collision()
    tester.reset_position()

    actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 10.0,
                "init_y": -1.0,
                "init_a": -175.0,
                "velocity": 0.75,
                "goal_x": -10.0,
                "goal_y": -1.0,
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 19) # 19 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_path_length", 12.0) # 12.0 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "time_not_moving", 3.8) # 3.8 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case2_move_towards_a_pedestrian_and_avoid(tester):
    # 1.2 Pedestrian Obstruction
    tester.check_collision()
    tester.reset_position(x=-5.0)

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 5.0,
                "init_y": -0.5,
                "init_a": 180.0,
                "velocity": 0.0,  # does not move
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 21) # 21 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 12.2) # 12.2 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "time_not_moving", 4.3) # 4.3 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case2_pedestrian_standing_left(tester):
    # 1.2 Pedestrian Obstruction
    tester.check_collision()
    tester.reset_position(x=-5.0)

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": -5.0,
                "init_y": 1.0,
                "init_a": 0.0,
                "velocity": 0.0,  # does not move
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 23) # 23 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 17.5) # 17.5 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "time_not_moving", 3.5) # 3.5 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case2_pedestrian_standing_right(tester):
    # 1.2 Pedestrian Obstruction
    tester.check_collision()
    tester.reset_position(x=-5.0)

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": -5.0,
                "init_y": -1.0,
                "init_a": 0.0,
                "velocity": 0.0,  # does not move
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 24) # 24 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 17.5) # 17.5 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "time_not_moving", 3.8) # 3.8 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case4_robot_overtaking(tester):
    # 1.4 robot overtaking
    tester.check_collision()
    tester.reset_position()

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 1.0,
                "init_y": -0.5,
                "init_a": 0.0,
                "velocity": 0.25,  # very slow
                "stop_time": 5.0
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_gt(tester, "total_time", 11) # 11 is the average value of 5 runs (actor's speed is set to 0 and actor's init_x is set to 5.0) with a 40% margin removed.
    _add_metric_condition_lt(tester, "total_time", 26) # 26 is the average value of 5 runs (actor's speed is set to 0 and actor's init_x is set to 5.0) with a 40% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 15.6) # 15.6 is the average value of 5 runs (actor's speed is set to 0 and actor's init_x is set to 5.0) with a 40% margin added.
    _add_metric_condition_lt(tester, "maximum_distance_to_people", 7.5) # 7.5 is the average value of 5 runs (actor's speed is set to 0 and actor's init_x is set to 5.0) with a 40% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case5_down_the_path(tester):
    # 1.5 Down path
    tester.check_collision()
    tester.reset_position()

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 1.0,
                "init_y": -0.5,
                "init_a": 0.0,
                "velocity": 0.75,
                "stop_time": 5.0
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_gt(tester, "total_time", 16) # 16 is the average value of 5 runs with a 10% margin removed.
    _add_metric_condition_lt(tester, "total_time", 20) # 20 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "cumulative_heading_changes", 0.23) # 0.23 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_gt(tester, "minimum_distance_to_people", 0.28) # 0.28 is the average value of 5 runs with a 10% margin removed.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian1(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": 5.0,
                "init_a": -90.0,
                "velocity": 0.95,
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 22) # 22 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian2(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": -5.0,
                "init_a": 90.0,
                "velocity": 0.95,
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 21) # 21 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian3(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": 6.0,
                "init_a": -90.0,
                "velocity": 0.95,
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 22) # 22 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian4(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()

    actors = [
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": -6.0,
                "init_a": 90.0,
                "velocity": 0.95,
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 22) # 22 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian5(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()

    actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": 7.0,
                "init_a": -90.0,
                "velocity": 0.95,
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 18) # 18 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian6(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()

    actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": -7.0,
                "init_a": 90.0,
                "velocity": 0.95,
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 23) # 23 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category3_case1_move_across_a_pedestrian_proceed(tester):
    # 2.3 Intersection Proceed
    tester.check_collision()
    tester.reset_position()

    actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": 5.0,
                "init_y": 5.0,
                "init_a": -90.0,
                "velocity": 0.95,
                "pause_distance": 3.0
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 20) # 20 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "time_not_moving", 3.8) # 3.8 is the average value of 5 runs with a 10% margin added.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)

def test_category3_case2_approach_the_stationary_robot_from_left_repeat(tester):
    tester.set_people_detection_range(
        min_range=0.0,
        max_range=100.0,
        min_angle=-3.142,
        max_angle=3.142,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )
    tester.check_collision()
    tester.reset_position()

    actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": -0.5,
                "init_y": 5.0,
                "init_a": -90.0,
                "velocity": 0.95,
                "goal_x": -0.5,
                "goal_y": 3.0,
                "repeat": 1,
                "pause_distance": 3.0
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 18) # 18 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "time_not_moving", 3.6) # 3.6 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)
    tester.set_people_detection_range(
        min_range=0.29, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        max_range=7.07, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        min_angle=-2.28,
        max_angle=2.28,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )

def test_category3_case2_approach_the_stationary_robot_from_right_repeat(tester):
    tester.set_people_detection_range(
        min_range=0.0,
        max_range=100.0,
        min_angle=-3.142,
        max_angle=3.142,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )
    tester.check_collision()
    tester.reset_position()

    actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": -0.5,
                "init_y": -5.0,
                "init_a": 90.0,
                "velocity": 0.95,
                "goal_x": -0.5,
                "goal_y": -3.0,
                "repeat": 1,
                "pause_distance": 3.0
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 18) # 18 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "time_not_moving", 3.6) # 3.6 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)
    tester.set_people_detection_range(
        min_range=0.29, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        max_range=7.07, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        min_angle=-2.28,
        max_angle=2.28,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )

def test_category3_case2_approach_the_stationary_robot_from_behind_repeat(tester):
    tester.set_people_detection_range(
        min_range=0.0,
        max_range=100.0,
        min_angle=-3.142,
        max_angle=3.142,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )
    tester.check_collision()
    tester.reset_position()

    actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_across",
            "params": {
                "init_x": -5.0,
                "init_y": -0.5,
                "init_a": 0.0,
                "velocity": 0.95,
                "goal_x": -3.0,
                "goal_y": -0.5,
                "repeat": 1,
                "pause_distance": 3.0
            },
        },
    ]

    _setup_actors(tester, actors=actors)
    _add_metric_condition_lt(tester, "total_time", 18) # 18 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "time_not_moving", 3.6) # 3.6 is the average value of 5 runs with a 10% margin added when the actor is stationary.
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)
    tester.set_people_detection_range(
        min_range=0.29, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        max_range=7.07, # 0.07 is the distance from the center of the LiDAR to the front of the front camera.
        min_angle=-2.28,
        max_angle=2.28,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )
