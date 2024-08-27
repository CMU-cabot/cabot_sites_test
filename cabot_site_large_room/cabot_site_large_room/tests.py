# ******************************************************************************
#  Copyright (c) 2024  Carnegie Mellon University and Miraikan
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
    tester.wait_localization_started()


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


def _check_metric(tester, metric_name, fail_threshold, condition_operator="<="):
    """
    Generic function to check if a given metric meets the failure threshold.
    :param tester: The tester object
    :param metric_name: The name of the metric to check
    :param fail_threshold: The threshold value for the metric
    :param condition_operator: The operator to use in the condition string
    """
    condition = f"msg.name=='{metric_name}' and {fail_threshold}{condition_operator}msg.value"
    tester.check_topic_error(
        action=f"check {metric_name}",
        topic="/metric",
        topic_type="pedestrian_plugin_msgs/msg/Metric",
        condition=condition
    )


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
                "pause_distance": 1.0
            },
        },
    ])
    _check_metric(tester, "total_time", 26) # 26 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 12.6) # 12.6 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case2_move_towards_a_pedestrian_and_avoid(tester):
    # 1.2 Pedestrian Obstruction
    tester.check_collision()
    tester.reset_position(x=-5.0)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 5.0,
                "init_y": -0.5,
                "init_a": 180.0,
                "velocity": 0.0  # does not move
            },
        },
    ])
    _check_metric(tester, "total_time", 21) # 21 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 12.2) # 12.2 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "time_not_moving", 4.3) # 4.3 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case4_robot_overtaking(tester):
    # 1.4 robot overtaking
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 1.0,
                "init_y": -0.5,
                "init_a": 0.0,
                "velocity": 0.25  # very slow
            },
        },
    ])
    _check_metric(tester, "total_time", 11, condition_operator=">=") # 11 is the average value of 5 runs (actor's speed is set to 0 and actor's init_x is set to 5.0) with a 40% margin removed.
    _check_metric(tester, "total_time", 26) # 26 is the average value of 5 runs (actor's speed is set to 0 and actor's init_x is set to 5.0) with a 40% margin added.
    _check_metric(tester, "robot_path_length", 15.6) # 15.6 is the average value of 5 runs (actor's speed is set to 0 and actor's init_x is set to 5.0) with a 40% margin added.
    _check_metric(tester, "maximum_distance_to_people", 7.5) # 7.5 is the average value of 5 runs (actor's speed is set to 0 and actor's init_x is set to 5.0) with a 40% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category1_case5_down_the_path(tester):
    # 1.5 Down path
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.walk_straight",
            "params": {
                "init_x": 1.0,
                "init_y": -0.5,
                "init_a": 0.0,
                "velocity": 0.75
            },
        },
    ])
    _check_metric(tester, "total_time", 16, condition_operator=">=") # 16 is the average value of 5 runs with a 10% margin removed.
    _check_metric(tester, "total_time", 20) # 20 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "cumulative_heading_changes", 0.23) # 0.23 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "minimum_distance_to_people", 0.28, condition_operator=">=") # 0.28 is the average value of 5 runs with a 10% margin removed.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian1(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
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
    ])
    _check_metric(tester, "total_time", 22) # 22 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian2(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
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
    ])
    _check_metric(tester, "total_time", 21) # 21 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian3(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
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
    ])
    _check_metric(tester, "total_time", 22) # 22 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian4(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
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
    ])
    _check_metric(tester, "total_time", 22) # 22 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian5(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
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
    ])
    _check_metric(tester, "total_time", 18) # 18 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category2_case1_move_across_a_pedestrian6(tester):
    # 2.1 Intersection
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
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
    ])
    _check_metric(tester, "total_time", 23) # 23 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)


def test_category3_case1_move_across_a_pedestrian_proceed(tester):
    # 2.3 Intersection Proceed
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
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
    ])
    _check_metric(tester, "total_time", 20) # 20 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_path_length", 11.9) # 11.9 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "time_not_moving", "3.8") # 3.8 is the average value of 5 runs with a 10% margin added.
    _check_metric(tester, "robot_on_person_collision_count", 1)
    _goto_target1(tester)
