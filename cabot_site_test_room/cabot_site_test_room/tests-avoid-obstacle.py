# *****************************************************************************
# Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# *****************************************************************************

import math
from cabot_ui.geojson import NavigationMode

def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0

def wait_ready(tester):
    tester.wait_localization_started()

def test0(tester):
    #tester.reset_position(x=-7.5,y=-1.0, a=0.0) # left end position
    #tester.reset_position(x=2.5,y=-1.0, a=180.0) # right end position
    tester.wait_for(1)

def test1_climb_step_lower_than_5cm_and_slow_down(tester):
    STEP_HEIGHT = 0.030 # 3.0cm=30mm
    tester.reset_position(x=-7.5,y=-1.0,a=0.0)
    tester.floor_change(+1)
    tester.spawn_obstacle(
            name="10mm_step", \
            x=-2.75, y=-1.0, z=15., yaw=0., \
            width=4.5, height=2, depth=STEP_HEIGHT \
            )

    tester.goto_node('EDITOR_node_1730277138101')

    max_speed = 1.0
    cancel = tester.check_topic(
        action_name='is_speed_under_limit_goto_EDITOR_node_1730277130501',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition=f"msg.twist.twist.linear.x <= {max_speed}",
        timeout=10
    )
    tester.wait_topic(
        action_name='is_speed_closeto_limit_goto_EDITOR_node_1730277130501',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x > 0.3",
        timeout=10
    )

    tester.wait_mode_changed(NavigationMode.ClimbUpStep,timeout=5)
    cancel()

    max_speed = 0.3
    cancel = tester.check_topic(
        action_name='is_speed_closeto_limit_goto_EDITOR_node_1730277132929',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition=f"msg.twist.twist.linear.x <= {max_speed}",
        timeout=10
    )
    tester.wait_topic(
        action_name='is_speed_under_limit_goto_EDITOR_node_1730277132929',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition=f"msg.twist.twist.linear.x > ({max_speed}-0.1)",
        timeout=10
    )

    tester.wait_mode_changed(NavigationMode.Standard,timeout=20)
    cancel()

    max_speed = 1.0
    cancel = tester.check_topic(
        action_name='is_speed_closeto_limit_goto_EDITOR_node_1730277134693',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition=f"msg.twist.twist.linear.x <= {max_speed}",
        timeout=10
    )
    tester.wait_topic(
        action_name='is_speed_under_limit_goto_EDITOR_node_1730277134693',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition=f"msg.twist.twist.linear.x > ({max_speed}-0.1)",
        timeout=10
    )

    tester.wait_mode_changed(NavigationMode.ClimbDownStep,timeout=20)
    cancel()

    max_speed = 0.3
    cancel = tester.check_topic(
        action_name='is_speed_closeto_limit_goto_EDITOR_node_1730277136174',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition=f"msg.twist.twist.linear.x <= {max_speed}",
        timeout=10
    )
    tester.wait_topic(
        action_name='is_speed_under_limit_goto_EDITOR_node_1730277136174',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition=f"msg.twist.twist.linear.x > ({max_speed}-0.1)",
        timeout=10
    )

    tester.wait_navigation_arrived()
    cancel()
    tester.clean_obstacle()

# # To execute clean_obstacle, /obstacle_states topic must exist
# def test999_clean_obstacles(tester):
#     tester.wait_topic(
#         action_name='check_obstacle_states',
#         topic='/obstacle_states',
#         topic_type='pedestrian_plugin_msgs/msg/Agents',
#         condition="True",
#         timeout=10
#     )
#     tester.clean_obstacle()
