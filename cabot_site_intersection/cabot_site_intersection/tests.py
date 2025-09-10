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

import json
from cabot_common.util import setInterval as _setInterval
from .signals import SignalGeneratorDummy001RedFirst
from .signals import SignalGeneratorDummy001GreenFirst


def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0


def wait_ready(tester):
    # tester.wait_localization_started()
    tester.wait_ready()


@_setInterval(0.5)
def _publish_signals(tester, signal_generator):
    status = signal_generator.next()
    tester.pub_topic(
        action_name='sending intersection status',
        topic='/signal_response_intersection_status',
        topic_type='std_msgs/msg/String',
        message=f"data: '{json.dumps(status)}'",
    )


def _wait_moved(tester, timeout=10):
    tester.wait_topic(
        action_name='check_speed nonzero speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x > 0.1",
        timeout=timeout
    )


def _wait_stopped(tester, timeout=10):
    tester.wait_topic(
        action_name='check_speed nonzero speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x < 0.001",
        timeout=timeout
    )


def _check_stopped(tester):
    tester.wait_topic(
        action_name='check_speed stopped',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x < 0.001",
        once=True,
    )


def _check_navigation_arrived_error(tester):
    return tester.check_topic_error(
        action_name='check_navigation_arrived error',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category=='cabot/navigation' and msg.text=='navigation' and msg.memo=='arrived'",
        timeout=60
    )


def test01_stop_by_red_signal(tester):
    tester.reset_position(x=7.0, y=6.5, a=0.0)
    stop = _publish_signals(tester, SignalGeneratorDummy001RedFirst(start=15))
    tester.goto_node('EDITOR_node_1757425364512')
    _wait_moved(tester, 10)
    _wait_stopped(tester, 10)
    tester.wait_navigation_arrived(timeout=30)
    stop.set()


def test02_stop_without_signal_info(tester):
    tester.reset_position(x=7.0, y=6.5, a=0.0)
    # publish signal status for a while and then stop publishing
    stop = _publish_signals(tester, SignalGeneratorDummy001GreenFirst())
    tester.wait_for(5)
    stop.set()
    tester.wait_for(5)

    # SignalPOI should invalidate the old signal info
    cancel = _check_navigation_arrived_error(tester)
    tester.goto_node('EDITOR_node_1757425364512')
    _wait_moved(tester, 10)
    _wait_stopped(tester)
    tester.wait_for(30)
    _check_stopped(tester)
    cancel()


def test03_check_remaining_time_stop(tester):
    tester.reset_position(x=7.0, y=6.5, a=0.0)
    tester.set_speed(1.0)
    stop = _publish_signals(tester, SignalGeneratorDummy001GreenFirst(start=15))
    tester.goto_node('EDITOR_node_1757425364512')
    tester.wait_for(5)
    _check_stopped(tester)
    tester.wait_navigation_arrived(timeout=60)
    stop.set()


def test04_1_check_remaining_time_go(tester):
    tester.reset_position(x=7.0, y=6.5, a=0.0)
    tester.set_speed(1.0)
    stop = _publish_signals(tester, SignalGeneratorDummy001GreenFirst(start=10))
    tester.goto_node('EDITOR_node_1757425364512')
    tester.wait_navigation_arrived(timeout=30)
    stop.set()


def test04_2_check_remaining_time_go(tester):
    tester.reset_position(x=7.0, y=6.5, a=0.0)
    tester.set_speed(0.5)
    stop = _publish_signals(tester, SignalGeneratorDummy001GreenFirst(start=10))
    tester.goto_node('EDITOR_node_1757425364512')
    tester.wait_for(5)
    _check_stopped(tester)
    tester.wait_navigation_arrived(timeout=90)
    stop.set()


def test05_cross_two_crossings(tester):
    tester.reset_position(x=7.0, y=6.5, a=0.0)
    tester.set_speed(1.0)
    stop = _publish_signals(tester, SignalGeneratorDummy001RedFirst(start=15))
    tester.goto_node('EDITOR_node_1757425176316')
    tester.wait_for(5)
    _check_stopped(tester)
    tester.wait_goal("CrosswalkGoal")
    tester.wait_goal("NavGoal")
    tester.info("Reached first goal, going to the next crossing")
    _wait_moved(tester, 5)
    _wait_stopped(tester, 5)
    tester.wait_goal("CrosswalkGoal")
    stop.set()
