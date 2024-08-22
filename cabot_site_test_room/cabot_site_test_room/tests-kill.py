import subprocess
import os
import time

def _get_ros2_node_pids(tester, node_name):
    ps_output = subprocess.check_output(['ps', 'aux'], text=True)
    pids = []
    for line in ps_output.splitlines():
        if node_name in line:
            pid = int(line.split()[1])
            pids.append(pid)
    return pids

def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def test1_kill_planner(tester):
    tester.reset_position()
    pids = _get_ros2_node_pids(tester, "planner_server")
    for pid in pids:
        try:
            tester.info(pid)
            os.kill(pid, 2)
        except:
            pass
    tester.wait_for(10)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_navigation_arrived(timeout=30)


def test2_kill_planner(tester):
    tester.reset_position()
    pids = _get_ros2_node_pids(tester, "planner_server")
    for pid in pids:
        try:
            tester.info(pid)
            os.kill(pid, 2)
        except:
            pass
    tester.wait_for(2)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_for(2)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_for(2)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_navigation_arrived(timeout=30)

def test3_kill_navigator_after_sending_goal(tester):
    tester.reset_position()
    tester.info("navigate to a node")
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_for(3)
    tester.info("killing bt_navigator with signal 9")
    pids = _get_ros2_node_pids(tester, "bt_navigator")
    for pid in pids:
        try:
            tester.info(pid)
            os.kill(pid, 9)
        except:
            pass
    tester.wait_for(2)
    tester.button_down(3)  # pause
    tester.wait_for(2)
    cancel = tester.check_topic(
        action_name='wait_pausing_message',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category=='cabot/interface' and msg.text=='navigation' and msg.memo=='pausing'",
    )
    tester.button_down(4)  # resume - will timeout to cancel
    tester.wait_for(7)
    cancel()
    tester.button_down(4)  # resume - this may fail due to delay of lifecycle bond check, and will retry to resume
    tester.wait_navigation_arrived(timeout=30)
