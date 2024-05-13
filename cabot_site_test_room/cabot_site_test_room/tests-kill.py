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
