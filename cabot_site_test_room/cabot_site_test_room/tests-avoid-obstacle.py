import math

def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()

def test1_ignore_step_lower_than_1cm(tester):
    STEP_HEIGHT = 0.009 # 0.9cm=9mm
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    tester.check_collision_obstacle()
    tester.spawn_obstacle(
            name="10mm_step", \
            x=3, y=0.7, z=0., yaw=0., \
            width=1, height=1, depth=STEP_HEIGHT \
            )
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_navigation_arrived()
    tester.clean_obstacle()

def test2_recognize_step_higher_than_1cm_and_slow_down(tester):
    STEP_HEIGHT = 0.011 # 1.1cm=11mm
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    tester.check_collision_obstacle()
    tester.spawn_obstacle(
            name="10mm_step", \
            x=3, y=0.7, z=0., yaw=0., \
            width=1, height=1, depth=STEP_HEIGHT \
            )
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_navigation_arrived()
    tester.clean_obstacle()

def test3_recognize_step_lower_than_5cm_and_slow_down(tester):
    STEP_HEIGHT = 0.049 # 4.9cm=49mm
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    tester.check_collision_obstacle()
    tester.spawn_obstacle(
            name="10mm_step", \
            x=3, y=0.7, z=0., yaw=0., \
            width=1, height=1, depth=STEP_HEIGHT \
            )
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_navigation_arrived()
    tester.clean_obstacle()

def test4_recognize_step_higher_than_5cm_and_avoid_step(tester):
    STEP_HEIGHT = 0.051 # 5.1cm=51mm
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    tester.check_no_collision_obstacle()
    tester.spawn_obstacle(
            name="10mm_step", \
            x=3, y=0.7, z=0., yaw=0., \
            width=1, height=1, depth=STEP_HEIGHT \
            )
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_navigation_arrived()
    tester.clean_obstacle()

def test5_recognize_step_higher_than_20cm_and_avoid_step(tester):
    STEP_HEIGHT = 0.200 # 20cm=200mm
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    tester.check_no_collision_obstacle()
    tester.spawn_obstacle(
            name="10mm_step", \
            x=3, y=0.7, z=0., yaw=0., \
            width=1, height=1, depth=STEP_HEIGHT \
            )
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_navigation_arrived()
    tester.clean_obstacle()
