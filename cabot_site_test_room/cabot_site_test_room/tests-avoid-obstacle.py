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
    tester.spawn_obstacle(
            name="10mm_step", \
            x=-2.75, y=-1.0, z=0., yaw=0., \
            width=4.5, height=2, depth=STEP_HEIGHT \
            )
    tester.goto_node('EDITOR_node_1720772383943')
    tester.wait_mode_changed(NavigationMode.ClimbUpStep,timeout=20)
    tester.wait_mode_changed(NavigationMode.ClimbDownStep,timeout=20)
    tester.wait_navigation_arrived()
    tester.clean_obstacle()

def test1_recognize_step_higher_than_1cm_and_slow_down(tester):
    # this does NOT check if the robot slow down yet
    STEP_HEIGHT = 0.011 # 1.1cm=11mm
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    cancel = tester.check_collision_obstacle()
    tester.spawn_obstacle(
            name="10mm_step", \
            x=3, y=0.7, z=0., yaw=0., \
            width=1, height=1, depth=STEP_HEIGHT \
            )
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_navigation_arrived()
    tester.clean_obstacle()
    cancel()

#def test2_recognize_step_lower_than_5cm_and_slow_down(tester):
#    # this does NOT check if the robot slow down yet
#    STEP_HEIGHT = 0.049 # 4.9cm=49mm
#    tester.reset_position(x=0.0, y=0.0, a=0.0)
#    cancel = tester.check_collision_obstacle()
#    tester.spawn_obstacle(
#            name="10mm_step", \
#            x=3, y=0.7, z=0., yaw=0., \
#            width=1, height=1, depth=STEP_HEIGHT \
#            )
#    tester.goto_node('EDITOR_node_1707899150598')
#    tester.wait_navigation_arrived()
#    tester.clean_obstacle()
#    cancel()
#
#def test3_recognize_step_higher_than_5cm_and_avoid_step(tester):
#    STEP_HEIGHT = 0.051 # 5.1cm=51mm
#    tester.reset_position(x=0.0, y=0.0, a=0.0)
#    cancel = tester.check_no_collision_obstacle()
#    tester.spawn_obstacle(
#            name="10mm_step", \
#            x=3, y=0.7, z=0., yaw=0., \
#            width=1, height=1, depth=STEP_HEIGHT \
#            )
#    tester.goto_node('EDITOR_node_1707899150598')
#    tester.wait_navigation_arrived()
#    tester.clean_obstacle()
#    cancel()

def test4_recognize_step_higher_than_or_equal_to_20cm_and_avoid_step(tester):
    STEP_HEIGHT = 1.500 # 20cm=200mm
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    cancel = tester.check_no_collision_obstacle()
    tester.spawn_obstacle(
            name="10mm_step", \
            x=3, y=0.7, z=0., yaw=0., \
            width=1, height=1, depth=STEP_HEIGHT \
            )
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_navigation_arrived()
    tester.clean_obstacle()
    cancel()

def test999_clean_obstacles(tester):
    tester.wait_topic(
        action_name='check_obstacle_states',
        topic='/obstacle_states',
        topic_type='pedestrian_plugin_msgs/msg/Agents',
        condition="True",
        timeout=10
    )
    tester.clean_obstacle()