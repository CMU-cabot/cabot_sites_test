def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0


def wait_ready(tester):
    tester.wait_ready()


def test0_navigation_to_a_goal(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899365026')
    tester.wait_navigation_arrived(timeout=90)


# cabot planner can be dead due to a goal that is too close to the current position
def test1_start_on_narrow_path_start(tester):
    tester.reset_position(x=-0.7, y=0.7, a=90.0)
    tester.goto_node('EDITOR_node_1707899365026')
    tester.wait_navigation_arrived(timeout=90)


def test2_navigation_to_a_goal_and_return(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899365026')
    tester.wait_navigation_arrived(timeout=90)
    tester.goto_node('EDITOR_node_1707899162797')
    tester.wait_navigation_arrived(timeout=90)

def test3_start_partway_along_the_narrow_path_1(tester):
    tester.reset_position(x=3.0, y=-4.0, a=90.0)
    tester.goto_node('EDITOR_node_1707899365026')
    tester.wait_navigation_arrived(timeout=180)

def test4_start_partway_along_the_narrow_path_2(tester):
    tester.reset_position(x=-5.0, y=5.0, a=180.0)
    tester.goto_node('EDITOR_node_1707899365026')
    tester.wait_navigation_arrived(timeout=90)

def test5_start_and_goal_partway_along_the_narrow_path(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707898959554')
    tester.wait_navigation_arrived(timeout=90)
    tester.goto_node('EDITOR_node_1707899162797')
    tester.wait_navigation_arrived(timeout=180)