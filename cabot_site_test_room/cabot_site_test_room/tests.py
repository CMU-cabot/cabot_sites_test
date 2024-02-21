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
    tester.wait_navigation_arrived()


def test1_navigation_to_a_goal(tester):
    tester.reset_position(x=-0.7, y=0.7, a=90.0)
    tester.goto_node('EDITOR_node_1707899365026')
    tester.wait_navigation_arrived()
