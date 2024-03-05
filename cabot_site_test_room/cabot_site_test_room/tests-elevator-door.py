def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0


def wait_ready(tester):
    tester.wait_ready()


def test0_navigation_to_a_goal(tester):
    tester.reset_position()
    tester.spawn_door(name="test", x=1, y=0, yaw=0)
    tester.wait_for(seconds=3)
    tester.delete_door(name="test")
