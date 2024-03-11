def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def _check_interface_event(tester, event, **kwargs):
    tester.check_topic(**dict(
        dict(
            action_name=f'check_interface_{event}',
            topic='/cabot/activity_log',
            topic_type='cabot_msgs/msg/Log',
            condition=f"msg.category=='cabot/interface' and msg.text=='navigation' and msg.memo=='{event}'",
            timeout=60
        ),
        **kwargs)
    )


def _check_interface_event_error(tester, event, **kwargs):
    tester.check_topic_error(**dict(
        dict(
            action_name=f'check_interface_{event}_error',
            topic='/cabot/activity_log',
            topic_type='cabot_msgs/msg/Log',
            condition=f"msg.category=='cabot/interface' and msg.text=='navigation' and msg.memo=='{event}'",
            timeout=60
        ),
        **kwargs)
    )


def test6_navigation_to_a_goal(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_navigation_arrived(timeout=90)


def test0_navigation_to_a_goal(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899365026')
    _check_interface_event(tester, "please_follow_behind")
    _check_interface_event(tester, "please_return_position")
    tester.wait_navigation_arrived(timeout=90)


# cabot planner can be dead due to a goal that is too close to the current position
def test1_start_on_narrow_path_start(tester):
    tester.reset_position(x=-0.7, y=0.7, a=90.0)
    tester.goto_node('EDITOR_node_1707899365026')
    _check_interface_event(tester, "please_follow_behind")
    _check_interface_event(tester, "please_return_position")
    tester.wait_navigation_arrived(timeout=90)


def test2_navigation_to_a_goal_and_return(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899365026')
    _check_interface_event(tester, "please_follow_behind")
    _check_interface_event(tester, "please_return_position")
    tester.wait_navigation_arrived(timeout=90)
    tester.goto_node('EDITOR_node_1707899162797')
    _check_interface_event(tester, "please_follow_behind")
    _check_interface_event(tester, "please_return_position")
    tester.wait_navigation_arrived(timeout=90)


def test3_start_partway_along_the_narrow_path_1(tester):
    tester.reset_position(x=3.0, y=-4.0, a=90.0)
    tester.goto_node('EDITOR_node_1707899150598')
    _check_interface_event_error(tester, "please_follow_behind")
    _check_interface_event_error(tester, "please_return_position")
    tester.wait_navigation_arrived(timeout=180)


def test4_start_partway_along_the_narrow_path_2(tester):
    tester.reset_position(x=-5.0, y=5.0, a=180.0)
    tester.goto_node('EDITOR_node_1707899150598')
    _check_interface_event(tester, "please_follow_behind")
    _check_interface_event(tester, "please_return_position")
    tester.wait_navigation_arrived(timeout=90)


def test5_start_and_goal_partway_along_the_narrow_path(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707898959554')
    _check_interface_event(tester, "please_follow_behind")
    _check_interface_event(tester, "please_return_position")
    tester.wait_navigation_arrived(timeout=90)
    tester.goto_node('EDITOR_node_1707899162797')
    _check_interface_event(tester, "please_follow_behind")
    _check_interface_event(tester, "please_return_position")
    tester.wait_navigation_arrived(timeout=180)
