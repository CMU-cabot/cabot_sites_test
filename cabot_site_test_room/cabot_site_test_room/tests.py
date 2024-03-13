def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def _check_cabot_event(tester, event, **kwargs):
    return tester.check_topic(**dict(
        dict(
            action_name=f'check_interface_{event}',
            topic='/cabot/event',
            topic_type='std_msgs/msg/String',
            condition=f"msg.data=='{event}'",
            timeout=60
        ),
        **kwargs)
    )


def _check_cabot_event_error(tester, event, **kwargs):
    return tester.check_topic_error(**dict(
        dict(
            action_name=f'check_interface_{event}_error',
            topic='/cabot/event',
            topic_type='std_msgs/msg/String',
            condition=f"msg.data=='{event}'",
            timeout=60
        ),
        **kwargs)
    )


def test1_navigation_to_a_goal(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_navigation_arrived(timeout=90)


# navigation_arrived event should be issued after navigation is completed
def test2_navigation_to_a_goal_and_check_event(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899314416@-90')
    cancel = _check_cabot_event_error(tester, "navigation_arrived")
    tester.wait_navigation_arrived()
    tester.wait_for(seconds=1)
    cancel()
    _check_cabot_event(tester, "navigation_arrived")
    tester.wait_navigation_completed()
