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


def test1_navigation_to_a_goal(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_navigation_arrived(timeout=90)

