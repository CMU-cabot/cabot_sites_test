def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def _check_cabot_event(tester, event, **kwargs):
    tester.check_topic(**dict(
        dict(
            action_name=f'check_cabot_event_{event}',
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
            action_name=f'check_cabot_event_{event}_error',
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


# navigation starts from leaf and narrow link
def test3_navigation_to_a_goal(tester):
    tester.reset_position(x=-8.0, y=-4.25, a=0.0)
    tester.goto_node('EDITOR_node_1710181919804')
    tester.wait_goal('NarrowGoal', timeout=15)
    tester.check_position(x=-5.4, y=-4.2, floor=1, tolerance=0.5)
    tester.wait_navigation_arrived(timeout=15)


# navigation starts from leaf and narrow link and goal
def test4_navigation_to_a_goal(tester):
    tester.reset_position(x=-8.0, y=-4.25, a=0.0)
    tester.goto_node('EDITOR_node_1710181891921')
    tester.wait_goal('NarrowGoal', timeout=15)


# navigation starts from leaf and end at the nodea
def test5_navigation_to_a_goal(tester):
    tester.reset_position(x=-8.0, y=-4.25, a=0.0)
    tester.goto_node('EDITOR_node_1708914021679')
    tester.wait_navigation_arrived(timeout=15)


# navigation starts from leaf and normal link and goal
def test6_navigation_to_a_goal(tester):
    tester.reset_position(x=-5.5, y=-1.5, a=-90.0)
    tester.goto_node('EDITOR_node_1710181891921')
    tester.wait_navigation_arrived(timeout=15)


def test7_read_facility_right(tester):
    tester.reset_position(x=10.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899216479')
    tester.wait_topic(
        action_name=f'check_speech',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition=f"msg.category=='speech request' and msg.text=='Test Exhibit is on your right'",
        timeout=30
    )
    tester.wait_navigation_arrived(timeout=30)


def test8_facility_left(tester):
    tester.reset_position(x=10.0, y=5.0, a=-90.0)
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_topic(
        action_name=f'check_speech',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition=f"msg.category=='speech request' and msg.text=='Test Exhibit is on your left'",
        timeout=30
    )
    tester.wait_navigation_arrived(timeout=30)


def test9_navcog_path_bug(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1710807879215')
    tester.wait_navigation_arrived(timeout=30)
    tester.goto_node('EDITOR_node_1707899150598')
    tester.wait_topic(
        action_name='check_path',
        topic='/path',
        topic_type='nav_msgs/msg/Path',
        condition="msg.poses[0].pose.position.x > 5",
        timeout=30
    )
    tester.wait_navigation_arrived(timeout=30)
