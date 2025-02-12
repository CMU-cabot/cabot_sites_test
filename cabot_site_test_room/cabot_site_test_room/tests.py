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


def test01_navigation_to_a_goal(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_navigation_arrived(timeout=90)


# navigation_arrived event should be issued after navigation is completed
def test02_navigation_to_a_goal_and_check_event(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899314416@-90')
    cancel = _check_cabot_event_error(tester, "navigation_arrived")
    tester.wait_navigation_arrived()
    tester.wait_for(seconds=1)
    cancel()
    _check_cabot_event(tester, "navigation_arrived")
    tester.wait_navigation_completed()


# navigation starts from leaf and narrow link
def test03_navigation_to_a_goal(tester):
    tester.reset_position(x=-8.0, y=-4.25, a=0.0)
    tester.goto_node('EDITOR_node_1710181919804')
    tester.wait_goal('NarrowGoal', timeout=15)
    tester.check_position(x=-5.4, y=-4.2, floor=1, tolerance=0.5)
    tester.wait_navigation_arrived(timeout=15)


# navigation starts from leaf and narrow link and goal
def test04_navigation_to_a_goal(tester):
    tester.reset_position(x=-8.0, y=-4.25, a=0.0)
    tester.goto_node('EDITOR_node_1710181891921')
    tester.wait_goal('NarrowGoal', timeout=15)


# navigation starts from leaf and end at the nodea
def test05_navigation_to_a_goal(tester):
    tester.reset_position(x=-8.0, y=-4.25, a=0.0)
    tester.goto_node('EDITOR_node_1708914021679')
    tester.wait_navigation_arrived(timeout=15)


# navigation starts from leaf and normal link and goal
def test06_navigation_to_a_goal(tester):
    tester.reset_position(x=-5.5, y=-1.5, a=-90.0)
    tester.goto_node('EDITOR_node_1710181891921')
    tester.wait_navigation_arrived(timeout=15)


def test07_read_facility_right(tester):
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


def test08_facility_left(tester):
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


def test08_2_ignore_facility_nearby_goal(tester):
    tester.reset_position(x=10.0, y=5.0, a=-90.0)
    tester.goto_node('EDITOR_node_1707899314416@90')
    tester.check_topic_error(
        action_name=f'check_speech',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition=f"msg.category=='speech request' and msg.text=='Test Exhibit is on your left'",
        timeout=30
    )
    tester.wait_navigation_completed(timeout=30)


def test09_navcog_path_bug(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899216479')
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


def test10_speed_poi(tester):
    tester.reset_position(x=1.0, y=2.0, a=0.0)
    tester.goto_node('EDITOR_node_1711172494427')
    tester.wait_topic(
        action_name='check_speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x > 0.5",
        timeout=10
    )
    tester.wait_topic(
        action_name='check_speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x < 0.12",
        timeout=10
    )
    tester.wait_topic(
        action_name='check_speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x > 0.5",
        timeout=10
    )
    tester.wait_topic(
        action_name='check_speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x < 0.12",
        timeout=10
    )
    tester.wait_navigation_arrived(timeout=20)


def test10_2_check_poi_with_line_mode(tester):
    tester.reset_position(x=1.0, y=2.0, a=0.0)
    tester.spawn_obstacle(
        name="obstacle10_2",
        x=4.0,
        y=2.0,
        z=0,
        width=0.2,
        height=3.0,
        depth=1,
        yaw=0)
    tester.wait_for(3)
    tester.goto_node('EDITOR_node_1711172494427')
    cancel = tester.check_topic(
        action_name='check_info',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'speech request' and msg.text == 'Hello'",
        timeout=10
    )
    tester.wait_topic(
        action_name='check_speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x > 0.5",
        timeout=10
    )
    tester.wait_topic(
        action_name='check_speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x < 0.12",
        timeout=10
    )
    tester.wait_topic(
        action_name='check_speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x > 0.5",
        timeout=10
    )
    tester.wait_topic(
        action_name='check_speed',
        topic='/odom',
        topic_type='nav_msgs/msg/Odometry',
        condition="msg.twist.twist.linear.x < 0.12",
        timeout=10
    )
    tester.wait_navigation_arrived(timeout=30)
    tester.delete_obstacle(name="obstacle10_2")
    cancel()


def test11_skip_in_navgoal(tester):
    tester.reset_position(x=0.0, y=0.0)
    tester.goto_node('EDITOR_node_1710181891921')
    tester.wait_for(10)
    tester.info("push left button to pause")
    tester.button_down(3)
    tester.wait_for(2)
    tester.reset_position(x=4.0, y=-4.0, a=-180.0)
    tester.info("push right button to resume")
    tester.button_down(4)
    tester.check_topic_error(
        action_name='check_path',
        topic='/path',
        topic_type='nav_msgs/msg/Path',
        condition="msg.poses[0].pose.position.y > -3",
        timeout=30
    )
    tester.wait_navigation_arrived(timeout=30)


def test12_skip_in_navgoal(tester):
    tester.reset_position(x=4.0, y=-4.0, a=-180.0)
    tester.goto_node('EDITOR_node_1710181891921')
    tester.wait_goal("NarrowGoal")
    tester.info("push left button to pause")
    tester.button_down(3)
    tester.wait_for(2)
    tester.info("push right button to resume")
    tester.button_down(4)
    tester.check_topic_error(
        action_name='check_path',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category=='cabot/navigation' and msg.text=='goal_completed' and msg.memo=='NarrowGoal'",
    )
    tester.wait_navigation_arrived(timeout=30)


def test13_rotation_shim(tester):
    tester.reset_position(x=0.0, y=0.0, a=-180.0)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_navigation_arrived(timeout=30)


def test14_check_footprint_size(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_for(5)
    tester.button_down(3)
    tester.cancel_navigation()
    tester.wait_for(2)
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_navigation_completed(timeout=90)
    tester.wait_for(2)
    tester.wait_topic(
        action_name='check_footprint_size',
        topic='/global_costmap/footprint',
        topic_type='geometry_msgs/msg/Polygon',
        condition="abs(msg.points[0].x - 0.2) < 0.001",
        timeout=5,
    )


def test15_check_short_link_before_narrow(tester):
    for i in range(3):
        tester.reset_position(x=-0.75, y=0.62, a=-90)
        tester.goto_node("EDITOR_node_1707899105269")
        tester.wait_navigation_completed(timeout=30)



def test16_robot_pause_change_destination_and_resume(tester):
    tester.reset_position(x=-0.5, y=1.0)
    tester.goto_node("EDITOR_node_1707899105269")
    tester.wait_for(5)
    tester.button_down(3)
    tester.wait_for(2)
    tester.cancel_navigation()
    tester.check_topic(
        action_name="check navigation_next",
        topic="/cabot/event",
        topic_type="std_msgs/msg/String",
        condition="msg.data=='navigation_next'"
    )
    tester.wait_for(2)
    tester.button_down(4)
    tester.wait_for(2)


def test17_sharp_turn(tester):
    tester.reset_position(a=90)
    tester.goto_node("EDITOR_node_1714592758467")
    tester.wait_navigation_completed(timeout=30)
    tester.cancel_navigation()

def test17_sharp_turn_case2(tester):
    tester.reset_position(x=9.0, y=2.5, a=-90, z=10)
    tester.goto_node("EDITOR_node_1709594307711")
    tester.wait_navigation_completed(timeout=30)
    tester.cancel_navigation()

def test18_across_static_with_narrow(tester):
    tester.reset_position(a=-180, x=-0.5, y=2.5, z=10, floor=2)
    tester.goto_node("EDITOR_node_1719362182201")
    tester.wait_navigation_completed(timeout=30)


def test19_stack_at_start(tester):
    tester.reset_position(x=9.95, y=-4.5, a=135)
    tester.goto_node("EDITOR_node_1708914074632")
    tester.wait_navigation_completed(timeout=90)


def test20_retry_goal(tester):
    tester.reset_position(x=1.0, y=-1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899216479')

    # error if navigation completed is coming after retry
    cancel = tester.check_topic_error(
        action_name='check_wrong_goal',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category=='cabot/navigation' and msg.text=='completed'"
    )

    # subscribe to debug/goal_id to cancel goal intentionally
    from unique_identifier_msgs.msg import UUID
    from rclpy.action import ActionClient
    from rclpy.action.client import ClientGoalHandle
    from nav2_msgs.action import NavigateToPose
    client = ActionClient(tester.node, NavigateToPose, "/navigate_to_pose")
    client.count = 0
    def goal_id_callback(msg):
        client.handle = ClientGoalHandle(client, msg, None)
    sub = tester.node.create_subscription(UUID, "/debug/goal_id", goal_id_callback, 10)

    # try to cancel goal 3 times
    tester.wait_for(5)
    client.handle.cancel_goal_async()
    tester.wait_for(5)
    client.handle.cancel_goal_async()
    tester.wait_for(5)
    client.handle.cancel_goal_async()
    tester.wait_for(5)
    # cancel check
    cancel()

    tester.wait_navigation_completed(timeout=90)
    # need to destroy created client and subscription
    client.destroy()
    sub.destroy()
