from rclpy.qos import QoSProfile, DurabilityPolicy


def _env():
    return [
        "CABOT_HANDLE_BUTTON_MAPPING=1"
    ]


def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0
    tester.set_speed(1.0)


def wait_ready(tester):
    # tester.wait_localization_started()
    tester.wait_ready()


def _button_down(tester, button, **kwargs):
    tester.pub_topic(**dict(
        dict(
            action_name=f'button down({button})',
            topic='/cabot/event',
            topic_type='std_msgs/msg/String',
            message=f"data: 'button_down_{button}'"
        ),
        **kwargs)
    )


def _button_up(tester, button, **kwargs):
    tester.pub_topic(**dict(
        dict(
            action_name=f'button up({button})',
            topic='/cabot/event',
            topic_type='std_msgs/msg/String',
            message=f"data: 'button_up_{button}'"
        ),
        **kwargs)
    )


def _click(tester, button, **kwargs):
    tester.pub_topic(**dict(
        dict(
            action_name=f'click({button})',
            topic='/cabot/event',
            topic_type='std_msgs/msg/String',
            message=f"data: 'click_{button}_1'"
        ),
        **kwargs)
    )


def _holddown(tester, button, duration, **kwargs):
    tester.pub_topic(**dict(
        dict(
            action_name=f'holddown({button}, {duration})',
            topic='/cabot/event',
            topic_type='std_msgs/msg/String',
            message=f"data: 'holddown_{button}_{duration}'"
        ),
        **kwargs)
    )


# click up
def test01_speed_up(tester):
    tester.reset_position()
    tester.set_speed(0.95)
    tester.wait_for(3)
    tester.check_topic(
        action_name='check /cabot/user_speed',
        topic='/cabot/user_speed',
        topic_type='std_msgs/msg/Float32',
        qos=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        condition="abs(msg.data - 1.00) < 0.01",
    )
    _click(tester, 1)
    tester.wait_for(3)


# click down
def test02_speed_down(tester):
    tester.reset_position()
    tester.set_speed(1.0)
    tester.wait_for(3)
    tester.check_topic(
        action_name='check /cabot/user_speed',
        topic='/cabot/user_speed',
        topic_type='std_msgs/msg/Float32',
        qos=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        condition="abs(msg.data - 0.95) < 0.01",
    )
    _click(tester, 2)
    tester.wait_for(3)


# click right
def test03_navigation_resume(tester):
    tester.reset_position()
    tester.check_topic(
        action_name='check /cabot/event',
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        condition="msg.data == 'navigation_next'",
    )
    _click(tester, 4)
    tester.wait_for(3)


# button down left and click right
def test04_navigation_pause_and_resume(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899314416')
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/navigation' and msg.text == 'pause'",
    )
    tester.wait_for(3)
    _button_down(tester, 3)
    tester.wait_for(3)
    cancel()
    _click(tester, 4)
    tester.wait_navigation_completed()


# hold down left 3 secs and click right
def test05_navigation_idle_and_resume(tester):
    tester.reset_position()
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/navigation' and msg.text == 'pause_control' and msg.memo == 'True'",
    )
    _holddown(tester, 3, 3)
    tester.wait_for(3)
    cancel()
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/navigation' and msg.text == 'pause_control' and msg.memo == 'False'",
    )
    tester.wait_for(3)
    _click(tester, 4)
    tester.wait_for(3)


# hold down right 1,2,3 secs and button up right
def test06_description(tester):
    tester.reset_position()
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/interface' and msg.text == 'requesting_describe_surround_stop_reason'"
    )
    _holddown(tester, 4, 1)
    _button_up(tester, 4)
    tester.wait_for(3)
    cancel()
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/interface' and msg.text == 'requesting_describe_surround_stop_reason'"
    )
    _holddown(tester, 4, 2)
    _button_up(tester, 4)
    tester.wait_for(3)
    cancel()
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/interface' and msg.text == 'requesting_describe_surround'"
    )
    _holddown(tester, 4, 3)
    _button_up(tester, 4)
    tester.wait_for(3)
    cancel()
