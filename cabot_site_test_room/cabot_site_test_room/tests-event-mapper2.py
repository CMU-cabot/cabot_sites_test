from rclpy.qos import QoSProfile, DurabilityPolicy


def _env():
    return [
        "CABOT_HANDLE_BUTTON_MAPPING=2"
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


def _click(tester, button, count=1, **kwargs):
    tester.pub_topic(**dict(
        dict(
            action_name=f'click({button}, {count})',
            topic='/cabot/event',
            topic_type='std_msgs/msg/String',
            message=f"data: 'click_{button}_{count}'"
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


# hold down up 1 sec
def test01_speed_up(tester):
    tester.reset_position()
    tester.set_speed(0.95)
    cancel = tester.check_topic(
        action_name='check /cabot/user_speed',
        topic='/cabot/user_speed',
        topic_type='std_msgs/msg/Float32',
        qos=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        condition="abs(msg.data - 1.00) < 0.01",
    )
    tester.wait_for(3)
    _holddown(tester, 1, 1)
    _button_up(tester, 1)
    tester.wait_for(3)
    cancel()


# hold down down 1 sec
def test02_speed_down(tester):
    tester.reset_position()
    tester.set_speed(1.0)
    cancel = tester.check_topic(
        action_name='check /cabot/user_speed',
        topic='/cabot/user_speed',
        topic_type='std_msgs/msg/Float32',
        qos=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        condition="abs(msg.data - 0.95) < 0.01",
    )
    tester.wait_for(3)
    _holddown(tester, 2, 1)
    _button_up(tester, 2)
    tester.wait_for(3)
    cancel()


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


# hold down left 1sec and click right
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
    _holddown(tester, 3, 1)
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


# navigation and click right again
def test06_navigation_pause_and_resume(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899314416')
    tester.wait_for(3)
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/interface' and msg.text == 'requesting_describe_surround_stop_reason'"
    )
    tester.wait_for(3)
    tester.set_speed(0.0)
    tester.wait_for(3)
    _click(tester, 4)
    tester.wait_for(3)
    cancel()
    tester.set_speed(1.0)
    tester.wait_for(3)
    tester.wait_navigation_completed()


# multiple click up
def test07_description(tester):
    tester.reset_position()
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/interface' and msg.text == 'requesting_describe_surround'"
    )
    _click(tester, 1, 1)
    tester.wait_for(3)
    cancel()
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/interface' and msg.text == 'requesting_describe_surround'"
    )
    _click(tester, 1, 2)
    tester.wait_for(3)
    cancel()
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'cabot/interface' and msg.text == 'requesting_describe_surround'"
    )
    _click(tester, 1, 3)
    tester.wait_for(3)
    cancel()


# multiple click right
def test08_description(tester):
    tester.reset_position()
    tester.pub_topic(
        action_name='enable speaker',
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        message="data: 'navigation_speaker_enable;true'"
    )
    tester.pub_topic(
        action_name='set speaker audio file',
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        message="data: 'navigation_speaker_audio_file;dummy.wav'"
    )
    tester.pub_topic(
        action_name='set speaker audio file',
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        message="data: 'navigation_speaker_volume;50'"
    )
    tester.wait_for(3)
    cancel = tester.check_topic(
        action_name='check /cabot/activity_log',
        topic='/cabot/activity_log',
        topic_type='cabot_msgs/msg/Log',
        condition="msg.category == 'speaker_alert'"
    )
    _click(tester, 4, 2)
    tester.wait_for(3)
    cancel()


# click down
def test09_conversation(tester):
    tester.reset_position()
    cancel = tester.check_topic(
        action_name='check /cabot/event',
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        condition="msg.data == 'navigation_toggleconversation'"
    )
    _click(tester, 2, 1)
    tester.wait_for(3)
    cancel()


# click down
def test10_stop_speak(tester):
    tester.reset_position()
    cancel = tester.check_topic(
        action_name='check /cabot/event',
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        condition="msg.data == 'navigation_togglespeakstate'"
    )
    _click(tester, 3, 1)
    tester.wait_for(3)
    cancel()
