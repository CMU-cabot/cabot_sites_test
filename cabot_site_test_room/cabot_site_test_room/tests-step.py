def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def _expect_move(tester):
    cancel = tester.check_topic(
        topic="/cabot/cmd_vel",
        topic_type="geometry_msgs/msg/Twist",
        condition="abs(msg.linear.x) > 0.0",
    )
    tester.pub_topic(
        topic="/cmd_vel",
        topic_type="geometry_msgs/msg/Twist",
        message="linear:\n  x: 0.5\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: 0.0",
    )
    tester.wait_for(1)
    cancel()


def _expect_no_move(tester):
    cancel = tester.check_topic_error(
        topic="/cabot/cmd_vel",
        topic_type="geometry_msgs/msg/Twist",
        condition="abs(msg.linear.x) > 0.0",
    )
    tester.pub_topic(
        topic="/cmd_vel",
        topic_type="geometry_msgs/msg/Twist",
        message="linear:\n  x: 0.5\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: 0.0",
    )
    tester.wait_for(1)
    cancel()


def test00_step_2_5(tester):
    tester.clean_obstacle()
    tester.delete_obstacle(name="f1_door")
    tester.reset_position(x=-1.0, y=0.0, a=0.0)
    tester.spawn_obstacle(
        name="f1_door",
        x=0.65,
        y=0,
        z=0,
        width=1,
        height=1,
        depth=0.025,
        yaw=0)
    tester.wait_for(1)
    tester.reset_position(x=0.0, y=0.0, z=0.05, a=0.0)
    tester.wait_for(1)
    _expect_move(tester)


def test01_step_3_5(tester):
    tester.clean_obstacle()
    tester.delete_obstacle(name="f1_door")
    tester.reset_position(x=-1.0, y=0.0, a=0.0)
    tester.spawn_obstacle(
        name="f1_door",
        x=0.65,
        y=0,
        z=0,
        width=1,
        height=1,
        depth=0.035,
        yaw=0)
    tester.wait_for(1)
    tester.reset_position(x=0.0, y=0.0, z=0.05, a=0.0)
    tester.wait_for(1)
    _expect_no_move(tester)


def test01_2_step_3_5(tester):
    tester.clean_obstacle()
    tester.delete_obstacle(name="f1_door")
    tester.reset_position(x=-1.0, y=0.0, a=0.0)
    tester.spawn_obstacle(
        name="f1_door",
        x=0.65,
        y=0,
        z=0,
        width=1,
        height=1,
        depth=0.035,
        yaw=0)
    tester.wait_for(1)
    tester.reset_position(x=0.0, y=0.0, z=0.05, a=0.0)
    tester.wait_for(1)
    _expect_no_move(tester)
    tester.wait_for(1)
    tester.delete_obstacle(name="f1_door")
    tester.wait_for(1)
    _expect_move(tester)


def test02_step_down_2_5(tester):
    tester.clean_obstacle()
    tester.delete_obstacle(name="f1_door")
    tester.reset_position(x=1.0, y=0.0, a=0.0)
    tester.spawn_obstacle(
        name="f1_door",
        x=-0.35,
        y=0,
        z=0,
        width=1,
        height=1,
        depth=0.025,
        yaw=0)
    tester.wait_for(1)
    tester.reset_position(x=0.0, y=0.0, z=0.05, a=0.0)
    tester.wait_for(1)
    _expect_move(tester)


def test03_step_down_3_5(tester):
    tester.clean_obstacle()
    tester.delete_obstacle(name="f1_door")
    tester.reset_position(x=1.0, y=0.0, a=0.0)
    tester.spawn_obstacle(
        name="f1_door",
        x=-0.35,
        y=0,
        z=0,
        width=1,
        height=1,
        depth=0.035,
        yaw=0)
    tester.wait_for(1)
    tester.reset_position(x=-0.00, y=0.0, z=0.035, a=0.0)
    tester.wait_for(1)
    _expect_move(tester)
