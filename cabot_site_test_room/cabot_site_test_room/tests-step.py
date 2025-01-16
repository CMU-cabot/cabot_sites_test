def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def test00_step_2_5(tester):
    tester.clean_door()
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
    tester.wait_for(1000)


def test01_step_3_5(tester):
    tester.clean_door()
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
    tester.wait_for(1000)


def test02_step_down_2_5(tester):
    tester.clean_door()
    tester.reset_position(x=1.0, y=0.0, a=0.0)
    tester.spawn_obstacle(
        name="f1_door",
        x=-0.55,
        y=0,
        z=0,
        width=1,
        height=1,
        depth=0.025,
        yaw=0)
    tester.wait_for(1)
    tester.reset_position(x=0.0, y=0.0, z=0.05, a=0.0)
    tester.wait_for(1000)


def test03_step_down_3_5(tester):
    tester.clean_door()
    tester.reset_position(x=1.0, y=0.0, a=0.0)
    tester.spawn_obstacle(
        name="f1_door",
        x=-0.55,
        y=0,
        z=0,
        width=1,
        height=1,
        depth=0.035,
        yaw=0)
    tester.wait_for(1)
    tester.reset_position(x=0.0, y=0.0, z=0.05, a=0.0)
    tester.wait_for(1000)