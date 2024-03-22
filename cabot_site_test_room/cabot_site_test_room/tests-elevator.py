def config(tester):
    tester.config['init_x'] = 10.0
    tester.config['init_y'] = 5.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 90.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()
    # tester.wait_ready()


def test9_go_to_elevator_out(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1709594305617')
    tester.wait_goal("ElevatorTurnGoal")
    tester.floor_change(+1)
    tester.wait_navigation_arrived(timeout=60)


def test8_check_elevator_button_direction_and_goal_in_the_cab(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1707899235671')
    tester.wait_goal("NavGoal")
    tester.wait_topic(
        action="check_button_direction",
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.memo != 'en' or (msg.category=='speech request' and 'front left' in msg.text)"
    )
    tester.wait_navigation_arrived(timeout=60)


def test7_start_in_front_of_elevator(tester):
    tester.reset_position(x=11.5, y=8.5, z=0)
    tester.goto_node('EDITOR_node_1709594309586')
    tester.wait_goal("ElevatorTurnGoal")
    tester.floor_change(+1)
    tester.wait_navigation_arrived(timeout=60)


def test6_elevator_skip_and_resume_in_elevator(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1709594309586')
    tester.wait_goal("ElevatorWaitGoal")
    tester.button_down(3)
    tester.reset_position(x=13, y=8.5, z=0)
    tester.floor_change(+1)
    tester.button_down(4)
    tester.wait_navigation_arrived(timeout=60)


def test5_elevator_skip(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1709594309586')
    tester.wait_goal("ElevatorWaitGoal")
    tester.button_down(3)
    tester.floor_change(+1)
    tester.wait_for(seconds=10)
    tester.button_down(4)
    tester.wait_navigation_arrived(timeout=60)


def test4_cancel_while_elevator_floor_goal(tester):
    tester.clean_door()
    tester.reset_position()
    tester.goto_node('EDITOR_node_1709594309586')
    tester.wait_goal("ElevatorTurnGoal", timeout=60)
    tester.info("cancel elevator floor goal")
    tester.wait_for(seconds=2)
    tester.info("push left button to pause")
    tester.button_down(3)
    tester.wait_for(seconds=3)
    # navigation is paused, so ElevatorFloorGoal should not be published
    cancel = tester.check_topic_error(
        action="check_elevator_floor_goal_error",
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/navigation' and msg.text=='goal_canceled' and msg.memo=='ElevatorFloorGoal'",
    )
    tester.floor_change(+1)
    tester.wait_for(seconds=10)
    cancel()
    tester.info("push right button to resume")
    tester.button_down(4)
    tester.wait_goal("ElevatorOutGoal", timeout=20)
    tester.wait_navigation_arrived(timeout=30)


def test3_door_close_while_elevator_out(tester):
    tester.clean_door()
    tester.reset_position()
    tester.goto_node('EDITOR_node_1709594309586')
    tester.info("close elvator doors")
    tester.spawn_door(name="f1_door", x=12, y=8.5, z=0, yaw=0)
    tester.spawn_door(name="f3_door", x=12, y=8.5, z=10, yaw=0)
    tester.wait_goal("ElevatorWaitGoal")
    tester.wait_for(seconds=5)
    tester.delete_door(name="f1_door")
    tester.wait_goal("ElevatorTurnGoal", timeout=60)
    tester.floor_change(+1)
    tester.wait_for(seconds=5)
    tester.info("open 3F door")
    tester.delete_door(name="f3_door")
    tester.info("close 3F door to abort Elevator Out")
    tester.spawn_door(name="f3_door", x=12, y=8.5, z=10, yaw=0)
    tester.wait_for(seconds=5)
    tester.info("open 3F door again")
    tester.delete_door(name="f3_door")
    tester.wait_navigation_arrived(timeout=30)


def test2_door_close_while_elevator_in(tester):
    tester.clean_door()
    tester.reset_position()
    tester.goto_node('EDITOR_node_1709594309586')
    tester.info("close elvator doors")
    tester.spawn_door(name="f1_door", x=12, y=8.5, z=0, yaw=0)
    tester.spawn_door(name="f3_door", x=12, y=8.5, z=10, yaw=0)
    tester.wait_goal("ElevatorWaitGoal")
    tester.wait_for(seconds=5)
    for i in range(0, 3):
        tester.info("open/close 1F door")
        tester.delete_door(name="f1_door")
        tester.spawn_door(name="f1_door", x=12, y=8.5, z=0, yaw=0)
        tester.wait_for(seconds=5)
    tester.info("open 1F door again")
    tester.delete_door(name="f1_door")
    tester.wait_goal("ElevatorTurnGoal", timeout=30)
    tester.floor_change(+1)
    tester.wait_for(seconds=5)
    tester.delete_door(name="f3_door")
    tester.wait_navigation_arrived(timeout=60)


def test1_navigation_to_a_goal_with_doors(tester):
    tester.clean_door()
    tester.reset_position()
    tester.goto_node('EDITOR_node_1709594309586')
    tester.info("close elvator doors")
    tester.spawn_door(name="f1_door", x=12, y=8.5, z=0, yaw=0)
    tester.spawn_door(name="f3_door", x=12, y=8.5, z=10, yaw=0)
    tester.wait_goal("ElevatorWaitGoal")
    tester.wait_for(seconds=5)
    tester.info("open 1F door")
    tester.delete_door(name="f1_door")
    tester.wait_goal("ElevatorTurnGoal")
    tester.floor_change(+1)
    tester.wait_for(seconds=5)
    tester.info("open 3F door")
    tester.delete_door(name="f3_door")
    tester.wait_navigation_arrived(timeout=60)


def test0_navigation_to_a_goal(tester):
    tester.clean_door()
    tester.reset_position()
    tester.goto_node('EDITOR_node_1709594309586')
    tester.wait_goal("ElevatorTurnGoal")
    tester.floor_change(+1)
    tester.wait_navigation_arrived(timeout=180)
