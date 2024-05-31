def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    tester.wait_localization_started()


def test1_limit_speed_when_people_topic_is_dead(tester):
    # go straight
    tester.reset_position(x=0.0, y=1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899150598')
    # actor for publishing /people
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 10.0,
                "init_y": 11.0,
                "init_a": -90.0
            },
        }
    ])
    # check if the limit is over 0.75
    tester.wait_topic(
        action_name='check_speed1',
        topic='/cabot/people_speed',
        topic_type='std_msgs/msg/Float32',
        condition="msg.data > 0.75",
        timeout=10
    )
    tester.wait_for(5)
    # stop /people
    tester.delete_actor(name="actor0")
    tester.wait_for(2)
    # error if the limit is over 0.75
    cancel = tester.check_topic_error(
        action_name='check_speed2',
        topic='/cabot/people_speed',
        topic_type='std_msgs/msg/Float32',
        condition="msg.data > 0.75"
    )
    # check if the robot speed is over 0.5
    tester.wait_topic(
        action_name='check_speed3',
        topic='/cabot/people_speed',
        topic_type='std_msgs/msg/Float32',
        condition="msg.data < 0.75",
        timeout=10
    )
    tester.wait_navigation_arrived(timeout=90)
    cancel()


def test2_limit_speed_when_people_topic_is_dead_and_restored(tester):
    # go straight
    tester.reset_position(x=0.0, y=1.0, a=0.0)
    tester.goto_node('EDITOR_node_1707899150598')
    # actor for publishing /people
    tester.setup_actors(actors=[
        {
            "name": 'actor1',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 10.0,
                "init_y": 11.0,
                "init_a": -90.0
            },
        }
    ])
    # check if the robot speed is over 0.75
    tester.wait_topic(
        action_name='check_speed1',
        topic='/cabot/people_speed',
        topic_type='std_msgs/msg/Float32',
        condition="msg.data > 0.75",
        timeout=10
    )
    tester.wait_for(5)
    # stop /people
    tester.delete_actor(name="actor1")
    tester.wait_for(2)
    # error if the limit is over 0.75
    cancel = tester.check_topic_error(
        action_name='check_speed2',
        topic='/cabot/people_speed',
        topic_type='std_msgs/msg/Float32',
        condition="msg.data > 0.75"
    )
    # check if the robot speed is over 0.5
    tester.wait_topic(
        action_name='check_speed3',
        topic='/cabot/people_speed',
        topic_type='std_msgs/msg/Float32',
        condition="msg.data < 0.75",
        timeout=10
    )
    tester.wait_navigation_arrived(timeout=90)
    cancel()
    tester.goto_node('EDITOR_node_1707899216479')
    # actor for publishing /people
    tester.setup_actors(actors=[
        {
            "name": 'actor2',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 10.0,
                "init_y": 11.0,
                "init_a": -90.0
            },
        }
    ])
    tester.wait_topic(
        action_name='check_speed4',
        topic='/cabot/people_speed',
        topic_type='std_msgs/msg/Float32',
        condition="msg.data > 0.75",
        timeout=10
    )
    tester.wait_navigation_arrived(timeout=90)

def test3_obstacle_avoidance(tester):
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor9',
            "module": "pedestrian.obstacle",
            "params": {
                "init_x": 5.0,
                "init_y": 2.5,
                "init_a": 180.0,
                "velocity": 0.5,
                "decel_distance": 1.5,
                "pause_distance": 1.0
            },
        },
    ])
