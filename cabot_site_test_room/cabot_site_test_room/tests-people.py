import math

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
    tester.wait_topic(
        # tester.clean_obstacle() needs tester.remaining to information of all obstacles
        topic='/obstacle_states',
        topic_type='pedestrian_plugin_msgs/msg/Agents',
        condition="True",
        timeout=10
    )
    tester.clean_obstacle()
    tester.check_collision()
    tester.reset_position()
    tester.setup_actors(actors=[
        {
            "name": 'actor8',
            "module": "pedestrian.pool",
            "params": {
                "init_x": 3.5,
                "init_y": 0.0,
                "init_a": 180.0,
            },
        },
    ])
    tester.check_collision()
    tester.spawn_obstacle(name="10mm_step", x=10.0, y=0., z=0., yaw=0.,\
                          width=1., height=1., depth=0.2)
    tester.wait_for(8)
    tester.clean_obstacle()

def test4_collision_module_test(tester):
    robot_radius = 0.45
    obstacle_pos_x = 4.
    obstacle_pos_y = -2.
    side_length = 1.
    depth = 0.2
    # 0.45 (default robot_radius) + 0.5 (obstacle width/2) = 0.95
    # obstacle returns true if x < 0.95 and false if x >= 0.95
    tester.spawn_obstacle(
            name="10mm_step", \
            x=obstacle_pos_x, y=obstacle_pos_y, z=0., yaw=0., \
            width=side_length, height=side_length, depth=depth \
            )
    tester.wait_for(1)
    for i in range(8):
        robot_pos_angle = 2.*math.pi*i/8.
        # Place robot around the polygon and check the collision
        if(i%2 != 0):
            lim_collision_x = (robot_radius + side_length/2.*math.sqrt(2))*math.cos(robot_pos_angle)
            lim_collision_y = (robot_radius + side_length/2.*math.sqrt(2))*math.sin(robot_pos_angle)
        else:
            lim_collision_x = (robot_radius + side_length/2.)*math.cos(robot_pos_angle)
            lim_collision_y = (robot_radius + side_length/2.)*math.sin(robot_pos_angle)
        # Place robot inside. Collision must be detected.
        robot_pose = {'x':obstacle_pos_x+lim_collision_x*0.8,
                      'y':obstacle_pos_y+lim_collision_y*0.8,
                      'a':0.0}
        tester.reset_position(**robot_pose)
        tester.check_collision_obstacle()
    #    ## Place robot outside. Collision must NOT be detected.
    #    #robot_pose = {'x':obstacle_pos_x+lim_collision_x*1.1,
    #    #              'y':obstacle_pos_y+lim_collision_y*1.1,
    #    #              'a':0.0}
    #    #tester.reset_position(**robot_pose)
    #    ## tester.check_no_collision_obstacle()
    #tester.clean_obstacle()

def test5(tester):
    tester.spawn_obstacle(
            name="10mm_step", \
            x=0, y=0, z=0., yaw=0., \
            width=5, height=5, depth=0.2 \
            )
    tester.reset_position(x=10.0, y=0.0, a=0.0)
    tester.check_no_collision_obstacle()
    tester.reset_position(x=0.0, y=0.0, a=0.0)
    tester.check_collision_obstacle()
