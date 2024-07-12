import math

def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0

def wait_ready(tester):
    tester.wait_localization_started()

def test1_collision_module_test(tester):
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
        cancel = tester.check_collision_obstacle()
        tester.wait_for(1)
        cancel()
        # Place robot outside. Collision must NOT be detected.
        robot_pose = {'x':obstacle_pos_x+lim_collision_x*1.1,
                      'y':obstacle_pos_y+lim_collision_y*1.1,
                      'a':0.0}
        tester.reset_position(**robot_pose)
        cancel = tester.check_no_collision_obstacle()
        tester.wait_for(1)
        cancel()
    tester.clean_obstacle()

def test2_collision_module_rotation_test(tester):
    robot_radius = 0.45
    obstacle_pos_x = 4.
    obstacle_pos_y = -2.
    width = 3.
    height = 5.
    depth = 0.2
    yaw = 30*math.pi/180
    lim_polygon_points = [
            [width/2.,     0],
            [width/2.,     height/2.*0.5],
            [width/2.,     height/2.],
            [width/2.*0.5, height/2.],
            [0,            height/2.],
            [-width/2.*0.5,height/2.],
            [-width/2.,    height/2.],
            [-width/2.,    height/2.*0.5],
            [-width/2.,    0],
            [-width/2.,    -height/2.*0.5],
            [-width/2.,    -height/2.],
            [-width/2.*0.5,-height/2.],
            [0,            -height/2.],
            [width/2.*0.5, -height/2.],
            [width/2.,     -height/2.],
            [width/2.,     -height/2.*0.5]
            ]

    tester.spawn_obstacle(
            name=f"{int(depth*1e3)}mm_step", \
            x=obstacle_pos_x, y=obstacle_pos_y, z=0., yaw=yaw, \
            width=width, height=height, depth=depth \
            )
    tester.wait_for(1)
    for lim_polygon_point in lim_polygon_points:
        # Place robot inside. Collision must be detected.
        def rotatePoint(x,y,yaw):
            x_rotated = x*math.cos(yaw) - y*math.sin(yaw)
            y_rotated = x*math.sin(yaw) + y*math.cos(yaw)
            return x_rotated, y_rotated

        lim_collision = [0,0]
        if(abs(lim_polygon_point[0])==width/2.
                and abs(lim_polygon_point[1])==height/2.):
            lim_collision[0] = lim_polygon_point[0] \
                               + math.copysign(robot_radius/math.sqrt(2), \
                                               lim_polygon_point[0])
            lim_collision[1] = lim_polygon_point[1] \
                               + math.copysign(robot_radius/math.sqrt(2), \
                                               lim_polygon_point[1])
        elif(abs(lim_polygon_point[0])==width/2.):
            lim_collision[0] = lim_polygon_point[0] \
                               + math.copysign(robot_radius, \
                                               lim_polygon_point[0])
            lim_collision[1] = lim_polygon_point[1]
        else:
            lim_collision[0] = lim_polygon_point[0]
            lim_collision[1] = lim_polygon_point[1] \
                               + math.copysign(robot_radius, \
                                               lim_polygon_point[1])
        lim_collision_x, lim_collision_y = rotatePoint(*lim_collision,yaw=yaw)
        robot_pose = {'x':obstacle_pos_x+lim_collision_x*0.8,
                      'y':obstacle_pos_y+lim_collision_y*0.8,
                      'a':0.0}
        tester.reset_position(**robot_pose)
        cancel = tester.check_collision_obstacle()
        tester.wait_for(1)
        cancel()
        # Place robot outside. Collision must NOT be detected.
        robot_pose = {'x':obstacle_pos_x+lim_collision_x*1.1,
                      'y':obstacle_pos_y+lim_collision_y*1.1,
                      'a':0.0}
        tester.reset_position(**robot_pose)
        cancel = tester.check_no_collision_obstacle()
        tester.wait_for(1)
        cancel()
    tester.clean_obstacle()
