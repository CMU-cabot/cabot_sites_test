def config(tester):
    tester.config['init_x'] = 0.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0
    tester.config['init_floor'] = 0


def wait_ready(tester):
    # tester.wait_localization_started()
    tester.wait_ready()


def test1_queue(tester):
    tester.reset_position(x=1.0, y=-4.0, a=180.0)
    tester.goto_node('EDITOR_node_1710182288787')    
    tester.wait_goal("QueueNavFirstGoal")
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueNavLastGoal")


def test2_queue_with_pedestrian(tester):
    tester.reset_position(x=1.0, y=-4.0, a=180.0)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": -3.5,
                "init_y": -8.0,
                "init_a": 90.0
            },
        },
    ])
    tester.goto_node('EDITOR_node_1710182288787')    
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueTurnGoal")
    tester.wait_for(seconds=20)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": -3.5,
                "init_y": 0.0,
                "init_a": 90.0
            },
        },
    ])
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueNavLastGoal")


def test3_queue_nav_partway(tester):
    tester.reset_position(x=-3.5, y=-11.0, a=90.0)
    tester.goto_node('EDITOR_node_1710182288787')    
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueNavLastGoal")


def test4_queue_with_pedestrian(tester):
    tester.reset_position(x=-3.5, y=-11.0, a=90.0)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": -3.5,
                "init_y": -8.0,
                "init_a": 90.0
            },
        },
    ])
    tester.goto_node('EDITOR_node_1710182288787')    
    tester.wait_for(seconds=20)
    tester.setup_actors(actors=[
        {
            "name": 'actor0',
            "module": "pedestrian.pool",
            "params": {
                "init_x": -3.5,
                "init_y": 0.0,
                "init_a": 90.0
            },
        },
    ])
    tester.wait_goal("QueueTurnGoal")
    tester.wait_goal("QueueNavGoal")
    tester.wait_goal("QueueNavLastGoal")

