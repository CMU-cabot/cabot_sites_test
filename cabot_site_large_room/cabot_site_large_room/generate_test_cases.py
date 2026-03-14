import random
import os
import math

# Configuration Variables
MIN_PEOPLE = 30
MAX_PEOPLE = 35
MIN_X = -8.0
MAX_X = 8.0
MIN_Y = -8.0
MAX_Y = 8.0
VELOCITY_MIN = 0.8
VELOCITY_MAX = 1.2
MIN_NUM_PAIRS = 3
MAX_NUM_PAIRS = 5 # Number of pairs to generate per test case will be random between these
CASES = 100

# Child Parameters
# These are default values, used if interval_settings is NOT provided
CHILD_CHANGE_INTERVAL_MIN = 7
CHILD_CHANGE_INTERVAL_MAX = 9
CHILD_CHANGE_PROBABILITY = 1.0
CHILD_VELOCITY_RANGE = [0.8, 1.2]
CHILD_RANDOM_SEED = 100

# Output File Settings
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

def get_header():
    return """# ******************************************************************************
#  Copyright (c) 2024, 2025  Carnegie Mellon University and Miraikan
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# ******************************************************************************

import time


def config(tester):
    # original config
    tester.config['init_x'] = -9.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 0.0

    tester.set_evaluation_parameters(
        robot_radius=0.25,
        metrics=[
            "total_time",
            "robot_path_length",
            "time_not_moving",
            "avg_robot_linear_speed",
            "cumulative_heading_changes",
            "minimum_distance_to_people",
            "minimum_distance_to_child",
            "minimum_distance_to_adult",
            "maximum_distance_to_people",
            "robot_on_person_collision_count",
            "person_on_robot_collision_count",
            "collision",
            "proximity_violation"
        ],
    )

    tester.set_people_detection_range(
        min_range=0.29,
        max_range=7.07,
        min_angle=-2.28,
        max_angle=2.28,
        occlusion_radius=0.25,
        divider_distance_m=0.05,
        divider_angle_deg=1.0
    )


def checks(tester):
    tester.check_topic_error(
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/interface' and msg.text=='vibration' and msg.memo=='unknown'"
    )


def wait_ready(tester):
    tester.wait_ready()


def _goto_target1(tester):
    tester.pub_topic(
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        message="data: 'navigation;destination;EDITOR_node_1705948557561'"
    )
    # start computing evaluation metrics after publshing the destination
    tester.start_evaluation()
    tester.wait_topic(
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/navigation' and msg.text=='completed'",
        timeout=120
    )

    # Send navigation cancellation in case it ends due to timeout
    tester.pub_topic(
        topic='/cabot/event',
        topic_type='std_msgs/msg/String',
        message="data: 'navigation;cancel'"
    )


def _add_metric_condition_lt(tester, metric_name, success_threshold):
    condition = f"{success_threshold} > value"
    tester.add_metric_condition({"name": metric_name, "condition": condition})


def _add_metric_condition_gt(tester, metric_name, success_threshold):
    condition = f"{success_threshold} < value"
    tester.add_metric_condition({"name": metric_name, "condition": condition})

def _setup_actors(tester, actors):
    tester.setup_actors(actors=actors, timeout=180)
    time.sleep(5)
"""

def generate_actor_config(name, init_x, init_y, init_a, velocity, goal_x, goal_y, n_actors, is_child=False, 
                         interval_min=CHILD_CHANGE_INTERVAL_MIN, interval_max=CHILD_CHANGE_INTERVAL_MAX):
    module = "pedestrian.walk_sfm_child" if is_child else "pedestrian.walk_sfm"
    name_suffix = "_child" if is_child else ""
    
    if is_child:
        # Generate random sub-goals for child to wander around
        sub_goals = []
        # Add original goal
        sub_goals.append(f"[{goal_x:.2f}, {goal_y:.2f}]")
        # Add random goals within the area
        # This allows the child to change their destination based on change_probability
        for _ in range(3):
            rx = random.uniform(MIN_X, MAX_X)
            ry = random.uniform(MIN_Y, MAX_Y)
            sub_goals.append(f"[{rx:.2f}, {ry:.2f}]")
        
        goals_str = "[" + ", ".join(sub_goals) + "]"

        return f"""        {{
            "name": '{name}{name_suffix}',
            "module": "{module}",
            "params": {{
                "radius": 0.25,
                "init_x": {init_x:.2f},
                "init_y": {init_y:.2f},
                "init_a": {init_a:.2f},
                "velocity": {velocity:.2f},
                "goals": {goals_str},
                "change_interval_min": {interval_min},
                "change_interval_max": {interval_max},
                "change_probability": {CHILD_CHANGE_PROBABILITY},
                "velocity_range": {CHILD_VELOCITY_RANGE},
                "n_actors": {n_actors},
                "random_seed": {CHILD_RANDOM_SEED},
            }},
        }},"""
    else:
        return f"""        {{
            "name": '{name}',
            "module": "{module}",
            "params": {{
                "radius": 0.25,
                "init_x": {init_x:.2f},
                "init_y": {init_y:.2f},
                "init_a": {init_a:.2f},
                "velocity": {velocity:.2f},
                "goal_x": {goal_x:.2f},
                "goal_y": {goal_y:.2f},
                "n_actors": {n_actors},
            }},
        }},"""

def get_base_scenario(case_index, is_stopped):
    # Deterministic generation based on case_index
    # We use a specific seed for each case_index to ensure 
    # the "shape" of the scenario is consistent whenever called for the same index.
    # However, to be consistent across different calls (different files),
    # we just need to re-seed with case_index.
    
    seed = case_index * 1000 + (1 if is_stopped else 2)
    rng = random.Random(seed)
    
    n_people = rng.randint(MIN_PEOPLE, MAX_PEOPLE)
    actors_setup = []
    
    # Generate abstract actors (pair or single, position, velocity, goal)
    # But WITHOUT "type" (adult/child) yet.
    
    pair_idx = 0
    single_idx = 0
    
    # Randomly successful number of pairs
    num_pairs = rng.randint(MIN_NUM_PAIRS, MAX_NUM_PAIRS)

    # Pairs (try to make num_pairs)
    pairs_cnt = 0
    while pairs_cnt < num_pairs and (n_people - len(actors_setup)) >= 2:
        px = rng.uniform(MIN_X, MAX_X)
        py = rng.uniform(MIN_Y, MAX_Y)
        pa = rng.uniform(-180, 180)
        
        if is_stopped:
            # Set velocity same as walking actors, but distance is 0 (start==goal)
            vel = rng.uniform(VELOCITY_MIN, VELOCITY_MAX)
            gx, gy = px, py
        else:
            vel = rng.uniform(VELOCITY_MIN, VELOCITY_MAX)
            gx = rng.uniform(MIN_X, MAX_X)
            gy = rng.uniform(MIN_Y, MAX_Y)
            
        offset_dist = 1.0 # Increased from 0.5 to avoid potential overlap issues
        offset_ang = rng.uniform(0, 360) 
        
        # Ensure velocity is not exactly 0.0 even for stopped actors if that causes issues,
        # but usually 0.0 is fine.
        
        p1 = {
            "name": f"actor_pair{pair_idx}_0",
            "init_x": px, "init_y": py, "init_a": pa,
            "velocity": vel, "goal_x": gx, "goal_y": gy
        }
        
        p2_x = px + offset_dist * math.cos(math.radians(offset_ang))
        p2_y = py + offset_dist * math.sin(math.radians(offset_ang))
        
        p2 = {
            "name": f"actor_pair{pair_idx}_1",
            "init_x": p2_x, "init_y": p2_y, "init_a": pa,
            "velocity": vel, "goal_x": gx, "goal_y": gy
        }
        
        actors_setup.append(p1)
        actors_setup.append(p2)
        pairs_cnt += 1
        pair_idx += 1
        
    # Singles
    while len(actors_setup) < n_people:
        sx = rng.uniform(MIN_X, MAX_X)
        sy = rng.uniform(MIN_Y, MAX_Y)
        sa = rng.uniform(-180, 180)
        
        vel = rng.uniform(VELOCITY_MIN, VELOCITY_MAX)
        gx = rng.uniform(MIN_X, MAX_X)
        gy = rng.uniform(MIN_Y, MAX_Y)
            
        actor = {
            "name": f"actor_single{single_idx}",
            "init_x": sx, "init_y": sy, "init_a": sa,
            "velocity": vel, "goal_x": gx, "goal_y": gy
        }
        actors_setup.append(actor)
        single_idx += 1
        
    return actors_setup

def generate_test_case(adult_ratio, child_ratio, case_index, is_stopped, interval_settings=None):
    suffix = "stopped" if is_stopped else "walking"
    if interval_settings:
        interval_min = interval_settings["min"]
        interval_max = interval_settings["max"]
        if "label" in interval_settings:
            # We add suffix to the test case function name as well
            suffix += f"_{interval_settings['label']}"
    else:
        interval_min = CHILD_CHANGE_INTERVAL_MIN
        interval_max = CHILD_CHANGE_INTERVAL_MAX

    case_name = f"tests_adult_{adult_ratio}_child_{child_ratio}_test_case_{case_index:02d}_{suffix}"
    
    # 1. Get Base Scenario (Positions, Goals, etc.) - Identical for all ratios for same case_index
    actors_info = get_base_scenario(case_index, is_stopped)
    n_actors = len(actors_info)
    
    # 2. Assign Types (Adult/Child) based on ratio
    rng_type = random.Random(case_index * 10000 + child_ratio)
    
    n_children = int(n_actors * (child_ratio / 100.0))
    # Ensure counts match exactly if 0 or 100
    if child_ratio == 0: n_children = 0
    if child_ratio == 100: n_children = n_actors
        
    # Create list of types
    types = [True] * n_children + [False] * (n_actors - n_children) # True for child
    rng_type.shuffle(types)
    
    # Prepare actors string
    actors_str = ""
    for i, actor in enumerate(actors_info):
        is_child = types[i]
        actors_str += generate_actor_config(
            actor["name"], 
            actor["init_x"], actor["init_y"], actor["init_a"], 
            actor["velocity"], actor["goal_x"], actor["goal_y"], 
            n_actors, is_child, 
            interval_min=interval_min, interval_max=interval_max
        ) + "\n"

    content = f"""
def {case_name}(tester):
    # Pairs Moving: {not is_stopped}
    tester.check_collision()
    tester.reset_position()
    
    n_actors = {n_actors}
    
    _setup_actors(tester, actors=[
{actors_str}    ])
    
    _add_metric_condition_lt(tester, "total_time", 75)
    _add_metric_condition_lt(tester, "robot_path_length", 30.0)
    _add_metric_condition_lt(tester, "robot_on_person_collision_count", 1)
    
    _goto_target1(tester)
"""
    return content


def generate_file(adult_ratio, child_ratio, limit_case_id=0, interval_settings=None):
    filename = f"tests_adult_{adult_ratio}_child_{child_ratio}.py"
    if interval_settings and "label" in interval_settings:
        filename = f"tests_adult_{adult_ratio}_child_{child_ratio}_{interval_settings['label']}.py"
        
    filepath = os.path.join(OUTPUT_DIR, filename)
    
    # Check if file exists to preserve content of other cases if limits are applied
    existing_content = ""
    start_lines = {}
    end_lines = {}
    lines = []
    
    if os.path.exists(filepath):
        if limit_case_id != 0:
            with open(filepath, 'r') as f:
                lines = f.readlines()
                existing_content = "".join(lines)
                
            # Parse functions positions
            current_func = None
            for i, line in enumerate(lines):
                if line.startswith("def tests_adult_"):
                    func_name = line.strip().split("(")[0].replace("def ", "")
                    start_lines[func_name] = i
                    if current_func:
                        end_lines[current_func] = i - 1
                    current_func = func_name
            if current_func:
                end_lines[current_func] = len(lines)

    # Prepare logic to generate new content
    if limit_case_id == 0:
        content = get_header()
        
        # Determine for each case if it is stopped or walking randomly (but deterministic per case index)
        # Using a seed for this
        rng_scenario = random.Random(adult_ratio * 100 + child_ratio)

        for i in range(1, CASES + 1):
             is_stopped = rng_scenario.choice([True, False])
             content += generate_test_case(adult_ratio, child_ratio, i, is_stopped=is_stopped, interval_settings=interval_settings)
             
        with open(filepath, 'w') as f:
            f.write(content)
        print(f"Generated {filename}")
        
    else:
        rng_scenario = random.Random(adult_ratio * 100 + child_ratio)
        is_stopped = None
        for i in range(1, CASES + 1):
            val = rng_scenario.choice([True, False])
            if i == limit_case_id:
                is_stopped = val
                break
        
        if is_stopped is None:
             print(f"Case {limit_case_id} out of range (max {CASES})")
             return

        suffix = "stopped" if is_stopped else "walking"
        if interval_settings and "label" in interval_settings:
            suffix += f"_{interval_settings['label']}"
            
        target_func_name = f"tests_adult_{adult_ratio}_child_{child_ratio}_test_case_{limit_case_id:02d}_{suffix}"
        
        if target_func_name in start_lines:
             print(f"Refining {target_func_name} in {filename}...")
             new_code = generate_test_case(adult_ratio, child_ratio, limit_case_id, is_stopped, interval_settings=interval_settings)
             
             start_idx = start_lines[target_func_name]
             end_idx = end_lines[target_func_name]
             
             with open(filepath, 'w') as f:
                 f.writelines(lines[:start_idx])
                 f.write(new_code)
                 f.writelines(lines[end_idx+1:])

        else:
             print(f"Target case {limit_case_id} not found in {filename}. Regenerating full file recommended.")


if __name__ == "__main__":
    TARGET_CASE_ID = 0 # 0 means generate all
    
    ratios = [
        (100, 0), (90, 10), (80, 20), (70, 30), (60, 40), 
        (50, 50), (40, 60), (30, 70), (20, 80), (10, 90), (0, 100)
    ]
    
    interval_options = [
        {"label": "high", "min": 1, "max": 2},
        {"label": "medium", "min": 3, "max": 5},
        {"label": "low", "min": 7, "max": 9},
    ]

    for ar, cr in ratios:
        if cr == 0:
             generate_file(ar, cr, TARGET_CASE_ID)
        else:
             for interval in interval_options:
                 generate_file(ar, cr, TARGET_CASE_ID, interval_settings=interval)
