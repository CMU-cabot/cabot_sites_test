
import re
import math
import sys

file_path = "/home/ai-suitcase-1/nitta_workspace/cabot/cabot-navigation/cabot_sites/cabot_sites_test/cabot_site_large_room/cabot_site_large_room/tests_adult_100_child_0.py"

robot_init_x = -9.0
robot_init_y = 0.0
robot_goal_x = 9.0
robot_goal_y = 0.0
min_dist = 1.5

with open(file_path, 'r') as f:
    lines = f.readlines()

new_lines = []
in_params = False
current_params = {}
param_lines_indices = {}

for i, line in enumerate(lines):
    # Check for start of params block
    if '"params": {' in line:
        in_params = True
        current_params = {}
        param_lines_indices = {}
        new_lines.append(line)
        continue

    if in_params:
        # Check for end of params block
        if '},' in line and line.strip() == '},':
            # Process the collected params
            
            # 1. Check Init
            if 'init_x' in current_params and 'init_y' in current_params:
                ix = current_params['init_x']
                iy = current_params['init_y']
                dx = ix - robot_init_x
                dy = iy - robot_init_y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist < min_dist:
                    print(f"Adjusting Init at line {i}: dist={dist:.2f} < {min_dist}")
                    if dist < 0.01:
                        dx = 1.0
                        dy = 0.0
                        dist = 1.0
                    factor = min_dist / dist
                    new_ix = robot_init_x + dx * factor
                    new_iy = robot_init_y + dy * factor
                    
                    # Update lines
                    idx_x = param_lines_indices['init_x']
                    idx_y = param_lines_indices['init_y']
                    
                    # formatting assuming "key": value,
                    new_lines[idx_x] = re.sub(r':\s*[-0-9.]+,', f': {new_ix:.2f},', new_lines[idx_x])
                    new_lines[idx_y] = re.sub(r':\s*[-0-9.]+,', f': {new_iy:.2f},', new_lines[idx_y])

            # 2. Check Goal
            if 'goal_x' in current_params and 'goal_y' in current_params:
                gx = current_params['goal_x']
                gy = current_params['goal_y']
                dx = gx - robot_goal_x
                dy = gy - robot_goal_y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist < min_dist:
                    print(f"Adjusting Goal at line {i}: dist={dist:.2f} < {min_dist}")
                    new_gx = gx - 1.5
                    
                    # Update line
                    idx_x = param_lines_indices['goal_x']
                    
                    new_lines[idx_x] = re.sub(r':\s*[-0-9.]+,', f': {new_gx:.2f},', new_lines[idx_x])
            
            in_params = False
            new_lines.append(line)
            continue
        
        # Parse values
        match = re.search(r'"(init_x|init_y|goal_x|goal_y)":\s*([-0-9.]+),', line)
        if match:
            key = match.group(1)
            val = float(match.group(2))
            current_params[key] = val
            param_lines_indices[key] = len(new_lines) # index in new_lines where this line will be placed
            
        new_lines.append(line)
    else:
        new_lines.append(line)

with open(file_path, 'w') as f:
    f.writelines(new_lines)
