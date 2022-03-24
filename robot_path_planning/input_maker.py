import yaml
import random

if __name__ == '__main__':
    obstacle_list = []
    grid_map = []
    with open('../maps/logistics_world/Map_backup', 'r') as f:
        lines = f.readlines()
        for line in lines:
            grid_map.append(list(map(int, line.rstrip())))

    map_x = len(grid_map[0])
    map_y = len(grid_map)

    for i, row in enumerate(grid_map):
        for j, occupancy in enumerate(row):
            if occupancy:
                obstacle_list.append((j, i))

    agent_list = []
    for agent in range(10):
        while True:
            rand_x = random.randint(0, map_x - 1)
            rand_y = random.randint(0, map_y - 1)

            goal_x = random.randint(0, map_x - 1)
            goal_y = random.randint(0, map_y - 1)

            if rand_x == goal_x and goal_x == goal_y:
                continue

            if not grid_map[rand_y][rand_x] and not grid_map[goal_y][goal_x]:
                grid_map[rand_y][rand_x] = 1
                grid_map[goal_y][goal_x] = 1
                agent_list.append({
                    'start': [rand_x, rand_y],
                    'goal': [goal_x, goal_y],
                    'name': 'agent' + str(agent)
                })
                break

    with open('input.yaml', 'w') as f:
        yaml.safe_dump(
            {
                'agents': agent_list,
                'map': {
                    'dimensions': [map_x, map_y],
                    'obstacles': obstacle_list
                }
            }
            , f, default_flow_style=None)
