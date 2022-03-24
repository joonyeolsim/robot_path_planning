import yaml
import os
from time import sleep

if __name__ == '__main__':
    map = []
    with open('../maps/logistics_world/Map', 'r') as f:
        lines = f.readlines()
        for line in lines:
            map.append(list(line[:-1]))

    with open("output.yaml", 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    robot1_schedule = param["schedule"]["robot1"]
    robot2_schedule = param["schedule"]["robot2"]
    robot3_schedule = param["schedule"]["robot3"]
    robot4_schedule = param["schedule"]["robot4"]
    max_t = max(len(robot1_schedule), len(robot2_schedule))

    for t in range(max_t):
        if len(robot1_schedule) > t:
            map[robot1_schedule[t]['x']][robot1_schedule[t]['y']] = 'a'
        if len(robot2_schedule) > t:
            map[robot2_schedule[t]['x']][robot2_schedule[t]['y']] = 'b'
        if len(robot3_schedule) > t:
            map[robot3_schedule[t]['x']][robot3_schedule[t]['y']] = 'c'
        if len(robot4_schedule) > t:
            map[robot4_schedule[t]['x']][robot4_schedule[t]['y']] = 'd'
        for row in map:
            for occupancy in row:
                print(occupancy, end=' ')
            print()
        if len(robot1_schedule) > t:
            map[robot1_schedule[t]['x']][robot1_schedule[t]['y']] = '0'
        if len(robot2_schedule) > t:
            map[robot2_schedule[t]['x']][robot2_schedule[t]['y']] = '0'
        if len(robot3_schedule) > t:
            map[robot3_schedule[t]['x']][robot3_schedule[t]['y']] = '0'
        if len(robot4_schedule) > t:
            map[robot4_schedule[t]['x']][robot4_schedule[t]['y']] = '0'
        sleep(1)
        os.system('clear')