"""

Python implementation of Conflict-based search

author: Ashwin Bose (@atb033)

"""
import sys
import time

sys.path.insert(0, '../')
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy

from a_star import AStar


class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return str((self.x, self.y))


class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location.x) + str(self.location.y))

    def is_equal_except_time(self, state):
        return self.location == state.location

    def __str__(self):
        return str((self.time, self.location.x, self.location.y))


class Conflict(object):
    VERTEX = 1
    EDGE = 2

    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + ', ' + str(
            self.location_1) + ', ' + str(self.location_2) + ')'


class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location))

    def __str__(self):
        return '(' + str(self.time) + ', ' + str(self.location) + ')'


class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2

    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 and self.location_2 == other.location_2

    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))

    def __str__(self):
        return '(' + str(self.time) + ', ' + str(self.location_1) + ', ' + str(self.location_2) + ')'


class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints]) + "EC: " + str(
            [str(ec) for ec in self.edge_constraints])


class Environment(object):
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension
        self.obstacles = obstacles

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y + 1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y - 1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x - 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x + 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors

    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t + 1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t + 1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        return 0 <= state.location.x < self.dimension[0] \
               and 0 <= state.location.y < self.dimension[1] \
               and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
               and (state.location.x, state.location.y) not in self.obstacles

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)

    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']: {'start': start_state, 'goal': goal_state}})

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent: local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])

    def transfer_wait(self, schedule):
        # ouput.yaml을 불러옴
        robot_n = len(self.agents)
        # 가장 긴 타임스텝 길이를 구한다.
        max_val = -1
        for i in schedule:
            if max_val < len(schedule[i]):
                max_val = len(schedule[i])
        dir_list = []
        # 우선 모든 agnet의 방향을 제자리로 길이는 최대길이로 초기화
        for _ in range(0, robot_n):
            dir_list.append([])
        for sub_list in dir_list:
            for _ in range(0, max_val):
                sub_list.append(5)

        # 타임 스텝별 좌표인 schedule 데이터를 dir_list에 [방향,이동 전 좌표, 이동 후 좌표] 형식으로 저장
        for k, agent in enumerate(schedule):
            if len(schedule[agent]) == 1:
                dir_list[k][0] = [5, [schedule[agent][0]["x"], schedule[agent][0]["y"]],
                                  [schedule[agent][0]["x"], schedule[agent][0]["y"]]]

            for j in range(1, len(schedule[agent])):
                if schedule[agent][j]["x"] == schedule[agent][j - 1]["x"] + 1:
                    dir_list[k][schedule[agent][j]["t"] - 1] = [6,
                                                                [schedule[agent][j - 1]["x"],
                                                                 schedule[agent][j - 1]["y"]],
                                                                [schedule[agent][j]["x"], schedule[agent][j]["y"]]]
                elif schedule[agent][j]["x"] == schedule[agent][j - 1]["x"] - 1:
                    dir_list[k][schedule[agent][j]["t"] - 1] = [4,
                                                                [schedule[agent][j - 1]["x"],
                                                                 schedule[agent][j - 1]["y"]],
                                                                [schedule[agent][j]["x"], schedule[agent][j]["y"]]]
                elif schedule[agent][j]["y"] == schedule[agent][j - 1]["y"] + 1:
                    dir_list[k][schedule[agent][j]["t"] - 1] = [8,
                                                                [schedule[agent][j - 1]["x"],
                                                                 schedule[agent][j - 1]["y"]],
                                                                [schedule[agent][j]["x"], schedule[agent][j]["y"]]]
                elif schedule[agent][j]["y"] == schedule[agent][j - 1]["y"] - 1:
                    dir_list[k][schedule[agent][j]["t"] - 1] = [2,
                                                                [schedule[agent][j - 1]["x"],
                                                                 schedule[agent][j - 1]["y"]],
                                                                [schedule[agent][j]["x"], schedule[agent][j]["y"]]]
                else:
                    dir_list[k][schedule[agent][j]["t"] - 1] = [5,
                                                                [schedule[agent][j - 1]["x"],
                                                                 schedule[agent][j - 1]["y"]],
                                                                [schedule[agent][j]["x"], schedule[agent][j]["y"]]]

        # 위 작업의 연장으로 제자리일경우에 dir_list 형식을 맞추기 위한 작업
        for i in dir_list:
            for j in range(1, len(i)):
                if i[j] == 5:
                    i[j] = [5, [i[j - 1][1][0], i[j - 1][1][1]], [i[j - 1][1][0], i[j - 1][1][1]]]

        # 특정 타임 스텝에서 agnet1의 시작점과 agent2의 도착점이 대각선에 위치하고 agnet1의 도착점이 agnet2의 시작점과 같을때 충돌이 발생하는데 이 경우를 찾기 위한 작업
        temp_list = []
        for i in range(0, max_val):
            temp_list.append([])

        # agnet1의 시작점과 agent2의 도착점이 대각선에 위치하고 agnet1의 도착점이 agnet2의 시작점과 같을때를 temp_list에 추가
        for i in range(0, max_val):
            for j in range(0, robot_n):
                for k in range(0, robot_n):
                    if (j != k):
                        if (dir_list[j][i][1] == dir_list[k][i][2] and not (
                                dir_list[k][i][1][0] == dir_list[j][i][2][0] or dir_list[k][i][1][1] ==
                                dir_list[j][i][2][
                                    1])):
                            temp_list[i].append([dir_list[k][i][1], dir_list[j][i][1], k, j])

        # 조건에 해당되는 경우를 위에서 모두 temp_list에 추가한뒤 이 temp_list 때문에 agent가 대기함으로써 추가적으로 대기 해야 하는 agent들을 찾아서
        # 마찬가지로 temp_list에 추가
        for i in range(0, max_val):
            if temp_list[i]:
                for j in temp_list[i]:
                    a = j
                    flag = 1
                    while flag:
                        flag = 0
                        for k in range(0, robot_n):
                            if dir_list[k][i][2] == a[0]:
                                if not ([dir_list[k][i][1], dir_list[a[2]][i][1], k, a[2]] in temp_list[i]):
                                    temp_list[i].append([dir_list[k][i][1], dir_list[a[2]][i][1], k, a[2]])
                                    flag = 1
                                    a = [dir_list[k][i][1], dir_list[a[2]][i][1], k, a[2]]

        # 위 작업을 모두 마쳤으면 time step 별로 앞서서 이동하는 agent와 뒤따라가는 agnet간의 충돌 때문에 대기 해야하는 case를 모두 찾은 상태이므로
        # time step별로 경로를 재구성하기위해 충돌 case마다 요소 맨 뒤에 얼마나 대기해야하는지를 추가함. 또 해당 time_step에서 최대대기시간을 구해 temp_list의 마지막요소로 추가함.
        for i in temp_list:
            max_n = 0
            if i:
                for j in i:
                    a = j
                    n = 1
                    flag = 1
                    while flag:
                        flag = 0
                        for k in i:
                            if a[1] == k[0] and k != j:
                                flag = 1
                                if not (a[0][0] == k[1][0] or a[0][1] == k[1][1]):
                                    n = n + 1
                                a = k
                    j.append(n)
                    if n > max_n:
                        max_n = n
            max_val = max_val + max_n
            i.append(max_n)

        # dir_list2가 agent별 최종 방향 경로를 담을 리스트
        dir_list2 = []
        for i in range(0, robot_n):
            dir_list2.append([])

        for i in dir_list2:
            for j in range(0, max_val):
                i.append(5)

        idx = 0
        t = 0

        # 위의 결과들로 path를 재구성
        for i in temp_list:

            a = i[-1]  # time step 마다 최대 대기 길이

            # path 재구성 과정
            for j in range(0, robot_n):
                flag = 0
                for k in i:
                    # print(k)
                    if not isinstance(k, list):
                        break
                    if k[2] == j:
                        flag = 1
                        dir_list2[j][idx + k[4]] = dir_list[j][t][0]

                if flag == 0:
                    dir_list2[j][idx] = dir_list[j][t][0]

            idx = idx + a + 1  # 재구성되는 path의 인덱스
            t = t + 1  # 실제 path의 인덱스

        # 로봇의 pose로 다시 변경
        re_schedule = {}
        for i, agent_dir in enumerate(dir_list2):
            t = 0
            agent_name = "agent" + str(i)
            agent_schedule = [{'x': schedule[agent_name][0]['x'], 'y': schedule[agent_name][0]['y'], 't': t}]
            t += 1
            i = -1
            while agent_dir[i] == 5:
                i -= 1
            for direction in agent_dir[:i + 1]:
                x = agent_schedule[t - 1]['x']
                y = agent_schedule[t - 1]['y']
                if direction == 2:
                    y -= 1
                elif direction == 6:
                    x += 1
                elif direction == 8:
                    y += 1
                elif direction == 4:
                    x -= 1
                agent_schedule.append({'x': x, 'y': y, 't': t})
                t += 1
            re_schedule[agent_name] = agent_schedule
        return re_schedule


class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost


class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()

    def search(self):
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                print("solution found")

                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t': state.time, 'x': state.location.x, 'y': state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan


def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument("param", help="input file containing map and obstacles")
    # parser.add_argument("output", help="output file with the schedule")
    # args = parser.parse_args()

    # Read from input file
    with open("input.yaml", 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    obstacles = list(map(tuple, param["map"]["obstacles"]))
    agents = param['agents']

    env = Environment(dimension, agents, obstacles)

    start_time = time.time()
    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    if not solution:
        print(" Solution not found")
        return
    solution = env.transfer_wait(solution)
    end_time = time.time()
    print("수행 시간: " + str(round(end_time - start_time, 3)))

    # Write to output file
    with open("output.yaml", 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    print("비용: " + str(output["cost"]))
    with open("output.yaml", 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)


if __name__ == "__main__":
    main()
