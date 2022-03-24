import yaml
import time

# cbs 경로를 재구성하는 스크립트
# 에이전트들의 타임스텝별 좌표 형식인 ouput.yaml을 충돌을 고려해 재구성된 타임스텝별 방향으로 바꾼다.

if __name__ == "__main__":
    start_time = time.time()
    # ouput.yaml을 불러옴
    robot_n = 10
    with open("output.yaml") as states_file:
        schedule = yaml.load(states_file, Loader=yaml.FullLoader)
    schedule = schedule["schedule"]
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
            dir_list[k][0] = [5, [schedule[agent][0]["x"], schedule[agent][0]["y"]], [schedule[agent][0]["x"], schedule[agent][0]["y"]]]

        for j in range(1, len(schedule[agent])):
            if schedule[agent][j]["x"] == schedule[agent][j - 1]["x"] + 1:
                dir_list[k][schedule[agent][j]["t"] - 1] = [6, [schedule[agent][j - 1]["x"], schedule[agent][j - 1]["y"]], [schedule[agent][j]["x"], schedule[agent][j]["y"]]]
            elif schedule[agent][j]["x"] == schedule[agent][j - 1]["x"] - 1:
                dir_list[k][schedule[agent][j]["t"] - 1] = [4, [schedule[agent][j - 1]["x"], schedule[agent][j - 1]["y"]], [schedule[agent][j]["x"], schedule[agent][j]["y"]]]
            elif schedule[agent][j]["y"] == schedule[agent][j - 1]["y"] + 1:
                dir_list[k][schedule[agent][j]["t"] - 1] = [8, [schedule[agent][j - 1]["x"], schedule[agent][j - 1]["y"]], [schedule[agent][j]["x"], schedule[agent][j]["y"]]]
            elif schedule[agent][j]["y"] == schedule[agent][j - 1]["y"] - 1:
                dir_list[k][schedule[agent][j]["t"] - 1] = [2, [schedule[agent][j - 1]["x"], schedule[agent][j - 1]["y"]], [schedule[agent][j]["x"], schedule[agent][j]["y"]]]
            else:
                dir_list[k][schedule[agent][j]["t"] - 1] = [5, [schedule[agent][j - 1]["x"], schedule[agent][j - 1]["y"]], [schedule[agent][j]["x"], schedule[agent][j]["y"]]]

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
                            dir_list[k][i][1][0] == dir_list[j][i][2][0] or dir_list[k][i][1][1] == dir_list[j][i][2][
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

    cost = 0
    last_index = []
    for dir_list in dir_list2:
        i = -1
        while dir_list[i] == 5:
            i -= 1
        cost += len(dir_list[:i + 1])
        last_index.append(i + 1)

    # 로봇의 pose로 다시 변경
    re_schedule = {"cost": cost + 10, "schedule": {}}
    for i, agent_dir in enumerate(dir_list2):
        t = 0
        agent_name = "agent" + str(i)
        agent_start_pose = schedule[agent_name][0]
        agent_schedule = [{'x': schedule[agent_name][0]['x'], 'y': schedule[agent_name][0]['y'], 't': t}]
        t += 1
        for direction in agent_dir:
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
        re_schedule["schedule"][agent_name] = agent_schedule
    with open("output_test.yaml", 'w') as output_yaml:
        yaml.safe_dump(re_schedule, output_yaml)