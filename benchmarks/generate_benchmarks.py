import json
import os
import random


def main():
    print("Sto(ng) generannn i benchmark...")

    n_benchmark = 5
    n_robot_list = [1, 2, 5]
    n_goal_list = [2, 5]

    with open("template.json") as json_file:
        data = json.load(json_file)
        rooms = data["rooms"]

        for n_robot in n_robot_list:
            for n_goal in n_goal_list:
                for n in range(n_benchmark):
                    json_agents = []
                    for robot in range(n_robot):
                        agent = dict()
                        agent["ID"] = robot
                        agent["initPos"] = [
                            rooms[random.randint(0, len(rooms)-1)][0]]
                        agent["endPos"] = [
                            rooms[random.randint(0, len(rooms)-1)][0]]
                        agent["goalPos"] = [[rooms[random.randint(0, len(rooms)-1)][0]]
                                            for _ in range(0, n_goal-1)]
                        agent["priority"] = 0
                        agent["name"] = "Robot" + str(robot)
                        json_agents.append(agent)

                    benchmark = dict()
                    benchmark["nAgents"] = n_robot
                    benchmark["nGoals"] = n_goal
                    benchmark["agents"] = json_agents

                    with open(os.path.join("input", f"benchmark_r{n_robot}_g{n_goal}_{n}.json"), "w", encoding="utf-8") as f:
                        json.dump(benchmark, f, ensure_ascii=False, indent=4)


if __name__ == "__main__":
    main()
