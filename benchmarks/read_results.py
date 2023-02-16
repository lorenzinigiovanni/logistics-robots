import os
import json

# example of result
# {
#     "timedout": false,
#     "solution_found": "1",
#     "time": "1917",
#     "conflicts": "0",
#     "path_length": "152",
#     "largest_group": "2",
#     "nRobots": "2",
#     "nGoals": "2",
#     "test": "1",
#     "filename": "benchmark_r2_g2_1.json",
#     "MAPF": "A*+OD+ID",
#     "CF": "SIC",
#     "SAPF": "A*",
#     "H": "MANHATTAN"
# }

times = [1, 10, 60]


def main():
    print("Reading all the benchmarks...")

    output_dir = os.path.join("output")
    results = dict()

    for robot in os.listdir(output_dir):
        robot_dir = os.path.join(output_dir, robot)

        results[robot] = dict()

        for goal in os.listdir(robot_dir):
            goal_dir = os.path.join(robot_dir, goal)

            results[robot][goal] = dict()

            for algorithm in os.listdir(goal_dir):
                algorithm_dir = os.path.join(goal_dir, algorithm)

                results[robot][goal][algorithm] = dict()

                nTestCompleted = [0] * len(times)
                totalTimeTestCompleted = [0] * len(times)
                length = [0] * len(times)
                conflicts = [0] * len(times)
                largest_group = [0] * len(times)

                for filename in os.listdir(algorithm_dir):
                    file = os.path.join(algorithm_dir, filename)

                    with open(file, "r", encoding="utf-8") as json_file:
                        data = json.load(json_file)

                        if not data["timedout"]:
                            for i in range(len(times)):
                                if int(data["time"]) <= times[i] * 1000:
                                    nTestCompleted[i] += 1
                                    totalTimeTestCompleted[i] += int(data["time"])
                                    length[i] += int(data["path_length"])
                                    conflicts[i] += int(data["conflicts"])
                                    largest_group[i] += int(data["largest_group"])

                for i in range(len(times)):
                    if nTestCompleted[i] == 0:
                        totalTimeTestCompleted[i] = 0
                        length[i] = 0
                        conflicts[i] = 0
                        largest_group[i] = 0
                    else:
                        totalTimeTestCompleted[i] = totalTimeTestCompleted[i] / nTestCompleted[i]
                        length[i] = length[i] / nTestCompleted[i]
                        conflicts[i] = conflicts[i] / nTestCompleted[i]
                        largest_group[i] = largest_group[i] / nTestCompleted[i]

                results[robot][goal][algorithm]["nTestCompleted"] = nTestCompleted
                results[robot][goal][algorithm]["totalTimeTestCompleted"] = totalTimeTestCompleted
                results[robot][goal][algorithm]["length"] = length
                results[robot][goal][algorithm]["conflicts"] = conflicts
                results[robot][goal][algorithm]["largest_group"] = largest_group

    print("Writing the results to results.json...")
    with open(os.path.join("results.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, sort_keys=True, indent=4)


if __name__ == "__main__":
    main()
