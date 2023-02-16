import re
import os
import subprocess
import json

algorithms = [
    "A*",
    "A*+ID",
    "A*+OD",
    "A*+OD+ID",
    "ICTS",
    "ICTS+ID",
    # "CP",
]

regexes = [
    re.compile(r"Solution found: (?P<solution_found>\d)"),
    re.compile(r"Time: (?P<time>\d+)"),
    re.compile(r"Conflicts: (?P<conflicts>\d+)"),
    re.compile(r"Path length: (?P<path_length>\d+)"),
    re.compile(r"Largest group: (?P<largest_group>\d+)"),
]


def parse_output(output):
    dizionario = {}

    if ("Solution found:" in output):
        dizionario["timedout"] = False

        for regex in regexes:
            m = regex.search(output)
            if m:
                dizionario.update(m.groupdict())
    else:
        dizionario["timedout"] = True

    return dizionario


def main():
    print("Executing benchmarks...")

    input_directory = "input"
    output_path = "output"
    maof_exec_path = os.path.join(
        "..",
        "API",
        "src",
        "maof",
        "build",
        "MAOFexec",
    )
    max_duration = 60

    if not os.path.exists(output_path):
        os.mkdir(os.path.join(output_path))

    for algorithm in algorithms:

        for filename in os.listdir(input_directory):
            file = os.path.join(input_directory, filename)

            arguments = [
                "timeout",
                # f"--kill-after={1}",
                # "--signal=SIGKILL",
                str(max_duration),
                f"./{maof_exec_path}",
                file,
                algorithm,
                "SIC",
                "A*",
                "MANHATTAN",
            ]

            print(arguments)

            output = subprocess.run(
                arguments,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            results = parse_output(output.stdout.decode("UTF-8"))

            r = re.compile(
                r"benchmark_r(?P<nRobots>\d+)_g(?P<nGoals>\d+)_(?P<test>\d+).json")
            m = r.match(filename)
            results.update(m.groupdict())

            results["filename"] = filename
            results["MAPF"] = algorithm
            results["CF"] = "SIC"
            results["SAPF"] = "A*"
            results["H"] = "MANHATTAN"

            print(results)

            nRobots = results["nRobots"]
            nGoals = results["nGoals"]

            robot_path = os.path.join(output_path, f"r{nRobots}")
            if not os.path.exists(robot_path):
                os.mkdir(robot_path)

            goal_path = os.path.join(robot_path, f"g{nGoals}")
            if not os.path.exists(goal_path):
                os.mkdir(goal_path)

            algorithm_path = os.path.join(goal_path, algorithm)
            if not os.path.exists(algorithm_path):
                os.mkdir(algorithm_path)

            with open(os.path.join(algorithm_path, filename), "w", encoding="utf-8") as f:
                json.dump(results, f, ensure_ascii=False, indent=4)


if __name__ == "__main__":
    main()
