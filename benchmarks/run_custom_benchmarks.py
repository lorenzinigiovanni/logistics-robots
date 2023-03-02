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
    "ICR+A*+OD",
    "ICR+ICTS",
    "ICR+CP",
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
    print("Executing custom benchmarks...")

    input_directory = "custom_input"
    output_path = "custom_output"
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
        os.mkdir(output_path)

    for algorithm in algorithms:

        for filename in os.listdir(input_directory):
            inputFile = os.path.join(input_directory, filename)

            algo = algorithm
            subSolver = ""

            if algorithm.startswith("ICR"):
                subSolver = algorithm.split("+", 1)[1]
                algo = "ICR"

            results = {}
            results["filename"] = filename
            results["MAPF"] = algo
            results["CF"] = "SIC"
            results["SAPF"] = "A*"
            results["H"] = "MANHATTAN"
            results["subSolver"] = subSolver

            custom_test_path = os.path.join(output_path, filename.split(".")[0])
            if not os.path.exists(custom_test_path):
                os.mkdir(custom_test_path)

            outputFile = os.path.join(custom_test_path, f"{algorithm}.json")

            if (os.path.isfile(outputFile)):
                continue

            arguments = [
                "timeout",
                # f"--kill-after={1}",
                # "--signal=SIGKILL",
                str(max_duration),
                f"./{maof_exec_path}",
                inputFile,
                algo,
                "SIC",
                "A*",
                "MANHATTAN",
                subSolver,
            ]

            print(arguments)

            output = subprocess.run(
                arguments,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            results.update(parse_output(output.stdout.decode("UTF-8")))

            print(results)

            with open(outputFile, "w", encoding="utf-8") as f:
                json.dump(results, f, ensure_ascii=False, indent=4)


if __name__ == "__main__":
    main()
