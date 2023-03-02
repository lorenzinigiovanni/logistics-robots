import os
import json

times = [1, 10, 60]


def main():
    print("Reading all the custom benchmarks...")

    output_dir = os.path.join("custom_output")
    results = dict()

    for test in os.listdir(output_dir):
        test_dir = os.path.join(output_dir, test)

        results[test] = dict()

        for algorithm in os.listdir(test_dir):
            file = os.path.join(test_dir, algorithm)

            with open(file, "r", encoding="utf-8") as json_file:
                data = json.load(json_file)

                if "subSolver" in data:
                    algorithmName = data["MAPF"] + "+" + data["subSolver"]
                else:
                    algorithmName = data["MAPF"]

                results[test][algorithmName] = dict()
                results[test][algorithmName]["timedout"] = data["timedout"]

                if not data["timedout"]:
                    results[test][algorithmName]["solution_found"] = int(data["solution_found"])
                    results[test][algorithmName]["time"] = int(data["time"])
                    results[test][algorithmName]["conflicts"] = int(data["conflicts"])
                    results[test][algorithmName]["largest_group"] = int(data["largest_group"])
                    results[test][algorithmName]["length"] = int(data["path_length"])

    print("Writing the results to custom_results.json...")
    with open(os.path.join("custom_results.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, sort_keys=True, indent=4)


if __name__ == "__main__":
    main()
