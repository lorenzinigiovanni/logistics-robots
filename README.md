# Logistics Robots

ROS 2 implementation of a logistics network of robots for testing multiple multi-agent pathfinding (MAPF) algorithms in simulated and real environments.

The system objective is to handle multiple robot that have to accomplish a sequence of goals. In this case, multiple robots will act as postmen in order to deliver some packages from the user's office to another.

This system leverages on [maof](https://bitbucket.org/chaff800/maof/src/master/) to resolve the MAPF problem.

## Installation procedure

The system is composed of 4 different docker containers that allow to run it with ease. Hence **Docker is required** to run the system, please make sure to have it.

Clone the project:
```bash
$ git clone --recursive https://github.com/lorenzinigiovanni/logistics-robots.git
```

## People

This project is developed by:

- Giovanni Lorenzini [@lorenzinigiovanni](https://github.com/lorenzinigiovanni)
- Diego Planchenstainer [@diegoplanchenstainer](https://github.com/diegoplanchenstainer)

## License

[GPL-3](LICENSE)
