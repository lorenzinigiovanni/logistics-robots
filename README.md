# Logistics Robots

ROS 2 implementation of a logistics network of robots for testing multiple multi-agent pathfinding (MAPF) algorithms in simulated and real environments.

The system objective is to handle multiple robot that have to accomplish a sequence of goals. In this case, multiple robots will act as postmen in order to deliver some packages from the user's office to another.

This system leverages on [maof](https://bitbucket.org/chaff800/maof/src/master/) to resolve the MAPF problem.

## Installation

To run the system a machine with Ubuntu 22.04 is required. Otherwise is possible to run the system in a virtual machine or in docker (see [here](#installation-using-docker)).

```bash
$ git clone --recursive https://github.com/lorenzinigiovanni/logistics-robots.git
```

### ROS 2 Humble

Installations steps:

```bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
$ sudo apt update && sudo apt install curl
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-humble-desktop-full
$ sudo apt install ros-humble-gazebo-*
$ sudo apt install python3-colcon-common-extensions
```

Install necessary packages:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
```

Automatically source ROS 2 in your bashrc:

```bash
$ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
$ echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> ~/.bashrc
$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
$ source ~/.bashrc
```

Compile the project:

```bash
$ cd logistics-robots/ros2_ws
$ colcon build
```

### PostgreSQL database

Install PostgreSQL database:

```bash
$ sudo apt update
$ sudo apt install postgresql postgresql-contrib
$ sudo systemctl start postgresql.service
```

Configure PostgreSQL database:

```bash
$ sudo -i -u postgres
```

Create a new user called `logistics_robots`:

```bash
postgres@server:~$ createuser --interactive
```

Create a new database called `logistics_robots`:

```bash
postgres@server:~$ createdb logistics_robots
```

Set a password and create extension:

```bash
postgres@server:~$ psql
```
```sql
postgres=# ALTER USER logistics_robots WITH PASSWORD 'DbPassword';
postgres=# \connect logistics_robots 
postgres=# CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
```

### MAOF

Install gcc and cmake:

```bash
$ sudo apt install build-essential
$ sudo apt install cmake
```

Compile maof:

```bash
$ cd logistics-robots/API/src/maof
$ mkdir build
$ cmake -DCMAKE_BUILD_TYPE=Release -B build -S .
$ cmake --build build -- -j
```

### Node.js

Install Node.js:

```bash
$ curl -sL https://deb.nodesource.com/setup_18.x -o nodesource_setup.sh
$ sudo bash nodesource_setup.sh
$ sudo apt install nodejs
```

### Backend

Install python packages:

```bash
$ sudo apt install python3-pip
$ sudo apt install python3-venv
$ cd logistics-robots/API/src/scripts
$ python3 -m venv venv
$ source venv/bin/activate
$ pip install -r requirements.txt
```

Create a `.env` file in the `logistics-robots/API` folder with the following content (to be personalized):

```bash
DB_HOST=localhost
DB_USER=logistics_robots
DB_PASS=DbPassword
DB_NAME=logistics_robots
NODE_ENV=development
TOKEN_SECRET=secret
MAIL_ADDRESS=mail@example.com
MAIL_PASSWORD=MailPassword
MAIL_SERVER=mail.example.com
```

Compile the backend:

```bash
$ cd logistics-robots/API
$ npm install
$ npm run build
```

### Frontend

Compile the frontend:

```bash
$ cd logistics-robots/GUI
$ npm install
$ npm run build
```

## Usage

Start the simulator:

```bash
$ cd logistics-robots/ros2_ws
$ source install/setup.bash
$ ros2 launch turtlebot3_nav multi_tb3_simulation_launch.py
```

Start the backend:

```bash
$ cd logistics-robots/API
$ npm run start
```

Start the frontend:

```bash
$ cd logistics-robots/GUI
$ npm run debug
```

## Installation using docker

The system is composed of 4 different docker containers that allow to run it with ease.

Clone the project:
```bash
$ git clone --recursive https://github.com/lorenzinigiovanni/logistics-robots.git
```

## Usage using docker

```bash
$ docker-compose up
```

## People

This project is developed by:

- Giovanni Lorenzini [@lorenzinigiovanni](https://github.com/lorenzinigiovanni)
- Diego Planchenstainer [@diegoplanchenstainer](https://github.com/diegoplanchenstainer)

## License

[GPL-3](LICENSE)
