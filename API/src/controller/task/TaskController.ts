import express from 'express';
import fs from 'fs/promises';

import { MapNode } from '../../entity/map/MapNode';
import { MapEdge } from '../../entity/map/MapEdge';
import { Task, TaskStatus } from '../../entity/task/Task';
import { Robot } from '../../entity/robot/Robot';
import { Settings } from '../../entity/settings/Settings';
import { TaskToRoom } from '../../entity/task/TaskToRoom';
import { Room } from '../../entity/map/Room';
import { execShellCommand } from '../../tools/shell';
import { manhattanDistanceFromNodes, euclideanDistanceFromNodes } from '../../tools/distance';

const maofBuildDir = path.join(__dirname, '..', '..', 'maof', 'build');

export class TaskController {

    static route(app: express.Application): void {

        app.route('/tasks')
            .get(async (req, res) => {
                const tasks = await Task.createQueryBuilder('task')
                    .leftJoinAndSelect('task.robot', 'robot')
                    .leftJoinAndSelect('task.taskToRooms', 'taskToRooms')
                    .leftJoinAndSelect('taskToRooms.room', 'room')
                    .leftJoinAndSelect('room.node', 'node')
                    .orderBy({ 'task.createdAt': 'DESC', 'taskToRooms.order': 'ASC' })
                    .getMany();

                for (const task of tasks) {
                    if (task.taskToRooms) {
                        task.goals = task.taskToRooms.map(taskToRoom => taskToRoom.room);
                        delete task.taskToRooms;
                    }
                }

                res.status(200).send(tasks);
            })
            .post(async (req, res) => {
                const task = new Task();
                task.taskToRooms = [];

                for (const [order, goal] of req.body.goals.entries()) {
                    const taskToRoom = new TaskToRoom();
                    taskToRoom.room = await Room.findOneOrFail(goal.ID);
                    taskToRoom.order = order;
                    await taskToRoom.save();
                    task.taskToRooms.push(taskToRoom);
                }

                await task.save();

                res.status(200).send();
            });

        app.route('/generate-plan')
            .get(async (req, res) => {
                const settings = await Settings.findOne();

                if (settings == null) {
                    res.status(500).send();
                    return;
                }

                const tasks = await Task.createQueryBuilder('task')
                    .leftJoinAndSelect('task.robot', 'robot')
                    .where('task.status=:status', { status: TaskStatus.NOT_ASSIGNED })
                    .leftJoinAndSelect('task.taskToRooms', 'taskToRooms')
                    .leftJoinAndSelect('taskToRooms.room', 'room')
                    .leftJoinAndSelect('room.node', 'node')
                    .orderBy({ 'task.createdAt': 'ASC', 'taskToRooms.order': 'ASC' })
                    .getMany();

                for (const task of tasks) {
                    if (task.taskToRooms) {
                        task.goals = task.taskToRooms.map(taskToRoom => taskToRoom.room);
                    }
                }

                let robots = await Robot.createQueryBuilder('robot')
                    .leftJoinAndSelect('robot.tasks', 'task')
                    .leftJoinAndSelect('task.taskToRooms', 'taskToRooms')
                    .leftJoinAndSelect('taskToRooms.room', 'room')
                    .leftJoinAndSelect('room.node', 'node')
                    .orderBy({ 'robot.number': 'ASC', 'task.createdAt': 'ASC', 'taskToRooms.order': 'ASC' })
                    .getMany();

                for (const robot of robots) {
                    robot.tasks = robot.tasks.filter(task => task.status === TaskStatus.IN_EXECUTION || task.status === TaskStatus.ASSIGNED);
                    for (const task of robot.tasks) {
                        if (task.taskToRooms) {
                            task.goals = task.taskToRooms.map(taskToRoom => taskToRoom.room);
                            task.completedGoals = task.taskToRooms.filter(taskToRoom => taskToRoom.completed).map(taskToRoom => taskToRoom.room);
                        }
                    }
                }

                const nodes = await MapNode.createQueryBuilder('node')
                    .orderBy('node.value', 'ASC')
                    .getMany();

                const edges = await MapEdge
                    .createQueryBuilder('edge')
                    .leftJoin('edge.node1', 'node1')
                    .addSelect(['node1.ID', 'node1.value'])
                    .leftJoin('edge.node2', 'node2')
                    .addSelect(['node2.ID', 'node2.value'])
                    .getMany();

                // assign free task to agent basing on manhattan distance from last task of agent

                for (const task of tasks) {
                    let minDistance = Infinity;
                    let robotIndex = -1;
                    let i = 0;

                    for (const robot of robots) {
                        let newTaskFirstNode = null;
                        let lastTaskLastNode = null;
                        const robotCurrentPosition = await robot.getPosition();

                        if (task.goals) {
                            // get the first node of the new task
                            newTaskFirstNode = task.goals[0].node;
                        }

                        if (robot.tasks.length > 0) {
                            // get the last node of the last task of the robot
                            const lastTask = robot.tasks[robot.tasks.length - 1];
                            if (lastTask.goals) {
                                lastTaskLastNode = lastTask.goals[lastTask.goals.length - 1].node;
                            }
                        }

                        // if a task is present compute the distance between the current position of the robot and the successive goals untill reaching the end,
                        // finally add distance between last goal and first new goal
                        // if there is no task the distance is the one between the robot and the first new goal
                        if (newTaskFirstNode) {
                            let distance = 0;

                            if (lastTaskLastNode) {
                                for (const [k, robotTask] of robot.tasks.entries()) {
                                    if (robotTask.goals) {
                                        if (robotTask.status === TaskStatus.IN_EXECUTION && robotTask.completedGoals?.length && robotTask.goals.length > robotTask.completedGoals?.length) {
                                            for (let j = robotTask.completedGoals?.length; j < robotTask.goals.length; j++) {
                                                if (j === robotTask.completedGoals?.length) {
                                                    distance += manhattanDistanceFromNodes(robotCurrentPosition, robotTask.goals[j].node);
                                                } else {
                                                    distance += manhattanDistanceFromNodes(robotTask.goals[j - 1].node, robotTask.goals[j].node);
                                                }
                                            }
                                        } else {
                                            for (let j = 0; j < robotTask.goals.length; j++) {
                                                if (j === 0 && k > 0) {
                                                    const previousTask = robot.tasks[k - 1];
                                                    if (previousTask.goals) {
                                                        distance += manhattanDistanceFromNodes(previousTask.goals[previousTask.goals.length - 1].node, robotTask.goals[j].node);
                                                    }
                                                } else if (j > 0) {
                                                    distance += manhattanDistanceFromNodes(robotTask.goals[j - 1].node, robotTask.goals[j].node);
                                                }
                                            }
                                        }
                                    }
                                }
                                // add distance from last task last node to new task first node
                                distance += manhattanDistanceFromNodes(lastTaskLastNode, newTaskFirstNode);
                            } else {
                                distance = manhattanDistanceFromNodes(robotCurrentPosition, newTaskFirstNode);
                            }

                            if (distance < minDistance) {
                                minDistance = distance;
                                robotIndex = i;
                            }
                        }

                        i++;
                    }

                    if (minDistance !== Infinity) {
                        task.status = TaskStatus.ASSIGNED;
                        task.robot = robots[robotIndex];
                        await task.save();

                        robots = await Robot.createQueryBuilder('robot')
                            .leftJoinAndSelect('robot.tasks', 'tasks')
                            .leftJoinAndSelect('tasks.taskToRooms', 'taskToRooms')
                            .leftJoinAndSelect('taskToRooms.room', 'room')
                            .leftJoinAndSelect('room.node', 'node')
                            .orderBy('robot.number', 'ASC')
                            .getMany();
                    }
                }

                // generate nodes list
                const numberOfNodes = nodes.length;

                const nodeList = Array.from(Array(numberOfNodes), () => Array(4).fill(0));
                for (let i = 0; i < numberOfNodes; i++) {
                    // value (starts from 1), ID (starts from 0), x, y
                    nodeList[i] = [nodes[i].value, nodes[i].value - 1, nodes[i].x, nodes[i].y];
                }

                // generate agents list
                const agentList = [];
                for (const robot of robots) {
                    let agentTask: number[] = [];
                    for (const task of robot.tasks) {
                        if (task.goals) {
                            for (const goal of task.goals) {
                                agentTask.push(goal.node.value);
                            }
                        }
                    }

                    agentTask = agentTask.flat();

                    const goalPositions: number[][] = [];
                    for (const task of agentTask.flat()) {
                        goalPositions.push([task]);
                    }

                    // search for the nearest node from the initial coordinates of the robot
                    let robotNodeValue = -1;
                    let minNodeDistance = Infinity;
                    for (const node of nodes) {
                        const dist = euclideanDistanceFromNodes(node, await robot.getPosition());
                        if (dist < minNodeDistance) {
                            minNodeDistance = dist;
                            robotNodeValue = node.value;
                        }
                    }

                    if (agentTask.length > 0) {
                        agentList.push({
                            'ID': robot.number,
                            'initPos': [robotNodeValue],
                            'endPos': [agentTask[agentTask.length - 1]],
                            'goalPos': goalPositions,
                            'priority': 0,
                            'name': robot.name,
                        });
                    }
                }

                // generate adjacency list
                const adjacencyList = Array.from(Array(numberOfNodes), () => Array(numberOfNodes).fill(0));
                for (let i = 0; i < numberOfNodes; i++) {
                    adjacencyList[i][i] = 1;

                    const node = nodes.find((n: MapNode) => n.value === i + 1);

                    if (node == null) {
                        res.status(404).send();
                        return;
                    }

                    const filteredEdges = edges.filter((e: MapEdge) => e.node1.ID === node.ID);

                    for (const edge of filteredEdges) {
                        const j = edge.node2.value - 1;

                        adjacencyList[i][j] = 1;
                        adjacencyList[j][i] = 1;
                    }
                }

                const jsonList = {
                    'nAgents': robots.length,
                    'nodes': nodeList,
                    'agents': agentList,
                    'connect': adjacencyList,
                    'MAPF': settings.MAPFalgorithm,
                    'SAPF': settings.SAPFalgorithm,
                    'costFunction': settings.costFunction,
                    'heuristic': '',
                };

                const json = JSON.stringify(jsonList);
                await fs.writeFile(path.join(maofBuildDir, 'input.json'), json, 'utf8');

                res.status(200).send();
            });


        app.route('/compute-plan')
            .get(async (req, res) => {

                const settings = await Settings.findOne();

                const cmd = 'cd ' + maofBuildDir + ' && ' + './MAOFexec' + ' ' + 'input.json' + ' ' + settings?.MAPFalgorithm + ' ' + settings?.costFunction + ' ' + settings?.SAPFalgorithm;
                const outputFile = path.join(maofBuildDir, 'output.json');

                try {
                    await execShellCommand(cmd);
                } catch (err) {
                    console.error(err);
                    res.status(500).send();
                    return;
                }

                let rawdata = '';
                try {
                    rawdata = await fs.readFile(outputFile, 'utf8');
                } catch (err) {
                    console.error(err);
                    res.status(500).send();
                    return;
                }
                const outputJson = JSON.parse(rawdata);

                await fs.unlink(path.join(maofBuildDir, 'input.json'));
                await fs.unlink(outputFile);

                res.status(200).send(outputJson);
            });
    }
}
