import express from 'express';
import fs from 'fs/promises';
import path from 'path';

import { MapNode } from '../../entity/map/MapNode';
import { MapEdge } from '../../entity/map/MapEdge';
import { Task, TaskStatus } from '../../entity/task/Task';
import { Robot } from '../../entity/robot/Robot';
import { Settings } from '../../entity/settings/Settings';
import { TaskToRoom } from '../../entity/task/TaskToRoom';
import { Room } from '../../entity/map/Room';
import { execShellCommand } from '../../tools/shell';
import { manhattanDistanceFromNodes } from '../../tools/distance';
import { Plan } from '../../entity/task/Plan';
import { PlanToNode } from '../../entity/task/PlanToNode';

const maofBuildDir = path.join(__dirname, '..', '..', '..', 'src', 'maof', 'build');

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
                        task.completedGoals = task.taskToRooms.filter(taskToRoom => taskToRoom.completed).map(taskToRoom => taskToRoom.room);
                        delete task.taskToRooms;
                    }
                }

                res.status(200).send(tasks);
            })
            .post(async (req, res) => {
                const newTask = new Task();
                newTask.taskToRooms = [];

                for (const [order, goal] of req.body.goals.entries()) {
                    const taskToRoom = new TaskToRoom();
                    taskToRoom.room = await Room.createQueryBuilder('room')
                        .leftJoinAndSelect('room.node', 'node')
                        .where('room.ID = :ID', { ID: goal.ID })
                        .getOneOrFail();
                    taskToRoom.order = order;
                    await taskToRoom.save();
                    newTask.taskToRooms.push(taskToRoom);
                }

                newTask.goals = newTask.taskToRooms.map(taskToRoom => taskToRoom.room);

                const robots = await Robot.createQueryBuilder('robot')
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

                // assign free task to agent basing on manhattan distance from last task of agent

                let minDistance = Infinity;
                let selectedRobot = null;

                for (const robot of robots) {
                    let newTaskFirstNode = null;
                    let lastTaskLastNode = null;
                    const robotCurrentPosition = await robot.getPosition();

                    // get the first node of the new task
                    newTaskFirstNode = newTask.goals[0].node;

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

                    let distance = 0;

                    if (lastTaskLastNode) {
                        // the robot is doing a task
                        // add distance from current position to the end of the in execution + assigned tasks
                        for (const [k, robotTask] of robot.tasks.entries()) {
                            if (robotTask.goals) {
                                if (k === 0 && robotTask.completedGoals?.length != null && robotTask.goals.length > robotTask.completedGoals?.length) {
                                    for (let j = robotTask.completedGoals?.length; j < robotTask.goals.length; j++) {
                                        if (j === robotTask.completedGoals?.length) {
                                            distance += manhattanDistanceFromNodes(robotCurrentPosition, robotTask.goals[j].node);
                                        } else {
                                            distance += manhattanDistanceFromNodes(robotTask.goals[j - 1].node, robotTask.goals[j].node);
                                        }
                                    }
                                } else {
                                    for (let j = 0; j < robotTask.goals.length; j++) {
                                        if (j === 0) {
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
                        // the robot is doing nothing
                        distance = manhattanDistanceFromNodes(robotCurrentPosition, newTaskFirstNode);
                    }

                    if (distance < minDistance) {
                        minDistance = distance;
                        selectedRobot = robot;
                    }
                }

                if (minDistance === Infinity || selectedRobot == null) {
                    res.status(500).send();
                    return;
                }

                newTask.status = TaskStatus.ASSIGNED;
                newTask.robot = selectedRobot;
                await newTask.save();

                try {
                    await this.computePlan();
                } catch (err) {
                    console.error(err);
                    res.status(500).send();
                    return;
                }

                res.status(200).send();
            });

        app.route('/plans')
            .get(async (req, res) => {
                const plans = await Plan.createQueryBuilder('plan')
                    .leftJoinAndSelect('plan.robot', 'robot')
                    .leftJoinAndSelect('plan.planToNodes', 'planToNodes')
                    .leftJoinAndSelect('planToNodes.node', 'node')
                    .orderBy({ 'planToNodes.order': 'ASC' })
                    .getMany();

                for (const plan of plans) {
                    if (plan.planToNodes) {
                        plan.nodes = plan.planToNodes.map(planToNode => planToNode.node);
                        delete plan.planToNodes;
                    }
                }

                res.status(200).send(plans);
            })
            .post(async (req, res) => {
                try {
                    await this.computePlan();
                } catch (err) {
                    console.error(err);
                    res.status(500).send();
                    return;
                }

                res.status(200).send();
            });
    }

    static async computePlan(): Promise<void> {
        const [settings] = await Settings.find();

        if (settings == null) {
            throw new Error('Settings not found');
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

        const robots = await Robot.createQueryBuilder('robot')
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
                    task.goals = task.taskToRooms.filter(taskToRoom => !taskToRoom.completed).map(taskToRoom => taskToRoom.room);
                }
            }
        }

        // generate nodes list
        const nodeList = Array.from(Array(nodes.length), () => Array(4).fill(0));
        for (let i = 0; i < nodes.length; i++) {
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
            const robotNode = await robot.getPosition();

            agentList.push({
                'ID': robot.number - 1,
                'initPos': [robotNode.value],
                'endPos': [agentTask.length > 0 ? agentTask[agentTask.length - 1] : robotNode.value],
                'goalPos': agentTask.slice(0, -1).map((value: number) => [value]),
                'priority': 0,
                'name': robot.name,
            });
        }

        // generate adjacency list
        const adjacencyList = Array.from(Array(nodes.length), () => Array(nodes.length).fill(0));
        for (let i = 0; i < nodes.length; i++) {
            adjacencyList[i][i] = 1;

            const node = nodes.find((n: MapNode) => n.value === i + 1);

            if (node == null) {
                throw new Error('Node not found');
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
            'heuristic': settings.heuristic,
        };

        const inputFilePath = path.join(maofBuildDir, 'input.json');
        const outputFilePath = path.join(maofBuildDir, 'output.json');

        const json = JSON.stringify(jsonList);
        await fs.writeFile(inputFilePath, json, 'utf8');

        const cmd = 'cd ' + maofBuildDir + ' && ' + './MAOFexec' + ' ' + 'input.json';

        await execShellCommand(cmd);

        let rawdata = '';
        rawdata = await fs.readFile(outputFilePath, 'utf8');
        const outputJson = JSON.parse(rawdata);

        await fs.unlink(inputFilePath);
        await fs.unlink(outputFilePath);

        await Plan.delete({});

        for (const robot of outputJson.agents) {
            const robotEntity = await Robot.findOneOrFail({ where: { number: robot.ID + 1 } });

            const plan = new Plan();
            await plan.save();
            robotEntity.plan = plan;

            for (const [order, node] of robot.plan.entries()) {
                try {
                    const planToNode = new PlanToNode();
                    planToNode.order = order;
                    planToNode.node = await MapNode.findOneOrFail({ where: { value: node[0] } });
                    planToNode.plan = robotEntity.plan;
                    await planToNode.save();
                } catch (err) {
                    console.error('TODO: Node not found, fix it: ' + node[0]);
                }
            }

            await robotEntity.save();
        }
    }
}
