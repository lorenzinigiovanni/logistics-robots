import express from 'express';
import fs from 'fs/promises';

import { MapNode } from '../../entity/map/MapNode';
import { MapEdge } from '../../entity/map/MapEdge';
import { Task } from '../../entity/task/Task';
import { Robot } from '../../entity/robot/Robot';
import { Settings } from '../../entity/settings/Settings';
import { TaskToRoom } from '../../entity/task/TaskToRoom';
import { Room } from '../../entity/map/Room';
import { execShellCommand } from '../../tools/shell';
import path from 'path';

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

                const robots = await Robot.createQueryBuilder('robot')
                    .leftJoinAndSelect('robot.tasks', 'tasks')
                    .leftJoinAndSelect('tasks.taskToRooms', 'taskToRooms')
                    .leftJoinAndSelect('taskToRooms.room', 'room')
                    .leftJoinAndSelect('room.node', 'node')
                    .orderBy('robot.number', 'ASC')
                    .getMany();

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

                if (settings == null || robots == null || nodes == null || edges == null) {
                    res.status(404).send();
                    return;
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
                        if (task.taskToRooms) {
                            for (const taskToRoom of task.taskToRooms) {
                                agentTask.push(taskToRoom.room.node.value);
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
                        const dist = (node.x - robot.x) ** 2 + (node.y - robot.y) ** 2;
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

                res.status(200).send(outputJson);
            });
    }
}
