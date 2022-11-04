import express from 'express';
import fs from 'fs/promises';

import { MapNode } from '../../entity/map/MapNode';
import { MapEdge } from '../../entity/map/MapEdge';
import { Task } from '../../entity/task/Task';
import { Robot } from '../../entity/robot/Robot';
import { Settings } from '../../entity/settings/Settings';
import { TaskToRoom } from '../../entity/task/TaskToRoom';
import { Room } from '../../entity/map/Room';


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

                // generate agent number
                const agent_num = robots.length;

                // generate nodes list
                const node_number = nodes.length;

                const node_list = Array.from(Array(node_number), () => Array(4).fill(0));
                for (let i = 0; i < node_number; i++) {
                    node_list[i] = [nodes[i].value, nodes[i].value, nodes[i].x, nodes[i].y];
                }

                // generate agents list
                const agent_list = [];
                for (const robot of robots) {
                    let agent_task: number[] = [];
                    for (const task of robot.tasks) {
                        if (task.taskToRooms) {
                            for (const taskToRoom of task.taskToRooms) {
                                agent_task.push(taskToRoom.room.node.value);
                            }
                        }
                    }

                    agent_task = agent_task.flat();

                    if (agent_task.length > 0) {
                        agent_list.push({
                            'ID': robot.number,
                            'initPos': [2], // TODO: fix initial position
                            'endPos': agent_task[agent_task.length - 1],
                            'goalPos': agent_task,
                            'priority': 0,
                            'name': robot.name,
                        });
                    }
                }

                // generate adjacency list
                const adjacency_list = Array.from(Array(node_number), () => Array(node_number).fill(0));
                for (let i = 0; i < node_number; i++) {
                    adjacency_list[i][i] = 1;

                    const node = nodes.find((n: MapNode) => n.value === i + 1);

                    if (node == null) {
                        res.status(404).send();
                        return;
                    }

                    const filteredEdges = edges.filter((e: MapEdge) => e.node1.ID === node.ID);

                    for (const edge of filteredEdges) {
                        const j = edge.node2.value - 1;

                        adjacency_list[i][j] = 1;
                        adjacency_list[j][i] = 1;
                    }
                }

                const json_list = {
                    'nAgents': agent_num,
                    'nodes': node_list,
                    'agents': agent_list,
                    'connect': adjacency_list,
                    'MAPF': settings.MAPFalgorithm,
                    'SAPF': settings.SAPFalgorithm,
                    'costFunction': settings.costFunction,
                    'heuristic': '',
                };
                const json = JSON.stringify(json_list);
                await fs.writeFile('test.json', json, 'utf8');

                res.status(200).send();
            });
    }
}
