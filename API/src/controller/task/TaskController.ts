import express from 'express';

import { Task } from '../../entity/task/Task';

export class TaskController {

    static route(app: express.Application): void {

        app.route('/tasks')
            .get(async (req, res) => {
                // TODO: fix in order to get rooms as array, rooms not nested in taskToRooms
                const task = await Task.createQueryBuilder('task')
                    .leftJoinAndSelect('task.robot', 'robot')
                    .leftJoinAndSelect('task.taskToRooms', 'taskToRooms')
                    .orderBy('taskToRooms.order')
                    .leftJoinAndSelect('taskToRooms.room', 'room')
                    .leftJoinAndSelect('room.node', 'node')
                    .getMany();

                res.status(200).send(task);
            })
            .post(async (req, res) => {
                // eslint-disable-next-line @typescript-eslint/ban-types
                const task = Task.create(req.body as Object);

                await task.save();

                res.status(200).send();
            });
    }
}
