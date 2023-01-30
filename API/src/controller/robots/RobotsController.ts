import express from 'express';

import { Robot } from '../../entity/robot/Robot';

export class RobotsController {

    static route(app: express.Application): void {

        app.route('/robots')
            .get(async (req, res) => {
                const robots = await Robot.createQueryBuilder('robot')
                    .orderBy('robot.number', 'ASC')
                    .getMany();

                res.status(200).send(robots);
            })
            .post(async (req, res) => {
                // eslint-disable-next-line @typescript-eslint/ban-types
                const robot = Robot.create(req.body as Object);

                await robot.save();

                res.status(200).send();
            });

        app.route('/robots/:ID')
            .get(async (req, res) => {
                const robot = await Robot.findOneBy({ ID: req.params.ID });

                if (robot == null) {
                    res.status(404).send();
                    return;
                }

                res.status(200).send(robot);
            })
            .put(async (req, res) => {
                const robot = await Robot.findOneBy({ ID: req.params.ID });

                if (robot == null) {
                    res.status(404).send();
                    return;
                }

                Robot.update(robot.ID, req.body);

                res.status(200).send();
            })
            .delete(async (req, res) => {
                const robot = await Robot.findOneBy({ ID: req.params.ID });

                if (robot == null) {
                    res.status(404).send();
                    return;
                }

                await Robot.remove(robot);

                res.status(200).send();
            });

    }

}
