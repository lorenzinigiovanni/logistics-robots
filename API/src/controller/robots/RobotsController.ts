import express from 'express';

import { Robot } from '../../entity/robot/Robot';
import { TopicSubscriber } from '../../ros2bridge/TopicSubscriber';
import { quaternionToEuler } from '../../tools/quaternion';

const positionTopicSubscribers = new Map<number, TopicSubscriber>();

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

                await this.subscribeToRobotPosition();

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

    static async subscribeToRobotPosition(): Promise<void> {
        const robots = await Robot.createQueryBuilder('robot')
            .orderBy('robot.number', 'ASC')
            .getMany();

        for (const robot of robots) {
            if (!positionTopicSubscribers.has(robot.number)) {
                const topicSubscriber = new TopicSubscriber(
                    process.env.ROS_URL || 'ws://localhost:9020',
                    '/robot' + robot.number + '/amcl_pose',
                    'geometry_msgs/PoseWithCovarianceStamped',
                    (msg) => {
                        const x = msg.pose.pose.position.x;
                        const y = msg.pose.pose.position.y;

                        const ox = msg.pose.pose.orientation.x;
                        const oy = msg.pose.pose.orientation.y;
                        const oz = msg.pose.pose.orientation.z;
                        const ow = msg.pose.pose.orientation.w;

                        const euler = quaternionToEuler(ox, oy, oz, ow);

                        Robot.update(robot.ID, { currentX: x, currentY: y, currentTheta: euler[2] });
                    },
                );

                await topicSubscriber.open();
                topicSubscriber.subscribe();

                positionTopicSubscribers.set(robot.number, topicSubscriber);
            }
        }
    }

}
