/* eslint-disable arrow-body-style */
import express from 'express';

import { Robot } from '../../entity/robot/Robot';
import { Plan } from '../../entity/task/Plan';
import { Task } from '../../entity/task/Task';
import { TaskToRoom } from '../../entity/task/TaskToRoom';

export class StatisticsController {

    static route(app: express.Application): void {

        app.route('/statistics')
            .get(async (req, res) => {
                const statistics = {
                    nRobots: await Robot.count(),
                    nTasks: await Task.count(),
                    nGoals: await TaskToRoom.count(),
                };

                res.status(200).send(statistics);
            });

        app.route('/statistics/tasks')
            .get(async (req, res) => {
                let query = '';

                if (req.query.period === 'day') { // one day period over every hour
                    query = `
                    SELECT EXTRACT(YEAR FROM d.rawdate) AS year,
                           EXTRACT(MONTH FROM d.rawdate) AS month,
                           EXTRACT(DAY FROM d.rawdate) AS day,
                           EXTRACT(HOUR FROM d.rawdate) AS hour,
                           COUNT(task."ID") AS "nTasks"
                    FROM (
                        SELECT to_char(date_trunc('hour', (now() - INTERVAL '1 hour' * offs)), 'YYYY-MM-DD HH24') AS date,
                               date_trunc('hour', (now() - INTERVAL '1 hour' * offs)) AS rawdate
                        FROM generate_series(0, 23) AS offs
                        ) d
                    LEFT OUTER JOIN task task
                        ON d.date = to_char(date_trunc('hour', task."createdAt"), 'YYYY-MM-DD HH24')
                    GROUP BY d.date, d.rawdate
                    ORDER BY d.rawdate;
                    `;
                } else if (req.query.period === 'month') { // one month period over every day
                    query = `
                    SELECT EXTRACT(YEAR FROM d.rawdate) AS year,
                           EXTRACT(MONTH FROM d.rawdate) AS month,
                           EXTRACT(DAY FROM d.rawdate) AS day,
                           COUNT(task."ID") AS "nTasks"
                    FROM (
                        SELECT to_char(date_trunc('day', (now() - INTERVAL '1 day' * offs)), 'YYYY-MM-DD') AS date,
                               date_trunc('day', (now() - INTERVAL '1 day' * offs)) AS rawdate
                        FROM generate_series(0, 30) AS offs
                        ) d
                    LEFT OUTER JOIN task task
                        ON d.date = to_char(date_trunc('day', task."createdAt"), 'YYYY-MM-DD')
                    GROUP BY d.date, d.rawdate
                    ORDER BY d.rawdate;
                    `;
                } else if (req.query.period === 'year') { // one year period over every month
                    query = `
                    SELECT EXTRACT(YEAR FROM d.rawdate) AS year,
                           EXTRACT(MONTH FROM d.rawdate) AS month,
                           COUNT(task."ID") AS "nTasks"
                    FROM (
                        SELECT to_char(date_trunc('month', (now() - INTERVAL '1 month' * offs)), 'YYYY-MM') AS date,
                               date_trunc('month', (now() - INTERVAL '1 month' * offs)) AS rawdate
                        FROM generate_series(0, 11) AS offs
                        ) d
                    LEFT OUTER JOIN task task
                        ON d.date = to_char(date_trunc('month', task."createdAt"), 'YYYY-MM')
                    GROUP BY d.rawdate
                    ORDER BY d.rawdate;
                    `;
                }

                res.status(200).send(await Task.query(query));
            });

        app.route('/statistics/robots')
            .get(async (req, res) => {
                const robots = Robot.createQueryBuilder('robot')
                    .leftJoin('robot.tasks', 'task', `task.createdAt >= (now() - INTERVAL '1 ${req.query.period}')`)

                    .select('robot.name', 'name')

                    .addSelect('COUNT(case task.status when \'completed\' then 1 else null end)', 'nCompletedTasks')
                    .addSelect('COUNT(case task.status when \'in_execution\' then 1 else null end)', 'nInExecutionTasks')
                    .addSelect('COUNT(case task.status when \'assigned\' then 1 else null end)', 'nAssignedTasks')

                    .orderBy('robot.number')
                    .groupBy('robot.number')
                    .addGroupBy('robot.name');


                res.status(200).send(await robots.getRawMany());
            });

        app.route('/statistics/plans')
            .get(async (req, res) => {
                const query = `
                SELECT c.length, COUNT(*)
                FROM (
                    SELECT COUNT("planToNodes"."ID") AS "length"
                    FROM "plan" "plan"
                    INNER JOIN plan_to_node "planToNodes"
                        ON "planToNodes"."planID" = "plan"."ID"
                    WHERE "plan"."createdAt" >= (now() - INTERVAL '1 ${req.query.period}')
                    GROUP BY "plan"."ID"
                ) "c"
                GROUP BY c.length
                ORDER BY c.length;
                `;

                res.status(200).send(await Plan.query(query));
            });

        app.route('/statistics/planners')
            .get(async (req, res) => {
                const planners = await Plan.createQueryBuilder('plan')
                    .select('plan.MAPFalgorithm', 'planner')
                    .addSelect('COUNT(*)', 'count')
                    .where(`plan.createdAt >= (now() - INTERVAL '1 ${req.query.period}')`)
                    .groupBy('plan.MAPFalgorithm')
                    .orderBy('count', 'DESC')
                    .getRawMany();

                res.status(200).send(planners);
            });

    }

}
