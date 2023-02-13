import express from 'express';

import { Settings } from '../../entity/settings/Settings';
import { Plan } from '../../entity/task/Plan';
import { Task } from '../../entity/task/Task';
import { TaskController } from '../task/TaskController';

export class SettingsController {

    static route(app: express.Application): void {

        app.route('/settings')
            .get(async (req, res) => {
                let [settings] = await Settings.find();

                if (settings == null) {
                    settings = new Settings();
                    await settings.save();
                }

                res.status(200).send(settings);
            })
            .put(async (req, res) => {
                const [settings] = await Settings.find();

                if (settings == null) {
                    res.status(404).send();
                    return;
                }

                Settings.update(settings.ID, req.body);

                res.status(200).send();
            });

        app.route('/settings/resetsettings')
            .post(async (req, res) => {
                await Settings.delete({});

                const settings = new Settings();
                await settings.save();

                res.status(200).send();
            });

        app.route('/settings/erasetasksplans')
            .post(async (req, res) => {
                await TaskController.stopPlan();

                await Plan.delete({});
                await Task.delete({});

                res.status(200).send();
            });

    }

}
