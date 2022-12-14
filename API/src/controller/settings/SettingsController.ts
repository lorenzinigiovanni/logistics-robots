import express from 'express';

import { Settings } from '../../entity/settings/Settings';


export class SettingsController {

    static route(app: express.Application): void {

        app.route('/settings')
            .get(async (req, res) => {
                const settings = await Settings.findOne();

                if (settings == null) {
                    res.status(404).send();
                    return;
                }

                res.status(200).send(settings);
            })
            .put(async (req, res) => {
                const settings = await Settings.findOne();

                if (settings == null) {
                    res.status(404).send();
                    return;
                }

                Settings.update(settings, req.body);

                res.status(200).send();
            });

    }

}
