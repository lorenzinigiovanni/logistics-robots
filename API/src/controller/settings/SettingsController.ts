import express from 'express';

import { Settings } from '../../entity/settings/Settings';


export class SettingsController {

    static route(app: express.Application): void {

        app.route('/settings')
            .get(async (req, res) => {
                let [settings] = await Settings.find();

                if (settings == null) {
                    settings = new Settings();
                    settings.save();
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

    }

}
