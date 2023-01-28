import dotenv from 'dotenv';
import { DataSource } from 'typeorm';
import express from 'express';
import cors from 'cors';
import { join } from 'path';

import { AuthController } from './controller/auth/AuthController';
import { authenticateToken } from './middlewares/AuthenticateToken';
import { UserController } from './controller/users/UserController';
import { MapController } from './controller/map/MapController';
import { SettingsController } from './controller/settings/SettingsController';
import { RobotsController } from './controller/robots/RobotsController';
import { TaskController } from './controller/task/TaskController';

export class Main {
    static start(): void {
        dotenv.config();

        const AppDataSource = new DataSource({
            type: 'postgres',
            host: process.env.DB_HOST,
            port: 5432,
            username: process.env.DB_USER,
            password: process.env.DB_PASS,
            database: process.env.DB_NAME,
            synchronize: true,
            entities: [join(__dirname, 'entity', '**', '*.{ts,js}')],
            migrations: [join(__dirname, 'migration', '**', '*.{ts,js}')],
            subscribers: [join(__dirname, 'subscriver', '**', '*.{ts,js}')],
        });

        AppDataSource.initialize()
            .then(async () => {
                // eslint-disable-next-line no-console
                console.log('DB connected ' + process.env.DB_NAME);

                const app = express();
                app.use(cors());
                app.use(express.json({ limit: '10mb' }));

                app.use(function (req, res, next) {
                    res.setHeader('Access-Control-Allow-Origin', '*');
                    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, PATCH, DELETE');
                    res.setHeader('Access-Control-Allow-Headers', 'X-Requested-With, Content-Type, Authorization, Origin');
                    res.setHeader('Access-Control-Expose-Headers', 'Authorization, Content-type, Content-disposition');

                    next();
                });

                app.use(authenticateToken);

                AuthController.route(app);
                UserController.route(app);
                MapController.route(app);
                SettingsController.route(app);
                RobotsController.route(app);
                TaskController.route(app);

                app.listen(3000);
                // eslint-disable-next-line no-console
                console.log('Express application is up and running on port 3000');

            })
            .catch(error => {
                console.error(error);
            });

    }
}

Main.start();
