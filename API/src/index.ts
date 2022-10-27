/* eslint-disable no-console */
import dotenv from 'dotenv';
import { createConnection } from 'typeorm';
import express from 'express';
import cors from 'cors';

import { AuthController } from './controller/auth/AuthController';
import { authenticateToken } from './middlewares/AuthenticateToken';
import { UserController } from './controller/users/UserController';
import { MapController } from './controller/map/MapController';

export class Main {
    static start(): void {
        dotenv.config();

        const rootDir = process.env.NODE_ENV === 'development' ? 'src' : 'build';

        createConnection({
            type: 'postgres',
            host: process.env.DB_HOST,
            port: 5432,
            username: process.env.DB_USER,
            password: process.env.DB_PASS,
            database: process.env.DB_NAME,
            synchronize: true,
            entities: [rootDir + '/entity/**/*.{js,ts}'],
            migrations: [rootDir + '/migration/**/*.{js,ts}'],
            subscribers: [rootDir + '/subscriber/**/*.{js,ts}'],
        }).then(async () => {

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

            app.listen(3000);
            console.log('Express application is up and running on port 3000');

        }).catch(error => {
            console.log(error);
        });

    }
}

Main.start();
