import express from 'express';
import jwt from 'jsonwebtoken';

import { User } from '../../entity/users/User';

export class UserController {

    static route(app: express.Application): void {

        app.route('/users')
            .get(async (req, res) => {
                const users = await User.find({ order: { name: 'ASC' } });

                res.status(200).send(users);
            });

        app.route('/users/:ID')
            .get(async (req, res) => {
                const user = await User.findOne({
                    where: {
                        ID: req.params.ID,
                    },
                });

                if (user == null) {
                    res.status(404).send();
                    return;
                }

                res.status(200).send(user);
            })
            .put(async (req, res) => {
                const authHeader = req.headers.authorization;
                const token = authHeader && authHeader.split(' ')[1];

                let reqUser = null;

                try {
                    const payload = <jwt.JwtPayload>jwt.verify(token!, process.env.TOKEN_SECRET as string);
                    reqUser = await User.findOneOrFail({ where: { ID: payload.ID } });
                } catch {
                    res.status(401).send();
                    return;
                }

                const user = await User.findOne({
                    where: {
                        ID: req.params.ID,
                    },
                });

                if (user == null) {
                    res.status(404).send();
                    return;
                }

                if (reqUser.ID !== req.params.ID && !reqUser.admin) {
                    res.status(403).send();
                    return;
                }

                if (user.email !== req.body.email && await User.findOne({ where: { email: req.body.email } })) {
                    res.status(409).send();
                    return;
                }

                user.name = req.body.name;
                user.email = req.body.email;

                if (reqUser.admin) {
                    user.admin = req.body.admin;
                    user.active = req.body.active;
                }

                await user.save();

                res.status(204).send();
            })
            .delete(async (req, res) => {
                const user = await User.findOne({
                    where: {
                        ID: req.params.ID,
                    },
                });

                if (user == null) {
                    res.status(404).send();
                    return;
                }

                await User.remove(user);

                res.status(204).send();
            });

        app.route('/users/:ID/password')
            .put(async (req, res) => {
                const user = await User.findOne({
                    where: {
                        ID: req.params.ID,
                    },
                    select: [
                        'ID',
                        'password',
                    ],
                });

                if (!user) {
                    res.status(404).send();
                    return;
                }

                const match = await user.verifyPassword(req.body.oldPassword);

                if (!match) {
                    res.status(401).send();
                    return;
                }

                await user.setPassword(req.body.newPassword);

                await user.save();

                res.status(204).send();
            });

    }

}
