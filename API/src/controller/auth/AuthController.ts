import express from 'express';
import jwt from 'jsonwebtoken';

import { User } from '../../entity/users/User';
import { Mail } from '../../mail/Mail';

export class AuthController {

    static route(app: express.Application): void {

        app.route('/auth/login')
            .post(async (req, res) => {
                const email = req.body.email;
                const password = req.body.password;

                const user = await User.findOne({
                    where: {
                        email: email,
                        active: true,
                    },
                    select: [
                        'ID',
                        'name',
                        'password',
                        'admin',
                    ],
                });

                if (!user) {
                    res.status(401).send();
                    return;
                }

                const match = await user.verifyPassword(password);

                if (!match) {
                    res.status(401).send();
                    return;
                }

                const token = this.generateAccessToken({
                    ID: user.ID,
                    name: user.name,
                    admin: user.admin,
                    picture: null,
                });

                res.status(200).send({ token: token });
            });

        app.route('/auth/register')
            .post(async (req, res) => {
                const user = new User();

                user.name = req.body.fullName;
                user.email = req.body.email;
                user.active = false;
                user.admin = false;

                await user.setPassword(req.body.password);

                await user.save();

                res.status(200).send();
            });

        app.route('/auth/request-pass')
            .post(async (req, res) => {
                const email = req.body.email;

                try {
                    const user = await User.findOneOrFail({ where: { email: email } });

                    const password = Math.random().toString(36).substring(2, 8);
                    await user.setPassword(password);

                    await user.save();

                    Mail.send(
                        email,
                        'Reset password Logistics Robots',
                        `Your new password is: ${password}
                        Change it as soon as possible.`,
                    );
                } catch {
                    res.status(401).send();
                    return;
                }

                res.status(200).send();
            });

        app.route('/auth/logout')
            .delete(async (req, res) => {
                res.status(200).send();
            });

    }

    static generateAccessToken(payload: any): string {
        return jwt.sign(payload, process.env.TOKEN_SECRET as string, { expiresIn: '28d' });
    }

}
