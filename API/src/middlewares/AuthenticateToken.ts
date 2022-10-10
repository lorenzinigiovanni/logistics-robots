import { Request, Response, NextFunction } from 'express';
import jwt from 'jsonwebtoken';

import { User } from '../entity/users/User';

export async function authenticateToken(req: Request, res: Response, next: NextFunction): Promise<void> {
    if (req.originalUrl.startsWith('/auth')) {
        return next();
    }

    const authHeader = req.headers.authorization;
    const token = authHeader && authHeader.split(' ')[1];

    if (token == null) {
        res.status(401).send();
        return;
    }

    try {
        const payload = <jwt.JwtPayload>jwt.verify(token, process.env.TOKEN_SECRET as string);
        await User.findOneOrFail({ where: { ID: payload.ID } });
    } catch {
        res.status(401).send();
        return;
    }

    next();
}
