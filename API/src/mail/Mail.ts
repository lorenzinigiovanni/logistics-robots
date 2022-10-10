import nodemailer from 'nodemailer';

export class Mail {
    static transporter = nodemailer.createTransport({
        host: process.env.MAIL_SERVER,
        port: 465,
        secure: true,
        auth: {
            user: process.env.MAIL_ADDRESS,
            pass: process.env.MAIL_PASSWORD,
        },
    });

    static async send(to: string, subject: string, text: string): Promise<void> {
        await this.transporter.sendMail({
            from: '"Logistics Robots" <' + process.env.MAIL_ADDRESS + '>',
            to: to,
            subject: subject,
            text: text,
        });
    }

}
