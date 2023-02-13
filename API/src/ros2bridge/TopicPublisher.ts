import { Ros } from './Ros';

export class TopicPublisher {
    private ros: Ros | null = null;

    private url: string;
    private name: string;
    private type: string;

    constructor(
        url: string,
        name: string,
        type: string,
    ) {
        this.url = url;
        this.name = name;
        this.type = type;
    }

    async open(): Promise<void> {
        this.ros = new Ros(this.url);
        await this.ros.open();
    }

    publish(message: any): void {
        this.ros?.send({
            operation: 'publish',
            topic: this.name,
            type: this.type,
            message: message,
        });
    }
}
