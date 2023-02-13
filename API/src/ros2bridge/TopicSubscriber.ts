import { Ros } from './Ros';

export class TopicSubscriber {
    private ros: Ros | null = null;

    private url: string;
    private name: string;
    private type: string;

    private callback?: (response: any) => void;

    constructor(
        url: string,
        name: string,
        type: string,
        callback?: (response: any) => void,
    ) {

        this.url = url;
        this.name = name;
        this.type = type;

        this.callback = callback;
    }

    async open(): Promise<void> {
        this.ros = new Ros(this.url);
        await this.ros.open();

        this.registerCallbacks();
    }

    subscribe(): void {
        this.ros?.send({
            operation: 'subscribe',
            topic: this.name,
            type: this.type,
        });
    }

    unsubscribe(): void {
        this.ros?.send({
            operation: 'subscribe',
            topic: this.name,
            type: this.type,
            unsubscribe: true,
        });
    }

    private registerCallbacks(): void {
        this.ros?.on('message', (message: any) => {
            this.callback?.(message.message);
        });
    }
}
