import { Ros } from './Ros';

export class ServiceClient {
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
        this.create();
    }

    call(message: any): void {
        this.ros?.send({
            operation: 'srv_client',
            action: 'call',
            srv_name: this.name,
            srv_type: this.type,
            message: message,
        });
    }

    private registerCallbacks(): void {
        this.ros?.on('message', (message: any) => {
            this.callback?.(message.message);
        });
    }

    private create(): void {
        this.ros?.send({
            operation: 'srv_client',
            action: 'create',
            srv_name: this.name,
            srv_type: this.type,
        });
    }
}
