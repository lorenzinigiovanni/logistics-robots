import { EventEmitter, once } from 'events';
import WebSocket from 'ws';
import { randomUUID } from 'crypto';

export class Ros extends EventEmitter {
    private url: string;
    private ws?: WebSocket;

    constructor(url: string) {
        super();
        this.url = url;

        this.onOpen = this.onOpen.bind(this);
        this.onClose = this.onClose.bind(this);
        this.onMessage = this.onMessage.bind(this);
    }

    async open(): Promise<void> {
        try {
            this.ws = new WebSocket(this.url);

            this.ws.on('close', this.onClose);
            this.ws.on('message', this.onMessage);

            await once(this.ws, 'open');
        } catch (e) {
            console.error(e);
        }
    }

    close(): void {
        this.ws?.close();
    }

    private onOpen(): void {
        this.emit('open');
    }

    private onClose(): void {
        this.emit('close');
    }

    private onMessage(data: any): void {
        const message = JSON.parse(data);
        this.emit('message', message);
    }

    send(message: any): void {
        this.ws?.send(JSON.stringify(message));
    }

    getRandomUUID(): string {
        return randomUUID();
    }

}
