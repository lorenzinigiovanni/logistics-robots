import { Ros } from './Ros';
import { ServiceClient } from './ServiceClient';

export class ActionClient {
    private ros: Ros | null = null;

    private url: string;
    private name: string;
    private type: string;

    private responseCallback?: (response: any) => void;
    private feedbackCallback?: (feedback: any) => void;
    private resultCallback?: (result: any) => void;

    constructor(
        url: string,
        name: string,
        type: string,
        responseCallback?: (response: any) => void,
        feedbackCallback?: (feedback: any) => void,
        resultCallback?: (result: any) => void,
    ) {
        this.url = url;
        this.name = name;
        this.type = type;

        this.responseCallback = responseCallback;
        this.feedbackCallback = feedbackCallback;
        this.resultCallback = resultCallback;
    }

    async open(): Promise<void> {
        this.ros = new Ros(this.url);
        await this.ros.open();

        this.registerCallbacks();
        this.create();
    }

    call(message: any): void {
        this.ros?.send({
            operation: 'action_client',
            action: 'call',
            action_name: this.name,
            action_type: this.type,
            message: message,
        });
    }

    async cancel(): Promise<void> {
        const cancelService = new ServiceClient(
            this.url,
            this.name + '/_action/cancel_goal',
            'action_msgs/CancelGoal',
        );

        await cancelService.open();
        cancelService.call({
            goal_info: {
                goal_id: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                stamp: {
                    sec: 0,
                    nanosec: 0,
                },
            },
        });
    }

    private registerCallbacks(): void {
        this.ros?.on('message', (message: any) => {
            if (message.action_response === 'response') {
                this.responseCallback?.(message.message);
            } else if (message.action_response === 'feedback') {
                this.feedbackCallback?.(message.message);
            } else if (message.action_response === 'result') {
                this.resultCallback?.(message.message);
            }
        });
    }

    private create(): void {
        this.ros?.send({
            operation: 'action_client',
            action: 'create',
            action_name: this.name,
            action_type: this.type,
        });
    }
}
