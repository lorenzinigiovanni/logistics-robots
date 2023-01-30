import { BaseEntity, Entity, PrimaryGeneratedColumn, Column, OneToMany, JoinColumn, OneToOne } from 'typeorm';
import { euclideanDistance } from '../../tools/distance';
import { MapNode } from '../map/MapNode';
import { Plan } from '../task/Plan';
import { Task } from '../task/Task';

@Entity()
export class Robot extends BaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    name!: string;

    @Column()
    number!: number;

    @Column('float')
    x!: number;

    @Column('float')
    y!: number;

    @Column('float')
    theta!: number;

    @OneToMany(() => Task, task => task.robot)
    tasks!: Task[];

    @OneToOne(() => Plan, plan => plan.robot, { onDelete: 'SET NULL' })
    @JoinColumn()
    plan?: Plan;

    async getPosition(): Promise<MapNode> {
        const nodes = await MapNode.createQueryBuilder('node')
            .orderBy('node.value', 'ASC')
            .getMany();

        // TODO: obtain from ROS x,y position, for now use this.x, this.y
        const robotX = this.x;
        const robotY = this.y;

        let minDistance = Infinity;
        let nearestNode = null;
        for (const node of nodes) {
            const distance = euclideanDistance(node.x, node.y, robotX, robotY);
            if (distance < minDistance) {
                minDistance = distance;
                nearestNode = node;
            }
        }

        // eslint-disable-next-line @typescript-eslint/no-non-null-assertion
        return nearestNode!;
    }
}
