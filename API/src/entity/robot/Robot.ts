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

    @Column('float', { nullable: true })
    currentX = 0.0;

    @Column('float', { nullable: true })
    currentY = 0.0;

    @Column('float', { nullable: true })
    currentTheta = 0.0;

    async getPosition(): Promise<MapNode> {
        const nodes = await MapNode.createQueryBuilder('node')
            .getMany();

        let minDistance = Infinity;
        let nearestNode = null;
        for (const node of nodes) {
            const distance = euclideanDistance(node.x, node.y, this.currentX, this.currentY);
            if (distance < minDistance) {
                minDistance = distance;
                nearestNode = node;
            }
        }

        // eslint-disable-next-line @typescript-eslint/no-non-null-assertion
        return nearestNode!;
    }
}
