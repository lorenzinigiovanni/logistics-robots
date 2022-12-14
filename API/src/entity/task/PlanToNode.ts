import { Entity, Column, ManyToOne, PrimaryGeneratedColumn } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';
import { MapNode } from '../map/MapNode';
import { Plan } from './Plan';

@Entity()
export class PlanToNode extends CustomBaseEntity {
    @PrimaryGeneratedColumn('uuid')
    public ID!: string;

    @Column()
    public order!: number;

    @ManyToOne(() => Plan, (plan) => plan.planToNodes, { onDelete: 'CASCADE' })
    public plan!: Plan;

    @ManyToOne(() => MapNode, { onDelete: 'CASCADE' })
    public node!: MapNode;
}
