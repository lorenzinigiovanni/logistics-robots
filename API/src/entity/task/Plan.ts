import { Entity, PrimaryGeneratedColumn, OneToMany, OneToOne } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';

import { MapNode } from '../map/MapNode';
import { Robot } from '../robot/Robot';
import { PlanToNode } from './PlanToNode';


@Entity()
export class Plan extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @OneToOne(() => Robot, robot => robot.plan, { onDelete: 'CASCADE' })
    robot!: Robot;

    @OneToMany(() => PlanToNode, planToNodes => planToNodes.plan, { onDelete: 'CASCADE' })
    public planToNodes?: PlanToNode[];

    public nodes?: MapNode[];

}
