import { Entity, PrimaryGeneratedColumn, ManyToOne, JoinColumn } from 'typeorm';
import { CustonBaseEntity } from '../CustomBaseEntity';
import { MapNode } from './MapNode';

@Entity()
export class MapEdge extends CustonBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @ManyToOne(() => MapNode)
    @JoinColumn()
    node1!: MapNode

    @ManyToOne(() => MapNode)
    @JoinColumn()
    node2!: MapNode

}
