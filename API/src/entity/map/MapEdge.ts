import { Entity, PrimaryGeneratedColumn, ManyToOne, JoinColumn } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';
import { MapNode } from './MapNode';

@Entity()
export class MapEdge extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @ManyToOne(() => MapNode, { onDelete: 'CASCADE' })
    @JoinColumn()
    node1!: MapNode

    @ManyToOne(() => MapNode, { onDelete: 'CASCADE' })
    @JoinColumn()
    node2!: MapNode

}
