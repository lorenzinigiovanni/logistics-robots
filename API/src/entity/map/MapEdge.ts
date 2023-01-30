import { BaseEntity, Entity, PrimaryGeneratedColumn, ManyToOne, JoinColumn } from 'typeorm';
import { MapNode } from './MapNode';

@Entity()
export class MapEdge extends BaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @ManyToOne(() => MapNode, { onDelete: 'CASCADE' })
    @JoinColumn()
    node1!: MapNode;

    @ManyToOne(() => MapNode, { onDelete: 'CASCADE' })
    @JoinColumn()
    node2!: MapNode;

}
