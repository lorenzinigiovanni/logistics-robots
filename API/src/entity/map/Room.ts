import { Entity, PrimaryGeneratedColumn, Column, OneToOne, JoinColumn } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';
import { MapNode } from './MapNode';

@Entity()
export class Room extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column({nullable: true})
    name?: string;

    @Column()
    polygon!: string;

    @OneToOne(() => MapNode)
    @JoinColumn()
    node!: MapNode

}
