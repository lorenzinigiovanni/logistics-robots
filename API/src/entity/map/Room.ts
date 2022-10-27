import { Entity, PrimaryGeneratedColumn, Column, OneToOne, JoinColumn } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';
import { MapNode } from './MapNode';

@Entity()
export class Room extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    name!: string;

    @Column({nullable: true})
    polygon?: string;

    @OneToOne(() => MapNode)
    @JoinColumn()
    node!: MapNode

}
