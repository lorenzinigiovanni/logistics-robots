import { Entity, PrimaryGeneratedColumn, Column, OneToOne, JoinColumn } from 'typeorm';
import { CustonBaseEntity } from '../CustomBaseEntity';
import { MapNode } from './MapNode';

@Entity()
export class Room extends CustonBaseEntity {

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
