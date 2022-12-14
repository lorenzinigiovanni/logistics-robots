import { Entity, PrimaryGeneratedColumn, Column, OneToOne, JoinColumn, OneToMany } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';
import { MapNode } from './MapNode';
import { TaskToRoom } from '../task/TaskToRoom';

@Entity()
export class Room extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column({ nullable: true })
    name?: string;

    @Column()
    polygon!: string;

    @OneToOne(() => MapNode, { onDelete: 'CASCADE' })
    @JoinColumn()
    node!: MapNode

    @OneToMany(() => TaskToRoom, taskToRooms => taskToRooms.room, { onDelete: 'CASCADE' })
    public taskToRooms!: TaskToRoom[];

}
