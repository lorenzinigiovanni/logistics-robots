import { Entity, Column, ManyToOne, PrimaryGeneratedColumn } from 'typeorm';
import { Room } from '../map/Room';
import { Task } from './Task';

@Entity()
export class TaskToRoom {
    @PrimaryGeneratedColumn('uuid')
    public ID!: string;

    @Column()
    public order!: number

    @ManyToOne(() => Task, (task) => task.taskToRooms)
    public task!: Task

    @ManyToOne(() => Room, (room) => room.taskToRooms)
    public room!: Room
}
