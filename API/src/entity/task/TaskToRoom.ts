import { Entity, Column, ManyToOne, PrimaryGeneratedColumn } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';
import { Room } from '../map/Room';
import { Task } from './Task';

@Entity()
export class TaskToRoom extends CustomBaseEntity {
    @PrimaryGeneratedColumn('uuid')
    public ID!: string;

    @Column()
    public order!: number;

    @ManyToOne(() => Task, (task) => task.taskToRooms, { onDelete: 'CASCADE' })
    public task!: Task;

    @ManyToOne(() => Room, (room) => room.taskToRooms, { onDelete: 'CASCADE' })
    public room!: Room;

    @Column('boolean', {default: false})
    public completed!: boolean;
}
