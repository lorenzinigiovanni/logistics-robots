import { Entity, PrimaryGeneratedColumn, ManyToOne, OneToMany } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';
import { Robot } from '../robot/Robot';
import { TaskToRoom } from './TaskToRoom';


@Entity()
export class Task extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @ManyToOne(() => Robot, robot => robot.tasks, { onDelete: 'CASCADE' })
    robot!: Robot

    @OneToMany(() => TaskToRoom, taskToRooms => taskToRooms.task, { onDelete: 'CASCADE' })
    public taskToRooms!: TaskToRoom[];

}
