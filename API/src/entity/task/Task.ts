import { BaseEntity, Entity, PrimaryGeneratedColumn, ManyToOne, OneToMany, CreateDateColumn, Column } from 'typeorm';
import { Room } from '../map/Room';
import { Robot } from '../robot/Robot';
import { TaskToRoom } from './TaskToRoom';

export enum TaskStatus {
    NOT_ASSIGNED = 'not_assigned',
    ASSIGNED = 'assigned',
    IN_EXECUTION = 'in_execution',
    COMPLETED = 'completed',
}

@Entity()
export class Task extends BaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @ManyToOne(() => Robot, robot => robot.tasks, { onDelete: 'CASCADE' })
    robot!: Robot;

    @OneToMany(() => TaskToRoom, taskToRooms => taskToRooms.task, { onDelete: 'CASCADE' })
    public taskToRooms?: TaskToRoom[];

    public goals?: Room[];

    public completedGoals?: Room[];

    @CreateDateColumn()
    public createdAt!: Date;

    @Column({
        type: 'enum',
        enum: TaskStatus,
        default: TaskStatus.NOT_ASSIGNED,
    })
    public status!: TaskStatus;

}
