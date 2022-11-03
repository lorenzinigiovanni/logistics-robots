import { Entity, PrimaryGeneratedColumn, Column, OneToMany } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';
import { Task } from '../task/Task';

@Entity()
export class Robot extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    name!: string;

    @Column()
    number!: number;

    @Column('float')
    x!: number;

    @Column('float')
    y!: number;

    @Column('float')
    theta!: number;

    @OneToMany(() => Task, task => task.robot)
    tasks!: Task[]
}
