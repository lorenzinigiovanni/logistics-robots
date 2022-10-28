import { Entity, PrimaryGeneratedColumn, Column } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';

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

}
