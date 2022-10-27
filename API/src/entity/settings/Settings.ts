import { Entity, PrimaryGeneratedColumn, Column } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';

@Entity()
export class Settings extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    algorithm!: string;

    @Column('float')
    robotRadius!: number;

    @Column('float')
    discretizationDistance!: number;

    @Column('float')
    doorSize!: number;

    @Column('float')
    meterPerPixel!: number;

}
