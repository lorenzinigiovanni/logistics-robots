import { BaseEntity, Entity, PrimaryGeneratedColumn, Column } from 'typeorm';

@Entity()
export class Settings extends BaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    MAPFalgorithm!: string;

    @Column()
    SAPFalgorithm!: string;

    @Column()
    costFunction!: string;

    @Column({ nullable: true })
    heuristic!: string;

    @Column('float')
    robotRadius!: number;

    @Column('float')
    discretizationDistance!: number;

    @Column('float')
    doorSize!: number;

    @Column('float')
    meterPerPixel!: number;

}
