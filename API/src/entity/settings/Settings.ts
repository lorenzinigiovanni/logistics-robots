import { BaseEntity, Entity, PrimaryGeneratedColumn, Column } from 'typeorm';

@Entity()
export class Settings extends BaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column('varchar')
    MAPFalgorithm = 'A*';

    @Column('varchar')
    SAPFalgorithm = 'A*';

    @Column('varchar')
    costFunction = 'SIC';

    @Column('varchar', { nullable: true })
    heuristic = 'MANHATTAN';

    @Column('float')
    robotRadius = 0.2;

    @Column('float')
    discretizationDistance = 1.3;

    @Column('float')
    doorSize = 1.2;

    @Column('float')
    meterPerPixel = 0.0254;

}
