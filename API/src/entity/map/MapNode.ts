import { BaseEntity, Entity, PrimaryGeneratedColumn, Column } from 'typeorm';

@Entity()
export class MapNode extends BaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    value!: number;

    @Column('float')
    x!: number;

    @Column('float')
    y!: number;

}
