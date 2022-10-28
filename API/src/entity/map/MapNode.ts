import { Entity, PrimaryGeneratedColumn, Column } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';

@Entity()
export class MapNode extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    value!: number;

    @Column('float')
    x!: number;

    @Column('float')
    y!: number;

}
