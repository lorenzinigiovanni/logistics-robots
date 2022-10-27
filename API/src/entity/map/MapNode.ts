import { Entity, PrimaryGeneratedColumn, Column } from 'typeorm';
import { CustonBaseEntity } from '../CustomBaseEntity';

@Entity()
export class MapNode extends CustonBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    value!: number;

    @Column('float')
    x!: number;

    @Column('float')
    y!: number;

}
