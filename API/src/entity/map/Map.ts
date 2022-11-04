import { Entity, PrimaryGeneratedColumn, Column } from 'typeorm';
import { CustomBaseEntity } from '../CustomBaseEntity';

@Entity()
export class Map extends CustomBaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column()
    svg!: string;

}
