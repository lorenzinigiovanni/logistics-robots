import { BaseEntity, Entity, PrimaryGeneratedColumn, Column } from 'typeorm';
import * as argon2 from 'argon2';

@Entity()
export class User extends BaseEntity {

    @PrimaryGeneratedColumn('uuid')
    ID!: string;

    @Column({ nullable: false })
    name!: string;

    @Column({ nullable: false, unique: true })
    email!: string;

    @Column({ nullable: false, select: false })
    password!: string;

    @Column({ nullable: false })
    active!: boolean;

    @Column({ nullable: false })
    admin!: boolean;

    async setPassword(password: string): Promise<void> {
        this.password = await argon2.hash(password, { type: argon2.argon2id });
    }

    async verifyPassword(password: string): Promise<boolean> {
        return await argon2.verify(this.password, password);
    }

}
