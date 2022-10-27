import { BaseEntity, FindManyOptions } from 'typeorm';

export class CustomBaseEntity extends BaseEntity {

    public static async findById<T extends CustomBaseEntity>(this: (new () => T), ID: number | string, options?: FindManyOptions<T>): Promise<T | null> {
        const entity = new this();
        const result = await (<typeof BaseEntity>entity.constructor).findByIds([ID], <typeof BaseEntity>options);

        if (result.length > 0) {
            return <T>result[0];
        } else {
            return null;
        }
    }

}
