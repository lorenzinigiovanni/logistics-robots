import { Expose } from "class-transformer";

export class User {

  @Expose()
  ID!: string;

  @Expose()
  name!: string;

  @Expose()
  email!: string;

  @Expose()
  active!: boolean;

  @Expose()
  admin!: boolean;

}
