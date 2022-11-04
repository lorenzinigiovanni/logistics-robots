import { Expose } from "class-transformer";

export class Room {

  @Expose()
  ID!: string;

  @Expose()
  name?: string;

}
