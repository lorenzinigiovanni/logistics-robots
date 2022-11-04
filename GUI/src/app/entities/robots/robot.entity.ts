import { Expose } from "class-transformer";

export class Robot {

  @Expose()
  ID!: string;

  @Expose()
  name!: string;

  @Expose()
  number!: number;

  @Expose()
  x!: number;

  @Expose()
  y!: number;

  @Expose()
  theta!: number;

}
