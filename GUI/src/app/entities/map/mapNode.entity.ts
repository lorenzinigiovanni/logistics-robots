import { Expose } from "class-transformer";

export class MapNode {

  @Expose()
  ID!: string;

  @Expose()
  value?: number;

  @Expose()
  x?: number;

  @Expose()
  y?: number;

}
