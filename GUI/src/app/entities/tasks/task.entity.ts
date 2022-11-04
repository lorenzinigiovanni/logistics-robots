import { Expose, Type } from 'class-transformer/decorators';

import { Robot } from "../robots/robot.entity";
import { Room } from "../map/room.entity";

export class Task {

  @Expose()
  ID!: string;

  @Expose()
  @Type(() => Robot)
  robot?: Robot;

  @Expose()
  @Type(() => Room)
  goals!: Room[];

}
