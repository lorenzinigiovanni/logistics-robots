import { Type } from 'class-transformer/decorators';

import { Robot } from "../robots/robot.entity";
import { Room } from "../map/room.entity";

export class Task {

  ID!: string;

  @Type(() => Robot)
  robot?: Robot;

  @Type(() => Room)
  goals!: Room[];

}
