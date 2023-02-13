import { Expose, Type } from 'class-transformer';

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

  @Expose()
  @Type(() => Room)
  completedGoals?: Room[];

  @Expose()
  @Type(() => Date)
  createdAt!: Date;

  @Expose()
  public status!: string;

}
