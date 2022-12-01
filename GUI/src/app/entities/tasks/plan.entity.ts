import { Expose, Type } from 'class-transformer/decorators';

import { MapNode } from '../map/mapNode.entity';
import { Robot } from "../robots/robot.entity";

export class Plan {

  @Expose()
  ID!: string;

  @Expose()
  @Type(() => Robot)
  robot?: Robot;

  @Expose()
  @Type(() => MapNode)
  nodes!: MapNode[];

}
