import { Injectable } from '@angular/core';
import { Observable } from 'rxjs';
import { ApiService } from '../api.service';

import { Robot } from 'app/entities/robots/robot.entity';

@Injectable({
  providedIn: 'root',
})
export class RobotsService extends ApiService {
  getRobots(): Observable<Robot[]> {
    return this.getAll(Robot, `/robots`);
  }

  getRobot(ID: string): Observable<Robot> {
    return this.getOne(Robot, `/robots/${ID}`);
  }

  postRobot(robot: Robot): Observable<Robot> {
    return this.postOne(Robot, `/robots`, robot);
  }

  putRobot(robot: Robot, ID: string): Observable<Robot> {
    return this.putOne(Robot, `/robots/${ID}`, robot);
  }

  deleteRobot(ID: string): Observable<Robot> {
    return this.deleteOne(Robot, `/robots/${ID}`);
  }

}
