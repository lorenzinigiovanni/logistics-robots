import { NgModule } from '@angular/core';
import { Routes, RouterModule } from '@angular/router';

import { RobotsComponent } from './robots.component';
import { RobotComponent } from './robot/robot.component';

const routes: Routes = [
  {
    path: '',
    component: RobotsComponent,
  },
  {
    path: ':ID',
    component: RobotComponent,
  },
];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule],
})
export class RobotsRoutingModule { }

export const routedComponents = [
  RobotsComponent,
  RobotComponent,
];
