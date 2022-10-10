import { NgModule } from '@angular/core';
import {
  NbCardModule,
} from '@nebular/theme';
import { ThemeModule } from '../../@theme/theme.module';

import { TasksRoutingModule, routedComponents } from './tasks-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    ThemeModule,
    TasksRoutingModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class TasksModule { }
