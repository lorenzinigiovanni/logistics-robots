import { NgModule } from '@angular/core';
import {
  NbButtonModule,
  NbCardModule,
  NbIconModule,
} from '@nebular/theme';
import { ThemeModule } from '../../@theme/theme.module';

import { TasksRoutingModule, routedComponents } from './tasks-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    ThemeModule,
    TasksRoutingModule,
    NbIconModule,
    NbButtonModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class TasksModule { }
