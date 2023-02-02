import { NgModule } from '@angular/core';
import {
  NbButtonModule,
  NbCardModule,
  NbIconModule,
  NbProgressBarModule,
} from '@nebular/theme';
import { EnumPipe } from 'app/@theme/pipes';
import { ThemeModule } from '../../@theme/theme.module';

import { TasksRoutingModule, routedComponents } from './tasks-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    ThemeModule,
    TasksRoutingModule,
    NbIconModule,
    NbButtonModule,
    NbProgressBarModule,
  ],
  declarations: [
    ...routedComponents,
    EnumPipe,
  ],
})
export class TasksModule { }
