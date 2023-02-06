import { NgModule } from '@angular/core';
import {
  NbCardModule, NbProgressBarModule, NbSelectModule,
} from '@nebular/theme';
import { NgxEchartsModule } from 'ngx-echarts';
import { ThemeModule } from '../../@theme/theme.module';

import { DashboardRoutingModule, routedComponents } from './dashboard-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    ThemeModule,
    DashboardRoutingModule,
    NbProgressBarModule,
    NbSelectModule,
    NgxEchartsModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class DashboardModule { }
