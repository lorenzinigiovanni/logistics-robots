import { NgModule } from '@angular/core';
import {
  NbButtonModule,
  NbCardModule, NbIconModule, NbProgressBarModule, NbSelectModule,
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
    NbButtonModule,
    NbIconModule,
    NgxEchartsModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class DashboardModule { }
