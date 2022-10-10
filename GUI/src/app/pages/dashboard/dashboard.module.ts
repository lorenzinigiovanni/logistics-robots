import { NgModule } from '@angular/core';
import {
  NbCardModule,
} from '@nebular/theme';
import { ThemeModule } from '../../@theme/theme.module';

import { DashboardRoutingModule, routedComponents } from './dashboard-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    ThemeModule,
    DashboardRoutingModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class DashboardModule { }
