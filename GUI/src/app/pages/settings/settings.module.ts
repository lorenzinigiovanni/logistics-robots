import { NgModule } from '@angular/core';
import {
  NbCardModule,
  NbSelectModule,
  NbButtonModule,
  NbInputModule,
  NbSpinnerModule,
  NbAlertModule,
} from '@nebular/theme';
import { ThemeModule } from '../../@theme/theme.module';
import { ReactiveFormsModule } from '@angular/forms';

import { SettingsRoutingModule, routedComponents } from './settings-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    NbSelectModule,
    NbButtonModule,
    NbInputModule,
    NbSpinnerModule,
    NbAlertModule,
    ThemeModule,
    SettingsRoutingModule,
    ReactiveFormsModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class SettingsModule { }
