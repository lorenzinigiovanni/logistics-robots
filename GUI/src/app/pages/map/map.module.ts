import { NgModule } from '@angular/core';
import { ReactiveFormsModule } from '@angular/forms';
import {
  NbAlertModule,
  NbButtonModule,
  NbCardModule,
  NbIconModule,
  NbInputModule,
  NbToastrModule,
  NbTooltipModule,
} from '@nebular/theme';
import { ThemeModule } from '../../@theme/theme.module';

import { MapRoutingModule, routedComponents } from './map-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    ThemeModule,
    MapRoutingModule,
    NbTooltipModule,
    NbIconModule,
    NbButtonModule,
    NbInputModule,
    ReactiveFormsModule,
    NbAlertModule,
    NbToastrModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class MapModule { }
