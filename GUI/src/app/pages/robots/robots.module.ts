import { NgModule } from '@angular/core';
import { ReactiveFormsModule } from '@angular/forms';
import {
  NbButtonModule,
  NbCardModule,
  NbIconModule,
  NbInputModule,
} from '@nebular/theme';
import { ThemeModule } from '../../@theme/theme.module';

import { RobotsRoutingModule, routedComponents } from './robots-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    ThemeModule,
    RobotsRoutingModule,
    NbIconModule,
    NbInputModule,
    NbButtonModule,
    ReactiveFormsModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class RobotsModule { }
