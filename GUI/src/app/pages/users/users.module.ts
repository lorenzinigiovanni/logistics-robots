import { NgModule } from '@angular/core';
import { ReactiveFormsModule } from '@angular/forms';
import {
  NbButtonModule,
  NbCardModule,
  NbCheckboxModule,
  NbIconModule,
  NbInputModule,
  NbToggleModule,
} from '@nebular/theme';
import { ThemeModule } from '../../@theme/theme.module';

import { UsersRoutingModule, routedComponents } from './users-routing.module';

@NgModule({
  imports: [
    NbCardModule,
    ThemeModule,
    UsersRoutingModule,
    NbIconModule,
    NbInputModule,
    NbButtonModule,
    NbCheckboxModule,
    NbToggleModule,
    ReactiveFormsModule,
  ],
  declarations: [
    ...routedComponents,
  ],
})
export class UsersModule { }
