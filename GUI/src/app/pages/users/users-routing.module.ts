import { NgModule } from '@angular/core';
import { Routes, RouterModule } from '@angular/router';
import { AdminGuard } from 'app/services/admin-guard.service';

import { UsersComponent } from './users.component';
import { UserComponent } from './user/user.component';

const routes: Routes = [
  {
    path: '',
    canActivate: [AdminGuard],
    component: UsersComponent,
  },
  {
    path: ':ID',
    component: UserComponent,
  },
];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule],
})
export class UsersRoutingModule { }

export const routedComponents = [
  UsersComponent,
  UserComponent,
];
