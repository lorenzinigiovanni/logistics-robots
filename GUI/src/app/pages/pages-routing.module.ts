import { RouterModule, Routes } from '@angular/router';
import { NgModule } from '@angular/core';
import { AdminGuard } from './../services/admin-guard.service';

import { PagesComponent } from './pages.component';
import { NotFoundComponent } from './miscellaneous/not-found/not-found.component';

const routes: Routes = [{
  path: '',
  component: PagesComponent,
  children: [
    {
      path: 'dashboard',
      loadChildren: () => import('./dashboard/dashboard.module')
        .then(m => m.DashboardModule),
    },
    {
      path: 'tasks',
      loadChildren: () => import('./tasks/tasks.module')
      .then(m => m.TasksModule),
    },
    {
      path: 'settings',
      canActivate: [AdminGuard],
      loadChildren: () => import('./settings/settings.module')
        .then(m => m.SettingsModule),
    },
    {
      path: 'robots',
      canActivate: [AdminGuard],
      loadChildren: () => import('./robots/robots.module')
        .then(m => m.RobotsModule),
    },
    {
      path: 'users',
      loadChildren: () => import('./users/users.module')
        .then(m => m.UsersModule),
    },
    {
      path: '',
      redirectTo: 'dashboard',
      pathMatch: 'full',
    },
    {
      path: '**',
      component: NotFoundComponent,
    },
  ],
}];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule],
})
export class PagesRoutingModule {
}
