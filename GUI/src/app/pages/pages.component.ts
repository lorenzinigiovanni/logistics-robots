import { Component } from '@angular/core';
import { NbAuthJWTToken, NbAuthService } from '@nebular/auth';
import { NbMenuItem, NbMenuService } from '@nebular/theme';

@Component({
  selector: 'lr-pages',
  styleUrls: ['pages.component.scss'],
  template: `
    <lr-one-column-layout>
      <nb-menu tag="menu" [items]="menu"></nb-menu>
      <router-outlet></router-outlet>
    </lr-one-column-layout>
  `,
})
export class PagesComponent {

  menu: NbMenuItem[] = [];

  constructor(private authService: NbAuthService, private menuService: NbMenuService) {
    this.authService.onTokenChange()
      .subscribe((token: NbAuthJWTToken) => {
        const admin = token.isValid() && token.getPayload()['admin'];

        this.menuService.addItems([
          {
            title: 'Dashboard',
            icon: 'bar-chart-outline',
            link: '/pages/dashboard',
            home: true,
          },
          {
            title: 'Map',
            icon: 'map-outline',
            link: '/pages/map',
          },
          {
            title: 'Tasks',
            icon: 'list-outline',
            link: '/pages/tasks',
          },
          {
            title: 'Robots',
            icon: 'car-outline',
            link: '/pages/robots',
            hidden: !admin,
          },
          {
            title: 'Settings',
            icon: 'settings-2-outline',
            link: '/pages/settings',
            hidden: !admin,
          },
          {
            title: 'Users',
            icon: 'people-outline',
            link: '/pages/users',
            hidden: !admin,
          },
        ], 'menu');

      });
  }

}
