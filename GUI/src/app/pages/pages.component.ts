import { Component } from '@angular/core';

import { MENU_ITEMS } from './pages-menu';

@Component({
  selector: 'lr-pages',
  styleUrls: ['pages.component.scss'],
  template: `
    <lr-one-column-layout>
      <nb-menu [items]="menu"></nb-menu>
      <router-outlet></router-outlet>
    </lr-one-column-layout>
  `,
})
export class PagesComponent {

  menu = MENU_ITEMS;
}
