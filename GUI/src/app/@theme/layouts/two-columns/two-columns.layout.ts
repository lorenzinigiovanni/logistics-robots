import { Component } from '@angular/core';

@Component({
  selector: 'lr-two-columns-layout',
  styleUrls: ['./two-columns.layout.scss'],
  template: `
    <nb-layout>
      <nb-layout-header fixed>
        <lr-header></lr-header>
      </nb-layout-header>

      <nb-sidebar class="menu-sidebar" tag="menu-sidebar">
        <ng-content select="nb-menu"></ng-content>
      </nb-sidebar>

      <nb-layout-column class="small">
      </nb-layout-column>

      <nb-layout-column>
        <ng-content select="router-outlet"></ng-content>
      </nb-layout-column>

      <nb-layout-footer fixed>
        <lr-footer></lr-footer>
      </nb-layout-footer>

    </nb-layout>
  `,
})
export class TwoColumnsLayoutComponent {}
