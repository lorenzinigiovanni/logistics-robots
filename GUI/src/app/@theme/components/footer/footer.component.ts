import { Component } from '@angular/core';

@Component({
  selector: 'lr-footer',
  styleUrls: ['./footer.component.scss'],
  template: `
    <span class="created-by">
      Created by <b><a href="https://www.lorenzinigiovanni.com" target="_blank">Lorenzini Giovanni</a></b> and Diego Planchenstainer
    </span>
    <div class="socials">
      <a href="https://github.com/lorenzinigiovanni/logistics-robots" target="_blank"><nb-icon icon="github"></nb-icon></a>
    </div>
  `,
})
export class FooterComponent {
}
