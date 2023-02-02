import { Pipe, PipeTransform } from '@angular/core';

@Pipe({ name: 'lrEnum' })
export class EnumPipe implements PipeTransform {

  transform(input: string): string {
    let x = input.replace('_', ' ');
    x = x.replace(/\w\S*/g, (txt) => txt.charAt(0).toUpperCase() + txt.substring(1).toLowerCase());
    return x;
  }
}
