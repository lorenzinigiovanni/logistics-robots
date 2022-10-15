import { Injectable } from '@angular/core';
import { CanActivate } from '@angular/router';
import { NbAuthJWTToken, NbAuthService } from '@nebular/auth';
import { map } from 'rxjs/operators';

@Injectable()
export class AdminGuard implements CanActivate {

  constructor(private authService: NbAuthService) {
  }

  canActivate() {
    return this.authService.onTokenChange()
      .pipe(
        map((token: NbAuthJWTToken) => {
          return token.isValid() && token.getPayload()['admin'];
        }),
      );
  }
}
