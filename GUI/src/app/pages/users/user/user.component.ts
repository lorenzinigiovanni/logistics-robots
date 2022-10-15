import { Component, OnInit } from '@angular/core';
import { FormControl, FormGroup, Validators } from '@angular/forms';
import { ActivatedRoute, Router } from '@angular/router';
import { NbAuthJWTToken, NbAuthService } from '@nebular/auth';
import { NbToastrService } from '@nebular/theme';

import { User } from 'app/entities/users/user.entity';
import { UsersService } from 'app/services/users/users.service';

@Component({
  selector: 'lr-users',
  templateUrl: './user.component.html',
  styleUrls: ['./user.component.scss'],
})
export class UserComponent implements OnInit {

  user: User;

  admin: boolean;
  userID: string;

  form = new FormGroup({
    name: new FormControl(null, [
      Validators.required,
    ]),
    email: new FormControl(null, [
      Validators.required,
      Validators.email,
    ]),
    active: new FormControl(null),
    admin: new FormControl(null),
  });

  passwordForm = new FormGroup({
    oldPassword: new FormControl(null, [
      Validators.required,
    ]),
    newPassword: new FormControl(null, [
      Validators.required,
    ]),
  });

  constructor(
    private activatedRoute: ActivatedRoute,
    private usersService: UsersService,
    private toastrService: NbToastrService,
    private authService: NbAuthService,
    private router: Router,
  ) {

  }

  ngOnInit() {
    this.activatedRoute.paramMap.subscribe(params => {
      this.usersService.getUser(params.get('ID')).subscribe(user => {
        this.user = user;

        this.form.controls.name.setValue(user.name);
        this.form.controls.email.setValue(user.email);
        this.form.controls.active.setValue(user.active);
        this.form.controls.admin.setValue(user.admin);
      });
    });

    this.authService.onTokenChange()
      .subscribe((token: NbAuthJWTToken) => {
        this.admin = token.isValid() && token.getPayload()['admin'];
        this.userID = token.isValid() && token.getPayload()['ID'];
      });
  }

  onSubmit(value: any) {
    this.user.name = value.name;
    this.user.email = value.email;
    this.user.active = value.active;
    this.user.admin = value.admin;

    this.usersService.putUser(this.user, this.user.ID).subscribe(user => {
      this.toastrService.show(
        `edited successfully`,
        `User ${this.user.name}`,
        {
          status: 'success',
        },
      );
    });
  }

  changePassword(value: any) {
    this.usersService.changePassword(this.user.ID, value.oldPassword, value.newPassword).subscribe(() => {
      this.toastrService.show(
        `changed successfully`,
        `Password`,
        {
          status: 'success',
        },
      );
    });
  }

  deleteUser() {
    this.usersService.deleteUser(this.user.ID).subscribe(() => {
      this.toastrService.show(
        `deleted successfully`,
        `User ${this.user.name}`,
        {
          status: 'success',
        },
      );
    });
    this.router.navigate(['/', 'pages', 'users']);
  }

}
