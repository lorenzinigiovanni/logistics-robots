import { Component, OnInit } from '@angular/core';
import { Router } from '@angular/router';

import { User } from 'app/entities/users/user.entity';
import { UsersService } from 'app/services/users/users.service';
import { Observable } from 'rxjs';

@Component({
  selector: 'lr-users',
  templateUrl: './users.component.html',
  styleUrls: ['./users.component.scss'],
})
export class UsersComponent implements OnInit {

  users: Observable<User[]>;

  constructor(
    private usersService: UsersService,
    private router: Router,
  ) {

  }

  ngOnInit() {
    this.users = this.usersService.getUsers();
  }

  onEdit(user: User): void {
    this.router.navigate(['/', 'pages', 'users', user.ID]);
  }

}
