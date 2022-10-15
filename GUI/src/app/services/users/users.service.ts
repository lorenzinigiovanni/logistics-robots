import { Injectable } from '@angular/core';
import { Observable } from 'rxjs';
import { ApiService } from '../api.service';

import { User } from 'app/entities/users/user.entity';

@Injectable({
  providedIn: 'root',
})
export class UsersService extends ApiService {
  getUsers(): Observable<User[]> {
    return this.getAll(User, `/users`);
  }

  getUser(ID: string): Observable<User> {
    return this.getOne(User, `/users/${ID}`);
  }

  postUser(user: User): Observable<User> {
    return this.postOne(User, `/users`, user);
  }

  putUser(user: User, ID: string): Observable<User> {
    return this.putOne(User, `/users/${ID}`, user);
  }

  deleteUser(ID: string): Observable<User> {
    return this.deleteOne(User, `/users/${ID}`);
  }

  changePassword(ID: string, oldPassword: string, newPassword: string): Observable<object> {
    return this.putRaw(`/users/${ID}/password`, {
      oldPassword,
      newPassword,
    });
  }
}
