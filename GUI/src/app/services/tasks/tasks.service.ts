import { Injectable } from '@angular/core';
import { Observable } from 'rxjs';
import { ApiService } from '../api.service';

import { Task } from 'app/entities/tasks/task.entity';

@Injectable({
  providedIn: 'root',
})
export class TasksService extends ApiService {
  getTasks(): Observable<Task[]> {
    return this.getAll(Task, `/tasks`);
  }

  getTask(ID: string): Observable<Task> {
    return this.getOne(Task, `/tasks/${ID}`);
  }

  postTask(task: Task): Observable<Task> {
    return this.postOne(Task, `/tasks`, task);
  }

  putTask(task: Task, ID: string): Observable<Task> {
    return this.putOne(Task, `/tasks/${ID}`, task);
  }

  deleteTask(ID: string): Observable<Task> {
    return this.deleteOne(Task, `/tasks/${ID}`);
  }

}
