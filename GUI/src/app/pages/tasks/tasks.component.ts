import { Component, OnInit } from '@angular/core';
import { Observable } from 'rxjs';

import { Task } from 'app/entities/tasks/task.entity';
import { TasksService } from 'app/services/tasks/tasks.service';

@Component({
  selector: 'lr-tasks',
  templateUrl: './tasks.component.html',
  styleUrls: ['./tasks.component.scss'],
})
export class TasksComponent implements OnInit {

  tasks: Observable<Task[]>;

  constructor(
    private tasksService: TasksService,
  ) {

  }

  ngOnInit() {
    this.tasks = this.tasksService.getTasks();
  }

}
