import { Injectable } from '@angular/core';
import { Observable } from 'rxjs';
import { ApiService } from '../api.service';

import { Plan } from 'app/entities/tasks/plan.entity';

@Injectable({
  providedIn: 'root',
})
export class PlansService extends ApiService {
  getPlans(): Observable<Plan[]> {
    return this.getAll(Plan, `/plans`);
  }
}
