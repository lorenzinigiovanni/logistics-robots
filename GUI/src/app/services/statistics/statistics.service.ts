import { Injectable } from '@angular/core';
import { Observable } from 'rxjs';
import { ApiService } from '../api.service';

@Injectable({
	providedIn: 'root',
})
export class StatisticsService extends ApiService {
	getStatistics(): Observable<object> {
		return this.getRaw(`/statistics`);
	}

	getTasksStatistics(period: string): Observable<object> {
		return this.getRaw(`/statistics/tasks?period=${period}`);
	}

	getRobotsStatistics(period: string): Observable<object> {
		return this.getRaw(`/statistics/robots?period=${period}`);
	}

	getPlansStatistics(period: string): Observable<object> {
		return this.getRaw(`/statistics/plans?period=${period}`);
	}

	getPlannersStatistics(period: string): Observable<object> {
		return this.getRaw(`/statistics/planners?period=${period}`);
	}
}
