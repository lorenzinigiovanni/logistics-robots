import { Injectable } from '@angular/core';
import { Observable } from 'rxjs';
import { ApiService } from '../api.service';
import { Settings } from 'app/entities/settings/settings.entity';

@Injectable({
	providedIn: 'root',
})
export class SettingsService extends ApiService {
	getSettings(): Observable<Settings> {
		return this.getOne(Settings, `/settings`);
	}

	putSettings(settings: Settings): Observable<Settings> {
		return this.putOne(Settings, `/settings`, settings);
	}

	uploadMap(file: File): Observable<any> {
		return this.uploadFile(`/map`, file);
	}

	resetSettings(): Observable<object> {
		return this.postRaw(`/settings/resetsettings`, {});
	}

	eraseTasksPlans(): Observable<object> {
		return this.postRaw(`/settings/erasetasksplans`, {});
	}
}
