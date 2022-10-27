import { Injectable } from '@angular/core';
import { Observable } from 'rxjs';
import { ApiService } from '../api.service';
import { Settings } from 'app/entities/settings/settings.entity';

@Injectable({
	providedIn: 'root',
})
export class SettingsService extends ApiService {
	uploadMap(file: File): Observable<any> {
		return this.uploadFile(`/map`, file);
	}
}
