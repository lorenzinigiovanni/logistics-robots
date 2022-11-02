import { Injectable } from '@angular/core';
import { Observable } from 'rxjs';
import { ApiService } from '../api.service';

import { Room } from 'app/entities/map/room.entity';

@Injectable({
	providedIn: 'root',
})
export class MapService extends ApiService {
	getMap(): Observable<string> {
		return this.getString(`/map/svg`);
	}

	getRooms(): Observable<Room[]> {
		return this.getAll(Room, `/map/rooms`);
	}

	putRoom(room: Room, ID: string): Observable<Room> {
		return this.putOne(Room, `/map/rooms/${ID}`, room);
	}
}
