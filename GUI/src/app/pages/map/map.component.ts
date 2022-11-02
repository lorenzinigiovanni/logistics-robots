import { Component, OnInit, TemplateRef, ViewChild } from '@angular/core';
import { NbDialogService, NbToastrService, NbTooltipDirective } from '@nebular/theme';

import { Room } from 'app/entities/map/room.entity';
import { Task } from 'app/entities/tasks/task.entity';
import { MapService } from '../../services/map/map.service';

@Component({
  selector: 'lr-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.scss'],
})
export class MapComponent implements OnInit {
  @ViewChild('dialogRoomName') dialogRoomName: TemplateRef<any>;

  @ViewChild('map') map;

  @ViewChild('tooltip') tooltipHtml;
  @ViewChild(NbTooltipDirective) tooltip: NbTooltipDirective;
  tooltipTitle = '';

  disabledNew = false;
  disabledSend = true;
  disabledCancel = true;

  acquiring = false;

  task = new Task();
  rooms: Room[];

  constructor(
    private mapService: MapService,
    private dialogService: NbDialogService,
    private toastrService: NbToastrService,
  ) {

  }

  async ngOnInit() {
    const svg = await this.mapService.getMap().toPromise();
    this.map.nativeElement.innerHTML = svg;

    this.rooms = await this.mapService.getRooms().toPromise();
  }

  onClick(e): void {
    if (this.acquiring) {
      if (e.target.id) {
        const room = this.rooms.find(r => r.ID === e.target.id);
        this.task.goals.push(room);
      }
    }
  }

  onRightClick(e): boolean {
    if (!this.acquiring) {
      if (e.target.id) {
        this.dialogService
          .open(this.dialogRoomName)
          .onClose.subscribe(result => {
            if (result) {
              const room = this.rooms.find(r => r.ID === e.target.id);
              room.name = result;
              this.mapService.putRoom(room, room.ID).subscribe(() => {
                this.toastrService.show(
                  `updated successfully`,
                  `Room ${room.name}`,
                  {
                    status: 'success',
                  },
                );
              });
            }
          });
      }
    }

    return false;
  }

  onMouseMove(e): void {
    if (e.target.id) {
      const room = this.rooms.find(r => r.ID === e.target.id);
      this.tooltipTitle = room.name;
    }
    else {
      this.tooltipTitle = '';
    }

    this.tooltipHtml.nativeElement.style.top = e.pageY + 'px';
    this.tooltipHtml.nativeElement.style.left = e.pageX + 'px';

    this.tooltip.show();
  }

  onMouseLeave(): void {
    this.tooltip.hide();
  }

  onNew(): void {
    this.disabledNew = true;
    this.disabledSend = false;
    this.disabledCancel = false;

    this.acquiring = true;

    this.task.goals = [];
  }

  onSend(): void {
    this.disabledNew = false;
    this.disabledSend = true;
    this.disabledCancel = true;

    this.acquiring = false;

    console.log(this.task);

    // this.task.goals = [];
  }

  onCancel(): void {
    this.disabledNew = false;
    this.disabledSend = true;
    this.disabledCancel = true;

    this.acquiring = false;

    this.task.goals = [];
  }

  onDelete(i: number): void {
    this.task.goals.splice(i, 1);
  }

}
