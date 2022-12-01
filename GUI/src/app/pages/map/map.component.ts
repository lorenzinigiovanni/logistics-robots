import { Component, OnInit, Renderer2, TemplateRef, ViewChild } from '@angular/core';
import { NbDialogService, NbToastrService, NbTooltipDirective } from '@nebular/theme';

import { Room } from 'app/entities/map/room.entity';
import { Task } from 'app/entities/tasks/task.entity';
import { TasksService } from 'app/services/tasks/tasks.service';
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

  computingTask = false;

  task = new Task();
  rooms: Room[];

  constructor(
    private mapService: MapService,
    private tasksService: TasksService,
    private dialogService: NbDialogService,
    private toastrService: NbToastrService,
    private renderer: Renderer2,
  ) {

  }

  async ngOnInit() {
    await this.loadMap();
    this.rooms = await this.mapService.getRooms().toPromise();
  }

  async loadMap(): Promise<void> {
    const svg = await this.mapService.getMap().toPromise();
    this.map.nativeElement.innerHTML = svg;
  }

  onClick(e): void {
    if (this.acquiring) {
      if (e.target.id) {
        const room = this.rooms.find(r => r.ID === e.target.id);
        const polyline = this.map.nativeElement.querySelector(`[id="${room.ID}"]`);
        this.renderer.addClass(polyline, 'selected');
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

  async onSend(): Promise<void> {
    this.computingTask = true;

    await this.tasksService.postTask(this.task).toPromise();

    this.toastrService.show(
      `created successfully`,
      `Task`,
      {
        status: 'success',
      },
    );

    this.onCancel();

    this.loadMap();

    this.computingTask = false;
  }

  onCancel(): void {
    this.disabledNew = false;
    this.disabledSend = true;
    this.disabledCancel = true;

    this.acquiring = false;

    const polylines = this.map.nativeElement.querySelectorAll(`polyline`);
    for (const polyline of polylines) {
      this.renderer.removeClass(polyline, 'selected');
    }

    this.task.goals = [];
  }

  onDelete(i: number): void {
    const room = this.rooms.find(r => r.ID === this.task.goals[i].ID);
    const polyline = this.map.nativeElement.querySelector(`[id="${room.ID}"]`);
    this.renderer.removeClass(polyline, 'selected');

    this.task.goals.splice(i, 1);
  }

}
