import { Component, OnInit, TemplateRef, ViewChild } from '@angular/core';
import { NbDialogService } from '@nebular/theme';

import { Robot } from 'app/entities/robots/robot.entity';
import { RobotsService } from 'app/services/robots/robots.service';
import { Observable } from 'rxjs';

@Component({
  selector: 'lr-robots',
  templateUrl: './robots.component.html',
  styleUrls: ['./robots.component.scss'],
})
export class RobotsComponent implements OnInit {

  robots: Observable<Robot[]>;

  @ViewChild('dialogDeleteRobot') dialogDeleteRobot: TemplateRef<any>;

  constructor(
    private robotsService: RobotsService,
    private dialogService: NbDialogService,
  ) {

  }

  ngOnInit() {
    this.updateRobots();
  }

  updateRobots() {
    this.robots = this.robotsService.getRobots();
  }

  onDelete(ID: string): void {
    this.dialogService
      .open(this.dialogDeleteRobot, { context: `Are you sure you want to delete the robot?` })
      .onClose.subscribe(result => {
        if (result === true) {
          this.robotsService.deleteRobot(ID).subscribe(x => {
            this.updateRobots();
          });
        }
      });
  }

}
