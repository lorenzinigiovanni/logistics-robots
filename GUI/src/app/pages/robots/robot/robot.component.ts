import { Component, OnInit } from '@angular/core';
import { FormControl, FormGroup, Validators } from '@angular/forms';
import { NbToastrService } from '@nebular/theme';
import { ActivatedRoute, Router } from '@angular/router';

import { Robot } from 'app/entities/robots/robot.entity';
import { RobotsService } from 'app/services/robots/robots.service';

@Component({
  selector: 'lr-robot',
  templateUrl: './robot.component.html',
  styleUrls: ['./robot.component.scss'],
})
export class RobotComponent implements OnInit {

  robot = new Robot();

  new = false;
  ID = '';

  robotForm = new FormGroup({
    number: new FormControl(null, [
      Validators.required,
      Validators.min(1),
      Validators.max(100),
      Validators.pattern("^[0-9]*$"),
    ]),
    name: new FormControl(null, [
      Validators.required,
    ]),
    x: new FormControl(null, [
      Validators.required,
      Validators.min(0),
      Validators.max(1000),
    ]),
    y: new FormControl(null, [
      Validators.required,
      Validators.min(0),
      Validators.max(1000),
    ]),
    theta: new FormControl(null, [
      Validators.required,
      Validators.min(0),
      Validators.max(6.28),
    ]),
  });

  constructor(
    private toastrService: NbToastrService,
    private router: Router,
    private activatedRoute: ActivatedRoute,
    private robotsService: RobotsService,
  ) {

  }

  ngOnInit() {
    this.activatedRoute.paramMap.subscribe(params => {
      this.ID = params.get('ID')
      if (this.ID == 'new') {
        this.new = true;
      } else {
        this.robotsService.getRobot(this.ID).subscribe(robot => {
          this.robot = robot;
          this.robotForm.patchValue(this.robot);
        });
      }
    });
  }

  onSubmit(value: any) {
    this.robot = { ...this.robot, ...value }

    if (this.new) {
      this.robotsService.postRobot(this.robot).subscribe(robot => {
        this.toastrService.show(
          `succesfully created`,
          `Robot ${this.robot.name}`,
          {
            status: 'success',
          },
        );
        this.router.navigate(['/', 'pages', 'robots']);
      });
    } else {
      this.robotsService.putRobot(this.robot, this.ID).subscribe(robot => {
        this.toastrService.show(
          `succesfully modified`,
          `Robot ${this.robot.name}`,
          {
            status: 'success',
          },
        );
        this.router.navigate(['/', 'pages', 'robots']);
      });
    }
  }
}
