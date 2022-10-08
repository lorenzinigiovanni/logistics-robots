import { Component, OnInit } from '@angular/core';
import { FormGroup, FormControl, Validators } from '@angular/forms';

@Component({
  selector: 'lr-settings',
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.scss'],
})
export class SettingsComponent implements OnInit {

  algorithmForm = new FormGroup({
    algorithmType: new FormControl(null,
      [
        Validators.required,
      ]),
  });

  robotsForm = new FormGroup({
    robotRadius: new FormControl(null,
      [
        Validators.required,
        Validators.min(0),
        Validators.max(1),
      ]),
    robotSpeed: new FormControl(null,
      [
        Validators.required,
        Validators.min(0),
        Validators.max(5),
      ]),
  });

  constructor(

  ) {

  }

  ngOnInit() {

  }

  onSubmit(value: any) {
    console.log(this.algorithmForm.value);
  }

}
