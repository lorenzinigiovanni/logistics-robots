import { Component, OnInit } from '@angular/core';
import { FormGroup, FormControl, Validators } from '@angular/forms';
import { SettingsService } from 'app/services/settings/settings.service';

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


  loading: boolean = false; // Flag variable
  file: File = null; // Variable to store file

  constructor(private settingsService: SettingsService) { }

  ngOnInit(): void {
  }

  onSubmit(value: any) {
    console.log(this.algorithmForm.value);
  }

  // On file Select
  onChange(event) {
    this.file = event.target.files[0];
  }

  // OnClick of button Upload
  onUpload() {
    this.loading = !this.loading;
    console.log(this.file);
    this.settingsService.uploadMap(this.file).subscribe(
      (event: any) => {
        this.loading = false;
      }
    );
  }

}
