import { Component, OnInit } from '@angular/core';
import { FormGroup, FormControl, Validators } from '@angular/forms';
import { NbToastrService } from '@nebular/theme';
import { Settings } from 'app/entities/settings/settings.entity';
import { SettingsService } from 'app/services/settings/settings.service';

@Component({
  selector: 'lr-settings',
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.scss'],
})
export class SettingsComponent implements OnInit {

  settingsForm = new FormGroup({
    // algorithm
    MAPFalgorithm: new FormControl(null,
      [
        Validators.required,
      ]),
    SAPFalgorithm: new FormControl(null,
      [
        Validators.required,
      ]),
    costFunction: new FormControl(null,
      [
        Validators.required,
      ]),
    heuristic: new FormControl(null),
    // robot
    robotRadius: new FormControl(null,
      [
        Validators.required,
        Validators.min(0),
        Validators.max(1),
      ]),
    // map 
    discretizationDistance: new FormControl(null,
      [
        Validators.required,
        Validators.min(0),
        Validators.max(10),
      ]),
    doorSize: new FormControl(null,
      [
        Validators.required,
        Validators.min(0),
        Validators.max(5),
      ]),
    meterPerPixel: new FormControl(null,
      [
        Validators.required,
        Validators.min(0),
        Validators.max(10),
      ]),
  });

  loading: boolean = false; // Flag variable
  enabled_upload_button: boolean = false;
  file: File = null; // Variable to store file

  settings: Settings;

  constructor(
    private settingsService: SettingsService,
    private toastrService: NbToastrService,
  ) {

  }

  ngOnInit(): void {
    this.settingsService.getSettings().subscribe(settings => {
      this.settings = settings;
      this.settingsForm.patchValue(settings);
    });
  }

  onSubmit(value: any) {
    this.settings = { ...this.settings, ...value }

    this.settingsService.putSettings(this.settings).subscribe(settings => {
      this.toastrService.show(
        `updated successfully`,
        `Settings`,
        {
          status: 'success',
        },
      );
    });
  }

  onCancel() {
    window.location.reload();
  }

  // On file Select
  onChange(event) {
    this.file = event.target.files[0];
    if (this.file) {
      this.enabled_upload_button = true;
    }
    else {
      this.enabled_upload_button = false;
    }
  }

  // OnClick of button Upload
  onUpload() {
    this.loading = true;
    console.log(this.file);
    this.settingsService.uploadMap(this.file).subscribe(
      (event: any) => {
        this.loading = false;
      }
    );
  }

  async resetSettings() {
    await this.settingsService.resetSettings().toPromise();
    this.ngOnInit();
  }

  async eraseTasksPlans() {
    await this.settingsService.eraseTasksPlans().toPromise();
  }

}
