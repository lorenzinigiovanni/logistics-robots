import { Injectable } from '@angular/core';
import { HttpClient, HttpErrorResponse } from '@angular/common/http';
import { Observable, throwError } from 'rxjs';
import { catchError, map } from 'rxjs/operators';
import { environment } from 'environments/environment';
import { plainToClass } from 'class-transformer';
import { NbToastrService } from '@nebular/theme';

@Injectable({
  providedIn: 'root',
})
export abstract class ApiService {
  API_URL = environment.apiUrl;

  constructor(private http: HttpClient, private toastrService: NbToastrService) { }

  protected getAll<T>(t: new () => T, url: string): Observable<T[]> {
    return this.http
      .get(this.API_URL + url)
      .pipe(
        map(res => plainToClass(t, res as Object[], { excludeExtraneousValues: true })),
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected getOne<T>(t: new () => T, url: string): Observable<T> {
    return this.http
      .get(this.API_URL + url)
      .pipe(
        map(res => plainToClass(t, res as Object, { excludeExtraneousValues: true })),
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected postOne<T>(t: new () => T, url: string, object: T): Observable<T> {
    return this.http
      .post(this.API_URL + url, object)
      .pipe(
        map(res => plainToClass(t, res as Object, { excludeExtraneousValues: true })),
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected putOne<T>(t: new () => T, url: string, object: T): Observable<T> {
    return this.http
      .put(this.API_URL + url, object)
      .pipe(
        map(res => plainToClass(t, res as Object, { excludeExtraneousValues: true })),
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected deleteOne<T>(t: new () => T, url: string): Observable<T> {
    return this.http
      .delete(this.API_URL + url)
      .pipe(
        map(res => plainToClass(t, res as Object, { excludeExtraneousValues: true })),
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected getRaw(url: string): Observable<object> {
    return this.http
      .get(this.API_URL + url)
      .pipe(
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected postRaw(url: string, body: object): Observable<object> {
    return this.http
      .post(this.API_URL + url, body)
      .pipe(
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected getString(url: string): Observable<string> {
    return this.http
      .get(this.API_URL + url, { responseType: 'text' })
      .pipe(
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected putRaw(url: string, body: object): Observable<object> {
    return this.http
      .put(this.API_URL + url, body)
      .pipe(
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        }),
      );
  }

  protected uploadFile(url: string, file: File): Observable<object> {
    const formData: FormData = new FormData();
    formData.append('file', file, file.name);

    return this.http
      .post(this.API_URL + url, formData)
      .pipe(
        catchError(error => {
          this.handleError(error);
          return throwError(error);
        })
      );
  }

  private handleError(error: HttpErrorResponse) {
    if (error.error instanceof ErrorEvent) {
      this.toastrService.show(
        `${error.error.message}`,
        `Errore`,
        {
          status: 'danger',
        },
      );
    } else {
      this.toastrService.show(
        `${error.statusText}`,
        `Errore ${error.status}`,
        {
          status: 'danger',
        },
      );
    }
  }
}
