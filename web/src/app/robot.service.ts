import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';

@Injectable({
    providedIn: 'root'
})
export class RobotService {
    constructor(private http: HttpClient) { }

    battery() {
      return this.http.get('/api/battery')
    }

    predict() {
      return this.http.post('/api/predict', {})
    }

    act() {
      return this.http.post('/api/act', {})
    }

    turn(degrees: number) {
      return this.http.post('/api/turn', {amount: degrees});
    }

    straight() {
        return this.http.post('/api/straight', {})
    }

    halt() {
      return this.http.post('/api/halt', {});
    }

}
