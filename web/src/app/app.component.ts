import { Component, OnInit } from '@angular/core';
import { interval } from 'rxjs';
import { RobotService } from './robot.service';

@Component({
    selector: 'app-root',
    templateUrl: './app.component.html',
    styleUrls: ['./app.component.scss']
})
export class AppComponent implements OnInit {
    public battery = {};
    private prevIndex = -1;
    private prevPredictions: any = [0, 0, 0];
    public predictions: any = [0, 0, 0];
    public maxIndex = -1;
    public wait = true;
    private readonly delay = 0;
    public running = false;
    public command = '';

    constructor(private robot: RobotService) { }

    ngOnInit() {
      this.updateBattery();
      interval(1000).subscribe(_ => this.countdown());
    }

    updateBattery() {
      this.robot.battery().subscribe(battery => this.battery = battery);
    }

    findMax(arr: [number]): number {
      let best = arr[0];
      let index = 0;
      for (let i = 1; i < arr.length; i++) {
        if (arr[i] > best) {
          best = arr[i];
          index = i;
        }
      }
      return index;
    }

    commandDone() {
      this.running = false;
      this.command = '';
    }

    countdown() {
      if (this.wait || this.running) {
        return;
      }
      this.running = true;
      this.updateBattery();
      this.command = 'PREDICT';
      this.robot.predict().subscribe(predictions => {
        this.prevIndex = this.maxIndex;
        this.prevPredictions = this.predictions;
        this.predictions = predictions;
        this.maxIndex = this.findMax(this.predictions);
        this.command = 'ACT';
        if ((this.prevIndex === 0 && this.maxIndex === 1) ||
            (this.prevIndex === 1 && this.maxIndex === 0)) {
          if (this.prevPredictions[2] > this.predictions[2]) {
            this.command = 'OVERRIDE';
            this.robot.straight().subscribe(_ => {
              this.maxIndex = 2;
              this.commandDone();
            });
            return;
          }
        }
        this.robot.act().subscribe(_ => {
          this.commandDone();
        });
      });
    }

    stop() {
      this.wait = true;
    }

    start() {
      this.wait = false;
    }

    turn(degrees: number) {
      this.running = true;
      this.command = 'TURN';
      this.robot.turn(degrees).subscribe(_ => this.commandDone());
    }

    straight() {
      this.running = true;
      this.command = 'STRAIGHT';
      this.robot.straight().subscribe(_ => this.commandDone());
    }

    halt() {
      this.running = true;
      this.command = 'HALT';
      this.robot.halt().subscribe();
    }
}
