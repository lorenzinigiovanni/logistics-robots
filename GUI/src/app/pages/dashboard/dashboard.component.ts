import { Component, OnInit } from '@angular/core';

import { StatisticsService } from '../../services/statistics/statistics.service';

@Component({
  selector: 'lr-dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.scss'],
})
export class DashboardComponent implements OnInit {
  statistics: any = {
    nRobots: 0,
    nTasks: 0,
    nGoals: 0,
  };

  tasksStatistics: any;
  tasksPeriod: string = 'month';
  tasksGraphOptions: any;

  robotsStatistics: any;
  robotsPeriod: string = 'month';
  robotsGraphOptions: any;

  plansStatistics: any;
  plansPeriod: string = 'month';
  plansGraphOptions: any;

  plannersStatistics: any;
  plannersPeriod: string = 'month';
  plannersGraphOptions: any;

  constructor(
    private statisticsService: StatisticsService,
  ) {

  }

  async ngOnInit() {
    this.statistics = await this.statisticsService.getStatistics().toPromise();
    await this.tasksGraphInit();
    await this.robotsGraphInit();
    await this.plansGraphInit();
    await this.plannersGraphInit();
  }

  async tasksGraphInit() {
    this.tasksStatistics = await this.statisticsService.getTasksStatistics(this.tasksPeriod).toPromise();

    const dataAxis = [];
    const data = [];

    for (const stat of this.tasksStatistics) {
      if (this.tasksPeriod === 'day') {
        dataAxis.push(stat.hour + ':00');
      } else if (this.tasksPeriod === 'month') {
        dataAxis.push(stat.day + '/' + stat.month);
      } else if (this.tasksPeriod === 'year') {
        dataAxis.push(stat.month + '/' + stat.year);
      }
      data.push(+stat.nTasks);
    }

    this.tasksGraphOptions = {
      grid: {
        left: 30,
        top: 20,
        right: 0,
        bottom: 20,
      },
      xAxis: {
        data: dataAxis,
        axisLabel: {
          color: '#fff',
        },
        axisTick: {
          show: false,
        },
        axisLine: {
          show: false,
        },
        z: 10,
      },
      yAxis: {
        axisLine: {
          show: false,
        },
        axisTick: {
          show: false,
        },
        axisLabel: {
          color: '#fff',
        },
      },
      series: [
        {
          type: 'bar',
          itemStyle: {
            color: '#007bff',
          },
          data,
        },
      ],
    };
  }

  async robotsGraphInit() {
    this.robotsStatistics = await this.statisticsService.getRobotsStatistics(this.robotsPeriod).toPromise();

    const names = [];
    const completed = [];
    const cancelled = [];
    const inExecution = [];
    const assigned = [];

    for (const stat of this.robotsStatistics) {
      names.push(stat.name);
      completed.push(+stat.nCompletedTasks);
      cancelled.push(+stat.nCancelledTasks);
      inExecution.push(+stat.nInExecutionTasks);
      assigned.push(+stat.nAssignedTasks);
    }

    this.robotsGraphOptions = {
      grid: {
        left: 30,
        top: 20,
        right: 10,
        bottom: 20,
      },
      xAxis: {
        axisLabel: {
          color: '#fff',
        },
      },
      yAxis: {
        type: 'category',
        data: names,
        axisLabel: {
          inside: true,
          color: '#fff',
        },
        axisLine: {
          show: false,
        },
        axisTick: {
          show: false,
        },
        inverse: true,
        z: 10,
      },
      series: [
        {
          name: "Completed",
          data: completed,
          type: 'bar',
          stack: 'x',
          itemStyle: {
            color: '#28a745',
          },
        },
        {
          name: "Cancelled",
          data: cancelled,
          type: 'bar',
          stack: 'x',
          itemStyle: {
            color: '#dc3545',
          },
        },
        {
          name: "In execution",
          data: inExecution,
          type: 'bar',
          stack: 'x',
          itemStyle: {
            color: '#007bff',
          },
        },
        {
          name: "Assigned",
          data: assigned,
          type: 'bar',
          stack: 'x',
          itemStyle: {
            color: '#6c757d',
          },
        },
      ],
      legend: {
        show: true,
        textStyle: {
          color: '#fff'
        }
      },
    };
  }

  async plansGraphInit() {
    this.plansStatistics = await this.statisticsService.getPlansStatistics(this.plansPeriod).toPromise();

    const dataAxis = [];
    const data = [];

    for (const stat of this.plansStatistics) {
      dataAxis.push(+stat.length);
      data.push(+stat.count);
    }

    this.plansGraphOptions = {
      grid: {
        left: 30,
        top: 20,
        right: 0,
        bottom: 20,
      },
      xAxis: {
        data: dataAxis,
        axisLabel: {
          color: '#fff',
        },
        axisTick: {
          show: false,
        },
        axisLine: {
          show: false,
        },
        z: 10,
      },
      yAxis: {
        axisLine: {
          show: false,
        },
        axisTick: {
          show: false,
        },
        axisLabel: {
          color: '#fff',
        },
      },
      series: [
        {
          type: 'bar',
          itemStyle: {
            color: '#007bff',
          },
          data,
        },
      ],
    };
  }

  async plannersGraphInit() {
    this.plannersStatistics = await this.statisticsService.getPlannersStatistics(this.plannersPeriod).toPromise();

    const data = [];

    for (const stat of this.plannersStatistics) {
      data.push({
        value: +stat.count,
        name: stat.planner,
      });
    }

    this.plannersGraphOptions = {
      grid: {
        left: 0,
        top: 0,
        right: 0,
        bottom: 0,
      },
      series: [
        {
          type: 'pie',
          radius: ['40%', '70%'],
          data,
          label: {
            color: '#fff',
          },
        },
      ],
    };
  }

  async tasksPeriodChange() {
    await this.tasksGraphInit();
  }

  async robotsPeriodChange() {
    await this.robotsGraphInit();
  }

  async plansPeriodChange() {
    await this.plansGraphInit();
  }

  async plannersPeriodChange() {
    await this.plannersGraphInit();
  }

  async onRefresh() {
    await this.ngOnInit();
  }

}
