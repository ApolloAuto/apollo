import {action, computed, observable, runInAction} from 'mobx';
import {LinearInterpolant} from 'three';

export default class Planning {
  @observable sequenceNum = 0;

  data = this.initData();

  latencyGraph = {
      planning: []
  };

  @action updateSequenceNum(newSequenceNum) {
    this.sequenceNum = newSequenceNum;
  }

  initData() {
    return {
      slGraph: {},
      stGraph: {},
      stSpeedGraph: {},
      speedGraph: {},
      kappaGraph: {},
      dkappaGraph: {},
    };
  }

  generateDataPoints(X, Y, transform) {
    if (!X || !Y || X.length !== Y.length) {
      return [];
    }

    const bound = [];
    for (let idx = 0; idx < Y.length; idx++) {
      const x = Number(X[idx]);
      let y = Number(Y[idx]);
      if (transform !== undefined) {
        y = transform(y);
      }
      bound.push({x: x, y: y});
    }
    return bound;
  }

  transformMapBound(l) {
    return (l > 10 || l < -10) ? (100 * l / Math.abs(l)) : l;
  }

  extractDataPoints(data, xField, yField, loopBack = false) {
    if (!data) {
      return [];
    }

    const points = data.map((point) => {
      return {x: point[xField], y: point[yField]};
    });

    if (loopBack && data.length) {
      points.push({x: data[0][xField], y: data[0][yField]});
    }

    return points;
  }

  updateSLFrame(slFrame) {
    const graph = this.data.slGraph;

    const sampledS = slFrame[0].sampledS;
    graph.mapLowerBound = this.generateDataPoints(
        sampledS, slFrame[0].mapLowerBound, this.transformMapBound);
    graph.mapUpperBound = this.generateDataPoints(
        sampledS, slFrame[0].mapUpperBound, this.transformMapBound);
    graph.staticObstacleLowerBound =
        this.generateDataPoints(sampledS, slFrame[0].staticObstacleLowerBound);
    graph.staticObstacleUpperBound =
        this.generateDataPoints(sampledS, slFrame[0].staticObstacleUpperBound);
    graph.dynamicObstacleLowerBound =
        this.generateDataPoints(sampledS, slFrame[0].dynamicObstacleLowerBound);
    graph.dynamicObstacleUpperBound =
        this.generateDataPoints(sampledS, slFrame[0].dynamicObstacleUpperBound);
    graph.pathLine = this.extractDataPoints(slFrame[0].slPath, 's', 'l');

    const aggregatedBoundaryS = slFrame[1].aggregatedBoundaryS;
    graph.aggregatedBoundaryLow = this.generateDataPoints(
        aggregatedBoundaryS, slFrame[1].aggregatedBoundaryLow);
    graph.aggregatedBoundaryHigh = this.generateDataPoints(
        aggregatedBoundaryS, slFrame[1].aggregatedBoundaryHigh);
  }

  updateSTGraph(stGraphs) {
    for (const stGraph of stGraphs) {
      this.data.stGraph[stGraph.name] = {obstaclesBoundary: {}};
      const graph = this.data.stGraph[stGraph.name];

      // obstacle boundary lines
      if (stGraph.boundary) {
        for (const boundary of stGraph.boundary) {
          const type = boundary.type.substring('ST_BOUNDARY_TYPE_'.length);
          const label = `${boundary.name}_${type}`;
          graph.obstaclesBoundary[label] =
              this.extractDataPoints(boundary.point, 't', 's', true);
        }
      }

      // curve line
      graph.curveLine = this.extractDataPoints(stGraph.speedProfile, 't', 's');

      // kernel cruise line
      if (stGraph.kernelCruiseRef) {
        graph.kernelCruise = this.generateDataPoints(
            stGraph.kernelCruiseRef.t, stGraph.kernelCruiseRef.cruiseLineS);
      }

      // kernel follow line
      if (stGraph.kernelFollowRef) {
        graph.kernelFollow = this.generateDataPoints(
            stGraph.kernelFollowRef.t, stGraph.kernelFollowRef.followLineS);
      }
    }
  }

  updateSTSpeedGraph(stGraphs) {
    for (const stGraph of stGraphs) {
      this.data.stSpeedGraph[stGraph.name] = {};
      const graph = this.data.stSpeedGraph[stGraph.name];

      // speed: limit
      graph.limit = this.extractDataPoints(stGraph.speedLimit, 's', 'v');

      // speed: planned
      graph.planned = this.extractDataPoints(stGraph.speedProfile, 's', 'v');

      // speed: constraint
      if (stGraph.speedConstraint) {
        const curveT = stGraph.speedProfile.map((point) => {
          return point.t;
        });
        const curveS = stGraph.speedProfile.map((point) => {
          return point.s;
        });
        const interpolant = new LinearInterpolant(curveT, curveS, 1, []);
        const speedConstraintS = stGraph.speedConstraint.t.map((point) => {
          return interpolant.evaluate(point)[0];
        });

        graph.lowerConstraint = this.generateDataPoints(
            speedConstraintS, stGraph.speedConstraint.lowerBound);
        graph.upperConstraint = this.generateDataPoints(
            speedConstraintS, stGraph.speedConstraint.upperBound);
      }
    }
  }

  updateSpeed(speedPlans, trajectory) {
    const graph = this.data.speedGraph;
    if (speedPlans) {
      for (const plan of speedPlans) {
        graph[plan.name] = this.extractDataPoints(plan.speedPoint, 't', 'v');
      }
    }

    if (trajectory) {
      graph.finalSpeed =
          this.extractDataPoints(trajectory, 'timestampSec', 'speed');
    }
  }

  updateKappaGraph(paths) {
    for (const path of paths) {
      this.data.kappaGraph[path.name] =
        this.extractDataPoints(path.pathPoint, 's', 'kappa');
    }
  }

  updateDkappaGraph(paths) {
    for (const path of paths) {
      this.data.dkappaGraph[path.name] =
        this.extractDataPoints(path.pathPoint, 's', 'dkappa');
    }
  }

  updadteLatencyGraph(currentTime, latencyStates) {
    const timeRange = 300000; // 5 min
    for (const moduleName in this.latencyGraph) {
      let graph = this.latencyGraph[moduleName];
      if (graph.length > 0) {
        const startTime = graph[0].x;
        const endTime = graph[graph.length - 1].x;
        const diff = currentTime - startTime;
        if (currentTime < endTime) {
          // new data set, clean up existing one
          this.latencyGraph[moduleName] = [];
          graph = this.latencyGraph[moduleName];

        } else if (diff > timeRange) {
          // shift out old data
          graph.shift();
        }
      }

      if (graph.length === 0 ||
          graph[graph.length - 1].x !== currentTime) {
          graph.push({x: currentTime, y: latencyStates.planning});
      }
    }
  }

  update(world) {
    this.updateSequenceNum(world.sequenceNum);
    this.data = this.initData();

    const planningData = world.planningData;
    if (planningData) {
      if (planningData.slFrame) {
        this.updateSLFrame(planningData.slFrame);
      }

      if (planningData.stGraph) {
        this.updateSTGraph(planningData.stGraph);
        this.updateSTSpeedGraph(planningData.stGraph);
      }

      if (planningData.speedPlan) {
        this.updateSpeed(planningData.speedPlan, world.planningTrajectory);
      }

      if (planningData.path) {
        this.updateKappaGraph(planningData.path);
        this.updateDkappaGraph(planningData.path);
      }
    }

    if (world.latency) {
      this.updadteLatencyGraph(world.timestampSec, world.latency);
    }
  }
}
