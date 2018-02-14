import {action, computed, observable, runInAction} from 'mobx';
import {LinearInterpolant} from 'three';

export default class PlanningData {
  @observable planningTime = null;

  data = this.initData();

  latencyGraph = {
      planning: []
  };

  @action updatePlanningTime(newTime) {
    this.planningTime = newTime;
  }

  initData() {
    return {
      slGraph: {},
      stGraph: {},
      stSpeedGraph: {},
      speedGraph: {},
      accelerationGraph: {},
      thetaGraph: {},
      kappaGraph: {},
      dkappaGraph: {},
      dpPolyGraph: {},
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

  extractDataPoints(data, xField, yField, loopBack = false, xOffset = 0) {
    if (!data) {
      return [];
    }

    const points = data.map((point) => {
      return {x: point[xField] + xOffset, y: point[yField]};
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
      graph.finalSpeed = this.extractDataPoints(
        trajectory, 'timestampSec', 'speed', false /* loop back */, -this.planningTime);
    }
  }

  updateAccelerationGraph(trajectory) {
    const graph = this.data.accelerationGraph;
    if (trajectory) {
      graph.acceleration = this.extractDataPoints(
        trajectory, 'timestampSec', 'speedAcceleration', false /* loop back */, -this.planningTime);
    }
  }

  updateThetaGraph(paths) {
    for (const path of paths) {
      const name = path.name === "planning_reference_line" ? "ReferenceLine" : path.name;
      this.data.thetaGraph[name] =
        this.extractDataPoints(path.pathPoint, 's', 'theta');
    }
  }

  updateKappaGraph(paths) {
    for (const path of paths) {
      const name = path.name === "planning_reference_line" ? "ReferenceLine" : path.name;
      this.data.kappaGraph[name] =
        this.extractDataPoints(path.pathPoint, 's', 'kappa');
    }
  }

  updateDkappaGraph(paths) {
    for (const path of paths) {
      const name = path.name === "planning_reference_line" ? "ReferenceLine" : path.name;
      this.data.dkappaGraph[name] =
        this.extractDataPoints(path.pathPoint, 's', 'dkappa');
    }
  }

  updateLatencyGraph(currentTime, latencyStates) {
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

  updateDpPolyGraph(dpPolyData) {
    const graph = this.data.dpPolyGraph;

    if (dpPolyData.sampleLayer) {
      graph.sampleLayer = [];
      for (const sample of dpPolyData.sampleLayer) {
        sample.slPoint.map(({s, l}) => {
          graph.sampleLayer.push({x:s, y:l});
        });
      }
    }

    if (dpPolyData.minCostPoint) {
      graph.minCostPoint = this.extractDataPoints(dpPolyData.minCostPoint, 's', 'l');
    }
  }

  update(world) {
    const planningData = world.planningData;
    if (planningData) {
      if (this.planningTime === world.planningTime) {
        return;
      }

      this.data = this.initData();

      if (planningData.slFrame && planningData.slFrame.length >= 2) {
        this.updateSLFrame(planningData.slFrame);
      }

      if (planningData.stGraph) {
        this.updateSTGraph(planningData.stGraph);
        this.updateSTSpeedGraph(planningData.stGraph);
      }

      if (planningData.speedPlan && world.planningTrajectory) {
        this.updateSpeed(planningData.speedPlan, world.planningTrajectory);
      }

      if (world.planningTrajectory) {
        this.updateAccelerationGraph(world.planningTrajectory);
      }

      if (planningData.path) {
        this.updateKappaGraph(planningData.path);
        this.updateDkappaGraph(planningData.path);
        this.updateThetaGraph(planningData.path);
      }

      if (planningData.dpPolyGraph) {
        this.updateDpPolyGraph(planningData.dpPolyGraph);
      }

      if (world.latency) {
        this.updateLatencyGraph(world.planningTime, world.latency);
      }

      this.updatePlanningTime(world.planningTime);
    }
  }
}
