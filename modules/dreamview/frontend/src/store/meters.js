import {
  observable, computed, action, runInAction,
} from 'mobx';

function roundToTens(percent) {
  return Math.round(percent / 10.0) * 10;
}

function toDrivingMode(disengageType) {
  switch (disengageType) {
    case 'DISENGAGE_MANUAL':
      return 'MANUAL';
    case 'DISENGAGE_NONE':
      return 'AUTO';
    case 'DISENGAGE_EMERGENCY':
      return 'DISENGAGED';
    case 'DISENGAGE_AUTO_STEER_ONLY':
      return 'AUTO STEER';
    case 'DISENGAGE_AUTO_SPEED_ONLY':
      return 'AUTO SPEED';
    case 'DISENGAGE_CHASSIS_ERROR':
      return 'CHASSIS ERROR';
    default:
      return 'UNKNOWN';
  }
}

function isAutoMode(disengageType) {
  return disengageType === 'DISENGAGE_NONE'
           || disengageType === 'DISENGAGE_AUTO_STEER_ONLY'
           || disengageType === 'DISENGAGE_AUTO_SPEED_ONLY';
}

function meterPerSecondToKmPerHour(speed) {
  return Math.round(speed * 3600 / 1000.0);
}

export default class Meters {
    @observable throttlePercent = 0;

    @observable brakePercent = 0;

    @observable speed = 0;

    @observable steeringAngle = 0;

    @observable steeringPercentage = 0;

    @observable batteryPercentage = 0;

    @observable drivingMode = 'UNKNOWN';

    @observable isAutoMode = false;

    @observable turnSignal = '';

    @action update(world) {
      if (world.autoDrivingCar) {
        if (world.autoDrivingCar.throttlePercentage !== undefined) {
          this.throttlePercent = roundToTens(world.autoDrivingCar.throttlePercentage);
        }
        if (world.autoDrivingCar.brakePercentage !== undefined) {
          this.brakePercent = roundToTens(world.autoDrivingCar.brakePercentage);
        }

        if (world.autoDrivingCar.speed !== undefined) {
          // Convert the unit from m/s to mph.
          this.speed = world.autoDrivingCar.speed;
        }

        if (world.autoDrivingCar.batteryPercentage !== undefined) {
            this.batteryPercentage = world.autoDrivingCar.batteryPercentage;
        }

        if (world.autoDrivingCar.steeringPercentage !== undefined
                && !isNaN(world.autoDrivingCar.steeringPercentage)) {
          this.steeringPercentage = Math.round(world.autoDrivingCar.steeringPercentage);
        }

        if (world.autoDrivingCar.steeringAngle !== undefined
                && !isNaN(world.autoDrivingCar.steeringAngle)) {
          this.steeringAngle = -Math.round(
            world.autoDrivingCar.steeringAngle * 180.0 / Math.PI,
          );
        }

        if (world.autoDrivingCar.disengageType !== undefined) {
          this.drivingMode = toDrivingMode(world.autoDrivingCar.disengageType);
          this.isAutoMode = isAutoMode(world.autoDrivingCar.disengageType);
        }

        if (world.autoDrivingCar.currentSignal !== undefined) {
          this.turnSignal = world.autoDrivingCar.currentSignal;
        }
      }
    }
}
