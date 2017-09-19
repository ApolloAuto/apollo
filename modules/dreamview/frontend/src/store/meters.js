import { observable, computed, action, runInAction } from "mobx";

function roundToTens(percent) {
    return Math.round(percent / 10.0) * 10;
}

function toDrivingMode(disengageType) {
    switch (disengageType) {
        case "DISENGAGE_MANUAL":
            return "MANUAL";
        case "DISENGAGE_NONE":
            return "AUTO";
        case "DISENGAGE_EMERGENCY":
            return "DISENGAGED";
        case "DISENGAGE_AUTO_STEER_ONLY":
            return "AUTO STEER ONLY";
        case "DISENGAGE_AUTO_SPEED_ONLY":
            return "AUTO SPEED ONLY";
        case "DISENGAGE_CHASSIS_ERROR":
            return "CHASSIS ERROR";
        default:
            return "?";
    }

}

function meterPerSecondToKmPerHour(speed) {
    return Math.round(speed * 3600 / 1000.0);
}

export default class Meters {
    @observable throttlePercent = 0;
    @observable brakePercent = 0;
    @observable speed = 0;
    @observable steeringAngle = 0;
    @observable drivingMode = "?";
    @observable turnSignal = "";

    @action update(world) {
        if (world.autoDrivingCar) {
            if (world.autoDrivingCar.throttlePercentage) {
                this.throttlePercent = roundToTens(
                    world.autoDrivingCar.throttlePercentage);
            }
            if (world.autoDrivingCar.brakePercentage) {
                this.brakePercent = roundToTens(
                    world.autoDrivingCar.brakePercentage);
            }

            if (world.autoDrivingCar.speed) {
                // Convert the unit from m/s to mph.
                this.speed = meterPerSecondToKmPerHour(
                    world.autoDrivingCar.speed);
            }

            if (world.autoDrivingCar.steeringAngle) {
                // TODO(siyangy): Avoid magic number here.
                this.steeringAngle =
                    -Math.round(
                        world.autoDrivingCar.steeringAngle * 4.7);
            }

            if (world.autoDrivingCar.disengageType) {
                this.drivingMode =
                    toDrivingMode(world.autoDrivingCar.disengageType);
            }

            if (world.autoDrivingCar.currentSignal) {
                this.turnSignal = world.autoDrivingCar.currentSignal;
            }
        }
    }
}
