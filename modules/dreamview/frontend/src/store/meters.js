import { observable, computed, action, runInAction } from "mobx";

function roundToTens(percent) {
    return Math.round(percent / 10.0) * 10;
}

function toDrivingMode(disengageType) {
    if (disengageType === "DISENGAGE_MANUAL") {
        return "MANUAL";
    } else if (disengageType === "DISENGAGE_NONE") {
        return "AUTO";
    }
    return "?";
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

    @action update(world) {
        if (world.autoDrivingCar) {
            this.throttlePercent = roundToTens(
                world.autoDrivingCar.throttlePercentage);
            this.brakePercent = roundToTens(
                world.autoDrivingCar.brakePercentage);
            // Convert the unit from m/s to mph.
            this.speed = meterPerSecondToKmPerHour(world.autoDrivingCar.speed);
            this.steeringAngle =
                -Math.round(world.autoDrivingCar.steeringAngle / 100 * 470);
            this.drivingMode =
                toDrivingMode(world.autoDrivingCar.disengageType);
            /* TODO Turn signal is not working. Please check and make sure
             * that it is set correctly in the backend.*/
        }
    }
}
