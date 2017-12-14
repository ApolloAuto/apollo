export function copyProperty(toObj, fromObj) {
    for (const property in fromObj) {
        if (fromObj.hasOwnProperty(property)) {
            toObj[property] = fromObj[property];
        }
    }
}

export function hideArrayObjects(objects, startIdx = 0) {
    if (objects.constructor === Array && objects.length > 0) {
        for (;startIdx < objects.length; startIdx++) {
            objects[startIdx].visible = false;
        }
    }
}


const MILLISECONDS_IN_A_SECOND = 1000;
const MILLISECONDS_IN_A_MINUTE = 1000 * 60;
const MILLISECONDS_IN_AN_HOUR = 1000 * 60 * 60;

export function millisecondsToTime(duration) {
    let milliseconds = Math.floor(duration % 1000);
    let seconds = Math.floor((duration / MILLISECONDS_IN_A_SECOND) % 60);
    let minutes = Math.floor((duration / MILLISECONDS_IN_A_MINUTE) % 60);
    let hours = Math.floor((duration / MILLISECONDS_IN_AN_HOUR) % 24);

    hours = (hours < 10) ? "0" + hours : hours;
    minutes = (minutes < 10) ? "0" + minutes : minutes;
    seconds = (seconds < 10) ? "0" + seconds : seconds;
    if (milliseconds < 10) {
        milliseconds = "00" + milliseconds;
    } else if (milliseconds < 100) {
        milliseconds = "0" + milliseconds;
    }

    return hours + ":" + minutes + ":" + seconds + "." + milliseconds;
}