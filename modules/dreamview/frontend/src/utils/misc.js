import polyval from 'compute-polynomial';
import _ from 'lodash';

export function copyProperty(toObj, fromObj) {
  for (const property in fromObj) {
    if (fromObj.hasOwnProperty(property)) {
      toObj[property] = fromObj[property];
    }
  }
}

export function hideArrayObjects(objects, startIdx = 0) {
  if (objects.constructor === Array && objects.length > 0) {
    let idx = startIdx;
    for (;idx < objects.length; idx += 1) {
      objects[idx].visible = false;
    }
  }
}

const MILLISECONDS_IN_A_SECOND = 1000;
const MILLISECONDS_IN_A_MINUTE = 1000 * 60;

export function millisecondsToTime(duration) {
  let milliseconds = Math.floor(duration % 1000);
  let seconds = Math.floor((duration / MILLISECONDS_IN_A_SECOND) % 60);
  let minutes = Math.floor(duration / MILLISECONDS_IN_A_MINUTE);

  minutes = (minutes < 10) ? `0${minutes}` : minutes;
  seconds = (seconds < 10) ? `0${seconds}` : seconds;
  if (milliseconds < 10) {
    milliseconds = `00${milliseconds}`;
  } else if (milliseconds < 100) {
    milliseconds = `0${milliseconds}`;
  }

  return `${minutes}:${seconds}.${milliseconds}`;
}

export function timestampMsToTimeString(timestampMs, showMilliseconds = false) {
  const date = new Date(timestampMs);
  let hours = date.getHours();
  let minutes = date.getMinutes();
  let seconds = date.getSeconds();
  hours = (hours < 10) ? `0${hours}` : hours;
  minutes = (minutes < 10) ? `0${minutes}` : minutes;
  seconds = (seconds < 10) ? `0${seconds}` : seconds;
  let timeString = `${hours}:${minutes}:${seconds}`;

  if (showMilliseconds) {
    let milliseconds = date.getMilliseconds();
    if (milliseconds < 10) {
      milliseconds = `00${milliseconds}`;
    } else if (milliseconds < 100) {
      milliseconds = `0${milliseconds}`;
    }
    timeString += `:${milliseconds}`;
  }

  return timeString;
}

export function calculateLaneMarkerPoints(autoDrivingCar, laneMarkerData) {
  if (!autoDrivingCar || !laneMarkerData) {
    return [];
  }

  const adcX = autoDrivingCar.positionX;
  const adcY = autoDrivingCar.positionY;
  const heading = autoDrivingCar.heading;

  const c0 = laneMarkerData.c0Position;
  const c1 = laneMarkerData.c1HeadingAngle;
  const c2 = laneMarkerData.c2Curvature;
  const c3 = laneMarkerData.c3CurvatureDerivative;
  const markerRange = laneMarkerData.viewRange;
  const markerCoef = [c3, c2, c1, c0];

  const points = [];
  for (let x = 0; x < markerRange; ++x) {
    const y = polyval(markerCoef, x);
    const newX = x * Math.cos(heading) - y * Math.sin(heading);
    const newY = y * Math.cos(heading) + x * Math.sin(heading);
    points.push({ x: adcX + newX, y: adcY + newY });
  }
  return points;
}

function GetCross(p1, p2, p) {
  return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

export function IsPointInRectangle(points, p) {
  const isPointIn = GetCross(points[0], points[1], p) * GetCross(points[2], points[3], p) >= 0
    && GetCross(points[1], points[2], p) * GetCross(points[3], points[0], p) >= 0;
  return isPointIn;
}

export function getPointDistance(p1, p2) {
  return Math.hypot(p1.x - p2.x, p1.y - p2.y);
}
