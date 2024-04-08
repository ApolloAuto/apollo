import * as THREE from 'three';
import { meanBy } from 'lodash';

const EPSILON = 1e-9;

export const getHeadingFromStopLine = (stopLine) => {
    if (!stopLine) {
        return NaN;
    }
    const points = stopLine.segment[0]?.lineSegment?.point;
    if (points && points.length >= 2) {
        const length = points.length;
        const stopLineDirection = Math.atan2(points[length - 1].y - points[0].y, points[length - 1].x - points[0].x);
        return Math.PI * 1.5 + stopLineDirection;
    }
    return NaN;
};
export const getHeadingFromStopLineAndTrafficLightBoundary = (signal) => {
    const boundaryPoints = signal.boundary.point;
    if (boundaryPoints.length < 3) {
        console.warn(`cannot get three points from boundary,signal_id:${signal.id.id}`);
        if (signal.stopLine[0]) {
            return getHeadingFromStopLine(signal.stopLine[0]);
        }
        return NaN;
    }
    const boundary1 = boundaryPoints[0];
    const boundary2 = boundaryPoints[1];
    const boundary3 = boundaryPoints[2];
    const orthogonalX =
        (boundary2.x - boundary1.x) * (boundary3.z - boundary1.z) -
        (boundary3.x - boundary1.x) * (boundary2.z - boundary1.z);
    const orthogonalY =
        (boundary2.y - boundary1.y) * (boundary3.z - boundary1.z) -
        (boundary3.y - boundary1.y) * (boundary2.z - boundary1.z);
    const orthogonalConstant = -orthogonalX * boundary1.x - orthogonalY * boundary1.y;

    const stopLinePoints = signal.stopLine[0]?.segment[0]?.lineSegment?.point;
    const len = stopLinePoints.length;
    if (len < 2) {
        console.warn(`Cannot get any stop line, signal_id: ${signal.id.id}`);
        return NaN;
    }
    // construct ax+by+c=0 ==> stopLineX*x+stopLineY*y+constant=0
    const stopLineX = stopLinePoints[len - 1].y - stopLinePoints[0].y;
    const stopLineY = stopLinePoints[0].x - stopLinePoints[len - 1].x;
    const stopLineConstant = -stopLineX * stopLinePoints[0].x - stopLineY * stopLinePoints[0].y;

    // calculate the intersection
    if (Math.abs(stopLineX * orthogonalY - orthogonalX * stopLineY) < EPSILON) {
        console.warn('The signal orthogonal direction is parallel to the stop line,', `signal_id: ${signal.id.id}`);
        return getHeadingFromStopLine(signal.stopLine[0]);
    }
    const intersectX =
        (stopLineY * orthogonalConstant - orthogonalY * stopLineConstant) /
        (stopLineX * orthogonalY - orthogonalX * stopLineY);
    const intersectY =
        stopLineY !== 0
            ? (-stopLineX * intersectX - stopLineConstant) / stopLineY
            : (-orthogonalX * intersectX - orthogonalConstant) / orthogonalY;
    let direction = Math.atan2(-orthogonalX, orthogonalY);

    // if the direction is not towards to intersection point, turn around
    if ((direction < 0 && intersectY > boundary1.y) || (direction > 0 && intersectY < boundary1.y)) {
        direction += Math.PI;
    }
    return direction;
};

export const getPositionAndHeading = (signal) => {
    const locations = [];
    if (signal.position && signal.heading) {
        return {
            position: signal.position,
            heading: signal.heading,
        };
    }
    if (!signal.subsignal || signal.subsignal.length === 0) {
        return {};
    }

    signal.subsignal.forEach((subsignal) => {
        if (subsignal.location) {
            locations.push(subsignal.location);
        }
    });

    if (locations.length === 0) {
        if (signal.boundary?.point?.length) {
            console.warn('subsignal locations not found,use signal bounday instead.');
            locations.push(...signal.boundary.point);
        } else {
            console.warn('unable to determine signal location,skip.');
            return {};
        }
    }

    const heading = getHeadingFromStopLineAndTrafficLightBoundary(signal);
    if (!Number.isNaN(heading)) {
        const position = new THREE.Vector3(0, 0, 0);
        position.x = meanBy(locations, (l) => l.x);
        position.y = meanBy(locations, (l) => l.y);
        return { position, heading };
    }
    return {};
};
