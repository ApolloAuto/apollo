import { coordinatesSame } from './common';
import { colorMapping } from '../constant/common';
import { drawSegmentsFromPoints } from './line';

export const drawPolygon = (points, polygonAttr) => {
    const {
        color = colorMapping.WHITE,
        linewidth = 1,
        zOffset = 0,
        opacity = 1,
        matrixAutoUpdate = true,
    } = polygonAttr;
    if (points.length < 3) {
        throw new Error('there are less than 3 points, the polygon cannot be drawn');
    }
    const length = points.length;
    if (!coordinatesSame(points[0], points[length - 1])) {
        points.push(points[0]);
    }
    return drawSegmentsFromPoints(points, {
        color,
        linewidth,
        zOffset,
        opacity,
        matrixAutoUpdate,
    });
};
