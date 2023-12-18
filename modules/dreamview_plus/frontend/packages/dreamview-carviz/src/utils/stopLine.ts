import { drawSegmentsFromPoints } from './line';
import { colorMapping, zOffset } from '../constant/common';

export const drawStopLine = (stopLine, coordinates) => {
    if (!stopLine) {
        return [];
    }
    const meshs = [];
    stopLine.segment?.forEach((segment) => {
        const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
        const mesh = drawSegmentsFromPoints(points, {
            color: colorMapping.PURE_WHITE,
            linewidth: 5,
            zOffset: zOffset.stopLine,
            opacity: 1,
        });
        meshs.push(mesh);
    });
    return meshs;
};
