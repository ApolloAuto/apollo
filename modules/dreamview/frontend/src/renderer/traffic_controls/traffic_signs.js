import * as THREE from "three";
import _ from "lodash";

import TrafficControlsBase from 'renderer/traffic_controls/traffic_controls_base';

export default class TrafficSigns extends TrafficControlsBase {
    getPositionAndHeading(sign, coordinates) {
        const heading = TrafficControlsBase.getHeadingFromStopLine(sign);

        if (!isNaN(heading)) {
            const stopLinePoint = _.last(sign.stopLine[0].segment[0].lineSegment.point);
            let position = new THREE.Vector3(stopLinePoint.x, stopLinePoint.y, 0);
            position = coordinates.applyOffset(position);

            return { "pos": position, "heading": heading - Math.PI / 2 };
        } else {
            console.error('Error loading traffic sign. Unable to determine heading.');
            return null;
        }
    }
}
