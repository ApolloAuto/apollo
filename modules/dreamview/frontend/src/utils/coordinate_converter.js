import Proj4 from "proj4";
import STORE from "store";

const WGS84_TEXT = "+proj=longlat +ellps=WGS84";
const UTM_TARGET_TEMPLATE = (utmZoneId) =>
`+proj=utm +zone=${utmZoneId} +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs `;

export function WGS84ToUTM(latitude, longitude) {
    const UTM_TARGET = UTM_TARGET_TEMPLATE(STORE.hmi.utmZoneId);
    return Proj4(WGS84_TEXT, UTM_TARGET, [longitude, latitude]);
}

export function UTMToWGS84(x, y, utmZoneId) {
    const UTM_TARGET = UTM_TARGET_TEMPLATE(STORE.hmi.utmZoneId);
    return Proj4(UTM_TARGET, WGS84_TEXT, [x, y]);
}