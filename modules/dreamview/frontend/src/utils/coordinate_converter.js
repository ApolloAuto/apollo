import Proj4 from "proj4";

const WGS84_TEXT = "+proj=longlat +ellps=WGS84";
const UTM_TARGET =
"+proj=utm +zone=10 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ";

export function WGS84ToUTM(longitude, latitude) {
    return Proj4(WGS84_TEXT, UTM_TARGET, [longitude, latitude]);
}

export function UTMToWGS84(x, y) {
    return Proj4(UTM_TARGET, WGS84_TEXT, [x, y]);
}