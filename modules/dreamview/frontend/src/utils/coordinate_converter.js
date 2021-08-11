import Proj4 from 'proj4';
import * as ChineseCoordinateConverter from 'chncrs';

import STORE from 'store';

const WGS84_TEXT = '+proj=longlat +ellps=WGS84';
const UTM_TARGET_TEMPLATE = (utmZoneId) =>
  `+proj=utm +zone=${utmZoneId} +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs`;

export function WGS84ToUTM(longitude, latitude) {
  const UTM_TARGET = UTM_TARGET_TEMPLATE(STORE.hmi.utmZoneId);
  return Proj4(WGS84_TEXT, UTM_TARGET, [longitude, latitude]);
}

export function UTMToWGS84(x, y) {
  const UTM_TARGET = UTM_TARGET_TEMPLATE(STORE.hmi.utmZoneId);
  return Proj4(UTM_TARGET, WGS84_TEXT, [x, y]);
}

export function WGS84ToGCJ02(longitude, latitude) {
  return ChineseCoordinateConverter.transform([longitude, latitude], 'WGS84', 'GCJ02');
}

export function WGS84ToBD09LL(longitude, latitude) {
  if (outOfChina(longitude, latitude)) {
    return [longitude, latitude];
  }
  const GCJ02 = WGS84ToGCJ02(longitude, latitude);
  return ChineseCoordinateConverter.transform(GCJ02, 'GCJ02', 'BD09LL');
}

function outOfChina(longitude, latitude) {
  if (longitude < 72.004 || longitude > 137.8347) {
    return true;
  }
  if (latitude < 0.8293 || latitude > 55.8271) {
    return true;
  }
  return false;
}
