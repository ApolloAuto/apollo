/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

function LoadGoogleMap(canvas_id, center_lat, center_lng, zoom) {
  var options = {
    center: new google.maps.LatLng(center_lat, center_lng),
    zoom: zoom,
    mapTypeId: google.maps.MapTypeId.ROADMAP
  };
  return new google.maps.Map(document.getElementById(canvas_id), options);
}

function DrawPolyline(map_obj, latlng_list, color, weight) {
  var coords = latlng_list.map(function(latlng) {
    return new google.maps.LatLng(latlng[0], latlng[1]);
  });

  new google.maps.Polyline({
    strokeColor: color,
    strokeWeight: weight,
    path: coords,
    clickable: false,
    map: map_obj
  });
}

function DrawCircle(map_obj, lat, lng, radius, color) {
  new google.maps.Circle({
    strokeColor: color,
    fillOpacity: 0,
    map: map_obj,
    center: new google.maps.LatLng(lat, lng),
    radius: radius
  });
}

function DrawInfoWindow(map_obj, lat, lng, info) {
  var infowindow = new google.maps.InfoWindow({content: info});
  var marker = new google.maps.Marker({
    position: { lat: lat, lng: lng },
    map: map_obj,
    title: 'Info'
  });
  marker.addListener('click', function() {infowindow.open(map_obj, marker);});
}
