import { WGS84ToGCJ02 } from 'utils/coordinate_converter';

export default class GoogleMapAdapter {
  constructor() {
    this.map = null;
    this.controls = [];
  }

  isInitialized() {
    return this.map !== null;
  }

  loadMap(initPoint, divElementName) {
    const mapOptions = {
      center: initPoint,
      zoom: 20,
      mapTypeId: google.maps.MapTypeId.ROADMAP,
      fullscreenControl: false,
    };
    this.map = new google.maps.Map(document.getElementById(divElementName), mapOptions);
  }

  setCenter(point) {
    this.map.setCenter(point);
  }

  setZoom(zoom) {
    this.map.setZoom(zoom);
  }

  addEventHandler(eventName, handler) {
    google.maps.event.addListener(this.map, eventName, (event) => {
      const clickedLatLng = event.latLng;
      handler(clickedLatLng);
    });
  }

  createPoint({ lat, lng }) {
    return new google.maps.LatLng(lat, lng);
  }

  createMarker(point, title, draggable = true) {
    const marker = new google.maps.Marker({
      position: point,
      label: title,
      draggable,
      map: this.map,
    });
    return marker;
  }

  createPolyline(path, color, opacity = 1.0, weight = 2.0) {
    const polyline = new google.maps.Polyline({
      path,
      geodesic: true,
      strokeColor: color,
      strokeOpacity: opacity,
      strokeWeight: weight,
      map: this.map,
    });
    return polyline;
  }

  createControl({
    text, tip, color, offsetX, offsetY, onClickHandler,
  }) {
    const controlDiv = document.createElement('div');

    // Set CSS for the control border.
    const controlUI = document.createElement('div');
    controlUI.style.backgroundColor = color;
    controlUI.style.border = '2px solid #fff';
    controlUI.style.borderRadius = '3px';
    controlUI.style.boxShadow = '0 2px 6px rgba(0,0,0,.3)';
    controlUI.style.cursor = 'pointer';
    controlUI.style.marginBottom = '22px';
    controlUI.style.textAlign = 'center';
    controlUI.title = tip;
    controlDiv.appendChild(controlUI);

    // Set CSS for the control interior.
    const controlText = document.createElement('div');
    controlText.style.color = 'rgb(25,25,25)';
    controlText.style.fontFamily = 'Roboto,Arial,sans-serif';
    controlText.style.fontSize = '16px';
    controlText.style.lineHeight = '38px';
    controlText.style.paddingLeft = '5px';
    controlText.style.paddingRight = '5px';
    controlText.innerHTML = text;
    controlUI.appendChild(controlText);

    // Setup the click event listeners: simply set the map to Chicago.
    controlUI.addEventListener('click', () => {
      onClickHandler(controlText);
    });

    this.map.controls[google.maps.ControlPosition.TOP_LEFT].push(controlDiv);
    this.controls.push(controlDiv);
  }

  disableControls() {
    this.controls.forEach((control) => {
      control.style.display = 'none';
    });
  }

  enableControls() {
    this.controls.forEach((control) => {
      control.style.display = 'block';
    });
  }

  getMarkerPosition(marker) {
    const position = marker.getPosition();
    return { lat: position.lat(), lng: position.lng() };
  }

  updatePolyline(polyline, newPath) {
    polyline.setPath(newPath);
  }

  removePolyline(polyline) {
    polyline.setMap(null);
  }

  applyCoordinateOffset([lng, lat]) {
    return WGS84ToGCJ02(lng, lat);
  }
}
