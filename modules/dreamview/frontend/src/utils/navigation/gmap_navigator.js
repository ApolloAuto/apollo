import polyval from "compute-polynomial";
import protobuf from "protobufjs/light";
import { UTMToWGS84 } from "utils/coordinate_converter";

const root = protobuf.Root.fromJSON(
  require("proto_bundle/map_proto_bundle.json")
);
const navInfoMessage = root.lookupType("apollo.relative_map.NavigationInfo");

class GmapNavigator {
  constructor() {
    this.WS = null;
    this.vehicleMarker = null;
    this.map = null;
    this.routingPath = [];
    this.rightLaneMarker = false;
    this.leftLaneMarker = false;
    this.plannedPath = null;
    this.centerVehicle = true;
    this.destinationMarker = false;
  }

  initialize(WS) {
    this.WS = WS;
    this.loadGoogleMap();

    const centerControlDiv = document.createElement("div");
    this.createCenterControl(centerControlDiv);
    centerControlDiv.index = 1;
    this.map.controls[google.maps.ControlPosition.TOP_CENTER].push(
      centerControlDiv
    );

    const centerControlDiv2 = document.createElement("div");
    this.createRoutingControl(centerControlDiv2);
    centerControlDiv2.index = 1;
    this.map.controls[google.maps.ControlPosition.TOP_CENTER].push(
      centerControlDiv2
    );

    const centerControlDiv3 = document.createElement("div");
    this.createCanadaRoutingWestControl(centerControlDiv3);
    centerControlDiv3.index = 1;
    this.map.controls[google.maps.ControlPosition.TOP_CENTER].push(
      centerControlDiv3
    );

    const centerControlDiv4 = document.createElement("div");
    this.createCanadaRoutingEastControl(centerControlDiv4);
    centerControlDiv4.index = 1;
    this.map.controls[google.maps.ControlPosition.TOP_CENTER].push(
      centerControlDiv4
    );

    google.maps.event.addListener(this.map, "click", event => {
      const clickedLocation = event.latLng;
      if (this.destinationMarker === false) {
        this.destinationMarker = new google.maps.Marker({
          position: clickedLocation,
          map: this.map,
          label: "D",
          draggable: true //make it draggable
        });
        //Listen for drag events!
        google.maps.event.addListener(this.vehicleMarker, "dragend", () => {
          markerLocation();
        });
      } else {
        //Marker has already been added, so just change its location.
        this.destinationMarker.setPosition(clickedLocation);
      }
    });
  }

  loadGoogleMap() {
    const latlng = new google.maps.LatLng(37.415885, -122.014487);
    const mapOptions = {
      center: latlng,
      zoom: 20,
      mapTypeId: google.maps.MapTypeId.ROADMAP
    };
    this.map = new google.maps.Map(
      document.getElementById("map_canvas"),
      mapOptions
    );
    this.vehicleMarker = new google.maps.Marker({
      position: latlng
    });
    this.vehicleMarker.setMap(this.map);
  }

  update(data) {
    const adc = data.autoDrivingCar;
    if (adc) {
      this.updateCenterVehicle(adc);
      this.updateLaneMarkers(adc, data.laneMarker);
      this.updatePlanningPath(adc, data.planningTrajectory);
    }
  }

  updateCenterVehicle(autoDrivingCar) {
    if (!autoDrivingCar) {
      return;
    }

    const x = autoDrivingCar.positionX;
    const y = autoDrivingCar.positionY;
    const heading = autoDrivingCar.heading;

    const [longitude, latitude] = UTMToWGS84(x, y);
    const latLng = new google.maps.LatLng(latitude, longitude);
    if (this.centerVehicle) {
      this.map.setCenter(latLng);
    }
    this.vehicleMarker.setPosition(latLng);
  }

  calculateLaneMarkerPath(autoDrivingCar, laneMarkerData) {
    if (!autoDrivingCar || !laneMarkerData) {
      return;
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

    const lane = [];
    for (let x = 0; x < markerRange; ++x) {
      const y = -1 * polyval(markerCoef, x);
      const newX = x * Math.cos(heading) - y * Math.sin(heading);
      const newY = y * Math.cos(heading) + x * Math.sin(heading);
      const [plon, plat] = UTMToWGS84(adcX + newX, adcY + newY);
      lane.push({ lat: plat, lng: plon });
    }
    return lane;
  }

  updateLaneMarkers(autoDrivingCar, laneMarker) {
    if (!autoDrivingCar || !laneMarker) {
      return;
    }

    const rightLane = this.calculateLaneMarkerPath(
      autoDrivingCar,
      laneMarker.rightLaneMarker
    );
    if (this.rightLaneMarker !== false) {
      this.rightLaneMarker.setMap(null);
    }
    this.rightLaneMarker = new google.maps.Polyline({
      path: rightLane,
      geodesic: true,
      strokeColor: "#0000FF",
      strokeOpacity: 1.0,
      strokeWeight: 2,
      map: this.map
    });

    const leftLane = this.calculateLaneMarkerPath(
      autoDrivingCar,
      laneMarker.leftLaneMarker
    );
    if (this.leftLaneMarker !== false) {
      this.leftLaneMarker.setMap(null);
    }
    this.leftLaneMarker = new google.maps.Polyline({
      path: leftLane,
      geodesic: true,
      strokeColor: "#0000FF",
      strokeOpacity: 1.0,
      strokeWeight: 2,
      map: this.map
    });
  }

  updatePlanningPath(autoDrivingCar, trajectory) {
    if (!autoDrivingCar || !trajectory) {
      return;
    }

    const adcX = autoDrivingCar.positionX;
    const adcY = autoDrivingCar.positionY;
    const heading = autoDrivingCar.heading;

    const path = trajectory.map(point => {
      const x = point.positionX;
      const y = point.positionY;
      const newX = x * Math.cos(heading) - y * Math.sin(heading);
      const newY = y * Math.cos(heading) + x * Math.sin(heading);
      const [plon, plat] = UTMToWGS84(adcX + newX, adcY + newY);
      return { lat: plat, lng: plon };
    });

    if (this.plannedPath) {
      this.plannedPath.setMap(null);
    }
    this.plannedPath = new google.maps.Polyline({
      path: path,
      geodesic: true,
      strokeColor: "#00FF00",
      strokeOpacity: 1.0,
      strokeWeight: 2
      // map: this.map,
    });
    this.plannedPath.setMap(this.map);
  }

  createCenterControl(controlDiv) {
    const map = this.map;

    // Set CSS for the control border.
    const controlUI = document.createElement("div");
    controlUI.style.backgroundColor = "#fff";
    controlUI.style.border = "2px solid #fff";
    controlUI.style.borderRadius = "3px";
    controlUI.style.boxShadow = "0 2px 6px rgba(0,0,0,.3)";
    controlUI.style.cursor = "pointer";
    controlUI.style.marginBottom = "22px";
    controlUI.style.textAlign = "center";
    controlUI.title = "Click to recenter the vehicle";
    controlDiv.appendChild(controlUI);

    // Set CSS for the control interior.
    const controlText = document.createElement("div");
    controlText.style.color = "rgb(25,25,25)";
    controlText.style.fontFamily = "Roboto,Arial,sans-serif";
    controlText.style.fontSize = "16px";
    controlText.style.lineHeight = "38px";
    controlText.style.paddingLeft = "5px";
    controlText.style.paddingRight = "5px";
    controlText.innerHTML = "Center Vehicle is ON";
    controlUI.appendChild(controlText);

    // Setup the click event listeners: simply set the map to Chicago.
    controlUI.addEventListener("click", () => {
      if (this.centerVehicle) {
        this.centerVehicle = false;
        controlText.innerHTML = "Center Vehicle is OFF";
        map.setZoom(15);
      } else {
        this.centerVehicle = true;
        controlText.innerHTML = "Center Vehicle is ON";
        map.setZoom(20);
      }
    });
  }

  requestRouting(startLat, startLng, endLat, endLng) {
    if (!startLat || !startLng || !endLat || !endLng) {
      return;
    }

    const url =
      "http://navi-env.axty8vi3ic.us-west-2.elasticbeanstalk.com" +
      "/dreamview/navigation" +
      `?origin=${startLat},${startLng}` +
      `&destination=${endLat},${endLng}` +
      "&heading=0";
    fetch(url, {
      method: "GET",
      mode: "cors"
    })
      .then(response => {
        return response.arrayBuffer();
      })
      .then(response => {
        if (!response.byteLength) {
          console.warn("No navigation info received.");
          return;
        }

        this.WS.publishNavigationInfo(response);

        const navigationInfo = navInfoMessage.toObject(
          navInfoMessage.decode(new Uint8Array(response)),
          { enums: String }
        );

        const paths = navigationInfo.navigationPath.map(naviPath => {
          return naviPath.path.pathPoint.map(point => {
            const [lon, lat] = UTMToWGS84(point.x, point.y);
            return { lat: lat, lng: lon };
          });
        });

        if (this.routingPath) {
          this.routingPath.forEach(path => {
            path.setMap(null);
          });
          this.routingPath = [];
        }
        paths.forEach(path => {
          this.routingPath.push(
            new google.maps.Polyline({
              path: paths[0],
              geodesic: true,
              strokeColor: "#cd5c5c",
              strokeOpacity: 0.8,
              strokeWeight: 6,
              map: this.map
            })
          );
        });
      })
      .catch(error => {
        console.error("Failed to retrieve navigation data:", error);
      });
  }

  createControlElement(parentDiv, backgroundColor, text) {
    // Set CSS for the control border.
    const controlUI = document.createElement("div");
    controlUI.style.backgroundColor = backgroundColor;
    controlUI.style.border = "2px solid #fff";
    controlUI.style.borderRadius = "3px";
    controlUI.style.boxShadow = "0 2px 6px rgba(0,0,0,.3)";
    controlUI.style.cursor = "pointer";
    controlUI.style.marginBottom = "22px";
    controlUI.style.textAlign = "center";
    controlUI.title = "Click to send routing request";
    parentDiv.appendChild(controlUI);

    // Set CSS for the control interior.
    const controlText = document.createElement("div");
    controlText.style.color = "rgb(25,25,25)";
    controlText.style.fontFamily = "Roboto,Arial,sans-serif";
    controlText.style.fontSize = "16px";
    controlText.style.lineHeight = "38px";
    controlText.style.paddingLeft = "5px";
    controlText.style.paddingRight = "5px";
    controlText.innerHTML = text;
    controlUI.appendChild(controlText);

    return controlUI;
  }

  createRoutingControl(controlDiv) {
    const controlUI = this.createControlElement(
      controlDiv,
      "#cd5c5c",
      "routing request"
    );

    controlUI.addEventListener("click", () => {
      if (!this.destinationMarker) {
        alert("please select a destination point.");
        return;
      }

      const start = this.vehicleMarker.getPosition();
      const end = this.destinationMarker.getPosition();
      this.requestRouting(start.lat(), start.lng(), end.lat(), end.lng());
    });
  }

  createCanadaRoutingWestControl(controlDiv) {
    const controlUI = this.createControlElement(
      controlDiv,
      "#ff8c00",
      "TO Cananda West"
    );

    controlUI.addEventListener("click", () => {
      const start = this.vehicleMarker.getPosition();
      const endLat = 37.506039;
      const endLng = -122.340299;
      this.requestRouting(start.lat(), start.lng(), endLat, endLng);
    });
  }

  createCanadaRoutingEastControl(controlDiv) {
    const controlUI = this.createControlElement(
      controlDiv,
      "#00bfff",
      "TO Cananda East"
    );

    controlUI.addEventListener("click", () => {
      const start = this.vehicleMarker.getPosition();
      const endLat = 37.464198;
      const endLng = -122.298453;
      this.requestRouting(start.lat(), start.lng(), endLat, endLng);
    });
  }

  markerLocation() {
    const currentLocation = this.destinationMarker.getPosition();
    //Add lat and lng values to a field that we can save.
    document.getElementById("lat").value = currentLocation.lat(); //latitude
    document.getElementById("lng").value = currentLocation.lng(); //longitude
  }
}

const GMAP_NAVIGATOR = new GmapNavigator();
export default GMAP_NAVIGATOR;
