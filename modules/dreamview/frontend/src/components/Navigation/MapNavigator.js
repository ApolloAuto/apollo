import { UTMToWGS84 } from "utils/coordinate_converter";
import { calculateLaneMarkerPoints } from "utils/misc";

class MapNavigator {
    constructor() {
        this.mapAPILoaded = false;
        this.WS = null;
        this.mapAdapter = null;

        this.routingPaths = [];
        this.plannedPath = null;
        this.vehicleMarker = null;
        this.rightLaneMarker = null;
        this.leftLaneMarker = null;
        this.destinationMarker = null;
        this.centerVehicle = true;

        this.routingRequestPoints = [];
    }

    initialize(WS, mapAdapter) {
        this.WS = WS;
        this.mapAdapter = mapAdapter;

        const initLatlng = { lng: -122.014487, lat: 37.415885 };
        const point = this.mapAdapter.createPoint(initLatlng);
        const divElementName = "map_canvas";
        this.mapAdapter.loadMap(point, divElementName);
        this.vehicleMarker = this.mapAdapter.createMarker(point, null, false);
        this.createControls();

        this.mapAdapter.addEventHandler("click", clickedLatLng => {
            if (!this.destinationMarker) {
                this.destinationMarker = this.mapAdapter.createMarker(clickedLatLng, "D");
            } else {
                this.destinationMarker.setPosition(clickedLatLng);
            }
        });
    }

    isInitialized() {
        return this.WS && this.mapAdapter && this.mapAdapter.isInitialized();
    }

    reset() {
        this.routingPaths.forEach(path => {
            this.mapAdapter.removePolyline(path);
        });
        this.routingPaths = [];

        if (this.plannedPath) {
            this.mapAdapter.removePolyline(this.plannedPath);
            this.plannedPath = null;
        }

        if (this.rightLaneMarker) {
            this.mapAdapter.removePolyline(this.rightLaneMarker);
            this.rightLaneMarker = null;
        }

        if (this.leftLaneMarker) {
            this.mapAdapter.removePolyline(this.leftLaneMarker);
            this.leftLaneMarker = null;
        }

        this.WS = null;
        this.mapAdapter = null;
        this.vehicleMarker = null;
        this.destinationMarker = null;
        this.centerVehicle = true;
    }

    createControls() {
        this.mapAdapter.createControl({
            text: "CarView ON",
            tip: "Click to recenter the vehicle",
            color: "#FFFFFF",
            offsetX: 130,
            offsetY: 0,
            onClickHandler: textElementDiv => {
                if (this.centerVehicle) {
                    this.centerVehicle = false;
                    textElementDiv.innerHTML = "CarView OFF";
                    this.mapAdapter.setZoom(18);
                } else {
                    this.centerVehicle = true;
                    textElementDiv.innerHTML = "CarView ON";
                    this.mapAdapter.setZoom(20);
                }
            },
        });

        this.mapAdapter.createControl({
            text: "Route",
            tip: "Click to send routing request",
            color: "#CD5C5C",
            offsetX: 235,
            offsetY: 0,
            onClickHandler: textElementDiv => {
                if (!this.destinationMarker) {
                    alert("please select a destination point.");
                    return;
                }

                const start = this.mapAdapter.getMarkerPosition(this.vehicleMarker);
                const end = this.mapAdapter.getMarkerPosition(this.destinationMarker);
                this.requestRoute(start.lat, start.lng, end.lat, end.lng);
            },
        });
    }

    disableControls() {
        this.mapAdapter.disableControls();
    }

    enableControls() {
        this.mapAdapter.enableControls();
    }

    update(data) {
        if (!this.isInitialized()) {
            return;
        }

        const adc = data.autoDrivingCar;
        this.updateCenterVehicle(adc);
        this.updateNavigationPath(data.navigationPath);
        this.updateLaneMarkers(adc, data.laneMarker);
        this.updatePlanningPath(adc, data.planningTrajectory);
    }

    updateCenterVehicle(autoDrivingCar) {
        if (!autoDrivingCar) {
            return;
        }

        const x = autoDrivingCar.positionX;
        const y = autoDrivingCar.positionY;

        const [longitude, latitude] = this.mapAdapter.applyCoordinateOffset(UTMToWGS84(x, y));
        const latLng = this.mapAdapter.createPoint({
            lat: latitude,
            lng: longitude,
        });
        if (this.centerVehicle) {
            this.mapAdapter.setCenter(latLng);
        }
        this.vehicleMarker.setPosition(latLng);
    }

    calculateLaneMarkerPath(autoDrivingCar, laneMarkerData) {
        const path = calculateLaneMarkerPoints(autoDrivingCar, laneMarkerData);
        return path.map(point => {
            const [plon, plat] = this.mapAdapter.applyCoordinateOffset(
                UTMToWGS84(point.x, point.y)
            );
            return this.mapAdapter.createPoint({ lat: plat, lng: plon });
        });
    }

    updateLaneMarkers(autoDrivingCar, laneMarker) {
        if (!autoDrivingCar || !laneMarker) {
            return;
        }

        const rightLane = this.calculateLaneMarkerPath(autoDrivingCar, laneMarker.rightLaneMarker);
        if (!this.rightLaneMarker) {
            this.rightLaneMarker = this.mapAdapter.createPolyline(rightLane, "#0000FF");
        } else {
            this.mapAdapter.updatePolyline(this.rightLaneMarker, rightLane);
        }

        const leftLane = this.calculateLaneMarkerPath(autoDrivingCar, laneMarker.leftLaneMarker);
        if (!this.leftLaneMarker) {
            this.leftLaneMarker = this.mapAdapter.createPolyline(leftLane, "#0000FF");
        } else {
            this.mapAdapter.updatePolyline(this.leftLaneMarker, leftLane);
        }
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
            const [plon, plat] = this.mapAdapter.applyCoordinateOffset(
                UTMToWGS84(adcX + newX, adcY + newY)
            );
            return this.mapAdapter.createPoint({ lat: plat, lng: plon });
        });

        if (!this.plannedPath) {
            this.plannedPath = this.mapAdapter.createPolyline(path, "#00FF00");
        } else {
            this.mapAdapter.updatePolyline(this.plannedPath, path);
        }
    }

    updateNavigationPath(navigationPaths) {
        if (!navigationPaths) {
            return;
        }

        const paths = navigationPaths.map(navigationPath => {
            return navigationPath.pathPoint.map(point => {
                const [lng, lat] = this.mapAdapter.applyCoordinateOffset(
                    UTMToWGS84(point.x, point.y)
                );
                return this.mapAdapter.createPoint({ lat: lat, lng: lng });
            });
        });

        if (this.routingPaths.length < paths.length) {
            // adding more polyline(s)
            while (this.routingPaths.length < paths.length) {
                this.routingPaths.push(this.mapAdapter.createPolyline(null, "#CD5C5C", 0.7, 6));
            }
        } else if (this.routingPaths.length > paths.length) {
            // removing extra polyline(s)
            while (this.routingPaths.length > paths.length) {
                this.mapAdapter.removePolyline(this.routingPaths[this.routingPaths.length - 1]);
                this.routingPaths.pop();
            }
        }

        this.routingPaths.forEach((polyline, index) => {
            this.mapAdapter.updatePolyline(polyline, paths[index]);
        });
    }

    requestRoute(startLat, startLng, endLat, endLng) {
        if (!startLat || !startLng || !endLat || !endLng) {
            return;
        }

        const url =
            "http://navi-env.axty8vi3ic.us-west-2.elasticbeanstalk.com" +
            "/dreamview/navigation" +
            `?origin=${startLat},${startLng}` +
            `&destination=${endLat},${endLng}` +
            "&heading=0";
        fetch(encodeURI(url), {
            method: "GET",
            mode: "cors",
        }).then(response => {
            return response.arrayBuffer();
        }).then(response => {
            if (!response.byteLength) {
                alert("No navigation info received.");
                return;
            }

            this.WS.publishNavigationInfo(response);
        }).catch(error => {
            console.error("Failed to retrieve navigation data:", error);
        });
    }

    sendRoutingRequest() {
        if (this.routingRequestPoints) {
            const start =
                this.routingRequestPoints.length > 1
                    ? this.routingRequestPoints[0]
                    : this.mapAdapter.getMarkerPosition(this.vehicleMarker);
            const end = this.routingRequestPoints[this.routingRequestPoints.length - 1];
            this.routingRequestPoints = [];

            this.requestRoute(start.lat, start.lng, end.lat, end.lng);
            return true;
        } else {
            alert("Please select a route");
            return false;
        }
    }

    addDefaultEndPoint(points) {
        points.forEach(point => {
            const [lng, lat] = this.mapAdapter.applyCoordinateOffset(UTMToWGS84(point.x, point.y));
            this.routingRequestPoints.push({ lat: lat, lng: lng });
        });
    }
}

const MAP_NAVIGATOR = new MapNavigator();
export default MAP_NAVIGATOR;
