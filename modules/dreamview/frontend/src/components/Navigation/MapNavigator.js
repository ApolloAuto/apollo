import polyval from "compute-polynomial";
import { UTMToWGS84 } from "utils/coordinate_converter";

class MapNavigator {
    constructor() {
        this.WS = null;
        this.mapAdapter = null;

        this.routingPaths = [];
        this.plannedPath = null;
        this.vehicleMarker = null;
        this.rightLaneMarker = null;
        this.leftLaneMarker = null;
        this.destinationMarker = null;
        this.centerVehicle = true;
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

    createControls() {
        this.mapAdapter.createControl({
            text: "Center Vehicle is ON",
            tip: "Click to recenter the vehicle",
            color: "#FFFFFF",
            offsetX: 430,
            offsetY: 0,
            onClickHandler: textElementDiv => {
                if (this.centerVehicle) {
                    this.centerVehicle = false;
                    textElementDiv.innerHTML = "Center Vehicle is OFF";
                    this.mapAdapter.setZoom(15);
                } else {
                    this.centerVehicle = true;
                    textElementDiv.innerHTML = "Center Vehicle is ON";
                    this.mapAdapter.setZoom(20);
                }
            },
        });

        this.mapAdapter.createControl({
            text: "Routing Request",
            tip: "Click to send routing request",
            color: "#CD5C5C",
            offsetX: 298,
            offsetY: 0,
            onClickHandler: textElementDiv => {
                if (!this.destinationMarker) {
                    alert("please select a destination point.");
                    return;
                }

                const start = this.mapAdapter.getMarkerPosition(this.vehicleMarker);
                const end = this.mapAdapter.getMarkerPosition(this.destinationMarker);
                this.requestRouting(start.lat, start.lng, end.lat, end.lng);
            },
        });

        this.mapAdapter.createControl({
            text: "TO Cananda West",
            tip: "Click to send routing request",
            color: "#FF8C00",
            offsetX: 152,
            offsetY: 0,
            onClickHandler: textElementDiv => {
                const start = this.mapAdapter.getMarkerPosition(this.vehicleMarker);
                const endLat = 37.50582457077844;
                const endLng = -122.34000922633726;
                this.requestRouting(start.lat, start.lng, endLat, endLng);
            },
        });

        this.mapAdapter.createControl({
            text: "TO Cananda East",
            tip: "Click to send routing request",
            color: "#00BFFF",
            offsetX: 10,
            offsetY: 0,
            onClickHandler: textElementDiv => {
                const start = this.mapAdapter.getMarkerPosition(this.vehicleMarker);
                const endLat = 37.464198;
                const endLng = -122.298453;
                this.requestRouting(start.lat, start.lng, endLat, endLng);
            },
        });
    }

    update(data) {
        if (!this.WS || !this.mapAdapter || !this.mapAdapter.isInitialized()) {
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
        const heading = autoDrivingCar.heading;

        const [longitude, latitude] = UTMToWGS84(x, y);
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
            const y = polyval(markerCoef, x);
            const newX = x * Math.cos(heading) - y * Math.sin(heading);
            const newY = y * Math.cos(heading) + x * Math.sin(heading);
            const [plon, plat] = UTMToWGS84(adcX + newX, adcY + newY);
            lane.push(this.mapAdapter.createPoint({ lat: plat, lng: plon }));
        }
        return lane;
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
            const [plon, plat] = UTMToWGS84(adcX + newX, adcY + newY);
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
                const [lng, lat] = UTMToWGS84(point.x, point.y);
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
            mode: "cors",
        }).then(response => {
            return response.arrayBuffer();
        }).then(response => {
            if (!response.byteLength) {
                console.warn("No navigation info received.");
                return;
            }

            this.WS.publishNavigationInfo(response);
        }).catch(error => {
            console.error("Failed to retrieve navigation data:", error);
        });
    }
}

const MAP_NAVIGATOR = new MapNavigator();
export default MAP_NAVIGATOR;
