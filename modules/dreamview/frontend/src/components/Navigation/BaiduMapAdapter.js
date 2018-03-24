import { WGS84ToBD09LL } from "utils/coordinate_converter";

export default class BaiduMapAdapter {
    constructor() {
        this.map = null;
        this.controls = [];
    }

    isInitialized() {
        return this.map !== null;
    }

    loadMap(initPoint, divElementName) {
        this.map = new BMap.Map(divElementName, { enableMapClick: false });
        this.map.centerAndZoom(initPoint, 19);
        this.map.enableScrollWheelZoom();
        this.map.addControl(
            new BMap.MapTypeControl({
                anchor: BMAP_ANCHOR_TOP_LEFT,
                type: BMAP_NAVIGATION_CONTROL_SMALL,
            })
        );
        this.map.addControl(
            new BMap.NavigationControl({
                anchor: BMAP_ANCHOR_BOTTOM_RIGHT,
                type: BMAP_NAVIGATION_CONTROL_SMALL,
                enableGeolocation: false,
            })
        );
    }

    setCenter(point) {
        this.map.setCenter(point);
    }

    setZoom(zoom) {
        this.map.setZoom(zoom);
    }

    addEventHandler(eventName, handlerFunction) {
        this.map.addEventListener(eventName, event => {
            const clickedLatLng = event.point;
            handlerFunction(clickedLatLng);
        });
    }

    createPoint({ lat, lng }) {
        return new BMap.Point(lng, lat);
    }

    createMarker(point, title, draggable = true) {
        let label = null;
        if (title) {
            label = new BMap.Label(title, {
                point: point,
                offset: new BMap.Size(15, -15),
            });
        }

        const marker = new BMap.Marker(point, {
            label: label,
            enableDragging: draggable,
            rotation: 5,
        });
        marker.setLabel(label);
        this.map.addOverlay(marker);
        return marker;
    }

    createPolyline(path, color, opacity = 1.0, weight = 2.0) {
        const options = {
            geodesic: true,
            strokeColor: color,
            strokeOpacity: opacity,
            strokeWeight: weight,
        };
        const polyline = new BMap.Polyline(path, options);
        this.map.addOverlay(polyline);
        return polyline;
    }

    createControl({ text, tip, color, offsetX, offsetY, onClickHandler }) {
        const myControl = new NavigationControl(
            text,
            tip,
            color,
            new BMap.Size(offsetX, offsetY),
            onClickHandler
        );
        this.map.addControl(myControl);
        this.controls.push(myControl);
    }

    disableControls() {
        this.controls.forEach(control => {
            this.map.removeControl(control);
        });
    }

    enableControls() {
        this.controls.forEach(control => {
            this.map.addControl(control);
        });
    }

    getMarkerPosition(marker) {
        return marker.getPosition();
    }

    updatePolyline(polyline, newPath) {
        polyline.setPath(newPath);
    }

    removePolyline(polyline) {
        this.map.removeOverlay(polyline);
    }

    applyCoordinateOffset([lng, lat]) {
        return WGS84ToBD09LL(lng, lat);
    }
}

class NavigationControl extends BMap.Control {
    constructor(text, tip, color, offset, onClickHandler, ...args) {
        super(...args);
        this.defaultAnchor = BMAP_ANCHOR_TOP_LEFT;
        this.defaultOffset = offset;
        this.onClickHandler = onClickHandler;
        this.title = tip;
        this.text = text;
        this.backgroundColor = color;
    }

    initialize(map) {
        const controlDiv = document.createElement("div");

        // Set CSS for the control border.
        const controlUI = document.createElement("div");
        controlUI.style.backgroundColor = this.backgroundColor;
        controlUI.style.border = "2px solid #fff";
        controlUI.style.borderRadius = "3px";
        controlUI.style.boxShadow = "0 2px 6px rgba(0,0,0,.3)";
        controlUI.style.cursor = "pointer";
        controlUI.style.marginBottom = "22px";
        controlUI.style.textAlign = "center";
        controlUI.title = this.title;
        controlDiv.appendChild(controlUI);

        // // Set CSS for the control interior.
        const controlText = document.createElement("div");
        controlText.style.color = "rgb(25,25,25)";
        controlText.style.fontFamily = "Roboto,Arial,sans-serif";
        controlText.style.fontSize = "16px";
        controlText.style.lineHeight = "38px";
        controlText.style.paddingLeft = "5px";
        controlText.style.paddingRight = "5px";
        controlText.innerHTML = this.text;
        controlUI.appendChild(controlText);

        map.getContainer().appendChild(controlDiv);

        controlUI.addEventListener("click", () => {
            this.onClickHandler(controlText);
        });

        return controlDiv;
    }
}
