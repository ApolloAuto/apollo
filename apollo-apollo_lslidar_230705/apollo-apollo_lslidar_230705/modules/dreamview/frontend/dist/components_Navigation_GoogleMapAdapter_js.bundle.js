(self["webpackChunk"] = self["webpackChunk"] || []).push([["components_Navigation_GoogleMapAdapter_js"],{

/***/ "./components/Navigation/GoogleMapAdapter.js":
/*!***************************************************!*\
  !*** ./components/Navigation/GoogleMapAdapter.js ***!
  \***************************************************/
/***/ ((module, exports, __webpack_require__) => {

/* provided dependency */ var __react_refresh_utils__ = __webpack_require__(/*! ../node_modules/@pmmmwh/react-refresh-webpack-plugin/lib/runtime/RefreshUtils.js */ "../node_modules/@pmmmwh/react-refresh-webpack-plugin/lib/runtime/RefreshUtils.js");
/* provided dependency */ var __react_refresh_error_overlay__ = __webpack_require__(/*! ../node_modules/@pmmmwh/react-refresh-webpack-plugin/overlay/index.js */ "../node_modules/@pmmmwh/react-refresh-webpack-plugin/overlay/index.js");
__webpack_require__.$Refresh$.runtime = __webpack_require__(/*! ../node_modules/react-refresh/runtime.js */ "../node_modules/react-refresh/runtime.js");

"use strict";

Object.defineProperty(exports, "__esModule", ({
  value: true
}));
exports["default"] = void 0;

var _coordinate_converter = __webpack_require__(/*! utils/coordinate_converter */ "./utils/coordinate_converter.js");

function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { var _i = arr == null ? null : typeof Symbol !== "undefined" && arr[Symbol.iterator] || arr["@@iterator"]; if (_i == null) return; var _arr = []; var _n = true; var _d = false; var _s, _e; try { for (_i = _i.call(arr); !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } }

function _createClass(Constructor, protoProps, staticProps) { if (protoProps) _defineProperties(Constructor.prototype, protoProps); if (staticProps) _defineProperties(Constructor, staticProps); Object.defineProperty(Constructor, "prototype", { writable: false }); return Constructor; }

var GoogleMapAdapter = /*#__PURE__*/function () {
  function GoogleMapAdapter() {
    _classCallCheck(this, GoogleMapAdapter);

    this.map = null;
    this.controls = [];
  }

  _createClass(GoogleMapAdapter, [{
    key: "isInitialized",
    value: function isInitialized() {
      return this.map !== null;
    }
  }, {
    key: "loadMap",
    value: function loadMap(initPoint, divElementName) {
      var mapOptions = {
        center: initPoint,
        zoom: 20,
        mapTypeId: google.maps.MapTypeId.ROADMAP,
        fullscreenControl: false
      };
      this.map = new google.maps.Map(document.getElementById(divElementName), mapOptions);
    }
  }, {
    key: "setCenter",
    value: function setCenter(point) {
      this.map.setCenter(point);
    }
  }, {
    key: "setZoom",
    value: function setZoom(zoom) {
      this.map.setZoom(zoom);
    }
  }, {
    key: "addEventHandler",
    value: function addEventHandler(eventName, handler) {
      google.maps.event.addListener(this.map, eventName, function (event) {
        var clickedLatLng = event.latLng;
        handler(clickedLatLng);
      });
    }
  }, {
    key: "createPoint",
    value: function createPoint(_ref) {
      var lat = _ref.lat,
          lng = _ref.lng;
      return new google.maps.LatLng(lat, lng);
    }
  }, {
    key: "createMarker",
    value: function createMarker(point, title) {
      var draggable = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : true;
      var marker = new google.maps.Marker({
        position: point,
        label: title,
        draggable: draggable,
        map: this.map
      });
      return marker;
    }
  }, {
    key: "createPolyline",
    value: function createPolyline(path, color) {
      var opacity = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 1.0;
      var weight = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : 2.0;
      var polyline = new google.maps.Polyline({
        path: path,
        geodesic: true,
        strokeColor: color,
        strokeOpacity: opacity,
        strokeWeight: weight,
        map: this.map
      });
      return polyline;
    }
  }, {
    key: "createControl",
    value: function createControl(_ref2) {
      var text = _ref2.text,
          tip = _ref2.tip,
          color = _ref2.color,
          offsetX = _ref2.offsetX,
          offsetY = _ref2.offsetY,
          onClickHandler = _ref2.onClickHandler;
      var controlDiv = document.createElement('div'); // Set CSS for the control border.

      var controlUI = document.createElement('div');
      controlUI.style.backgroundColor = color;
      controlUI.style.border = '2px solid #fff';
      controlUI.style.borderRadius = '3px';
      controlUI.style.boxShadow = '0 2px 6px rgba(0,0,0,.3)';
      controlUI.style.cursor = 'pointer';
      controlUI.style.marginBottom = '22px';
      controlUI.style.textAlign = 'center';
      controlUI.title = tip;
      controlDiv.appendChild(controlUI); // Set CSS for the control interior.

      var controlText = document.createElement('div');
      controlText.style.color = 'rgb(25,25,25)';
      controlText.style.fontFamily = 'Roboto,Arial,sans-serif';
      controlText.style.fontSize = '16px';
      controlText.style.lineHeight = '38px';
      controlText.style.paddingLeft = '5px';
      controlText.style.paddingRight = '5px';
      controlText.innerHTML = text;
      controlUI.appendChild(controlText); // Setup the click event listeners: simply set the map to Chicago.

      controlUI.addEventListener('click', function () {
        onClickHandler(controlText);
      });
      this.map.controls[google.maps.ControlPosition.TOP_LEFT].push(controlDiv);
      this.controls.push(controlDiv);
    }
  }, {
    key: "disableControls",
    value: function disableControls() {
      this.controls.forEach(function (control) {
        control.style.display = 'none';
      });
    }
  }, {
    key: "enableControls",
    value: function enableControls() {
      this.controls.forEach(function (control) {
        control.style.display = 'block';
      });
    }
  }, {
    key: "getMarkerPosition",
    value: function getMarkerPosition(marker) {
      var position = marker.getPosition();
      return {
        lat: position.lat(),
        lng: position.lng()
      };
    }
  }, {
    key: "updatePolyline",
    value: function updatePolyline(polyline, newPath) {
      polyline.setPath(newPath);
    }
  }, {
    key: "removePolyline",
    value: function removePolyline(polyline) {
      polyline.setMap(null);
    }
  }, {
    key: "applyCoordinateOffset",
    value: function applyCoordinateOffset(_ref3) {
      var _ref4 = _slicedToArray(_ref3, 2),
          lng = _ref4[0],
          lat = _ref4[1];

      return (0, _coordinate_converter.WGS84ToGCJ02)(lng, lat);
    }
  }]);

  return GoogleMapAdapter;
}();

exports["default"] = GoogleMapAdapter;

const $ReactRefreshModuleId$ = __webpack_require__.$Refresh$.moduleId;
const $ReactRefreshCurrentExports$ = __react_refresh_utils__.getModuleExports(
	$ReactRefreshModuleId$
);

function $ReactRefreshModuleRuntime$(exports) {
	if (true) {
		let errorOverlay;
		if (typeof __react_refresh_error_overlay__ !== 'undefined') {
			errorOverlay = __react_refresh_error_overlay__;
		}
		let testMode;
		if (typeof __react_refresh_test__ !== 'undefined') {
			testMode = __react_refresh_test__;
		}
		return __react_refresh_utils__.executeRuntime(
			exports,
			$ReactRefreshModuleId$,
			module.hot,
			errorOverlay,
			testMode
		);
	}
}

if (typeof Promise !== 'undefined' && $ReactRefreshCurrentExports$ instanceof Promise) {
	$ReactRefreshCurrentExports$.then($ReactRefreshModuleRuntime$);
} else {
	$ReactRefreshModuleRuntime$($ReactRefreshCurrentExports$);
}

/***/ })

}]);
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiLi9jb21wb25lbnRzX05hdmlnYXRpb25fR29vZ2xlTWFwQWRhcHRlcl9qcy5idW5kbGUuanMiLCJtYXBwaW5ncyI6Ijs7Ozs7Ozs7Ozs7Ozs7Ozs7OztBQUFBOzs7Ozs7Ozs7Ozs7Ozs7Ozs7OztJQUVxQkE7RUFDbkIsNEJBQWM7SUFBQTs7SUFDWixLQUFLQyxHQUFMLEdBQVcsSUFBWDtJQUNBLEtBQUtDLFFBQUwsR0FBZ0IsRUFBaEI7RUFDRDs7OztXQUVELHlCQUFnQjtNQUNkLE9BQU8sS0FBS0QsR0FBTCxLQUFhLElBQXBCO0lBQ0Q7OztXQUVELGlCQUFRRSxTQUFSLEVBQW1CQyxjQUFuQixFQUFtQztNQUNqQyxJQUFNQyxVQUFVLEdBQUc7UUFDakJDLE1BQU0sRUFBRUgsU0FEUztRQUVqQkksSUFBSSxFQUFFLEVBRlc7UUFHakJDLFNBQVMsRUFBRUMsTUFBTSxDQUFDQyxJQUFQLENBQVlDLFNBQVosQ0FBc0JDLE9BSGhCO1FBSWpCQyxpQkFBaUIsRUFBRTtNQUpGLENBQW5CO01BTUEsS0FBS1osR0FBTCxHQUFXLElBQUlRLE1BQU0sQ0FBQ0MsSUFBUCxDQUFZSSxHQUFoQixDQUFvQkMsUUFBUSxDQUFDQyxjQUFULENBQXdCWixjQUF4QixDQUFwQixFQUE2REMsVUFBN0QsQ0FBWDtJQUNEOzs7V0FFRCxtQkFBVVksS0FBVixFQUFpQjtNQUNmLEtBQUtoQixHQUFMLENBQVNpQixTQUFULENBQW1CRCxLQUFuQjtJQUNEOzs7V0FFRCxpQkFBUVYsSUFBUixFQUFjO01BQ1osS0FBS04sR0FBTCxDQUFTa0IsT0FBVCxDQUFpQlosSUFBakI7SUFDRDs7O1dBRUQseUJBQWdCYSxTQUFoQixFQUEyQkMsT0FBM0IsRUFBb0M7TUFDbENaLE1BQU0sQ0FBQ0MsSUFBUCxDQUFZWSxLQUFaLENBQWtCQyxXQUFsQixDQUE4QixLQUFLdEIsR0FBbkMsRUFBd0NtQixTQUF4QyxFQUFtRCxVQUFDRSxLQUFELEVBQVc7UUFDNUQsSUFBTUUsYUFBYSxHQUFHRixLQUFLLENBQUNHLE1B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aUJELE9BQWpCO0lBQ0Q7OztXQUVELHdCQUFldEMsUUFBZixFQUF5QjtNQUN2QkEsUUFBUSxDQUFDd0MsTUFBVCxDQUFnQixJQUFoQjtJQUNEOzs7V0FFRCxzQ0FBa0M7TUFBQTtNQUFBLElBQVhwRCxHQUFXO01BQUEsSUFBTkQsR0FBTTs7TUFDaEMsT0FBTyxJQUFBc0Qsa0NBQUEsRUFBYXJELEdBQWIsRUFBa0JELEdBQWxCLENBQVA7SUFDRCIsInNvdXJjZXMiOlsid2VicGFjazovLy8uL2NvbXBvbmVudHMvTmF2aWdhdGlvbi9Hb29nbGVNYXBBZGFwdGVyLmpzIl0sInNvdXJjZXNDb250ZW50IjpbImltcG9ydCB7IFdHUzg0VG9HQ0owMiB9IGZyb20gJ3V0aWxzL2Nvb3JkaW5hdGVfY29udmVydGVyJztcblxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgR29vZ2xlTWFwQWRhcHRlciB7XG4gIGNvbnN0cnVjdG9yKCkge1xuICAgIHRoaXMubWFwID0gbnVsbDtcbiAgICB0aGlzLmNvbnRyb2xzID0gW107XG4gIH1cblxuICBpc0luaXRpYWxpemVkKCkge1xuICAgIHJldHVybiB0aGlzLm1hcCAhPT0gbnVsbDtcbiAgfVxuXG4gIGxvYWRNYXAoaW5pdFBvaW50LCBkaXZFbGVtZW50TmFtZSkge1xuICAgIGNvbnN0IG1hcE9wdGlvbnMgPSB7XG4gICAgICBjZW50ZXI6IGluaXRQb2ludCxcbiAgICAgIHpvb206IDIwLFxuICAgICAgbWFwVHlwZUlkOiBnb29nbGUubWFwcy5NYXBUeXBlSWQuUk9BRE1BUCxcbiAgICAgIGZ1bGxzY3JlZW5Db250cm9sOiBmYWxzZSxcbiAgICB9O1xuICAgIHRoaXMubWFwID0gbmV3IGdvb2dsZS5tYXBzLk1hcChkb2N1bWVudC5nZXRFbGVtZW50QnlJZChkaXZFbGVtZW50TmFtZSksIG1hcE9wdGlvbnMpO1xuICB9XG5cbiAgc2V0Q2VudGVyKHBvaW50KSB7XG4gICAgdGhpcy5tYXAuc2V0Q2VudGVyKHBvaW50KTtcbiAgfVxuXG4gIHNldFpvb20oem9vbSkge1xuICAgIHRoaXMubWFwLnNldFpvb20oem9vbSk7XG4gIH1cblxuICBhZGRFdmVudEhhbmRsZXIoZXZlbnROYW1lLCBoYW5kbGVyKSB7XG4gICAgZ29vZ2xlLm1hcHMuZXZlbnQuYWRkTGlzdGVuZXIodGhpcy5tYXAsIGV2ZW50TmFtZSwgKGV2ZW50KSA9PiB7XG4gICAgICBjb25zdCBjbGlja2VkTGF0TG5nID0gZXZlbnQubGF0TG5nO1xuICAgICAgaGFuZGxlcihjbGlja2VkTGF0TG5nKTtcbiAgICB9KTtcbiAgfVxuXG4gIGNyZWF0ZVBvaW50KHsgbGF0LCBsbmcgfSkge1xuICAgIHJldHVybiBuZXcgZ29vZ2xlLm1hcHMuTGF0TG5nKGxhdCwgbG5nKTtcbiAgfVxuXG4gIGNyZWF0ZU1hcmtlcihwb2ludCwgdGl0bGUsIGRyYWdnYWJsZSA9IHRydWUpIHtcbiAgICBjb25zdCBtYXJrZXIgPSBuZXcgZ29vZ2xlLm1hcHMuTWFya2VyKHtcbiAgICAgIHBvc2l0aW9uOiBwb2ludCxcbiAgICAgIGxhYmVsOiB0aXRsZSxcbiAgICAgIGRyYWdnYWJsZSxcbiAgICAgIG1hcDogdGhpcy5tYXAsXG4gICAgfSk7XG4gICAgcmV0dXJuIG1hcmtlcjtcbiAgfVxuXG4gIGNyZWF0ZVBvbHlsaW5lKHBhdGgsIGNvbG9yLCBvcGFjaXR5ID0gMS4wLCB3ZWlnaHQgPSAyLjApIHtcbiAgICBjb25zdCBwb2x5bGluZSA9IG5ldyBnb29nbGUubWFwcy5Qb2x5bGluZSh7XG4gICAgICBwYXRoLFxuICAgICAgZ2VvZGVzaWM6IHRydWUsXG4gICAgICBzdHJva2VDb2xvcjogY29sb3IsXG4gICAgICBzdHJva2VPcGFjaXR5OiBvcGFjaXR5LFxuICAgICAgc3Ryb2tlV2VpZ2h0OiB3ZWlnaHQsXG4gICAgICBtYXA6IHRoaXMubWFwLFxuICAgIH0pO1xuICAgIHJldHVybiBwb2x5bGluZTtcbiAgfVxuXG4gIGNyZWF0ZUNvbnRyb2woe1xuICAgIHRleHQsIHRpcCwgY29sb3IsIG9mZnNldFgsIG9mZnNldFksIG9uQ2xpY2tIYW5kbGVyLFxuICB9KSB7XG4gICAgY29uc3QgY29udHJvbERpdiA9IGRvY3VtZW50LmNyZWF0ZUVsZW1lbnQoJ2RpdicpO1xuXG4gICAgLy8gU2V0IENTUyBmb3IgdGhlIGNvbnRyb2wgYm9yZGVyLlxuICAgIGNvbnN0IGNvbnRyb2xVSSA9IGRvY3VtZW50LmNyZWF0ZUVsZW1lbnQoJ2RpdicpO1xuICAgIGNvbnRyb2xVSS5zdHlsZS5iYWNrZ3JvdW5kQ29sb3IgPSBjb2xvcjtcbiAgICBjb250cm9sVUkuc3R5bGUuYm9yZGVyID0gJzJweCBzb2xpZCAjZmZmJztcbiAgICBjb250cm9sVUkuc3R5bGUuYm9yZGVyUmFkaXVzID0gJzNweCc7XG4gICAgY29udHJvbFVJLnN0eWxlLmJveFNoYWRvdyA9ICcwIDJweCA2cHggcmdiYSgwLDAsMCwuMyknO1xuICAgIGNvbnRyb2xVSS5zdHlsZS5jdXJzb3IgPSAncG9pbnRlcic7XG4gICAgY29udHJvbFVJLnN0eWxlLm1hcmdpbkJvdHRvbSA9ICcyMnB4JztcbiAgICBjb250cm9sVUkuc3R5bGUudGV4dEFsaWduID0gJ2NlbnRlcic7XG4gICAgY29udHJvbFVJLnRpdGxlID0gdGlwO1xuICAgIGNvbnRyb2xEaXYuYXBwZW5kQ2hpbGQoY29udHJvbFVJKTtcblxuICAgIC8vIFNldCBDU1MgZm9yIHRoZSBjb250cm9sIGludGVyaW9yLlxuICAgIGNvbnN0IGNvbnRyb2xUZXh0ID0gZG9jdW1lbnQuY3JlYXRlRWxlbWVudCgnZGl2Jyk7XG4gICAgY29udHJvbFRleHQuc3R5bGUuY29sb3IgPSAncmdiKDI1LDI1LDI1KSc7XG4gICAgY29udHJvbFRleHQuc3R5bGUuZm9udEZhbWlseSA9ICdSb2JvdG8sQXJpYWwsc2Fucy1zZXJpZic7XG4gICAgY29udHJvbFRleHQuc3R5bGUuZm9udFNpemUgPSAnMTZweCc7XG4gICAgY29udHJvbFRleHQuc3R5bGUubGluZUhlaWdodCA9ICczOHB4JztcbiAgICBjb250cm9sVGV4dC5zdHlsZS5wYWRkaW5nTGVmdCA9ICc1cHgnO1xuICAgIGNvbnRyb2xUZXh0LnN0eWxlLnBhZGRpbmdSaWdodCA9ICc1cHgnO1xuICAgIGNvbnRyb2xUZXh0LmlubmVySFRNTCA9IHRleHQ7XG4gICAgY29udHJvbFVJLmFwcGVuZENoaWxkKGNvbnRyb2xUZXh0KTtcblxuICAgIC8vIFNldHVwIHRoZSBjbGljayBldmVudCBsaXN0ZW5lcnM6IHNpbXBseSBzZXQgdGhlIG1hcCB0byBDaGljYWdvLlxuICAgIGNvbnRyb2xVSS5hZGRFdmVudExpc3RlbmVyKCdjbGljaycsICgpID0+IHtcbiAgICAgIG9uQ2xpY2tIYW5kbGVyKGNvbnRyb2xUZXh0KTtcbiAgICB9KTtcblxuICAgIHRoaXMubWFwLmNvbnRyb2xzW2dvb2dsZS5tYXBzLkNvbnRyb2xQb3NpdGlvbi5UT1BfTEVGVF0ucHVzaChjb250cm9sRGl2KTtcbiAgICB0aGlzLmNvbnRyb2xzLnB1c2goY29udHJvbERpdik7XG4gIH1cblxuICBkaXNhYmxlQ29udHJvbHMoKSB7XG4gICAgdGhpcy5jb250cm9scy5mb3JFYWNoKChjb250cm9sKSA9PiB7XG4gICAgICBjb250cm9sLnN0eWxlLmRpc3BsYXkgPSAnbm9uZSc7XG4gICAgfSk7XG4gIH1cblxuICBlbmFibGVDb250cm9scygpIHtcbiAgICB0aGlzLmNvbnRyb2xzLmZvckVhY2goKGNvbnRyb2wpID0+IHtcbiAgICAgIGNvbnRyb2wuc3R5bGUuZGlzcGxheSA9ICdibG9jayc7XG4gICAgfSk7XG4gIH1cblxuICBnZXRNYXJrZXJQb3NpdGlvbihtYXJrZXIpIHtcbiAgICBjb25zdCBwb3NpdGlvbiA9IG1hcmtlci5nZXRQb3NpdGlvbigpO1xuICAgIHJldHVybiB7IGxhdDogcG9zaXRpb24ubGF0KCksIGxuZzogcG9zaXRpb24ubG5nKCkgfTtcbiAgfVxuXG4gIHVwZGF0ZVBvbHlsaW5lKHBvbHlsaW5lLCBuZXdQYXRoKSB7XG4gICAgcG9seWxpbmUuc2V0UGF0aChuZXdQYXRoKTtcbiAgfVxuXG4gIHJlbW92ZVBvbHlsaW5lKHBvbHlsaW5lKSB7XG4gICAgcG9seWxpbmUuc2V0TWFwKG51bGwpO1xuICB9XG5cbiAgYXBwbHlDb29yZGluYXRlT2Zmc2V0KFtsbmcsIGxhdF0pIHtcbiAgICByZXR1cm4gV0dTODRUb0dDSjAyKGxuZywgbGF0KTtcbiAgfVxufVxuIl0sIm5hbWVzIjpbIkdvb2dsZU1hcEFkYXB0ZXIiLCJtYXAiLCJjb250cm9scyIsImluaXRQb2ludCIsImRpdkVsZW1lbnROYW1lIiwibWFwT3B0aW9ucyIsImNlbnRlciIsInpvb20iLCJtYXBUeXBlSWQiLCJnb29nbGUiLCJtYXBzIiwiTWFwVHlwZUlkIiwiUk9BRE1BUCIsImZ1bGxzY3JlZW5Db250cm9sIiwiTWFwIiwiZG9jdW1lbnQiLCJnZXRFbGVtZW50QnlJZCIsInBvaW50Iiwic2V0Q2VudGVyIiwic2V0Wm9vbSIsImV2ZW50TmFtZSIsImhhbmRsZXIiLCJldmVudCIsImFkZExpc3RlbmVyIiwiY2xpY2tlZExhdExuZyIsImxhdExuZyIsImxhdCIsImxuZyIsIkxhdExuZyIsInRpdGxlIiwiZHJhZ2dhYmxlIiwibWFya2VyIiwiTWFya2VyIiwicG9zaXRpb24iLCJsYWJlbCIsInBhdGgiLCJjb2xvciIsIm9wYWNpdHkiLCJ3ZWlnaHQiLCJwb2x5bGluZSIsIlBvbHlsaW5lIiwiZ2VvZGVzaWMiLCJzdHJva2VDb2xvciIsInN0cm9rZU9wYWNpdHkiLCJzdHJva2VXZWlnaHQiLCJ0ZXh0IiwidGlwIiwib2Zmc2V0WCIsIm9mZnNldFkiLCJvbkNsaWNrSGFuZGxlciIsImNvbnRyb2xEaXYiLCJjcmVhdGVFbGVtZW50IiwiY29udHJvbFVJIiwic3R5bGUiLCJiYWNrZ3JvdW5kQ29sb3IiLCJib3JkZXIiLCJib3JkZXJSYWRpdXMiLCJib3hTaGFkb3ciLCJjdXJzb3IiLCJtYXJnaW5Cb3R0b20iLCJ0ZXh0QWxpZ24iLCJhcHBlbmRDaGlsZCIsImNvbnRyb2xUZXh0IiwiZm9udEZhbWlseSIsImZvbnRTaXplIiwibGluZUhlaWdodCIsInBhZGRpbmdMZWZ0IiwicGFkZGluZ1JpZ2h0IiwiaW5uZXJIVE1MIiwiYWRkRXZlbnRMaXN0ZW5lciIsIkNvbnRyb2xQb3NpdGlvbiIsIlRPUF9MRUZUIiwicHVzaCIsImZvckVhY2giLCJjb250cm9sIiwiZGlzcGxheSIsImdldFBvc2l0aW9uIiwibmV3UGF0aCIsInNldFBhdGgiLCJzZXRNYXAiLCJXR1M4NFRvR0NKMDIiXSwic291cmNlUm9vdCI6IiJ9