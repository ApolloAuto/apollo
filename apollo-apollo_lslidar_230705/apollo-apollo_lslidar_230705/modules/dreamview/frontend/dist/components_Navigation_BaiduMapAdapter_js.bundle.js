(self["webpackChunk"] = self["webpackChunk"] || []).push([["components_Navigation_BaiduMapAdapter_js"],{

/***/ "./components/Navigation/BaiduMapAdapter.js":
/*!**************************************************!*\
  !*** ./components/Navigation/BaiduMapAdapter.js ***!
  \**************************************************/
/***/ ((module, exports, __webpack_require__) => {

/* provided dependency */ var __react_refresh_utils__ = __webpack_require__(/*! ../node_modules/@pmmmwh/react-refresh-webpack-plugin/lib/runtime/RefreshUtils.js */ "../node_modules/@pmmmwh/react-refresh-webpack-plugin/lib/runtime/RefreshUtils.js");
/* provided dependency */ var __react_refresh_error_overlay__ = __webpack_require__(/*! ../node_modules/@pmmmwh/react-refresh-webpack-plugin/overlay/index.js */ "../node_modules/@pmmmwh/react-refresh-webpack-plugin/overlay/index.js");
__webpack_require__.$Refresh$.runtime = __webpack_require__(/*! ../node_modules/react-refresh/runtime.js */ "../node_modules/react-refresh/runtime.js");

"use strict";

function _typeof(obj) { "@babel/helpers - typeof"; return _typeof = "function" == typeof Symbol && "symbol" == typeof Symbol.iterator ? function (obj) { return typeof obj; } : function (obj) { return obj && "function" == typeof Symbol && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : typeof obj; }, _typeof(obj); }

Object.defineProperty(exports, "__esModule", ({
  value: true
}));
exports["default"] = void 0;

var _coordinate_converter = __webpack_require__(/*! utils/coordinate_converter */ "./utils/coordinate_converter.js");

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function"); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, writable: true, configurable: true } }); Object.defineProperty(subClass, "prototype", { writable: false }); if (superClass) _setPrototypeOf(subClass, superClass); }

function _setPrototypeOf(o, p) { _setPrototypeOf = Object.setPrototypeOf ? Object.setPrototypeOf.bind() : function _setPrototypeOf(o, p) { o.__proto__ = p; return o; }; return _setPrototypeOf(o, p); }

function _createSuper(Derived) { var hasNativeReflectConstruct = _isNativeReflectConstruct(); return function _createSuperInternal() { var Super = _getPrototypeOf(Derived), result; if (hasNativeReflectConstruct) { var NewTarget = _getPrototypeOf(this).constructor; result = Reflect.construct(Super, arguments, NewTarget); } else { result = Super.apply(this, arguments); } return _possibleConstructorReturn(this, result); }; }

function _possibleConstructorReturn(self, call) { if (call && (_typeof(call) === "object" || typeof call === "function")) { return call; } else if (call !== void 0) { throw new TypeError("Derived constructors may only return object or undefined"); } return _assertThisInitialized(self); }

function _assertThisInitialized(self) { if (self === void 0) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return self; }

function _isNativeReflectConstruct() { if (typeof Reflect === "undefined" || !Reflect.construct) return false; if (Reflect.construct.sham) return false; if (typeof Proxy === "function") return true; try { Boolean.prototype.valueOf.call(Reflect.construct(Boolean, [], function () {})); return true; } catch (e) { return false; } }

function _getPrototypeOf(o) { _getPrototypeOf = Object.setPrototypeOf ? Object.getPrototypeOf.bind() : function _getPrototypeOf(o) { return o.__proto__ || Object.getPrototypeOf(o); }; return _getPrototypeOf(o); }

function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { var _i = arr == null ? null : typeof Symbol !== "undefined" && arr[Symbol.iterator] || arr["@@iterator"]; if (_i == null) return; var _arr = []; var _n = true; var _d = false; var _s, _e; try { for (_i = _i.call(arr); !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } }

function _createClass(Constructor, protoProps, staticProps) { if (protoProps) _defineProperties(Constructor.prototype, protoProps); if (staticProps) _defineProperties(Constructor, staticProps); Object.defineProperty(Constructor, "prototype", { writable: false }); return Constructor; }

var BaiduMapAdapter = /*#__PURE__*/function () {
  function BaiduMapAdapter() {
    _classCallCheck(this, BaiduMapAdapter);

    this.map = null;
    this.controls = [];
    this.initializedCenter = false;
  }

  _createClass(BaiduMapAdapter, [{
    key: "isInitialized",
    value: function isInitialized() {
      return this.map !== null && Object.keys(this.map).length > 0;
    }
  }, {
    key: "loadMap",
    value: function loadMap(initPoint, divElementName) {
      this.map = new BMap.Map(divElementName, {
        enableMapClick: false
      });
      this.map.enableScrollWheelZoom();
      this.map.addControl(new BMap.MapTypeControl({
        anchor: BMAP_ANCHOR_TOP_LEFT,
        type: BMAP_NAVIGATION_CONTROL_SMALL
      }));
      this.map.addControl(new BMap.NavigationControl({
        anchor: BMAP_ANCHOR_BOTTOM_RIGHT,
        type: BMAP_NAVIGATION_CONTROL_SMALL,
        enableGeolocation: false
      }));
    }
  }, {
    key: "setCenter",
    value: function setCenter(point) {
      if (this.initializedCenter) {
        this.map.setCenter(point);
      } else {
        this.map.centerAndZoom(point, 19);
        this.initializedCenter = true;
      }
    }
  }, {
    key: "setZoom",
    value: function setZoom(zoom) {
      this.map.setZoom(zoom);
    }
  }, {
    key: "addEventHandler",
    value: function addEventHandler(eventName, handlerFunction) {
      this.map.addEventListener(eventName, function (event) {
        var clickedLatLng = event.point;
        handlerFunction(clickedLatLng);
      });
    }
  }, {
    key: "createPoint",
    value: function createPoint(_ref) {
      var lat = _ref.lat,
          lng = _ref.lng;
      return new BMap.Point(lng, lat);
    }
  }, {
    key: "createMarker",
    value: function createMarker(point, title) {
      var draggable = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : true;
      var label = null;

      if (title) {
        label = new BMap.Label(title, {
          point: point,
          offset: new BMap.Size(15, -15)
        });
      }

      var marker = new BMap.Marker(point, {
        label: label,
        enableDragging: draggable,
        rotation: 5
      });
      marker.setLabel(label);
      this.map.addOverlay(marker);
      return marker;
    }
  }, {
    key: "createPolyline",
    value: function createPolyline(path, color) {
      var opacity = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 1.0;
      var weight = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : 2.0;
      var options = {
        geodesic: true,
        strokeColor: color,
        strokeOpacity: opacity,
        strokeWeight: weight
      };
      var polyline = new BMap.Polyline(path, options);
      this.map.addOverlay(polyline);
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
      var myControl = new NavigationControl(text, tip, color, new BMap.Size(offsetX, offsetY), onClickHandler);
      this.map.addControl(myControl);
      this.controls.push(myControl);
    }
  }, {
    key: "disableControls",
    value: function disableControls() {
      var _this = this;

      this.controls.forEach(function (control) {
        _this.map.removeControl(control);
      });
    }
  }, {
    key: "enableControls",
    value: function enableControls() {
      var _this2 = this;

      this.controls.forEach(function (control) {
        _this2.map.addControl(control);
      });
    }
  }, {
    key: "getMarkerPosition",
    value: function getMarkerPosition(marker) {
      return marker.getPosition();
    }
  }, {
    key: "updatePolyline",
    value: function updatePolyline(polyline, newPath) {
      polyline.setPath(newPath);
    }
  }, {
    key: "removePolyline",
    value: function removePolyline(polyline) {
      this.map.removeOverlay(polyline);
    }
  }, {
    key: "applyCoordinateOffset",
    value: function applyCoordinateOffset(_ref3) {
      var _ref4 = _slicedToArray(_ref3, 2),
          lng = _ref4[0],
          lat = _ref4[1];

      return (0, _coordinate_converter.WGS84ToBD09LL)(lng, lat);
    }
  }]);

  return BaiduMapAdapter;
}();

exports["default"] = BaiduMapAdapter;

var NavigationControl = /*#__PURE__*/function (_BMap$Control) {
  _inherits(NavigationControl, _BMap$Control);

  var _super = _createSuper(NavigationControl);

  function NavigationControl(text, tip, color, offset, onClickHandler) {
    var _this3;

    _classCallCheck(this, NavigationControl);

    for (var _len = arguments.length, args = new Array(_len > 5 ? _len - 5 : 0), _key = 5; _key < _len; _key++) {
      args[_key - 5] = arguments[_key];
    }

    _this3 = _super.call.apply(_super, [this].concat(args));
    _this3.defaultAnchor = BMAP_ANCHOR_TOP_LEFT;
    _this3.defaultOffset = offset;
    _this3.onClickHandler = onClickHandler;
    _this3.title = tip;
    _this3.text = text;
    _this3.backgroundColor = color;
    return _this3;
  }

  _createClass(NavigationControl, [{
    key: "initialize",
    value: function initialize(map) {
      var _this4 = this;

      var controlDiv = document.createElement('div'); // Set CSS for the control border.

      var controlUI = document.createElement('div');
      controlUI.style.backgroundColor = this.backgroundColor;
      controlUI.style.border = '2px solid #fff';
      controlUI.style.borderRadius = '3px';
      controlUI.style.boxShadow = '0 2px 6px rgba(0,0,0,.3)';
      controlUI.style.cursor = 'pointer';
      controlUI.style.marginBottom = '22px';
      controlUI.style.textAlign = 'center';
      controlUI.title = this.title;
      controlDiv.appendChild(controlUI); // // Set CSS for the control interior.

      var controlText = document.createElement('div');
      controlText.style.color = 'rgb(25,25,25)';
      controlText.style.fontFamily = 'Roboto,Arial,sans-serif';
      controlText.style.fontSize = '16px';
      controlText.style.lineHeight = '38px';
      controlText.style.paddingLeft = '5px';
      controlText.style.paddingRight = '5px';
      controlText.innerHTML = this.text;
      controlUI.appendChild(controlText);
      map.getContainer().appendChild(controlDiv);
      controlUI.addEventListener('click', function () {
        _this4.onClickHandler(controlText);
      });
      return controlDiv;
    }
  }]);

  return NavigationControl;
}(BMap.Control);

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
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiLi9jb21wb25lbnRzX05hdmlnYXRpb25fQmFpZHVNYXBBZGFwdGVyX2pzLmJ1bmRsZS5qcyIsIm1hcHBpbmdzIjoiOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7QUFBQTs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7OztJQUVxQkE7RUFDbkIsMkJBQWM7SUFBQTs7SUFDWixLQUFLQyxHQUFMLEdBQVcsSUFBWDtJQUNBLEtBQUtDLFFBQUwsR0FBZ0IsRUFBaEI7SUFDQSxLQUFLQyxpQkFBTCxHQUF5QixLQUF6QjtFQUNEOzs7O1dBRUQseUJBQWdCO01BQ2QsT0FBTyxLQUFLRixHQUFMLEtBQWEsSUFBYixJQUFxQkcsTUFBTSxDQUFDQyxJQUFQLENBQVksS0FBS0osR0FBakIsRUFBc0JLLE1BQXRCLEdBQStCLENBQTNEO0lBQ0Q7OztXQUVELGlCQUFRQyxTQUFSLEVBQW1CQyxjQUFuQixFQUFtQztNQUNqQyxLQUFLUCxHQUFMLEdBQVcsSUFBSVEsSUFBSSxDQUFDQyxHQUFULENBQWFGLGNBQWIsRUFBNkI7UUFBRUcsY0FBYyxFQUFFO01BQWxCLENBQTdCLENBQVg7TUFFQSxLQUFLVixHQUFMLENBQVNXLHFCQUFUO01BQ0EsS0FBS1gsR0FBTCxDQUFTWSxVQUFULENBQ0UsSUFBSUosSUFBSSxDQUFDSyxjQUFULENBQXdCO1FBQ3RCQyxNQUFNLEVBQUVDLG9CQURjO1FBRXRCQyxJQUFJLEVBQUVDO01BRmdCLENBQXhCLENBREY7TUFNQSxLQUFLakIsR0FBTCxDQUFTWSxVQUFULENBQ0UsSUFBSUosSUFBSSxDQUFDVSxpQkFBVCxDQUEyQjtRQUN6QkosTUFBTSxFQUFFSyx3QkFEaUI7UUFFekJILElBQUksRUFBRUMsNkJBRm1CO1FBR3pCRyxpQkFBaUIsRUFBRTtNQUhNLENBQTNCLENBREY7SUFPRDs7O1dBRUQsbUJBQVVDLEtBQVYsRUFBaUI7TUFDZixJQUFJLEtBQUtuQixpQkFBVCxFQUE0QjtRQUMxQixLQUFLRixHQUFMLENBQVNzQixTQUFULENBQW1CRCxLQUFu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O01BQ0FILFdBQVcsQ0FBQ1IsS0FBWixDQUFrQlksV0FBbEIsR0FBZ0MsS0FBaEM7TUFDQUosV0FBVyxDQUFDUixLQUFaLENBQWtCYSxZQUFsQixHQUFpQyxLQUFqQztNQUNBTCxXQUFXLENBQUNNLFNBQVosR0FBd0IsS0FBS3JDLElBQTdCO01BQ0FzQixTQUFTLENBQUNRLFdBQVYsQ0FBc0JDLFdBQXRCO01BRUF4RixHQUFHLENBQUMrRixZQUFKLEdBQW1CUixXQUFuQixDQUErQlgsVUFBL0I7TUFFQUcsU0FBUyxDQUFDbkQsZ0JBQVYsQ0FBMkIsT0FBM0IsRUFBb0MsWUFBTTtRQUN4QyxNQUFJLENBQUNpQyxjQUFMLENBQW9CMkIsV0FBcEI7TUFDRCxDQUZEO01BSUEsT0FBT1osVUFBUDtJQUNEOzs7O0VBNUM2QnBFLElBQUksQ0FBQ3dGIiwic291cmNlcyI6WyJ3ZWJwYWNrOi8vLy4vY29tcG9uZW50cy9OYXZpZ2F0aW9uL0JhaWR1TWFwQWRhcHRlci5qcyJdLCJzb3VyY2VzQ29udGVudCI6WyJpbXBvcnQgeyBXR1M4NFRvQkQwOUxMIH0gZnJvbSAndXRpbHMvY29vcmRpbmF0ZV9jb252ZXJ0ZXInO1xuXG5leHBvcnQgZGVmYXVsdCBjbGFzcyBCYWlkdU1hcEFkYXB0ZXIge1xuICBjb25zdHJ1Y3RvcigpIHtcbiAgICB0aGlzLm1hcCA9IG51bGw7XG4gICAgdGhpcy5jb250cm9scyA9IFtdO1xuICAgIHRoaXMuaW5pdGlhbGl6ZWRDZW50ZXIgPSBmYWxzZTtcbiAgfVxuXG4gIGlzSW5pdGlhbGl6ZWQoKSB7XG4gICAgcmV0dXJuIHRoaXMubWFwICE9PSBudWxsICYmIE9iamVjdC5rZXlzKHRoaXMubWFwKS5sZW5ndGggPiAwO1xuICB9XG5cbiAgbG9hZE1hcChpbml0UG9pbnQsIGRpdkVsZW1lbnROYW1lKSB7XG4gICAgdGhpcy5tYXAgPSBuZXcgQk1hcC5NYXAoZGl2RWxlbWVudE5hbWUsIHsgZW5hYmxlTWFwQ2xpY2s6IGZhbHNlIH0pO1xuXG4gICAgdGhpcy5tYXAuZW5hYmxlU2Nyb2xsV2hlZWxab29tKCk7XG4gICAgdGhpcy5tYXAuYWRkQ29udHJvbChcbiAgICAgIG5ldyBCTWFwLk1hcFR5cGVDb250cm9sKHtcbiAgICAgICAgYW5jaG9yOiBCTUFQX0FOQ0hPUl9UT1BfTEVGVCxcbiAgICAgICAgdHlwZTogQk1BUF9OQVZJR0FUSU9OX0NPTlRST0xfU01BTEwsXG4gICAgICB9KSxcbiAgICApO1xuICAgIHRoaXMubWFwLmFkZENvbnRyb2woXG4gICAgICBuZXcgQk1hcC5OYXZpZ2F0aW9uQ29udHJvbCh7XG4gICAgICAgIGFuY2hvcjogQk1BUF9BTkNIT1JfQk9UVE9NX1JJR0hULFxuICAgICAgICB0eXBlOiBCTUFQX05BVklHQVRJT05fQ09OVFJPTF9TTUFMTCxcbiAgICAgICAgZW5hYmxlR2VvbG9jYXRpb246IGZhbHNlLFxuICAgICAgfSksXG4gICAgKTtcbiAgfVxuXG4gIHNldENlbnRlcihwb2ludCkge1xuICAgIGlmICh0aGlzLmluaXRpYWxpemVkQ2VudGVyKSB7XG4gICAgICB0aGlzLm1hcC5zZXRDZW50ZXIocG9pbnQpO1xuICAgIH0gZWxzZSB7XG4gICAgICB0aGlzLm1hcC5jZW50ZXJBbmRab29tKHBvaW50LCAxOSk7XG4gICAgICB0aGlzLmluaXRpYWxpemVkQ2VudGVyID0gdHJ1ZTtcbiAgICB9XG4gIH1cblxuICBzZXRab29tKHpvb20pIHtcbiAgICB0aGlzLm1hcC5zZXRab29tKHpvb20pO1xuICB9XG5cbiAgYWRkRXZlbnRIYW5kbGVyKGV2ZW50TmFtZSwgaGFuZGxlckZ1bmN0aW9uKSB7XG4gICAgdGhpcy5tYXAuYWRkRXZlbnRMaXN0ZW5lcihldmVudE5hbWUsIChldmVudCkgPT4ge1xuICAgICAgY29uc3QgY2xpY2tlZExhdExuZyA9IGV2ZW50LnBvaW50O1xuICAgICAgaGFuZGxlckZ1bmN0aW9uKGNsaWNrZWRMYXRMbmcpO1xuICAgIH0pO1xuICB9XG5cbiAgY3JlYXRlUG9pbnQoeyBsYXQsIGxuZyB9KSB7XG4gICAgcmV0dXJuIG5ldyBCTWFwLlBvaW50KGxuZywgbGF0KTtcbiAgfVxuXG4gIGNyZWF0ZU1hcmtlcihwb2ludCwgdGl0bGUsIGRyYWdnYWJsZSA9IHRydWUpIHtcbiAgICBsZXQgbGFiZWwgPSBudWxsO1xuICAgIGlmICh0aXRsZSkge1xuICAgICAgbGFiZWwgPSBuZXcgQk1hcC5MYWJlbCh0aXRsZSwge1xuICAgICAgICBwb2ludCxcbiAgICAgICAgb2Zmc2V0OiBuZXcgQk1hcC5TaXplKDE1LCAtMTUpLFxuICAgICAgfSk7XG4gICAgfVxuXG4gICAgY29uc3QgbWFya2VyID0gbmV3IEJNYXAuTWFya2VyKHBvaW50LCB7XG4gICAgICBsYWJlbCxcbiAgICAgIGVuYWJsZURyYWdnaW5nOiBkcmFnZ2FibGUsXG4gICAgICByb3RhdGlvbjogNSxcbiAgICB9KTtcbiAgICBtYXJrZXIuc2V0TGFiZWwobGFiZWwpO1xuICAgIHRoaXMubWFwLmFkZE92ZXJsYXkobWFya2VyKTtcbiAgICByZXR1cm4gbWFya2VyO1xuICB9XG5cbiAgY3JlYXRlUG9seWxpbmUocGF0aCwgY29sb3IsIG9wYWNpdHkgPSAxLjAsIHdlaWdodCA9IDIuMCkge1xuICAgIGNvbnN0IG9wdGlvbnMgPSB7XG4gICAgICBnZW9kZXNpYzogdHJ1ZSxcbiAgICAgIHN0cm9rZUNvbG9yOiBjb2xvcixcbiAgICAgIHN0cm9rZU9wYWNpdHk6IG9wYWNpdHksXG4gICAgICBzdHJva2VXZWlnaHQ6IHdlaWdodCxcbiAgICB9O1xuICAgIGNvbnN0IHBvbHlsaW5lID0gbmV3IEJNYXAuUG9seWxpbmUocGF0aCwgb3B0aW9ucyk7XG4gICAgdGhpcy5tYXAuYWRkT3ZlcmxheShwb2x5bGluZSk7XG4gICAgcmV0dXJuIHBvbHlsaW5lO1xuICB9XG5cbiAgY3JlYXRlQ29udHJvbCh7XG4gICAgdGV4dCwgdGlwLCBjb2xvciwgb2Zmc2V0WCwgb2Zmc2V0WSwgb25DbGlja0hhbmRsZXIsXG4gIH0pIHtcbiAgICBjb25zdCBteUNvbnRyb2wgPSBuZXcgTmF2aWdhdGlvbkNvbnRyb2woXG4gICAgICB0ZXh0LFxuICAgICAgdGlwLFxuICAgICAgY29sb3IsXG4gICAgICBuZXcgQk1hcC5TaXplKG9mZnNldFgsIG9mZnNldFkpLFxuICAgICAgb25DbGlja0hhbmRsZXIsXG4gICAgKTtcbiAgICB0aGlzLm1hcC5hZGRDb250cm9sKG15Q29udHJvbCk7XG4gICAgdGhpcy5jb250cm9scy5wdXNoKG15Q29udHJvbCk7XG4gIH1cblxuICBkaXNhYmxlQ29udHJvbHMoKSB7XG4gICAgdGhpcy5jb250cm9scy5mb3JFYWNoKChjb250cm9sKSA9PiB7XG4gICAgICB0aGlzLm1hcC5yZW1vdmVDb250cm9sKGNvbnRyb2wpO1xuICAgIH0pO1xuICB9XG5cbiAgZW5hYmxlQ29udHJvbHMoKSB7XG4gICAgdGhpcy5jb250cm9scy5mb3JFYWNoKChjb250cm9sKSA9PiB7XG4gICAgICB0aGlzLm1hcC5hZGRDb250cm9sKGNvbnRyb2wpO1xuICAgIH0pO1xuICB9XG5cbiAgZ2V0TWFya2VyUG9zaXRpb24obWFya2VyKSB7XG4gICAgcmV0dXJuIG1hcmtlci5nZXRQb3NpdGlvbigpO1xuICB9XG5cbiAgdXBkYXRlUG9seWxpbmUocG9seWxpbmUsIG5ld1BhdGgpIHtcbiAgICBwb2x5bGluZS5zZXRQYXRoKG5ld1BhdGgpO1xuICB9XG5cbiAgcmVtb3ZlUG9seWxpbmUocG9seWxpbmUpIHtcbiAgICB0aGlzLm1hcC5yZW1vdmVPdmVybGF5KHBvbHlsaW5lKTtcbiAgfVxuXG4gIGFwcGx5Q29vcmRpbmF0ZU9mZnNldChbbG5nLCBsYXRdKSB7XG4gICAgcmV0dXJuIFdHUzg0VG9CRDA5TEwobG5nLCBsYXQpO1xuICB9XG59XG5cbmNsYXNzIE5hdmlnYXRpb25Db250cm9sIGV4dGVuZHMgQk1hcC5Db250cm9sIHtcbiAgY29uc3RydWN0b3IodGV4dCwgdGlwLCBjb2xvciwgb2Zmc2V0LCBvbkNsaWNrSGFuZGxlciwgLi4uYXJncykge1xuICAgIHN1cGVyKC4uLmFyZ3MpO1xuICAgIHRoaXMuZGVmYXVsdEFuY2hvciA9IEJNQVBfQU5DSE9SX1RPUF9MRUZUO1xuICAgIHRoaXMuZGVmYXVsdE9mZnNldCA9IG9mZnNldDtcbiAgICB0aGlzLm9uQ2xpY2tIYW5kbGVyID0gb25DbGlja0hhbmRsZXI7XG4gICAgdGhpcy50aXRsZSA9IHRpcDtcbiAgICB0aGlzLnRleHQgPSB0ZXh0O1xuICAgIHRoaXMuYmFja2dyb3VuZENvbG9yID0gY29sb3I7XG4gIH1cblxuICBpbml0aWFsaXplKG1hcCkge1xuICAgIGNvbnN0IGNvbnRyb2xEaXYgPSBkb2N1bWVudC5jcmVhdGVFbGVtZW50KCdkaXYnKTtcblxuICAgIC8vIFNldCBDU1MgZm9yIHRoZSBjb250cm9sIGJvcmRlci5cbiAgICBjb25zdCBjb250cm9sVUkgPSBkb2N1bWVudC5jcmVhdGVFbGVtZW50KCdkaXYnKTtcbiAgICBjb250cm9sVUkuc3R5bGUuYmFja2dyb3VuZENvbG9yID0gdGhpcy5iYWNrZ3JvdW5kQ29sb3I7XG4gICAgY29udHJvbFVJLnN0eWxlLmJvcmRlciA9ICcycHggc29saWQgI2ZmZic7XG4gICAgY29udHJvbFVJLnN0eWxlLmJvcmRlclJhZGl1cyA9ICczcHgnO1xuICAgIGNvbnRyb2xVSS5zdHlsZS5ib3hTaGFkb3cgPSAnMCAycHggNnB4IHJnYmEoMCwwLDAsLjMpJztcbiAgICBjb250cm9sVUkuc3R5bGUuY3Vyc29yID0gJ3BvaW50ZXInO1xuICAgIGNvbnRyb2xVSS5zdHlsZS5tYXJnaW5Cb3R0b20gPSAnMjJweCc7XG4gICAgY29udHJvbFVJLnN0eWxlLnRleHRBbGlnbiA9ICdjZW50ZXInO1xuICAgIGNvbnRyb2xVSS50aXRsZSA9IHRoaXMudGl0bGU7XG4gICAgY29udHJvbERpdi5hcHBlbmRDaGlsZChjb250cm9sVUkpO1xuXG4gICAgLy8gLy8gU2V0IENTUyBmb3IgdGhlIGNvbnRyb2wgaW50ZXJpb3IuXG4gICAgY29uc3QgY29udHJvbFRleHQgPSBkb2N1bWVudC5jcmVhdGVFbGVtZW50KCdkaXYnKTtcbiAgICBjb250cm9sVGV4dC5zdHlsZS5jb2xvciA9ICdyZ2IoMjUsMjUsMjUpJztcbiAgICBjb250cm9sVGV4dC5zdHlsZS5mb250RmFtaWx5ID0gJ1JvYm90byxBcmlhbCxzYW5zLXNlcmlmJztcbiAgICBjb250cm9sVGV4dC5zdHlsZS5mb250U2l6ZSA9ICcxNnB4JztcbiAgICBjb250cm9sVGV4dC5zdHlsZS5saW5lSGVpZ2h0ID0gJzM4cHgnO1xuICAgIGNvbnRyb2xUZXh0LnN0eWxlLnBhZGRpbmdMZWZ0ID0gJzVweCc7XG4gICAgY29udHJvbFRleHQuc3R5bGUucGFkZGluZ1JpZ2h0ID0gJzVweCc7XG4gICAgY29udHJvbFRleHQuaW5uZXJIVE1MID0gdGhpcy50ZXh0O1xuICAgIGNvbnRyb2xVSS5hcHBlbmRDaGlsZChjb250cm9sVGV4dCk7XG5cbiAgICBtYXAuZ2V0Q29udGFpbmVyKCkuYXBwZW5kQ2hpbGQoY29udHJvbERpdik7XG5cbiAgICBjb250cm9sVUkuYWRkRXZlbnRMaXN0ZW5lcignY2xpY2snLCAoKSA9PiB7XG4gICAgICB0aGlzLm9uQ2xpY2tIYW5kbGVyKGNvbnRyb2xUZXh0KTtcbiAgICB9KTtcblxuICAgIHJldHVybiBjb250cm9sRGl2O1xuICB9XG59XG4iXSwibmFtZXMiOlsiQmFpZHVNYXBBZGFwdGVyIiwibWFwIiwiY29udHJvbHMiLCJpbml0aWFsaXplZENlbnRlciIsIk9iamVjdCIsImtleXMiLCJsZW5ndGgiLCJpbml0UG9pbnQiLCJkaXZFbGVtZW50TmFtZSIsIkJNYXAiLCJNYXAiLCJlbmFibGVNYXBDbGljayIsImVuYWJsZVNjcm9sbFdoZWVsWm9vbSIsImFkZENvbnRyb2wiLCJNYXBUeXBlQ29udHJvbCIsImFuY2hvciIsIkJNQVBfQU5DSE9SX1RPUF9MRUZUIiwidHlwZSIsIkJNQVBfTkFWSUdBVElPTl9DT05UUk9MX1NNQUxMIiwiTmF2aWdhdGlvbkNvbnRyb2wiLCJCTUFQX0FOQ0hPUl9CT1RUT01fUklHSFQiLCJlbmFibGVHZW9sb2NhdGlvbiIsInBvaW50Iiwic2V0Q2VudGVyIiwiY2VudGVyQW5kWm9vbSIsInpvb20iLCJzZXRab29tIiwiZXZlbnROYW1lIiwiaGFuZGxlckZ1bmN0aW9uIiwiYWRkRXZlbnRMaXN0ZW5lciIsImV2ZW50IiwiY2xpY2tlZExhdExuZyIsImxhdCIsImxuZyIsIlBvaW50IiwidGl0bGUiLCJkcmFnZ2FibGUiLCJsYWJlbCIsIkxhYmVsIiwib2Zmc2V0IiwiU2l6ZSIsIm1hcmtlciIsIk1hcmtlciIsImVuYWJsZURyYWdnaW5nIiwicm90YXRpb24iLCJzZXRMYWJlbCIsImFkZE92ZXJsYXkiLCJwYXRoIiwiY29sb3IiLCJvcGFjaXR5Iiwid2VpZ2h0Iiwib3B0aW9ucyIsImdlb2Rlc2ljIiwic3Ryb2tlQ29sb3IiLCJzdHJva2VPcGFjaXR5Iiwic3Ryb2tlV2VpZ2h0IiwicG9seWxpbmUiLCJQb2x5bGluZSIsInRleHQiLCJ0aXAiLCJvZmZzZXRYIiwib2Zmc2V0WSIsIm9uQ2xpY2tIYW5kbGVyIiwibXlDb250cm9sIiwicHVzaCIsImZvckVhY2giLCJjb250cm9sIiwicmVtb3ZlQ29udHJvbCIsImdldFBvc2l0aW9uIiwibmV3UGF0aCIsInNldFBhdGgiLCJyZW1vdmVPdmVybGF5IiwiV0dTODRUb0JEMDlMTCIsImFyZ3MiLCJkZWZhdWx0QW5jaG9yIiwiZGVmYXVsdE9mZnNldCIsImJhY2tncm91bmRDb2xvciIsImNvbnRyb2xEaXYiLCJkb2N1bWVudCIsImNyZWF0ZUVsZW1lbnQiLCJjb250cm9sVUkiLCJzdHlsZSIsImJvcmRlciIsImJvcmRlclJhZGl1cyIsImJveFNoYWRvdyIsImN1cnNvciIsIm1hcmdpbkJvdHRvbSIsInRleHRBbGlnbiIsImFwcGVuZENoaWxkIiwiY29udHJvbFRleHQiLCJmb250RmFtaWx5IiwiZm9udFNpemUiLCJsaW5lSGVpZ2h0IiwicGFkZGluZ0xlZnQiLCJwYWRkaW5nUmlnaHQiLCJpbm5lckhUTUwiLCJnZXRDb250YWluZXIiLCJDb250cm9sIl0sInNvdXJjZVJvb3QiOiIifQ==