(self["webpackChunk"] = self["webpackChunk"] || []).push([["components_Navigation_index_js"],{

/***/ "./components/Navigation/WindowResizeControl.js":
/*!******************************************************!*\
  !*** ./components/Navigation/WindowResizeControl.js ***!
  \******************************************************/
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

var _react = _interopRequireDefault(__webpack_require__(/*! react */ "../node_modules/react/index.js"));

var _dimension = __webpack_require__(/*! store/dimension */ "./store/dimension.js");

function _interopRequireDefault(obj) { return obj && obj.__esModule ? obj : { "default": obj }; }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } }

function _createClass(Constructor, protoProps, staticProps) { if (protoProps) _defineProperties(Constructor.prototype, protoProps); if (staticProps) _defineProperties(Constructor, staticProps); Object.defineProperty(Constructor, "prototype", { writable: false }); return Constructor; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function"); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, writable: true, configurable: true } }); Object.defineProperty(subClass, "prototype", { writable: false }); if (superClass) _setPrototypeOf(subClass, superClass); }

function _setPrototypeOf(o, p) { _setPrototypeOf = Object.setPrototypeOf ? Object.setPrototypeOf.bind() : function _setPrototypeOf(o, p) { o.__proto__ = p; return o; }; return _setPrototypeOf(o, p); }

function _createSuper(Derived) { var hasNativeReflectConstruct = _isNativeReflectConstruct(); return function _createSuperInternal() { var Super = _getPrototypeOf(Derived), result; if (hasNativeReflectConstruct) { var NewTarget = _getPrototypeOf(this).constructor; result = Reflect.construct(Super, arguments, NewTarget); } else { result = Super.apply(this, arguments); } return _possibleConstructorReturn(this, result); }; }

function _possibleConstructorReturn(self, call) { if (call && (_typeof(call) === "object" || typeof call === "function")) { return call; } else if (call !== void 0) { throw new TypeError("Derived constructors may only return object or undefined"); } return _assertThisInitialized(self); }

function _assertThisInitialized(self) { if (self === void 0) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return self; }

function _isNativeReflectConstruct() { if (typeof Reflect === "undefined" || !Reflect.construct) return false; if (Reflect.construct.sham) return false; if (typeof Proxy === "function") return true; try { Boolean.prototype.valueOf.call(Reflect.construct(Boolean, [], function () {})); return true; } catch (e) { return false; } }

function _getPrototypeOf(o) { _getPrototypeOf = Object.setPrototypeOf ? Object.getPrototypeOf.bind() : function _getPrototypeOf(o) { return o.__proto__ || Object.getPrototypeOf(o); }; return _getPrototypeOf(o); }

var WindowResizeControl = /*#__PURE__*/function (_React$PureComponent) {
  _inherits(WindowResizeControl, _React$PureComponent);

  var _super = _createSuper(WindowResizeControl);

  function WindowResizeControl() {
    _classCallCheck(this, WindowResizeControl);

    return _super.apply(this, arguments);
  }

  _createClass(WindowResizeControl, [{
    key: "getMinimizingIcon",
    value: function getMinimizingIcon() {
      return /*#__PURE__*/_react["default"].createElement("svg", {
        viewBox: "0 0 20 20"
      }, /*#__PURE__*/_react["default"].createElement("defs", null, /*#__PURE__*/_react["default"].createElement("path", {
        d: "M20 0L0 20h20V0z",
        id: "a"
      }), /*#__PURE__*/_react["default"].createElement("path", {
        d: "M11.53 18.5l-.03-7h7",
        id: "b"
      }), /*#__PURE__*/_react["default"].createElement("path", {
        d: "M12 12l7 7",
        id: "c"
      })), /*#__PURE__*/_react["default"].createElement("use", {
        xlinkHref: "#a",
        opacity: ".8",
        fill: "#84b7FF"
      }), /*#__PURE__*/_react["default"].createElement("use", {
        xlinkHref: "#b",
        fillOpacity: "0",
        stroke: "#006AFF",
        strokeWidth: "2"
      }), /*#__PURE__*/_react["default"].createElement("use", {
        xlinkHref: "#c",
        fillOpacity: "0",
        stroke: "#006AFF",
        strokeWidth: "2"
      }));
    }
  }, {
    key: "getMaximizingIcon",
    value: function getMaximizingIcon() {
      return /*#__PURE__*/_react["default"].createElement("svg", {
        viewBox: "0 0 20 20"
      }, /*#__PURE__*/_react["default"].createElement("defs", null, /*#__PURE__*/_react["default"].createElement("path", {
        d: "M20 0L0 20h20V0z",
        id: "a"
      }), /*#__PURE__*/_react["default"].createElement("path", {
        d: "M18.47 11.5l.03 7h-7",
        id: "b"
      }), /*#__PURE__*/_react["default"].createElement("path", {
        d: "M11 11l7 7",
        id: "c"
      })), /*#__PURE__*/_react["default"].createElement("use", {
        xlinkHref: "#a",
        opacity: ".8",
        fill: "#84b7FF"
      }), /*#__PURE__*/_react["default"].createElement("use", {
        xlinkHref: "#b",
        fillOpacity: "0",
        stroke: "#006AFF",
        strokeWidth: "2"
      }), /*#__PURE__*/_react["default"].createElement("use", {
        xlinkHref: "#c",
        fillOpacity: "0",
        stroke: "#006AFF",
        strokeWidth: "2"
      }));
    }
  }, {
    key: "render",
    value: function render() {
      var _this$props = this.props,
          type = _this$props.type,
          onClick = _this$props.onClick;
      var icon = null;

      switch (type) {
        case _dimension.MAP_SIZE.FULL:
          icon = this.getMinimizingIcon();
          break;

        case _dimension.MAP_SIZE.DEFAULT:
          icon = this.getMaximizingIcon();
          break;

        default:
          console.error('Unknown window size found:', type);
          break;
      }

      return /*#__PURE__*/_react["default"].createElement("div", {
        className: "window-resize-control",
        onClick: onClick
      }, icon);
    }
  }]);

  return WindowResizeControl;
}(_react["default"].PureComponent);

exports["default"] = WindowResizeControl;

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

/***/ }),

/***/ "./components/Navigation/index.js":
/*!****************************************!*\
  !*** ./components/Navigation/index.js ***!
  \****************************************/
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

var _react = _interopRequireDefault(__webpack_require__(/*! react */ "../node_modules/react/index.js"));

var _MapNavigator = _interopRequireDefault(__webpack_require__(/*! components/Navigation/MapNavigator */ "./components/Navigation/MapNavigator.js"));

var _websocket = _interopRequireDefault(__webpack_require__(/*! store/websocket */ "./store/websocket/index.js"));

var _dimension = __webpack_require__(/*! store/dimension */ "./store/dimension.js");

var _script_loader = _interopRequireDefault(__webpack_require__(/*! utils/script_loader */ "./utils/script_loader.js"));

var _WindowResizeControl = _interopRequireDefault(__webpack_require__(/*! components/Navigation/WindowResizeControl */ "./components/Navigation/WindowResizeControl.js"));

function _interopRequireDefault(obj) { return obj && obj.__esModule ? obj : { "default": obj }; }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } }

function _createClass(Constructor, protoProps, staticProps) { if (protoProps) _defineProperties(Constructor.prototype, protoProps); if (staticProps) _defineProperties(Constructor, staticProps); Object.defineProperty(Constructor, "prototype", { writable: false }); return Constructor; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function"); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, writable: true, configurable: true } }); Object.defineProperty(subClass, "prototype", { writable: false }); if (superClass) _setPrototypeOf(subClass, superClass); }

function _setPrototypeOf(o, p) { _setPrototypeOf = Object.setPrototypeOf ? Object.setPrototypeOf.bind() : function _setPrototypeOf(o, p) { o.__proto__ = p; return o; }; return _setPrototypeOf(o, p); }

function _createSuper(Derived) { var hasNativeReflectConstruct = _isNativeReflectConstruct(); return function _createSuperInternal() { var Super = _getPrototypeOf(Derived), result; if (hasNativeReflectConstruct) { var NewTarget = _getPrototypeOf(this).constructor; result = Reflect.construct(Super, arguments, NewTarget); } else { result = Super.apply(this, arguments); } return _possibleConstructorReturn(this, result); }; }

function _possibleConstructorReturn(self, call) { if (call && (_typeof(call) === "object" || typeof call === "function")) { return call; } else if (call !== void 0) { throw new TypeError("Derived constructors may only return object or undefined"); } return _assertThisInitialized(self); }

function _assertThisInitialized(self) { if (self === void 0) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return self; }

function _isNativeReflectConstruct() { if (typeof Reflect === "undefined" || !Reflect.construct) return false; if (Reflect.construct.sham) return false; if (typeof Proxy === "function") return true; try { Boolean.prototype.valueOf.call(Reflect.construct(Boolean, [], function () {})); return true; } catch (e) { return false; } }

function _getPrototypeOf(o) { _getPrototypeOf = Object.setPrototypeOf ? Object.getPrototypeOf.bind() : function _getPrototypeOf(o) { return o.__proto__ || Object.getPrototypeOf(o); }; return _getPrototypeOf(o); }

var Navigation = /*#__PURE__*/function (_React$Component) {
  _inherits(Navigation, _React$Component);

  var _super = _createSuper(Navigation);

  function Navigation(props) {
    var _this;

    _classCallCheck(this, Navigation);

    _this = _super.call(this, props);
    _this.scriptOnLoadHandler = _this.scriptOnLoadHandler.bind(_assertThisInitialized(_this));

    if (!_MapNavigator["default"].mapAPILoaded) {
      var onLoad = function onLoad() {
        console.log('Map API script loaded.');
      };

      if (PARAMETERS.navigation.map === 'BaiduMap') {
        // For Baidu Map, the callback function is set in the window Object level
        window.initMap = _this.scriptOnLoadHandler;
      } else if (PARAMETERS.navigation.map === 'GoogleMap') {
        // For Google Map, the callback function is set from the <Script>
        onLoad = _this.scriptOnLoadHandler;
      }

      (0, _script_loader["default"])({
        url: PARAMETERS.navigation.mapAPiUrl,
        onLoad: onLoad,
        onError: function onError() {
          console.log('Failed to load map api');
        }
      });
    }

    return _this;
  }

  _createClass(Navigation, [{
    key: "componentDidMount",
    value: function componentDidMount() {
      if (_MapNavigator["default"].mapAPILoaded) {
        this.scriptOnLoadHandler();
      }
    }
  }, {
    key: "componentDidUpdate",
    value: function componentDidUpdate() {
      var _this$props = this.props,
          hasRoutingControls = _this$props.hasRoutingControls,
          size = _this$props.size;

      if (hasRoutingControls && size === _dimension.MAP_SIZE.FULL) {
        _MapNavigator["default"].enableControls();
      } else {
        _MapNavigator["default"].disableControls();
      }
    }
  }, {
    key: "scriptOnLoadHandler",
    value: function scriptOnLoadHandler() {
      __webpack_require__("./components/Navigation lazy recursive ^\\.\\/.*Adapter$")("./".concat(PARAMETERS.navigation.map, "Adapter")).then(function (mapAdapterModule) {
        var MapAdapterClass = mapAdapterModule["default"];
        var mapAdapter = new MapAdapterClass();
        _MapNavigator["default"].mapAPILoaded = true;

        _MapNavigator["default"].initialize(_websocket["default"], mapAdapter);

        _MapNavigator["default"].disableControls();
      });
    }
  }, {
    key: "componentWillUnmount",
    value: function componentWillUnmount() {
      _MapNavigator["default"].reset();
    }
  }, {
    key: "render",
    value: function render() {
      var _this$props2 = this.props,
          width = _this$props2.width,
          height = _this$props2.height,
          size = _this$props2.size,
          onResize = _this$props2.onResize;

      if (!['GoogleMap', 'BaiduMap'].includes(PARAMETERS.navigation.map)) {
        console.error("Map API ".concat(PARAMETERS.navigation.map, " is not supported."));
        return null;
      }

      return /*#__PURE__*/_react["default"].createElement("div", {
        displayname: "navigation",
        className: "navigation-view",
        style: {
          width: width,
          height: height
        }
      }, /*#__PURE__*/_react["default"].createElement("div", {
        id: "map_canvas"
      }), /*#__PURE__*/_react["default"].createElement(_WindowResizeControl["default"], {
        type: size,
        onClick: onResize
      }));
    }
  }]);

  return Navigation;
}(_react["default"].Component);

exports["default"] = Navigation;

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

/***/ }),

/***/ "./utils/script_loader.js":
/*!********************************!*\
  !*** ./utils/script_loader.js ***!
  \********************************/
/***/ ((module, exports, __webpack_require__) => {

/* provided dependency */ var __react_refresh_utils__ = __webpack_require__(/*! ../node_modules/@pmmmwh/react-refresh-webpack-plugin/lib/runtime/RefreshUtils.js */ "../node_modules/@pmmmwh/react-refresh-webpack-plugin/lib/runtime/RefreshUtils.js");
/* provided dependency */ var __react_refresh_error_overlay__ = __webpack_require__(/*! ../node_modules/@pmmmwh/react-refresh-webpack-plugin/overlay/index.js */ "../node_modules/@pmmmwh/react-refresh-webpack-plugin/overlay/index.js");
__webpack_require__.$Refresh$.runtime = __webpack_require__(/*! ../node_modules/react-refresh/runtime.js */ "../node_modules/react-refresh/runtime.js");

"use strict";

Object.defineProperty(exports, "__esModule", ({
  value: true
}));
exports["default"] = loadScriptAsync;

function loadScriptAsync(_ref) {
  var url = _ref.url,
      onLoad = _ref.onLoad,
      onError = _ref.onError;
  var script = document.createElement('script');
  script.src = url;
  script.type = 'text/javascript';
  script.async = true;
  script.onload = onLoad;
  script.onerror = onError;
  document.body.appendChild(script);
}

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

/***/ }),

/***/ "./components/Navigation lazy recursive ^\\.\\/.*Adapter$":
/*!**********************************************************************!*\
  !*** ./components/Navigation/ lazy ^\.\/.*Adapter$ namespace object ***!
  \**********************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

var map = {
	"./BaiduMapAdapter": [
		"./components/Navigation/BaiduMapAdapter.js",
		"components_Navigation_BaiduMapAdapter_js"
	],
	"./GoogleMapAdapter": [
		"./components/Navigation/GoogleMapAdapter.js",
		"components_Navigation_GoogleMapAdapter_js"
	]
};
function webpackAsyncContext(req) {
	if(!__webpack_require__.o(map, req)) {
		return Promise.resolve().then(() => {
			var e = new Error("Cannot find module '" + req + "'");
			e.code = 'MODULE_NOT_FOUND';
			throw e;
		});
	}

	var ids = map[req], id = ids[0];
	return __webpack_require__.e(ids[1]).then(() => {
		return __webpack_require__(id);
	});
}
webpackAsyncContext.keys = () => (Object.keys(map));
webpackAsyncContext.id = "./components/Navigation lazy recursive ^\\.\\/.*Adapter$";
module.exports = webpackAsyncContext;

/***/ })

}]);
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiLi9jb21wb25lbnRzX05hdmlnYXRpb25faW5kZXhfanMuYnVuZGxlLmpzIiwibWFwcGluZ3MiOiI7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7OztBQUFBOztBQUVBOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7SUFFcUJBOzs7Ozs7Ozs7Ozs7O1dBQ25CLDZCQUFvQjtNQUNsQixvQkFDUTtRQUFLLE9BQU8sRUFBQztNQUFiLGdCQUNJLDJEQUNJO1FBQU0sQ0FBQyxFQUFDLGtCQUFSO1FBQTJCLEVBQUUsRUFBQztNQUE5QixFQURKLGVBRUk7UUFBTSxDQUFDLEVBQUMsc0JBQVI7UUFBK0IsRUFBRSxFQUFDO01BQWxDLEVBRkosZUFHSTtRQUFNLENBQUMsRUFBQyxZQUFSO1FBQXFCLEVBQUUsRUFBQztNQUF4QixFQUhKLENBREosZUFNSTtRQUFLLFNBQVMsRUFBQyxJQUFmO1FBQW9CLE9BQU8sRUFBQyxJQUE1QjtRQUFpQyxJQUFJLEVBQUM7TUFBdEMsRUFOSixlQU9JO1FBQUssU0FBUyxFQUFDLElBQWY7UUFBb0IsV0FBVyxFQUFDLEdBQWhDO1FBQW9DLE1BQU0sRUFBQyxTQUEzQztRQUFxRCxXQUFXLEVBQUM7TUFBakUsRUFQSixlQVFJO1FBQUssU0FBUyxFQUFDLElBQWY7UUFBb0IsV0FBVyxFQUFDLEdBQWhDO1FBQW9DLE1BQU0sRUFBQyxTQUEzQztRQUFxRCxXQUFXLEVBQUM7TUFBakUsRUFSSixDQURSO0lBWUQ7OztXQUVELDZCQUFvQjtNQUNsQixvQkFDUTtRQUFLLE9BQU8sRUFBQztNQUFiLGdCQUNJLDJEQUNJO1FBQU0sQ0FBQyxFQUFDLGtCQUFSO1FBQTJCLEVBQUUsRUFBQztNQUE5QixFQURKLGVBRUk7UUFBTSxDQUFDLEVBQUMsc0JBQVI7UUFBK0IsRUFBRSxFQUFDO01BQWxDLEVBRkosZUFHSTtRQUFNLENBQUMsRUFBQyxZQUFSO1FBQXFCLEVBQUUsRUFBQztNQUF4QixFQUhKLENBREosZUFNSTtRQUFLLFNBQVMsRUFBQyxJQUFmO1FBQW9CLE9BQU8sRUFBQyxJ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Ozs7Ozs7Ozs7QUNUL0IsU0FBU25CLGVBQVQsT0FBbUQ7RUFBQSxJQUF4QkMsR0FBd0IsUUFBeEJBLEdBQXdCO0VBQUEsSUFBbkJSLE1BQW1CLFFBQW5CQSxNQUFtQjtFQUFBLElBQVhVLE9BQVcsUUFBWEEsT0FBVztFQUNoRSxJQUFNaUIsTUFBTSxHQUFHQyxRQUFRLENBQUNDLGFBQVQsQ0FBdUIsUUFBdkIsQ0FBZjtFQUNBRixNQUFNLENBQUNHLEdBQVAsR0FBYXRCLEdBQWI7RUFDQW1CLE1BQU0sQ0FBQzVDLElBQVAsR0FBYyxpQkFBZDtFQUNBNEMsTUFBTSxDQUFDSSxLQUFQLEdBQWUsSUFBZjtFQUNBSixNQUFNLENBQUNLLE1BQVAsR0FBZ0JoQyxNQUFoQjtFQUNBMkIsTUFBTSxDQUFDTSxPQUFQLEdBQWlCdkIsT0FBakI7RUFDQWtCLFFBQVEsQ0FBQ00sSUFBVCxDQUFjQyxXQUFkLENBQTBCUixNQUExQjtBQUNEOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7OztBQ1JEO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EsR0FBRztBQUNIOztBQUVBO0FBQ0E7QUFDQTtBQUNBLEVBQUU7QUFDRjtBQUNBO0FBQ0E7QUFDQSIsInNvdXJjZXMiOlsid2VicGFjazovLy8uL2NvbXBvbmVudHMvTmF2aWdhdGlvbi9XaW5kb3dSZXNpemVDb250cm9sLmpzIiwid2VicGFjazovLy8uL2NvbXBvbmVudHMvTmF2aWdhdGlvbi9pbmRleC5qcyIsIndlYnBhY2s6Ly8vLi91dGlscy9zY3JpcHRfbG9hZGVyLmpzIiwid2VicGFjazovLy8uL2NvbXBvbmVudHMvTmF2aWdhdGlvbi8gbGF6eSBeXFwuXFwvLipBZGFwdGVyJCBuYW1lc3BhY2Ugb2JqZWN0Il0sInNvdXJjZXNDb250ZW50IjpbImltcG9ydCBSZWFjdCBmcm9tICdyZWFjdCc7XG5cbmltcG9ydCB7IE1BUF9TSVpFIH0gZnJvbSAnc3RvcmUvZGltZW5zaW9uJztcblxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgV2luZG93UmVzaXplQ29udHJvbCBleHRlbmRzIFJlYWN0LlB1cmVDb21wb25lbnQge1xuICBnZXRNaW5pbWl6aW5nSWNvbigpIHtcbiAgICByZXR1cm4gKFxuICAgICAgICAgICAgPHN2ZyB2aWV3Qm94PVwiMCAwIDIwIDIwXCI+XG4gICAgICAgICAgICAgICAgPGRlZnM+XG4gICAgICAgICAgICAgICAgICAgIDxwYXRoIGQ9XCJNMjAgMEwwIDIwaDIwVjB6XCIgaWQ9XCJhXCIgLz5cbiAgICAgICAgICAgICAgICAgICAgPHBhdGggZD1cIk0xMS41MyAxOC41bC0uMDMtN2g3XCIgaWQ9XCJiXCIgLz5cbiAgICAgICAgICAgICAgICAgICAgPHBhdGggZD1cIk0xMiAxMmw3IDdcIiBpZD1cImNcIiAvPlxuICAgICAgICAgICAgICAgIDwvZGVmcz5cbiAgICAgICAgICAgICAgICA8dXNlIHhsaW5rSHJlZj1cIiNhXCIgb3BhY2l0eT1cIi44XCIgZmlsbD1cIiM4NGI3RkZcIiAvPlxuICAgICAgICAgICAgICAgIDx1c2UgeGxpbmtIcmVmPVwiI2JcIiBmaWxsT3BhY2l0eT1cIjBcIiBzdHJva2U9XCIjMDA2QUZGXCIgc3Ryb2tlV2lkdGg9XCIyXCIgLz5cbiAgICAgICAgICAgICAgICA8dXNlIHhsaW5rSHJlZj1cIiNjXCIgZmlsbE9wYWNpdHk9XCIwXCIgc3Ryb2tlPVwiIzAwNkFGRlwiIHN0cm9rZVdpZHRoPVwiMlwiIC8+XG4gICAgICAgICAgICA8L3N2Zz5cbiAgICApO1xuICB9XG5cbiAgZ2V0TWF4aW1pemluZ0ljb24oKSB7XG4gICAgcmV0dXJuIChcbiAgICAgICAgICAgIDxzdmcgdmlld0JveD1cIjAgMCAyMCAyMFwiPlxuICAgICAgICAgICAgICAgIDxkZWZzPlxuICAgICAgICAgICAgICAgICAgICA8cGF0aCBkPVwiTTIwIDBMMCAyMGgyMFYwelwiIGlkPVwiYVwiIC8+XG4gICAgICAgICAgICAgICAgICAgIDxwYXRoIGQ9XCJNMTguNDcgMTEuNWwuMDMgN2gtN1wiIGlkPVwiYlwiIC8+XG4gICAgICAgICAgICAgICAgICAgIDxwYXRoIGQ9XCJNMTEgMTFsNyA3XCIgaWQ9XCJjXCIgLz5cbiAgICAgICAgICAgICAgICA8L2RlZnM+XG4gICAgICAgICAgICAgICAgPHVzZSB4bGlua0hyZWY9XCIjYVwiIG9wYWNpdHk9XCIuOFwiIGZpbGw9XCIjODRiN0ZGXCIgLz5cbiAgICAgICAgICAgICAgICA8dXNlIHhsaW5rSHJlZj1cIiNiXCIgZmlsbE9wYWNpdHk9XCIwXCIgc3Ryb2tlPVwiIzAwNkFGRlwiIHN0cm9rZVdpZHRoPVwiMlwiIC8+XG4gICAgICAgICAgICAgICAgPHVzZSB4bGlua0hyZWY9XCIjY1wiIGZpbGxPcGFjaXR5PVwiMFwiIHN0cm9rZT1cIiMwMDZBRkZcIiBzdHJva2VXaWR0aD1cIjJcIiAvPlxuICAgICAgICAgICAgPC9zdmc+XG4gICAgKTtcbiAgfVxuXG4gIHJlbmRlcigpIHtcbiAgICBjb25zdCB7IHR5cGUsIG9uQ2xpY2sgfSA9IHRoaXMucHJvcHM7XG5cbiAgICBsZXQgaWNvbiA9IG51bGw7XG4gICAgc3dpdGNoICh0eXBlKSB7XG4gICAgICBjYXNlIE1BUF9TSVpFLkZVTEw6XG4gICAgICAgIGljb24gPSB0aGlzLmdldE1pbmltaXppbmdJY29uKCk7XG4gICAgICAgIGJyZWFrO1xuICAgICAgY2FzZSBNQVBfU0laRS5ERUZBVUxUOlxuICAgICAgICBpY29uID0gdGhpcy5nZXRNYXhpbWl6aW5nSWNvbigpO1xuICAgICAgICBicmVhaztcbiAgICAgIGRlZmF1bHQ6XG4gICAgICAgIGNvbnNvbGUuZXJyb3IoJ1Vua25vd24gd2luZG93IHNpemUgZm91bmQ6JywgdHlwZSk7XG4gICAgICAgIGJyZWFrO1xuICAgIH1cblxuICAgIHJldHVybiAoXG4gICAgICAgICAgICA8ZGl2IGNsYXNzTmFtZT1cIndpbmRvdy1yZXNpemUtY29udHJvbFwiIG9uQ2xpY2s9e29uQ2xpY2t9PlxuICAgICAgICAgICAgICAgIHtpY29ufVxuICAgICAgICAgICAgPC9kaXY+XG4gICAgKTtcbiAgfVxufVxuIiwiaW1wb3J0IFJlYWN0IGZyb20gJ3JlYWN0JztcblxuaW1wb3J0IE1BUF9OQVZJR0FUT1IgZnJvbSAnY29tcG9uZW50cy9OYXZpZ2F0aW9uL01hcE5hdmlnYXRvcic7XG5pbXBvcnQgV1MgZnJvbSAnc3RvcmUvd2Vic29ja2V0JztcbmltcG9ydCB7IE1BUF9TSVpFIH0gZnJvbSAnc3RvcmUvZGltZW5zaW9uJztcbmltcG9ydCBsb2FkU2NyaXB0QXN5bmMgZnJvbSAndXRpbHMvc2NyaXB0X2xvYWRlcic7XG5cbmltcG9ydCBXaW5kb3dSZXNpemVDb250cm9sIGZyb20gJ2NvbXBvbmVudHMvTmF2aWdhdGlvbi9XaW5kb3dSZXNpemVDb250cm9sJztcblxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgTmF2aWdhdGlvbiBleHRlbmRzIFJlYWN0LkNvbXBvbmVudCB7XG4gIGNvbnN0cnVjdG9yKHByb3BzKSB7XG4gICAgc3VwZXIocHJvcHMpO1xuXG4gICAgdGhpcy5zY3JpcHRPbkxvYWRIYW5kbGVyID0gdGhpcy5zY3JpcHRPbkxvYWRIYW5kbGVyLmJpbmQodGhpcyk7XG5cbiAgICBpZiAoIU1BUF9OQVZJR0FUT1IubWFwQVBJTG9hZGVkKSB7XG4gICAgICBsZXQgb25Mb2FkID0gKCkgPT4ge1xuICAgICAgICBjb25zb2xlLmxvZygnTWFwIEFQSSBzY3JpcHQgbG9hZGVkLicpO1xuICAgICAgfTtcbiAgICAgIGlmIChQQVJBTUVURVJTLm5hdmlnYXRpb24ubWFwID09PSAnQmFpZHVNYXAnKSB7XG4gICAgICAgIC8vIEZvciBCYWlkdSBNYXAsIHRoZSBjYWxsYmFjayBmdW5jdGlvbiBpcyBzZXQgaW4gdGhlIHdpbmRvdyBPYmplY3QgbGV2ZWxcbiAgICAgICAgd2luZG93LmluaXRNYXAgPSB0aGlzLnNjcmlwdE9uTG9hZEhhbmRsZXI7XG4gICAgICB9IGVsc2UgaWYgKFBBUkFNRVRFUlMubmF2aWdhdGlvbi5tYXAgPT09ICdHb29nbGVNYXAnKSB7XG4gICAgICAgIC8vIEZvciBHb29nbGUgTWFwLCB0aGUgY2FsbGJhY2sgZnVuY3Rpb24gaXMgc2V0IGZyb20gdGhlIDxTY3JpcHQ+XG4gICAgICAgIG9uTG9hZCA9IHRoaXMuc2NyaXB0T25Mb2FkSGFuZGxlcjtcbiAgICAgIH1cblxuICAgICAgbG9hZFNjcmlwdEFzeW5jKHtcbiAgICAgICAgdXJsOiBQQVJBTUVURVJTLm5hdmlnYXRpb24ubWFwQVBpVXJsLFxuICAgICAgICBvbkxvYWQsXG4gICAgICAgIG9uRXJyb3I6ICgpID0+IHtcbiAgICAgICAgICBjb25zb2xlLmxvZygnRmFpbGVkIHRvIGxvYWQgbWFwIGFwaScpO1xuICAgICAgICB9LFxuICAgICAgfSk7XG4gICAgfVxuICB9XG5cbiAgY29tcG9uZW50RGlkTW91bnQoKSB7XG4gICAgaWYgKE1BUF9OQVZJR0FUT1IubWFwQVBJTG9hZGVkKSB7XG4gICAgICB0aGlzLnNjcmlwdE9uTG9hZEhhbmRsZXIoKTtcbiAgICB9XG4gIH1cblxuICBjb21wb25lbnREaWRVcGRhdGUoKSB7XG4gICAgY29uc3QgeyBoYXNSb3V0aW5nQ29udHJvbHMsIHNpemUgfSA9IHRoaXMucHJvcHM7XG5cbiAgICBpZiAoaGFzUm91dGluZ0NvbnRyb2xzICYmIHNpemUgPT09IE1BUF9TSVpFLkZVTEwpIHtcbiAgICAgIE1BUF9OQVZJR0FUT1IuZW5hYmxlQ29udHJvbHMoKTtcbiAgICB9IGVsc2Uge1xuICAgICAgTUFQX05BVklHQVRPUi5kaXNhYmxlQ29udHJvbHMoKTtcbiAgICB9XG4gIH1cblxuICBzY3JpcHRPbkxvYWRIYW5kbGVyKCkge1xuICAgIGltcG9ydChgY29tcG9uZW50cy9OYXZpZ2F0aW9uLyR7UEFSQU1FVEVSUy5uYXZpZ2F0aW9uLm1hcH1BZGFwdGVyYCkudGhlbihcbiAgICAgIChtYXBBZGFwdGVyTW9kdWxlKSA9PiB7XG4gICAgICAgIGNvbnN0IE1hcEFkYXB0ZXJDbGFzcyA9IG1hcEFkYXB0ZXJNb2R1bGUuZGVmYXVsdDtcbiAgICAgICAgY29uc3QgbWFwQWRhcHRlciA9IG5ldyBNYXBBZGFwdGVyQ2xhc3MoKTtcbiAgICAgICAgTUFQX05BVklHQVRPUi5tYXBBUElMb2FkZWQgPSB0cnVlO1xuICAgICAgICBNQVBfTkFWSUdBVE9SLmluaXRpYWxpemUoV1MsIG1hcEFkYXB0ZXIpO1xuICAgICAgICBNQVBfTkFWSUdBVE9SLmRpc2FibGVDb250cm9scygpO1xuICAgICAgfSxcbiAgICApO1xuICB9XG5cbiAgY29tcG9uZW50V2lsbFVubW91bnQoKSB7XG4gICAgTUFQX05BVklHQVRPUi5yZXNldCgpO1xuICB9XG5cbiAgcmVuZGVyKCkge1xuICAgIGNvbnN0IHtcbiAgICAgIHdpZHRoLCBoZWlnaHQsIHNpemUsIG9uUmVzaXplLFxuICAgIH0gPSB0aGlzLnByb3BzO1xuXG4gICAgaWYgKCFbJ0dvb2dsZU1hcCcsICdCYWlkdU1hcCddLmluY2x1ZGVzKFBBUkFNRVRFUlMubmF2aWdhdGlvbi5tYXApKSB7XG4gICAgICBjb25zb2xlLmVycm9yKGBNYXAgQVBJICR7UEFSQU1FVEVSUy5uYXZpZ2F0aW9uLm1hcH0gaXMgbm90IHN1cHBvcnRlZC5gKTtcbiAgICAgIHJldHVybiBudWxsO1xuICAgIH1cblxuICAgIHJldHVybiAoXG4gICAgICAgICAgICA8ZGl2IGRpc3BsYXluYW1lPVwibmF2aWdhdGlvblwiIGNsYXNzTmFtZT1cIm5hdmlnYXRpb24tdmlld1wiIHN0eWxlPXt7IHdpZHRoLCBoZWlnaHQgfX0+XG4gICAgICAgICAgICAgICAgPGRpdiBpZD1cIm1hcF9jYW52YXNcIiAvPlxuICAgICAgICAgICAgICAgIDxXaW5kb3dSZXNpemVDb250cm9sIHR5cGU9e3NpemV9IG9uQ2xpY2s9e29uUmVzaXplfSAvPlxuICAgICAgICAgICAgPC9kaXY+XG4gICAgKTtcbiAgfVxufVxuIiwiZXhwb3J0IGRlZmF1bHQgZnVuY3Rpb24gbG9hZFNjcmlwdEFzeW5jKHsgdXJsLCBvbkxvYWQsIG9uRXJyb3IgfSkge1xuICBjb25zdCBzY3JpcHQgPSBkb2N1bWVudC5jcmVhdGVFbGVtZW50KCdzY3JpcHQnKTtcbiAgc2NyaXB0LnNyYyA9IHVybDtcbiAgc2NyaXB0LnR5cGUgPSAndGV4dC9qYXZhc2NyaXB0JztcbiAgc2NyaXB0LmFzeW5jID0gdHJ1ZTtcbiAgc2NyaXB0Lm9ubG9hZCA9IG9uTG9hZDtcbiAgc2NyaXB0Lm9uZXJyb3IgPSBvbkVycm9yO1xuICBkb2N1bWVudC5ib2R5LmFwcGVuZENoaWxkKHNjcmlwdCk7XG59XG4iLCJ2YXIgbWFwID0ge1xuXHRcIi4vQmFpZHVNYXBBZGFwdGVyXCI6IFtcblx0XHRcIi4vY29tcG9uZW50cy9OYXZpZ2F0aW9uL0JhaWR1TWFwQWRhcHRlci5qc1wiLFxuXHRcdFwiY29tcG9uZW50c19OYXZpZ2F0aW9uX0JhaWR1TWFwQWRhcHRlcl9qc1wiXG5cdF0sXG5cdFwiLi9Hb29nbGVNYXBBZGFwdGVyXCI6IFtcblx0XHRcIi4vY29tcG9uZW50cy9OYXZpZ2F0aW9uL0dvb2dsZU1hcEFkYXB0ZXIuanNcIixcblx0XHRcImNvbXBvbmVudHNfTmF2aWdhdGlvbl9Hb29nbGVNYXBBZGFwdGVyX2pzXCJcblx0XVxufTtcbmZ1bmN0aW9uIHdlYnBhY2tBc3luY0NvbnRleHQocmVxKSB7XG5cdGlmKCFfX3dlYnBhY2tfcmVxdWlyZV9fLm8obWFwLCByZXEpKSB7XG5cdFx0cmV0dXJuIFByb21pc2UucmVzb2x2ZSgpLnRoZW4oKCkgPT4ge1xuXHRcdFx0dmFyIGUgPSBuZXcgRXJyb3IoXCJDYW5ub3QgZmluZCBtb2R1bGUgJ1wiICsgcmVxICsgXCInXCIpO1xuXHRcdFx0ZS5jb2RlID0gJ01PRFVMRV9OT1RfRk9VTkQnO1xuXHRcdFx0dGhyb3cgZTtcblx0XHR9KTtcblx0fVxuXG5cdHZhciBpZHMgPSBtYXBbcmVxXSwgaWQgPSBpZHNbMF07XG5cdHJldHVybiBfX3dlYnBhY2tfcmVxdWlyZV9fLmUoaWRzWzFdKS50aGVuKCgpID0+IHtcblx0XHRyZXR1cm4gX193ZWJwYWNrX3JlcXVpcmVfXyhpZCk7XG5cdH0pO1xufVxud2VicGFja0FzeW5jQ29udGV4dC5rZXlzID0gKCkgPT4gKE9iamVjdC5rZXlzKG1hcCkpO1xud2VicGFja0FzeW5jQ29udGV4dC5pZCA9IFwiLi9jb21wb25lbnRzL05hdmlnYXRpb24gbGF6eSByZWN1cnNpdmUgXlxcXFwuXFxcXC8uKkFkYXB0ZXIkXCI7XG5tb2R1bGUuZXhwb3J0cyA9IHdlYnBhY2tBc3luY0NvbnRleHQ7Il0sIm5hbWVzIjpbIldpbmRvd1Jlc2l6ZUNvbnRyb2wiLCJwcm9wcyIsInR5cGUiLCJvbkNsaWNrIiwiaWNvbiIsIk1BUF9TSVpFIiwiRlVMTCIsImdldE1pbmltaXppbmdJY29uIiwiREVGQVVMVCIsImdldE1heGltaXppbmdJY29uIiwiY29uc29sZSIsImVycm9yIiwiUmVhY3QiLCJQdXJlQ29tcG9uZW50IiwiTmF2aWdhdGlvbiIsInNjcmlwdE9uTG9hZEhhbmRsZXIiLCJiaW5kIiwiTUFQX05BVklHQVRPUiIsIm1hcEFQSUxvYWRlZCIsIm9uTG9hZCIsImxvZyIsIlBBUkFNRVRFUlMiLCJuYXZpZ2F0aW9uIiwibWFwIiwid2luZG93IiwiaW5pdE1hcCIsImxvYWRTY3JpcHRBc3luYyIsInVybCIsIm1hcEFQaVVybCIsIm9uRXJyb3IiLCJoYXNSb3V0aW5nQ29udHJvbHMiLCJzaXplIiwiZW5hYmxlQ29udHJvbHMiLCJkaXNhYmxlQ29udHJvbHMiLCJ0aGVuIiwibWFwQWRhcHRlck1vZHVsZSIsIk1hcEFkYXB0ZXJDbGFzcyIsIm1hcEFkYXB0ZXIiLCJpbml0aWFsaXplIiwiV1MiLCJyZXNldCIsIndpZHRoIiwiaGVpZ2h0Iiwib25SZXNpemUiLCJpbmNsdWRlcyIsIkNvbXBvbmVudCIsInNjcmlwdCIsImRvY3VtZW50IiwiY3JlYXRlRWxlbWVudCIsInNyYyIsImFzeW5jIiwib25sb2FkIiwib25lcnJvciIsImJvZHkiLCJhcHBlbmRDaGlsZCJdLCJzb3VyY2VSb290IjoiIn0=