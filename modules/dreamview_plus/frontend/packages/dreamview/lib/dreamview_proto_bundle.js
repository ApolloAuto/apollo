/*eslint-disable */
'use strict';

var $protobuf = require('protobufjs/minimal');

// Common aliases
var $Reader = $protobuf.Reader,
    $Writer = $protobuf.Writer,
    $util = $protobuf.util;

// Exported root namespace
var $root = $protobuf.roots['default'] || ($protobuf.roots['default'] = {});

$root.apollo = (function () {
    /**
     * Namespace apollo.
     * @exports apollo
     * @namespace
     */
    var apollo = {};

    apollo.dreamview = (function () {
        /**
         * Namespace dreamview.
         * @memberof apollo
         * @namespace
         */
        var dreamview = {};

        dreamview.WebsocketInfo = (function () {
            /**
             * Properties of a WebsocketInfo.
             * @memberof apollo.dreamview
             * @interface IWebsocketInfo
             * @property {string|null} [websocketName] WebsocketInfo websocketName
             * @property {string|null} [websocketPipe] WebsocketInfo websocketPipe
             */

            /**
             * Constructs a new WebsocketInfo.
             * @memberof apollo.dreamview
             * @classdesc Represents a WebsocketInfo.
             * @implements IWebsocketInfo
             * @constructor
             * @param {apollo.dreamview.IWebsocketInfo=} [properties] Properties to set
             */
            function WebsocketInfo(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null) this[keys[i]] = properties[keys[i]];
            }

            /**
             * WebsocketInfo websocketName.
             * @member {string} websocketName
             * @memberof apollo.dreamview.WebsocketInfo
             * @instance
             */
            WebsocketInfo.prototype.websocketName = '';

            /**
             * WebsocketInfo websocketPipe.
             * @member {string} websocketPipe
             * @memberof apollo.dreamview.WebsocketInfo
             * @instance
             */
            WebsocketInfo.prototype.websocketPipe = '';

            /**
             * Creates a new WebsocketInfo instance using the specified properties.
             * @function create
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {apollo.dreamview.IWebsocketInfo=} [properties] Properties to set
             * @returns {apollo.dreamview.WebsocketInfo} WebsocketInfo instance
             */
            WebsocketInfo.create = function create(properties) {
                return new WebsocketInfo(properties);
            };

            /**
             * Encodes the specified WebsocketInfo message. Does not implicitly {@link apollo.dreamview.WebsocketInfo.verify|verify} messages.
             * @function encode
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {apollo.dreamview.IWebsocketInfo} message WebsocketInfo message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            WebsocketInfo.encode = function encode(message, writer) {
                if (!writer) writer = $Writer.create();
                if (message.websocketName != null && Object.hasOwnProperty.call(message, 'websocketName'))
                    writer.uint32(/* id 1, wireType 2 =*/ 10).string(message.websocketName);
                if (message.websocketPipe != null && Object.hasOwnProperty.call(message, 'websocketPipe'))
                    writer.uint32(/* id 2, wireType 2 =*/ 18).string(message.websocketPipe);
                return writer;
            };

            /**
             * Encodes the specified WebsocketInfo message, length delimited. Does not implicitly {@link apollo.dreamview.WebsocketInfo.verify|verify} messages.
             * @function encodeDelimited
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {apollo.dreamview.IWebsocketInfo} message WebsocketInfo message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            WebsocketInfo.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a WebsocketInfo message from the specified reader or buffer.
             * @function decode
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {apollo.dreamview.WebsocketInfo} WebsocketInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            WebsocketInfo.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader)) reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length,
                    message = new $root.apollo.dreamview.WebsocketInfo();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                        case 1: {
                            message.websocketName = reader.string();
                            break;
                        }
                        case 2: {
                            message.websocketPipe = reader.string();
                            break;
                        }
                        default:
                            reader.skipType(tag & 7);
                            break;
                    }
                }
                return message;
            };

            /**
             * Decodes a WebsocketInfo message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {apollo.dreamview.WebsocketInfo} WebsocketInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            WebsocketInfo.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader)) reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a WebsocketInfo message.
             * @function verify
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            WebsocketInfo.verify = function verify(message) {
                if (typeof message !== 'object' || message === null) return 'object expected';
                if (message.websocketName != null && message.hasOwnProperty('websocketName'))
                    if (!$util.isString(message.websocketName)) return 'websocketName: string expected';
                if (message.websocketPipe != null && message.hasOwnProperty('websocketPipe'))
                    if (!$util.isString(message.websocketPipe)) return 'websocketPipe: string expected';
                return null;
            };

            /**
             * Creates a WebsocketInfo message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {apollo.dreamview.WebsocketInfo} WebsocketInfo
             */
            WebsocketInfo.fromObject = function fromObject(object) {
                if (object instanceof $root.apollo.dreamview.WebsocketInfo) return object;
                var message = new $root.apollo.dreamview.WebsocketInfo();
                if (object.websocketName != null) message.websocketName = String(object.websocketName);
                if (object.websocketPipe != null) message.websocketPipe = String(object.websocketPipe);
                return message;
            };

            /**
             * Creates a plain object from a WebsocketInfo message. Also converts values to other types if specified.
             * @function toObject
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {apollo.dreamview.WebsocketInfo} message WebsocketInfo
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            WebsocketInfo.toObject = function toObject(message, options) {
                if (!options) options = {};
                var object = {};
                if (options.defaults) {
                    object.websocketName = '';
                    object.websocketPipe = '';
                }
                if (message.websocketName != null && message.hasOwnProperty('websocketName'))
                    object.websocketName = message.websocketName;
                if (message.websocketPipe != null && message.hasOwnProperty('websocketPipe'))
                    object.websocketPipe = message.websocketPipe;
                return object;
            };

            /**
             * Converts this WebsocketInfo to JSON.
             * @function toJSON
             * @memberof apollo.dreamview.WebsocketInfo
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            WebsocketInfo.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for WebsocketInfo
             * @function getTypeUrl
             * @memberof apollo.dreamview.WebsocketInfo
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            WebsocketInfo.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = 'type.googleapis.com';
                }
                return typeUrlPrefix + '/apollo.dreamview.WebsocketInfo';
            };

            return WebsocketInfo;
        })();

        dreamview.ChannelInfo = (function () {
            /**
             * Properties of a ChannelInfo.
             * @memberof apollo.dreamview
             * @interface IChannelInfo
             * @property {string|null} [channelName] ChannelInfo channelName
             * @property {string|null} [protoPath] ChannelInfo protoPath
             * @property {string|null} [msgType] ChannelInfo msgType
             */

            /**
             * Constructs a new ChannelInfo.
             * @memberof apollo.dreamview
             * @classdesc Represents a ChannelInfo.
             * @implements IChannelInfo
             * @constructor
             * @param {apollo.dreamview.IChannelInfo=} [properties] Properties to set
             */
            function ChannelInfo(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null) this[keys[i]] = properties[keys[i]];
            }

            /**
             * ChannelInfo channelName.
             * @member {string} channelName
             * @memberof apollo.dreamview.ChannelInfo
             * @instance
             */
            ChannelInfo.prototype.channelName = '';

            /**
             * ChannelInfo protoPath.
             * @member {string} protoPath
             * @memberof apollo.dreamview.ChannelInfo
             * @instance
             */
            ChannelInfo.prototype.protoPath = '';

            /**
             * ChannelInfo msgType.
             * @member {string} msgType
             * @memberof apollo.dreamview.ChannelInfo
             * @instance
             */
            ChannelInfo.prototype.msgType = '';

            /**
             * Creates a new ChannelInfo instance using the specified properties.
             * @function create
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {apollo.dreamview.IChannelInfo=} [properties] Properties to set
             * @returns {apollo.dreamview.ChannelInfo} ChannelInfo instance
             */
            ChannelInfo.create = function create(properties) {
                return new ChannelInfo(properties);
            };

            /**
             * Encodes the specified ChannelInfo message. Does not implicitly {@link apollo.dreamview.ChannelInfo.verify|verify} messages.
             * @function encode
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {apollo.dreamview.IChannelInfo} message ChannelInfo message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            ChannelInfo.encode = function encode(message, writer) {
                if (!writer) writer = $Writer.create();
                if (message.channelName != null && Object.hasOwnProperty.call(message, 'channelName'))
                    writer.uint32(/* id 1, wireType 2 =*/ 10).string(message.channelName);
                if (message.protoPath != null && Object.hasOwnProperty.call(message, 'protoPath'))
                    writer.uint32(/* id 2, wireType 2 =*/ 18).string(message.protoPath);
                if (message.msgType != null && Object.hasOwnProperty.call(message, 'msgType'))
                    writer.uint32(/* id 3, wireType 2 =*/ 26).string(message.msgType);
                return writer;
            };

            /**
             * Encodes the specified ChannelInfo message, length delimited. Does not implicitly {@link apollo.dreamview.ChannelInfo.verify|verify} messages.
             * @function encodeDelimited
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {apollo.dreamview.IChannelInfo} message ChannelInfo message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            ChannelInfo.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a ChannelInfo message from the specified reader or buffer.
             * @function decode
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {apollo.dreamview.ChannelInfo} ChannelInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            ChannelInfo.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader)) reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length,
                    message = new $root.apollo.dreamview.ChannelInfo();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                        case 1: {
                            message.channelName = reader.string();
                            break;
                        }
                        case 2: {
                            message.protoPath = reader.string();
                            break;
                        }
                        case 3: {
                            message.msgType = reader.string();
                            break;
                        }
                        default:
                            reader.skipType(tag & 7);
                            break;
                    }
                }
                return message;
            };

            /**
             * Decodes a ChannelInfo message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {apollo.dreamview.ChannelInfo} ChannelInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            ChannelInfo.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader)) reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a ChannelInfo message.
             * @function verify
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            ChannelInfo.verify = function verify(message) {
                if (typeof message !== 'object' || message === null) return 'object expected';
                if (message.channelName != null && message.hasOwnProperty('channelName'))
                    if (!$util.isString(message.channelName)) return 'channelName: string expected';
                if (message.protoPath != null && message.hasOwnProperty('protoPath'))
                    if (!$util.isString(message.protoPath)) return 'protoPath: string expected';
                if (message.msgType != null && message.hasOwnProperty('msgType'))
                    if (!$util.isString(message.msgType)) return 'msgType: string expected';
                return null;
            };

            /**
             * Creates a ChannelInfo message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {apollo.dreamview.ChannelInfo} ChannelInfo
             */
            ChannelInfo.fromObject = function fromObject(object) {
                if (object instanceof $root.apollo.dreamview.ChannelInfo) return object;
                var message = new $root.apollo.dreamview.ChannelInfo();
                if (object.channelName != null) message.channelName = String(object.channelName);
                if (object.protoPath != null) message.protoPath = String(object.protoPath);
                if (object.msgType != null) message.msgType = String(object.msgType);
                return message;
            };

            /**
             * Creates a plain object from a ChannelInfo message. Also converts values to other types if specified.
             * @function toObject
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {apollo.dreamview.ChannelInfo} message ChannelInfo
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            ChannelInfo.toObject = function toObject(message, options) {
                if (!options) options = {};
                var object = {};
                if (options.defaults) {
                    object.channelName = '';
                    object.protoPath = '';
                    object.msgType = '';
                }
                if (message.channelName != null && message.hasOwnProperty('channelName'))
                    object.channelName = message.channelName;
                if (message.protoPath != null && message.hasOwnProperty('protoPath'))
                    object.protoPath = message.protoPath;
                if (message.msgType != null && message.hasOwnProperty('msgType')) object.msgType = message.msgType;
                return object;
            };

            /**
             * Converts this ChannelInfo to JSON.
             * @function toJSON
             * @memberof apollo.dreamview.ChannelInfo
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            ChannelInfo.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for ChannelInfo
             * @function getTypeUrl
             * @memberof apollo.dreamview.ChannelInfo
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            ChannelInfo.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = 'type.googleapis.com';
                }
                return typeUrlPrefix + '/apollo.dreamview.ChannelInfo';
            };

            return ChannelInfo;
        })();

        dreamview.DataHandlerInfo = (function () {
            /**
             * Properties of a DataHandlerInfo.
             * @memberof apollo.dreamview
             * @interface IDataHandlerInfo
             * @property {string|null} [dataName] DataHandlerInfo dataName
             * @property {string|null} [protoPath] DataHandlerInfo protoPath
             * @property {string|null} [msgType] DataHandlerInfo msgType
             * @property {apollo.dreamview.IWebsocketInfo|null} [websocketInfo] DataHandlerInfo websocketInfo
             * @property {boolean|null} [differentForChannels] DataHandlerInfo differentForChannels
             * @property {Array.<apollo.dreamview.IChannelInfo>|null} [channels] DataHandlerInfo channels
             */

            /**
             * Constructs a new DataHandlerInfo.
             * @memberof apollo.dreamview
             * @classdesc Represents a DataHandlerInfo.
             * @implements IDataHandlerInfo
             * @constructor
             * @param {apollo.dreamview.IDataHandlerInfo=} [properties] Properties to set
             */
            function DataHandlerInfo(properties) {
                this.channels = [];
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null) this[keys[i]] = properties[keys[i]];
            }

            /**
             * DataHandlerInfo dataName.
             * @member {string} dataName
             * @memberof apollo.dreamview.DataHandlerInfo
             * @instance
             */
            DataHandlerInfo.prototype.dataName = '';

            /**
             * DataHandlerInfo protoPath.
             * @member {string} protoPath
             * @memberof apollo.dreamview.DataHandlerInfo
             * @instance
             */
            DataHandlerInfo.prototype.protoPath = '';

            /**
             * DataHandlerInfo msgType.
             * @member {string} msgType
             * @memberof apollo.dreamview.DataHandlerInfo
             * @instance
             */
            DataHandlerInfo.prototype.msgType = '';

            /**
             * DataHandlerInfo websocketInfo.
             * @member {apollo.dreamview.IWebsocketInfo|null|undefined} websocketInfo
             * @memberof apollo.dreamview.DataHandlerInfo
             * @instance
             */
            DataHandlerInfo.prototype.websocketInfo = null;

            /**
             * DataHandlerInfo differentForChannels.
             * @member {boolean} differentForChannels
             * @memberof apollo.dreamview.DataHandlerInfo
             * @instance
             */
            DataHandlerInfo.prototype.differentForChannels = false;

            /**
             * DataHandlerInfo channels.
             * @member {Array.<apollo.dreamview.IChannelInfo>} channels
             * @memberof apollo.dreamview.DataHandlerInfo
             * @instance
             */
            DataHandlerInfo.prototype.channels = $util.emptyArray;

            /**
             * Creates a new DataHandlerInfo instance using the specified properties.
             * @function create
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {apollo.dreamview.IDataHandlerInfo=} [properties] Properties to set
             * @returns {apollo.dreamview.DataHandlerInfo} DataHandlerInfo instance
             */
            DataHandlerInfo.create = function create(properties) {
                return new DataHandlerInfo(properties);
            };

            /**
             * Encodes the specified DataHandlerInfo message. Does not implicitly {@link apollo.dreamview.DataHandlerInfo.verify|verify} messages.
             * @function encode
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {apollo.dreamview.IDataHandlerInfo} message DataHandlerInfo message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            DataHandlerInfo.encode = function encode(message, writer) {
                if (!writer) writer = $Writer.create();
                if (message.dataName != null && Object.hasOwnProperty.call(message, 'dataName'))
                    writer.uint32(/* id 1, wireType 2 =*/ 10).string(message.dataName);
                if (message.protoPath != null && Object.hasOwnProperty.call(message, 'protoPath'))
                    writer.uint32(/* id 2, wireType 2 =*/ 18).string(message.protoPath);
                if (message.msgType != null && Object.hasOwnProperty.call(message, 'msgType'))
                    writer.uint32(/* id 3, wireType 2 =*/ 26).string(message.msgType);
                if (message.websocketInfo != null && Object.hasOwnProperty.call(message, 'websocketInfo'))
                    $root.apollo.dreamview.WebsocketInfo.encode(
                        message.websocketInfo,
                        writer.uint32(/* id 4, wireType 2 =*/ 34).fork(),
                    ).ldelim();
                if (message.differentForChannels != null && Object.hasOwnProperty.call(message, 'differentForChannels'))
                    writer.uint32(/* id 5, wireType 0 =*/ 40).bool(message.differentForChannels);
                if (message.channels != null && message.channels.length)
                    for (var i = 0; i < message.channels.length; ++i)
                        $root.apollo.dreamview.ChannelInfo.encode(
                            message.channels[i],
                            writer.uint32(/* id 6, wireType 2 =*/ 50).fork(),
                        ).ldelim();
                return writer;
            };

            /**
             * Encodes the specified DataHandlerInfo message, length delimited. Does not implicitly {@link apollo.dreamview.DataHandlerInfo.verify|verify} messages.
             * @function encodeDelimited
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {apollo.dreamview.IDataHandlerInfo} message DataHandlerInfo message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            DataHandlerInfo.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a DataHandlerInfo message from the specified reader or buffer.
             * @function decode
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {apollo.dreamview.DataHandlerInfo} DataHandlerInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            DataHandlerInfo.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader)) reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length,
                    message = new $root.apollo.dreamview.DataHandlerInfo();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                        case 1: {
                            message.dataName = reader.string();
                            break;
                        }
                        case 2: {
                            message.protoPath = reader.string();
                            break;
                        }
                        case 3: {
                            message.msgType = reader.string();
                            break;
                        }
                        case 4: {
                            message.websocketInfo = $root.apollo.dreamview.WebsocketInfo.decode(
                                reader,
                                reader.uint32(),
                            );
                            break;
                        }
                        case 5: {
                            message.differentForChannels = reader.bool();
                            break;
                        }
                        case 6: {
                            if (!(message.channels && message.channels.length)) message.channels = [];
                            message.channels.push($root.apollo.dreamview.ChannelInfo.decode(reader, reader.uint32()));
                            break;
                        }
                        default:
                            reader.skipType(tag & 7);
                            break;
                    }
                }
                return message;
            };

            /**
             * Decodes a DataHandlerInfo message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {apollo.dreamview.DataHandlerInfo} DataHandlerInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            DataHandlerInfo.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader)) reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a DataHandlerInfo message.
             * @function verify
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            DataHandlerInfo.verify = function verify(message) {
                if (typeof message !== 'object' || message === null) return 'object expected';
                if (message.dataName != null && message.hasOwnProperty('dataName'))
                    if (!$util.isString(message.dataName)) return 'dataName: string expected';
                if (message.protoPath != null && message.hasOwnProperty('protoPath'))
                    if (!$util.isString(message.protoPath)) return 'protoPath: string expected';
                if (message.msgType != null && message.hasOwnProperty('msgType'))
                    if (!$util.isString(message.msgType)) return 'msgType: string expected';
                if (message.websocketInfo != null && message.hasOwnProperty('websocketInfo')) {
                    var error = $root.apollo.dreamview.WebsocketInfo.verify(message.websocketInfo);
                    if (error) return 'websocketInfo.' + error;
                }
                if (message.differentForChannels != null && message.hasOwnProperty('differentForChannels'))
                    if (typeof message.differentForChannels !== 'boolean')
                        return 'differentForChannels: boolean expected';
                if (message.channels != null && message.hasOwnProperty('channels')) {
                    if (!Array.isArray(message.channels)) return 'channels: array expected';
                    for (var i = 0; i < message.channels.length; ++i) {
                        var error = $root.apollo.dreamview.ChannelInfo.verify(message.channels[i]);
                        if (error) return 'channels.' + error;
                    }
                }
                return null;
            };

            /**
             * Creates a DataHandlerInfo message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {apollo.dreamview.DataHandlerInfo} DataHandlerInfo
             */
            DataHandlerInfo.fromObject = function fromObject(object) {
                if (object instanceof $root.apollo.dreamview.DataHandlerInfo) return object;
                var message = new $root.apollo.dreamview.DataHandlerInfo();
                if (object.dataName != null) message.dataName = String(object.dataName);
                if (object.protoPath != null) message.protoPath = String(object.protoPath);
                if (object.msgType != null) message.msgType = String(object.msgType);
                if (object.websocketInfo != null) {
                    if (typeof object.websocketInfo !== 'object')
                        throw TypeError('.apollo.dreamview.DataHandlerInfo.websocketInfo: object expected');
                    message.websocketInfo = $root.apollo.dreamview.WebsocketInfo.fromObject(object.websocketInfo);
                }
                if (object.differentForChannels != null)
                    message.differentForChannels = Boolean(object.differentForChannels);
                if (object.channels) {
                    if (!Array.isArray(object.channels))
                        throw TypeError('.apollo.dreamview.DataHandlerInfo.channels: array expected');
                    message.channels = [];
                    for (var i = 0; i < object.channels.length; ++i) {
                        if (typeof object.channels[i] !== 'object')
                            throw TypeError('.apollo.dreamview.DataHandlerInfo.channels: object expected');
                        message.channels[i] = $root.apollo.dreamview.ChannelInfo.fromObject(object.channels[i]);
                    }
                }
                return message;
            };

            /**
             * Creates a plain object from a DataHandlerInfo message. Also converts values to other types if specified.
             * @function toObject
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {apollo.dreamview.DataHandlerInfo} message DataHandlerInfo
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            DataHandlerInfo.toObject = function toObject(message, options) {
                if (!options) options = {};
                var object = {};
                if (options.arrays || options.defaults) object.channels = [];
                if (options.defaults) {
                    object.dataName = '';
                    object.protoPath = '';
                    object.msgType = '';
                    object.websocketInfo = null;
                    object.differentForChannels = false;
                }
                if (message.dataName != null && message.hasOwnProperty('dataName')) object.dataName = message.dataName;
                if (message.protoPath != null && message.hasOwnProperty('protoPath'))
                    object.protoPath = message.protoPath;
                if (message.msgType != null && message.hasOwnProperty('msgType')) object.msgType = message.msgType;
                if (message.websocketInfo != null && message.hasOwnProperty('websocketInfo'))
                    object.websocketInfo = $root.apollo.dreamview.WebsocketInfo.toObject(
                        message.websocketInfo,
                        options,
                    );
                if (message.differentForChannels != null && message.hasOwnProperty('differentForChannels'))
                    object.differentForChannels = message.differentForChannels;
                if (message.channels && message.channels.length) {
                    object.channels = [];
                    for (var j = 0; j < message.channels.length; ++j)
                        object.channels[j] = $root.apollo.dreamview.ChannelInfo.toObject(message.channels[j], options);
                }
                return object;
            };

            /**
             * Converts this DataHandlerInfo to JSON.
             * @function toJSON
             * @memberof apollo.dreamview.DataHandlerInfo
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            DataHandlerInfo.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for DataHandlerInfo
             * @function getTypeUrl
             * @memberof apollo.dreamview.DataHandlerInfo
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            DataHandlerInfo.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = 'type.googleapis.com';
                }
                return typeUrlPrefix + '/apollo.dreamview.DataHandlerInfo';
            };

            return DataHandlerInfo;
        })();

        dreamview.DataHandlerConf = (function () {
            /**
             * Properties of a DataHandlerConf.
             * @memberof apollo.dreamview
             * @interface IDataHandlerConf
             * @property {Object.<string,apollo.dreamview.IDataHandlerInfo>|null} [dataHandlerInfo] DataHandlerConf dataHandlerInfo
             */

            /**
             * Constructs a new DataHandlerConf.
             * @memberof apollo.dreamview
             * @classdesc Represents a DataHandlerConf.
             * @implements IDataHandlerConf
             * @constructor
             * @param {apollo.dreamview.IDataHandlerConf=} [properties] Properties to set
             */
            function DataHandlerConf(properties) {
                this.dataHandlerInfo = {};
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null) this[keys[i]] = properties[keys[i]];
            }

            /**
             * DataHandlerConf dataHandlerInfo.
             * @member {Object.<string,apollo.dreamview.IDataHandlerInfo>} dataHandlerInfo
             * @memberof apollo.dreamview.DataHandlerConf
             * @instance
             */
            DataHandlerConf.prototype.dataHandlerInfo = $util.emptyObject;

            /**
             * Creates a new DataHandlerConf instance using the specified properties.
             * @function create
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {apollo.dreamview.IDataHandlerConf=} [properties] Properties to set
             * @returns {apollo.dreamview.DataHandlerConf} DataHandlerConf instance
             */
            DataHandlerConf.create = function create(properties) {
                return new DataHandlerConf(properties);
            };

            /**
             * Encodes the specified DataHandlerConf message. Does not implicitly {@link apollo.dreamview.DataHandlerConf.verify|verify} messages.
             * @function encode
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {apollo.dreamview.IDataHandlerConf} message DataHandlerConf message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            DataHandlerConf.encode = function encode(message, writer) {
                if (!writer) writer = $Writer.create();
                if (message.dataHandlerInfo != null && Object.hasOwnProperty.call(message, 'dataHandlerInfo'))
                    for (var keys = Object.keys(message.dataHandlerInfo), i = 0; i < keys.length; ++i) {
                        writer
                            .uint32(/* id 1, wireType 2 =*/ 10)
                            .fork()
                            .uint32(/* id 1, wireType 2 =*/ 10)
                            .string(keys[i]);
                        $root.apollo.dreamview.DataHandlerInfo.encode(
                            message.dataHandlerInfo[keys[i]],
                            writer.uint32(/* id 2, wireType 2 =*/ 18).fork(),
                        )
                            .ldelim()
                            .ldelim();
                    }
                return writer;
            };

            /**
             * Encodes the specified DataHandlerConf message, length delimited. Does not implicitly {@link apollo.dreamview.DataHandlerConf.verify|verify} messages.
             * @function encodeDelimited
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {apollo.dreamview.IDataHandlerConf} message DataHandlerConf message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            DataHandlerConf.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a DataHandlerConf message from the specified reader or buffer.
             * @function decode
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {apollo.dreamview.DataHandlerConf} DataHandlerConf
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            DataHandlerConf.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader)) reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length,
                    message = new $root.apollo.dreamview.DataHandlerConf(),
                    key,
                    value;
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                        case 1: {
                            if (message.dataHandlerInfo === $util.emptyObject) message.dataHandlerInfo = {};
                            var end2 = reader.uint32() + reader.pos;
                            key = '';
                            value = null;
                            while (reader.pos < end2) {
                                var tag2 = reader.uint32();
                                switch (tag2 >>> 3) {
                                    case 1:
                                        key = reader.string();
                                        break;
                                    case 2:
                                        value = $root.apollo.dreamview.DataHandlerInfo.decode(reader, reader.uint32());
                                        break;
                                    default:
                                        reader.skipType(tag2 & 7);
                                        break;
                                }
                            }
                            message.dataHandlerInfo[key] = value;
                            break;
                        }
                        default:
                            reader.skipType(tag & 7);
                            break;
                    }
                }
                return message;
            };

            /**
             * Decodes a DataHandlerConf message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {apollo.dreamview.DataHandlerConf} DataHandlerConf
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            DataHandlerConf.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader)) reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a DataHandlerConf message.
             * @function verify
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            DataHandlerConf.verify = function verify(message) {
                if (typeof message !== 'object' || message === null) return 'object expected';
                if (message.dataHandlerInfo != null && message.hasOwnProperty('dataHandlerInfo')) {
                    if (!$util.isObject(message.dataHandlerInfo)) return 'dataHandlerInfo: object expected';
                    var key = Object.keys(message.dataHandlerInfo);
                    for (var i = 0; i < key.length; ++i) {
                        var error = $root.apollo.dreamview.DataHandlerInfo.verify(message.dataHandlerInfo[key[i]]);
                        if (error) return 'dataHandlerInfo.' + error;
                    }
                }
                return null;
            };

            /**
             * Creates a DataHandlerConf message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {apollo.dreamview.DataHandlerConf} DataHandlerConf
             */
            DataHandlerConf.fromObject = function fromObject(object) {
                if (object instanceof $root.apollo.dreamview.DataHandlerConf) return object;
                var message = new $root.apollo.dreamview.DataHandlerConf();
                if (object.dataHandlerInfo) {
                    if (typeof object.dataHandlerInfo !== 'object')
                        throw TypeError('.apollo.dreamview.DataHandlerConf.dataHandlerInfo: object expected');
                    message.dataHandlerInfo = {};
                    for (var keys = Object.keys(object.dataHandlerInfo), i = 0; i < keys.length; ++i) {
                        if (typeof object.dataHandlerInfo[keys[i]] !== 'object')
                            throw TypeError('.apollo.dreamview.DataHandlerConf.dataHandlerInfo: object expected');
                        message.dataHandlerInfo[keys[i]] = $root.apollo.dreamview.DataHandlerInfo.fromObject(
                            object.dataHandlerInfo[keys[i]],
                        );
                    }
                }
                return message;
            };

            /**
             * Creates a plain object from a DataHandlerConf message. Also converts values to other types if specified.
             * @function toObject
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {apollo.dreamview.DataHandlerConf} message DataHandlerConf
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            DataHandlerConf.toObject = function toObject(message, options) {
                if (!options) options = {};
                var object = {};
                if (options.objects || options.defaults) object.dataHandlerInfo = {};
                var keys2;
                if (message.dataHandlerInfo && (keys2 = Object.keys(message.dataHandlerInfo)).length) {
                    object.dataHandlerInfo = {};
                    for (var j = 0; j < keys2.length; ++j)
                        object.dataHandlerInfo[keys2[j]] = $root.apollo.dreamview.DataHandlerInfo.toObject(
                            message.dataHandlerInfo[keys2[j]],
                            options,
                        );
                }
                return object;
            };

            /**
             * Converts this DataHandlerConf to JSON.
             * @function toJSON
             * @memberof apollo.dreamview.DataHandlerConf
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            DataHandlerConf.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for DataHandlerConf
             * @function getTypeUrl
             * @memberof apollo.dreamview.DataHandlerConf
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            DataHandlerConf.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = 'type.googleapis.com';
                }
                return typeUrlPrefix + '/apollo.dreamview.DataHandlerConf';
            };

            return DataHandlerConf;
        })();

        dreamview.StreamData = (function () {
            /**
             * Properties of a StreamData.
             * @memberof apollo.dreamview
             * @interface IStreamData
             * @property {string|null} [type] StreamData type
             * @property {string|null} [action] StreamData action
             * @property {string|null} [dataName] StreamData dataName
             * @property {string|null} [channelName] StreamData channelName
             * @property {Uint8Array|null} [data] StreamData data
             */

            /**
             * Constructs a new StreamData.
             * @memberof apollo.dreamview
             * @classdesc Represents a StreamData.
             * @implements IStreamData
             * @constructor
             * @param {apollo.dreamview.IStreamData=} [properties] Properties to set
             */
            function StreamData(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null) this[keys[i]] = properties[keys[i]];
            }

            /**
             * StreamData type.
             * @member {string} type
             * @memberof apollo.dreamview.StreamData
             * @instance
             */
            StreamData.prototype.type = '';

            /**
             * StreamData action.
             * @member {string} action
             * @memberof apollo.dreamview.StreamData
             * @instance
             */
            StreamData.prototype.action = '';

            /**
             * StreamData dataName.
             * @member {string} dataName
             * @memberof apollo.dreamview.StreamData
             * @instance
             */
            StreamData.prototype.dataName = '';

            /**
             * StreamData channelName.
             * @member {string} channelName
             * @memberof apollo.dreamview.StreamData
             * @instance
             */
            StreamData.prototype.channelName = '';

            /**
             * StreamData data.
             * @member {Uint8Array} data
             * @memberof apollo.dreamview.StreamData
             * @instance
             */
            StreamData.prototype.data = $util.newBuffer([]);

            /**
             * Creates a new StreamData instance using the specified properties.
             * @function create
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {apollo.dreamview.IStreamData=} [properties] Properties to set
             * @returns {apollo.dreamview.StreamData} StreamData instance
             */
            StreamData.create = function create(properties) {
                return new StreamData(properties);
            };

            /**
             * Encodes the specified StreamData message. Does not implicitly {@link apollo.dreamview.StreamData.verify|verify} messages.
             * @function encode
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {apollo.dreamview.IStreamData} message StreamData message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            StreamData.encode = function encode(message, writer) {
                if (!writer) writer = $Writer.create();
                if (message.type != null && Object.hasOwnProperty.call(message, 'type'))
                    writer.uint32(/* id 1, wireType 2 =*/ 10).string(message.type);
                if (message.action != null && Object.hasOwnProperty.call(message, 'action'))
                    writer.uint32(/* id 2, wireType 2 =*/ 18).string(message.action);
                if (message.dataName != null && Object.hasOwnProperty.call(message, 'dataName'))
                    writer.uint32(/* id 3, wireType 2 =*/ 26).string(message.dataName);
                if (message.channelName != null && Object.hasOwnProperty.call(message, 'channelName'))
                    writer.uint32(/* id 4, wireType 2 =*/ 34).string(message.channelName);
                if (message.data != null && Object.hasOwnProperty.call(message, 'data'))
                    writer.uint32(/* id 5, wireType 2 =*/ 42).bytes(message.data);
                return writer;
            };

            /**
             * Encodes the specified StreamData message, length delimited. Does not implicitly {@link apollo.dreamview.StreamData.verify|verify} messages.
             * @function encodeDelimited
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {apollo.dreamview.IStreamData} message StreamData message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            StreamData.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a StreamData message from the specified reader or buffer.
             * @function decode
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {apollo.dreamview.StreamData} StreamData
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            StreamData.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader)) reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length,
                    message = new $root.apollo.dreamview.StreamData();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                        case 1: {
                            message.type = reader.string();
                            break;
                        }
                        case 2: {
                            message.action = reader.string();
                            break;
                        }
                        case 3: {
                            message.dataName = reader.string();
                            break;
                        }
                        case 4: {
                            message.channelName = reader.string();
                            break;
                        }
                        case 5: {
                            message.data = reader.bytes();
                            break;
                        }
                        default:
                            reader.skipType(tag & 7);
                            break;
                    }
                }
                return message;
            };

            /**
             * Decodes a StreamData message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {apollo.dreamview.StreamData} StreamData
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            StreamData.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader)) reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a StreamData message.
             * @function verify
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            StreamData.verify = function verify(message) {
                if (typeof message !== 'object' || message === null) return 'object expected';
                if (message.type != null && message.hasOwnProperty('type'))
                    if (!$util.isString(message.type)) return 'type: string expected';
                if (message.action != null && message.hasOwnProperty('action'))
                    if (!$util.isString(message.action)) return 'action: string expected';
                if (message.dataName != null && message.hasOwnProperty('dataName'))
                    if (!$util.isString(message.dataName)) return 'dataName: string expected';
                if (message.channelName != null && message.hasOwnProperty('channelName'))
                    if (!$util.isString(message.channelName)) return 'channelName: string expected';
                if (message.data != null && message.hasOwnProperty('data'))
                    if (!((message.data && typeof message.data.length === 'number') || $util.isString(message.data)))
                        return 'data: buffer expected';
                return null;
            };

            /**
             * Creates a StreamData message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {apollo.dreamview.StreamData} StreamData
             */
            StreamData.fromObject = function fromObject(object) {
                if (object instanceof $root.apollo.dreamview.StreamData) return object;
                var message = new $root.apollo.dreamview.StreamData();
                if (object.type != null) message.type = String(object.type);
                if (object.action != null) message.action = String(object.action);
                if (object.dataName != null) message.dataName = String(object.dataName);
                if (object.channelName != null) message.channelName = String(object.channelName);
                if (object.data != null)
                    if (typeof object.data === 'string')
                        $util.base64.decode(
                            object.data,
                            (message.data = $util.newBuffer($util.base64.length(object.data))),
                            0,
                        );
                    else if (object.data.length >= 0) message.data = object.data;
                return message;
            };

            /**
             * Creates a plain object from a StreamData message. Also converts values to other types if specified.
             * @function toObject
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {apollo.dreamview.StreamData} message StreamData
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            StreamData.toObject = function toObject(message, options) {
                if (!options) options = {};
                var object = {};
                if (options.defaults) {
                    object.type = '';
                    object.action = '';
                    object.dataName = '';
                    object.channelName = '';
                    if (options.bytes === String) object.data = '';
                    else {
                        object.data = [];
                        if (options.bytes !== Array) object.data = $util.newBuffer(object.data);
                    }
                }
                if (message.type != null && message.hasOwnProperty('type')) object.type = message.type;
                if (message.action != null && message.hasOwnProperty('action')) object.action = message.action;
                if (message.dataName != null && message.hasOwnProperty('dataName')) object.dataName = message.dataName;
                if (message.channelName != null && message.hasOwnProperty('channelName'))
                    object.channelName = message.channelName;
                if (message.data != null && message.hasOwnProperty('data'))
                    object.data =
                        options.bytes === String
                            ? $util.base64.encode(message.data, 0, message.data.length)
                            : options.bytes === Array
                            ? Array.prototype.slice.call(message.data)
                            : message.data;
                return object;
            };

            /**
             * Converts this StreamData to JSON.
             * @function toJSON
             * @memberof apollo.dreamview.StreamData
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            StreamData.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for StreamData
             * @function getTypeUrl
             * @memberof apollo.dreamview.StreamData
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            StreamData.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = 'type.googleapis.com';
                }
                return typeUrlPrefix + '/apollo.dreamview.StreamData';
            };

            return StreamData;
        })();

        return dreamview;
    })();

    return apollo;
})();

module.exports = $root;
