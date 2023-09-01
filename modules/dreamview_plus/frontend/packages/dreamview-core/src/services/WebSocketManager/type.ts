/**
 * 定义与 WebSocketManager 元数据、连接状态、socket 名称、消息操作和有效负载字段相关的类型和接口。
 */

/**
 * 元数据项，包含数据名称、proto 文件路径、proto 描述、请求名称和 WebSocket 信息。
 */
export type MetadataItem = {
    /** 数据名称 */
    dataName: string;
    /** 数据频道 */
    channels: string[];
    /** 是否存在不同的频道 */
    differentForChannels: boolean;
    /** proto 文件路径 */
    protoPath: string;
    /** proto 描述 */
    protoDesc: string;
    /** 请求名称 */
    requestName: string;
    /** WebSocket 信息 */
    websocketInfo: {
        /** WebSocket 名称 */
        websocketName: string;
        /** WebSocket 管道 */
        websocketPipe: string;
    };
};

/**
 * 元数据，包含数据处理程序信息。
 */
export type Metadata = {
    /** 数据处理程序信息 */
    dataHandlerInfo: {
        [path_name: string]: MetadataItem;
    };
};

/**
 * 连接状态枚举，包含已连接、已断开连接、连接中和元数据。
 */
export enum ConnectionStatusEnum {
    /** 已断开连接 */
    DISCONNECTED,
    /** 连接中 */
    CONNECTING,
    /** 已连接 */
    CONNECTED,
    /** 元数据 */
    METADATA,
}

/**
 * WebSocket 名称枚举，包含主 WebSocket 和插件 WebSocket。
 */
export enum SocketNameEnum {
    /** 主 WebSocket */
    MAIN = 'websocket',
    /** 插件 WebSocket */
    PLUGIN = 'plugin',
}

/**
 * 请求消息操作枚举，包含请求消息类型、订阅消息类型和取消订阅消息类型。
 */
export enum RequestMessageActionEnum {
    /** 请求消息类型 */
    REQUEST_MESSAGE_TYPE = 'request',
    /** 订阅消息类型 */
    SUBSCRIBE_MESSAGE_TYPE = 'subscribe',
    /** 取消订阅消息类型 */
    UNSUBSCRIBE_MESSAGE_TYPE = 'unsubscribe',
}

/**
 * 响应消息操作枚举，包含元数据消息类型、响应消息类型和流消息类型。
 */
export enum ResponseMessageActionEnum {
    /** 元数据消息类型 */
    METADATA_MESSAGE_TYPE = 'metadata',
    /** 响应消息类型 */
    RESPONSE_MESSAGE_TYPE = 'response',
    /** 流消息类型 */
    STREAM_MESSAGE_TYPE = 'stream',
}

/**
 * 有效负载字段类型，包含字段名称、源、信息、目标、源类型、目标类型和请求 ID。
 */
export type PayloadField<T> = {
    /** 字段名称 */
    name: string;
    /** 源 */
    source: string;
    /** 信息 */
    info: T;
    /** 目标 */
    target: string;
    /** 源类型 */
    sourceType: string;
    /** 目标类型 */
    targetType: string;
    /** 请求 ID */
    requestId: string;
};

/**
 * 请求消息类型，包含类型、操作和数据。
 */
export interface RequestMessage<T> {
    /** 类型 */
    type: string;
    /** 操作 */
    action: RequestMessageActionEnum;
    /** 数据 */
    data: PayloadField<T>;
}

/**
 * 订阅消息类型，包含类型、操作、WebSocket 名称和数据频率（毫秒）。
 */
export interface SubscribePayload {
    /** WebSocket 名称 */
    websocketName: MetadataItem['websocketInfo']['websocketName'];
    /** 数据频率（毫秒） */
    dataFrequencyMs?: number;
    /** channel名称 */
    channelName?: string;
}

export type RequestStreamMessage = RequestMessage<SubscribePayload>;

/**
 * 响应数据字段类型，包含状态码、消息和数据。
 */
export type ResponseDataField<T> = {
    /** 状态码 */
    code: number;
    /** 消息 */
    message: string;
    /** 数据 */
    data: T;
};

/**
 * 响应消息类型，包含操作和数据。
 */
export interface ResponseMessage<T> {
    /** 操作 */
    action: ResponseMessageActionEnum;
    /** 数据 */
    data: PayloadField<ResponseDataField<T>>;
}

/**
 * 流消息类型，包含操作和数据。一次反序列化
 */
export interface StreamMessage {
    /** 操作 */
    action: ResponseMessageActionEnum.STREAM_MESSAGE_TYPE;
    /** WebSocket 名称 */
    type: MetadataItem['websocketInfo']['websocketName'];
    /** 数据名称 */
    dataName: MetadataItem['dataName'];
    /** 数据名称 */
    channelName?: string;
    /** 数据 */
    data: Uint8Array;
}

// 二次反序列化数据类型
export type StreamMessageData<T> = Omit<StreamMessage, 'data'> & { data: T };

/**
 * 响应流消息类型，包含操作和数据。
 */
export interface ResponseStreamMessage<T> {
    /** 操作 */
    action: ResponseMessageActionEnum;
    /** 数据 */
    data: PayloadField<ResponseDataField<T>>;
}
