/**
 * 定义与 WebSocketManager 元数据、连接状态、socket 名称、消息操作和有效负载字段相关的类型和接口。
 */

import { TaskInternal } from '@dreamview/dreamview-core/src/worker/type';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';

/**
 * channel
 */
export interface Channel {
    channelName: string;
    msgType: string;
    protoPath: string;
}

/**
 * 元数据项，包含数据名称、proto 文件路径、proto 描述、请求名称和 WebSocket 信息。
 */
export type MetadataItem = {
    /** 数据名称 */
    dataName: StreamDataNames;
    /** 数据频道 */
    channels: Channel[];
    /** 是否存在不同的频道 */
    differentForChannels: boolean;
    /** proto 文件路径 */
    protoPath: string;
    /** proto 描述 */
    msgType: string;
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
    /** 仿真 WebSocket */
    SIMULATION = '/plugins/sim/sim_websocket',
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
    /** 元数据增量更新消息类型 */
    METADATA_JOIN_TYPE = 'join',
    /** 元数据局部清理消息类型 */
    METADATA_LEAVE_TYPE = 'leave',
    /** 响应消息类型 */
    RESPONSE_MESSAGE_TYPE = 'response',
    /** 流消息类型 */
    STREAM_MESSAGE_TYPE = 'stream',
}

/**
 * 元数据操作类型，包含元数据消息类型、元数据增量更新消息类型和元数据局部清理消息类型。
 */
export type MetaActionType = 'join' | 'leave';

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
    /** 入参 */
    param?: any;
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
    /** 消息 */
    error_msg: string;
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
    /** 性能分析 */
    performance?: {
        startTimestamp?: number;
        endTimestamp?: number;
    };
}

// 二次反序列化数据类型
export type StreamMessageData<T> = Omit<StreamMessage, 'data'> & {
    data: T;
    // 用于性能评估
    performance?: {
        startTimestamp?: number;
        endTimestamp?: number;
    };
};

/**
 * 响应流消息类型，包含操作和数据。
 */
export interface ResponseStreamMessage<T> {
    /** 操作 */
    action: ResponseMessageActionEnum;
    /** 数据 */
    data: PayloadField<ResponseDataField<T>>;
}

/** ***child socket***** */

export type WorkerMessageType =
    // 初始化WebSocket (INIT_SOCKET):
    | 'INIT_SOCKET'
    // 销毁WebSocket (DESTROY_SOCKET):
    | 'DESTROY_SOCKET'
    // 中断WebSocket (INTERRUPT_SOCKET):
    | 'INTERRUPT_SOCKET'
    // 重新连接WebSocket (RECONNECT_SOCKET):
    | 'RECONNECT_SOCKET'
    // WebSocket消息 (SOCKET_MESSAGE):
    | 'SOCKET_MESSAGE'
    // WebSocket错误 (SOCKET_ERROR):
    | 'SOCKET_ERROR'
    // WebSocket打开 (SOCKET_OPEN):
    | 'SOCKET_OPEN'
    // WebSocket已关闭 (SOCKET_CLOSED):
    | 'SOCKET_CLOSED'
    // SOCKET_METADATA 更新元数据 (SOCKET_METADATA):
    | 'SOCKET_METADATA'
    // 接受首次反序列化的数据 (SOCKET_STREAM_MESSAGE):
    | 'SOCKET_STREAM_MESSAGE'
    // 二次反序列化的数据 (SOCKET_STREAM_MESSAGE_DATA):
    | 'SOCKET_STREAM_MESSAGE_DATA';

type INIT_SOCKET_Payload = { name: StreamDataNames; url: string };
type DESTROY_SOCKET_Payload = undefined;
type INTERRUPT_SOCKET_Payload = undefined;
type RECONNECT_SOCKET_Payload = { name: StreamDataNames; url: string };
type SOCKET_MESSAGE_Payload = unknown;
type SOCKET_ERROR_Payload = undefined;
type SOCKET_OPEN_Payload = undefined;
type SOCKET_CLOSED_Payload = undefined;
type SOCKET_METADATA_Payload = MetadataItem[];
type SOCKET_STREAM_MESSAGE_Payload = TaskInternal<StreamMessage>;
type SOCKET_STREAM_MESSAGE_DATA_Payload = TaskInternal<StreamMessage>;

export type WorkerMessagePayloads = {
    INIT_SOCKET: INIT_SOCKET_Payload;
    DESTROY_SOCKET: DESTROY_SOCKET_Payload;
    INTERRUPT_SOCKET: INTERRUPT_SOCKET_Payload;
    RECONNECT_SOCKET: RECONNECT_SOCKET_Payload;
    SOCKET_MESSAGE: SOCKET_MESSAGE_Payload;
    SOCKET_ERROR: SOCKET_ERROR_Payload;
    SOCKET_OPEN: SOCKET_OPEN_Payload;
    SOCKET_CLOSED: SOCKET_CLOSED_Payload;
    SOCKET_METADATA: SOCKET_METADATA_Payload;
    SOCKET_STREAM_MESSAGE: SOCKET_STREAM_MESSAGE_Payload;
    SOCKET_STREAM_MESSAGE_DATA: SOCKET_STREAM_MESSAGE_DATA_Payload;
};
export interface WorkerMessage<T extends WorkerMessageType> {
    type: T;
    payload?: WorkerMessagePayloads[T];
}

export function isMessageType<T extends WorkerMessageType>(
    message: WorkerMessage<WorkerMessageType>,
    type: T,
): message is WorkerMessage<T> {
    return message.type === type;
}

export type SocketState = 'OPEN' | 'CLOSED' | 'ERROR' | 'PENDING';

export type DispatchTaskOption = {
    // 分发任务执行后的回调
    callback?: () => void;
};
