import { Metadata, ResponseMessage, ResponseMessageActionEnum } from '../WebSocketManager/type';
import { ENUM_DOWNLOAD_STATUS } from '../api/types';

export type MetadataMessage = ResponseMessage<Metadata> & {
    action:
        | ResponseMessageActionEnum.METADATA_MESSAGE_TYPE
        | ResponseMessageActionEnum.METADATA_JOIN_TYPE
        | ResponseMessageActionEnum.METADATA_LEAVE_TYPE;
};

export type CheckCertResMessage = ResponseMessage<null> & {
    action: ResponseMessageActionEnum.RESPONSE_MESSAGE_TYPE;
};

export interface FileRecord {
    name: string;
    status: ENUM_DOWNLOAD_STATUS;
    percentage: number;
}

export interface Files {
    // key为实际下载使用字段， name为展示使用字段
    [key: string]: FileRecord;
}

export type RecordListResMessage = ResponseMessage<Files> & {
    action: ResponseMessageActionEnum.RESPONSE_MESSAGE_TYPE;
};

export interface DownloadRecord {
    percentage: number;
    record_id: string;
    resource_id: string;
    resource_type: string;
    status: string;
}

export type DownloadRecordResMessage = ResponseMessage<DownloadRecord> & {
    action: ResponseMessageActionEnum.RESPONSE_MESSAGE_TYPE;
};

export type HandleMessageType = MetadataMessage | CheckCertResMessage | RecordListResMessage | DownloadRecordResMessage;
