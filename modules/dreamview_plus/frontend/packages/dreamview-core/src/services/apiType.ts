import { Metadata, ResponseMessage, ResponseMessageActionEnum } from './WebSocketManager/type';

export type MetadataMessage = ResponseMessage<Metadata> & {
    action:
        | ResponseMessageActionEnum.METADATA_MESSAGE_TYPE
        | ResponseMessageActionEnum.METADATA_JOIN_TYPE
        | ResponseMessageActionEnum.METADATA_LEAVE_TYPE;
};

export type ResMessage = ResponseMessage<any> & {
    action: ResponseMessageActionEnum.RESPONSE_MESSAGE_TYPE;
};

export type HandleMessageType = MetadataMessage | ResMessage;
