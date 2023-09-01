import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';
import { MetadataItem } from '@dreamview/dreamview-core/src/services/WebSocketManager/type';

export enum ACTIONS {
    UPDATE_METADATA = 'UPDATE_METADATA',
}

export type UpdateMetaDataAction = PayloadAction<ACTIONS.UPDATE_METADATA, MetadataItem[]>;

export type CombineAction = UpdateMetaDataAction;
