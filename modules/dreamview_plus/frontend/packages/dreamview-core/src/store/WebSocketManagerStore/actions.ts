import { MetadataItem } from '@dreamview/dreamview-core/src/services/WebSocketManager/type';
import { ACTIONS } from './actionTypes';
import * as TYPES from './actionTypes';

export const UpdateAction = (payload: MetadataItem[]): TYPES.UpdateMetaDataAction => ({
    type: ACTIONS.UPDATE_METADATA,
    payload,
});
