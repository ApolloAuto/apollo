import { ACTIONS } from './actionTypes';
import * as TYPES from './actionTypes';

export const UpdateAction = (payload: TYPES.UpdatePayload): TYPES.UpdateAction => ({
    type: ACTIONS.UPDATE,
    payload,
});
