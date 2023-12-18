import { produce } from 'immer';

import { INCREMENT, DECREMENT, INCREMENTNUMBER } from './actionTypes';
import { CombineAction } from './actions';

export interface IInitState {
    num1: number;
    num2: number;
}

export const initState = {
    num1: 0,
    num2: 0,
};

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case INCREMENT:
                draftState.num1 += 1;
                break;
            case DECREMENT:
                draftState.num1 -= 1;
                break;
            case INCREMENTNUMBER:
                draftState.num2 += action.payload;
                break;
            default:
                break;
        }
    });
