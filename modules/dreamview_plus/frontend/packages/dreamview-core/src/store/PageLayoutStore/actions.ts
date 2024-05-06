import { Action, PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';
import { DECREMENT, INCREMENT, INCREMENTNUMBER } from './actionTypes';

type IncrementAction = Action<typeof INCREMENT>;

type DecrementAction = Action<typeof DECREMENT>;

type IncrementNumberAction = PayloadAction<typeof INCREMENTNUMBER, number>;

export const increment = (): IncrementAction => ({ type: INCREMENT });
export const decrement = (): DecrementAction => ({ type: DECREMENT });
export const incrementNumber = (payload: number): IncrementNumberAction => ({ type: INCREMENTNUMBER, payload });

export type CombineAction = IncrementAction | DecrementAction | IncrementNumberAction;
