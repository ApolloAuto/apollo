import React, { useState } from 'react';
import { Meta, StoryObj } from '@storybook/react';
import { produce } from 'immer';
import { Input, Button } from '@dreamview/dreamview-ui';
import { Factory } from './base';
import { PayloadAction } from './base/Reducer';

const ADD = 'ADD';
const MINUS = 'MINUS';
type CombineType = typeof ADD | typeof MINUS;

type CountPayload = {
    count: number;
};

type AddAction = PayloadAction<typeof ADD, CountPayload>;
type MinusAction = PayloadAction<typeof MINUS, CountPayload>;

type CombineAction = AddAction | MinusAction;

type IInitState = {
    count: number;
};

const initstate: IInitState = {
    count: 0,
};

const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case ADD:
                draftState.count = state.count + action.payload.count;
                break;
            case MINUS:
                draftState.count = state.count - action.payload.count;
                break;
            default:
                break;
        }
    });

export const { StoreProvider: CountStoreProvider, useStore: useCountStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initstate,
    reducer,
});

function Count() {
    const [{ count }, dispatch] = useCountStore();
    const [addCount, setAddCount] = useState(count);
    const [minusCount, setMinusCount] = useState(count);

    const handleClick = (type: CombineType, countNum: number) => {
        dispatch({ type, payload: { count: countNum } });
    };

    const handleInputAdd = (e: any) => {
        setAddCount(parseInt(e.target.value, 10));
    };

    const handleInputMinus = (e: any) => {
        setMinusCount(parseInt(e.target.value, 10));
    };

    return (
        <div style={{ width: 200 }}>
            <div>
                <Input placeholder='请输入增加数字大小' width={200} onChange={handleInputAdd} type='number' />
                <Button onClick={() => handleClick(ADD, addCount)} type='primary'>{`加${addCount}`}</Button>
            </div>
            <div>
                <Input placeholder='请输入减小数字大小' width={200} onChange={handleInputMinus} type='number' />
                <Button onClick={() => handleClick(MINUS, minusCount)} type='primary'>{`减${minusCount}`}</Button>
            </div>
            <div style={{ color: '#2E3438', fontSize: '15px' }}>{`当前数值：${count}`}</div>
        </div>
    );
}

export function Store() {
    return (
        <CountStoreProvider>
            <Count />
        </CountStoreProvider>
    );
}

const meta: Meta<typeof Store> = {
    title: 'Store/CommonStore',
    component: Store,
};

export default meta;
