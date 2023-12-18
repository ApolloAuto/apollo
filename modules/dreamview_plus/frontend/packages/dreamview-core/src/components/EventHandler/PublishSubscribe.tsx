import React, { useState } from 'react';
import { Button } from '@dreamview/dreamview-ui';
import { isNumber } from 'lodash';
import useCustomSubcribe from '../../hooks/useCustomSubcribe';
import useRegisterCustomSubcribe from '../../hooks/useRegisterCustomSubcribe';

function PublishSubscribe(props) {
    const [count, setCount] = useState(0);

    // 注册事件名称，并得到该事件发布最新数据的方法
    const addNum = useRegisterCustomSubcribe('addNum');

    /**
     * 订阅该事件，传递订阅回调函数
     * 注意该回调函数只会被注册一次,在组件卸载时事件会被清空
     */
    useCustomSubcribe('addNum', (val: unknown) => {
        if (isNumber(val)) {
            setCount((prev) => prev + val);
        }
    });

    return (
        <div>
            <span>
                count:
                {count}
            </span>
            <Button
                style={{
                    marginLeft: 10,
                }}
                onClick={() => {
                    addNum(10);
                }}
            >
                累 加
            </Button>
        </div>
    );
}

export default React.memo(PublishSubscribe);
