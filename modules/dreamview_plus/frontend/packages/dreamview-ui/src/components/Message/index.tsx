import IcToastFail from '@dreamview/dreamview-core/src/assets/svg/ic_toast_fail';
import IcToastSucceed from '@dreamview/dreamview-core/src/assets/svg/ic_toast_succeed';
import IcToastWarning from '@dreamview/dreamview-core/src/assets/svg/ic_toast_warning';
import IcToastLoading from '@dreamview/dreamview-core/src/assets/svg/ic_toast_loading';

import { message as internalmessage, MessageArgsProps } from 'antd';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import './index.less';

export function message(props: MessageArgsProps) {
    const { type } = props;

    let icon = null;
    switch (type) {
        case 'error':
            icon = (
                <span>
                    <IcToastFail />
                </span>
            );
            break;
        case 'success':
            icon = (
                <span>
                    <IcToastSucceed />
                </span>
            );
            break;
        case 'warning':
            icon = (
                <span>
                    <IcToastWarning />
                </span>
            );
            break;
        case 'loading':
            icon = (
                <span className='message-loading-icon'>
                    <IcToastLoading />
                </span>
            );
            break;
        default:
            break;
    }

    internalmessage[type]({ ...props, className: `dreamview-message-notice-${type}`, icon });
}

message.destory = (key: string) => {
    internalmessage.destroy(key);
};

internalmessage.config({
    prefixCls: getPrefixCls('message'),
    duration: 3,
    maxCount: 3,
});
