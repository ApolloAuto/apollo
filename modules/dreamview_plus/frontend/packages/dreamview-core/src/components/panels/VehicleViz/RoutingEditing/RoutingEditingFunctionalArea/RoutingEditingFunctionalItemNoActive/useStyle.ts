import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'routing-editing-item-no-active': {
            padding: '5px 10px',
            color: '#FFFFFF',
            fontSize: '14px',
            fontWeight: '400',
            fontFamily: 'PingFangSC-Regular',
        },
    }));
    return hoc();
}
