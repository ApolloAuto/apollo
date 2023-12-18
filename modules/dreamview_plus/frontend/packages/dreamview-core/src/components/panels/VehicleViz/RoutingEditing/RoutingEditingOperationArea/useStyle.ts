import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'routing-editing-op-con': {
            '& > button:nth-of-type(1)': {
                width: '72px',
                height: '32px',
                marginRight: '16px',
                backgroundColor: 'transparent',
                border: '1px solid rgba(124,136,153,1)',
                '&:hover': {
                    color: '#3288FA',
                    backgroundColor: 'transparent',
                    border: '1px solid #3288FA',
                },
                '&:active': {
                    color: '#1252C0',
                    backgroundColor: '#1252C0',
                    border: '1px solid #1252C0',
                },
            },
            '& > button:nth-of-type(2)': {
                width: '114px',
                height: '32px',
            },
        },
    }));
    return hoc();
}
