import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'download-status': {
            display: 'flex',
            alignItems: 'center',
            '& .anticon': {
                marginRight: '6px',
            },
            '& svg': {
                fontSize: '16px',
            },
        },
        downloaded: {
            color: theme.tokens.colors.success,
        },
        downloading: {
            color: theme.tokens.colors.brand2,
        },
        tobeupdate: {
            color: theme.tokens.colors.warn,
        },
        downloadfail: {
            color: theme.tokens.colors.error,
        },
    }));

    return hoc();
}
