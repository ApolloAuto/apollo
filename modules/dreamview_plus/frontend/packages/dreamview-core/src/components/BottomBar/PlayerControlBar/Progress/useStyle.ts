import { useMakeStyle } from '@dreamview/dreamview-theme';

interface IUseStyle {
    width: string;
}
export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'progress-container': {
            position: 'relative',
            width: '100%',
            padding: '4px 0',
            cursor: 'pointer',
            '& .progress-background': {
                backgroundColor: '#343947',
            },
            '& .progress-inner': {
                height: '4px',
                backgroundImage: 'linear-gradient(270deg, #559CFA 1%, #3288FA 100%)',
                // width: prop.width,
            },
            '& .progress-pointer': {
                position: 'absolute',
                borderRadius: '12px',
                width: '12px',
                height: '12px',
                backgroundColor: theme.tokens.colors.brand2,
                border: '2px solid white',
                boxShadow: '0px 6px 6px 0px rgba(0,75,179,0.5)',
                top: '0px',
                // left: `calc(${prop.width} - 6px)`,
                '&::after': {
                    content: '""',
                    position: 'absolute',
                    left: '-8px',
                    top: '-8px',
                    width: '24px',
                    height: '24px',
                    borderRadius: '50%',
                    background: 'rgba(50,136,250,0.20)',
                    transition: theme.tokens.transitions.easeInOut(),
                },
            },
        },
    }));

    return hoc();
}
