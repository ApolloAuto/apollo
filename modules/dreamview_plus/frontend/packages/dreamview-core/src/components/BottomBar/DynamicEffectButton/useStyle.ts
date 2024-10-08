import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import { keyframes } from 'tss-react';
import tinycolor from 'tinycolor2';
import { useImagePrak } from '@dreamview/dreamview-ui';

export enum DynamicEffectButtonStatus {
    DISABLE = 'DISABLE',
    START = 'START',
    STOP = 'STOP',
    RUNNING = 'RUNNING',
}

const useHoc = makeStylesWithProps<{ ImageAmbientLight: string }>()((theme, props) => ({
    'btn-container': {
        position: 'relative',
        height: '32px',
        borderRadius: '6px',
        padding: '6px 16px',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        fontFamily: 'PingFangSC-Regular',
        fontSize: '14px',
        fontWeight: 400,
        cursor: 'pointer',
        userSelect: 'none',
        '& .anticon': {
            display: 'block',
            color: '#FFFFFF',
            marginRight: theme.tokens.margin.speace,
        },
    },
    'btn-doing': {
        color: '#FFFFFF',
        backgroundColor: theme.tokens.colors.brand3,
        inset: 0,
        '& span': {
            position: 'relative',
            zIndex: 2,
        },
        '& .anticon': {
            fontSize: theme.tokens.font.size.large,
        },
    },
    'btn-border': {
        content: '""',
        left: '-3px',
        right: '-3px',
        top: '-3px',
        bottom: '-3px',
        position: 'absolute',
        backgroundImage: 'linear-gradient(0deg,#8DD0FF,#3288FA)',
        maskImage: 'linear-gradient(#fff 0 0),linear-gradient(#fff 0 0)',
        maskClip: 'content-box,border-box',
        maskComposite: 'xor',
        padding: '1px',
        borderRadius: '8px',
    },
    'btn-reactive': {
        '&:hover': {
            backgroundColor: theme.tokens.colors.brand2,
        },
        '&:active': {
            backgroundColor: theme.tokens.colors.brand1,
        },
    },
    'btn-start': {
        color: '#FFFFFF',
        backgroundColor: theme.tokens.colors.brand3,
        '& .anticon': {
            fontSize: theme.tokens.font.size.large,
        },
    },
    'btn-stop': {
        color: '#FFFFFF',
        backgroundColor: '#F75660',
        '& .anticon': {
            fontSize: theme.tokens.font.size.huge,
        },
        '&:active': {
            backgroundColor: tinycolor('#F75660').setAlpha(0.8).toRgbString(),
        },
    },
    'btn-disabled': {
        color: theme.tokens.colors.fontColor1,
        cursor: 'not-allowed',
        background: '#454A54',
        '& .anticon': {
            color: theme.tokens.colors.fontColor1,
        },
    },
    'btn-ripple': {
        position: 'absolute',
        left: '19px',
        top: '11px',
        width: '10px',
        height: '10px',
        borderRadius: '50%',
        backgroundColor: '#8DD0FF',
        opacity: '0.8',
        animation: `${keyframes`
            0% {
                transform: scale(1);
                opacity: 0.8;
            }
            100% {
                transform: scale(3);
                opacity: 0;
            }
        `} 2s infinite`,
    },
    'btn-running-image': {
        position: 'absolute',
        left: '0',
        bottom: '-12px',
        width: '100%',
        height: '16px',
        backgroundImage: `url(${props.ImageAmbientLight})`,
        backgroundSize: 'cover',
    },
}));

export function useStyle() {
    const ImageAmbientLight = useImagePrak('image_ambient_light');

    return useHoc({ ImageAmbientLight });
}
