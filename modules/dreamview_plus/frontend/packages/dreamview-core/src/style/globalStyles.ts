import baseTokens from '@dreamview/dreamview-theme/tokens/baseToken';
import guideStyles from './guideStyles';
import scrollBar from './scrollBar';

export default {
    '*': {
        margin: 0,
        padding: 0,
        boxSizing: 'border-box',
    },
    ul: {
        padding: 0,
        margin: 0,
    },
    li: {
        listStyle: 'none',
    },
    // react-mosaic-component样式重置
    // 隐藏掉react-mosaic-component自带的toolbar，重新实现
    '.mosaic .mosaic-window .mosaic-window-toolbar': {
        display: 'none',
    },
    // 隐藏掉窗口跟元素的padding的间隙
    '.mosaic-root': {
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        color: baseTokens.font.color.main,
        fontFamily: baseTokens.font.fontFamily,
        fontSize: baseTokens.font.size.regular,
        '.drop-target-container .drop-target-hover': {
            opacity: 0,
        },
    },
    '.mosaic-drop-target .drop-target-container .drop-target': {
        backgroundColor: 'rgba(41,125,236,0.10)',
        border: '2px solid rgba(41,125,236,0.4)',
    },
    // 调整窗口之间的间隙
    '.mosaic-tile': {
        margin: '0.5px',
    },
    '.mosaic-split': {
        zIndex: 2,
        backgroundClip: 'content-box !important',
    },
    '.mosaic-split.-row': {
        padding: '0 4px',
        width: '10px',
        marginLeft: '-4px',
    },
    '.mosaic-split.-column': {
        padding: '4px 0',
        height: '10px',
        marginTop: '-4px',
    },
    '.mosaic-split:hover': {
        backgroundColor: 'rgba(41,125,236,1)',
    },
    '.mosaic-window .mosaic-window-body, .mosaic-preview .mosaic-window-body': {
        backgroundColor: '#0F1014',
    },
    // /* 指定滚动条的宽度和颜色 */
    // '::-webkit-scrollbar': {
    //     width: '4px' /* 滚动条宽度 */,
    //     backgroundColor: 'transparent' /* 滚动条背景颜色 */,
    // },

    // /* 指定滚动条轨道的样式 */
    // '::-webkit-scrollbar-track': {
    //     backgroundColor: 'transparent' /* 滚动条轨道背景颜色 */,
    // },

    // /* 指定滚动条滑块的样式 */
    // '::-webkit-scrollbar-thumb': {
    //     backgroundColor: '#3D434E' /* 滚动条滑块颜色 */,
    //     borderRadius: '4px',
    // },
    '.mosaic-drop-target': {
        height: '100%',
    },
    ...guideStyles,
    ...scrollBar,
};
