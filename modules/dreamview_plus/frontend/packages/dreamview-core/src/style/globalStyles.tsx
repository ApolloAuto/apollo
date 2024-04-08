import React, { useMemo } from 'react';
import { GlobalStyles, CSSInterpolation } from 'tss-react';
import { useThemeContext } from '@dreamview/dreamview-theme';
import guideStyles from './guideStyles';
import scrollBar from './scrollBar';

function GlobalCss() {
    const { tokens: themeTokens } = useThemeContext();

    const style = useMemo(
        () => ({
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
                color: themeTokens.tokens.font.color.main,
                fontFamily: themeTokens.tokens.font.fontFamily,
                fontSize: themeTokens.tokens.font.size.regular,
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
                margin: '0',
                borderTop: `1px solid ${themeTokens.tokens.colors.divider2}`,
                borderLeft: `1px solid ${themeTokens.tokens.colors.divider2}`,
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
                backgroundColor: themeTokens.tokens.colors.background3,
            },
            '.mosaic-drop-target': {
                height: '100%',
            },
            '.ant-form-item .ant-form-item-label': {
                padding: 0,
                textAlign: 'right',
            },
            '.ant-form-item .ant-form-item-label > label::after': {
                content: '":"',
                position: 'relative',
                display: 'block',
                marginBlock: 0,
                marginInlineStart: '2px',
                marginInlineEnd: '8px',
            },
            ...guideStyles(themeTokens),
            ...scrollBar,
        }),
        [themeTokens],
    );
    return <GlobalStyles styles={style as CSSInterpolation} />;
}

export default React.memo(GlobalCss);
