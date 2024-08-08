import React from 'react';
import { Input as InternalInput, InputProps } from 'antd';
import { TextAreaProps } from 'antd/es/input';
import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
// import './index.less';

const { TextArea: InternalTextArea } = InternalInput;

const useHoc = makeStylesWithProps<{ classname: string; bordered: boolean }>()((theme, prop) => {
    const tokens = theme.components.input;
    return {
        [prop.classname]: {
            '&.dreamview-input': {
                height: '36px',
                lineHeight: '36px',
                padding: '0 14px',
                color: tokens.color,
                background: tokens.bgColor,
                boxShadow: tokens.boxShadow,
                border: tokens.borderInGray,
                '&:hover': {
                    background: tokens.bgColorHover,
                    boxShadow: tokens.boxShadowHover,
                },
            },
            '&.dreamview-input-affix-wrapper': {
                background: tokens.bgColor,
                border: tokens.borderInGray,
                color: tokens.color,
                boxShadow: tokens.boxShadow,
                '& input': {
                    background: tokens.bgColor,
                    color: tokens.color,
                },
                '&:hover': {
                    border: prop.bordered ? tokens.borderInWhite : tokens.borderInGray,
                    background: tokens.bgColorHover,
                    boxShadow: tokens.boxShadowHover,
                },
            },
            '& .dreamview-input-clear-icon': {
                fontSize: '16px',
                '& .anticon': {
                    display: 'block',
                    color: theme.tokens.font.iconReactive.main,
                    '&:hover': {
                        color: theme.tokens.font.iconReactive.hover,
                    },
                    '&:active': {
                        color: theme.tokens.font.iconReactive.active,
                    },
                },
            },
        },
        'border-line': {
            '&.dreamview-input': {
                border: tokens.borderInWhite,
            },
            '&.dreamview-input-affix-wrapper': {
                border: tokens.borderInWhite,
            },
        },
    };
});

function useStyle(classname: string, bordered: boolean) {
    return useHoc({ bordered, classname });
}

export function Input(props: InputProps) {
    const { prefixCls: customizePrefixCls, children, className, bordered = true, ...rest } = props;
    const prefixCls = getPrefixCls('input', customizePrefixCls);

    const { classes, cx } = useStyle(prefixCls, bordered);

    return (
        <InternalInput
            className={cx(classes[prefixCls], className, { [classes['border-line']]: bordered })}
            prefixCls={prefixCls}
            {...rest}
        >
            {children}
        </InternalInput>
    );
}
Input.propTypes = {};

Input.defaultProps = {};

Input.displayName = 'Input';

export function TextArea(props: TextAreaProps) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('text-area', customizePrefixCls);

    return (
        <InternalTextArea prefixCls={prefixCls} {...rest}>
            {children}
        </InternalTextArea>
    );
}
TextArea.propTypes = {};

TextArea.defaultProps = {};

TextArea.displayName = 'Text-Area';
