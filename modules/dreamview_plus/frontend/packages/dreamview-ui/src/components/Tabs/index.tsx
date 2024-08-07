import React from 'react';
import { Tabs as InternalTabs, TabsProps } from 'antd';
import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

const useStyle = makeStylesWithProps<{ classname: string }>()((theme, props) => ({
    [props.classname]: {
        '&.dreamview-tabs-top': {
            '&>.dreamview-tabs-nav::before': {
                border: 'none',
            },
        },
        '& .dreamview-tabs-nav .dreamview-tabs-nav-list': {
            display: 'inline-flex',
            flex: 'none',
            background: theme.components.tab.bgColor,
            borderRadius: '6px',
        },
        '.dreamview-tabs-tab': {
            padding: '5px 16px',
            minWidth: '106px',
            justifyContent: 'center',
            margin: '0 !important',
            backgroundColor: theme.components.tab.tabItemBgColor,
            color: theme.components.tab.color,
            fontFamily: 'PingFangSC-Regular',
            fontWeight: 400,
            borderRadius: '6px',
        },
        '.dreamview-tabs-ink-bar': {
            display: 'none',
        },
        '.dreamview-tabs-tab.dreamview-tabs-tab-active .dreamview-tabs-tab-btn': {
            color: theme.components.tab.activeColor,
        },
        '.dreamview-tabs-tab.dreamview-tabs-tab-active ': {
            backgroundColor: theme.components.tab.activeBgColor,
            borderRadius: '6px',
        },
    },
    'in-gray': {
        '.dreamview-tabs-tab': {
            background: theme.components.tab.bgColorInBackground,
        },
        '.dreamview-tabs-nav .dreamview-tabs-nav-list': {
            boxShadow: theme.components.tab.boxShadowInBackground,
        },
        '.dreamview-tabs-nav .dreamview-tabs-nav-wrap': {
            overflow: 'visible',
        },
    },
}));

export function Tabs(props: TabsProps & { inGray?: boolean }) {
    const { children, prefixCls: customClassNames, className, inGray = false, ...rest } = props;
    const prefixCls = getPrefixCls('tabs', customClassNames);
    const { classes, cx } = useStyle({ classname: prefixCls });
    return (
        <InternalTabs
            prefixCls={prefixCls}
            className={cx(classes[prefixCls], { [classes['in-gray']]: inGray }, className)}
            {...rest}
        >
            {children}
        </InternalTabs>
    );
}

Tabs.propTypes = {
    // /**
    //  * Is this the principal call to action on the page?
    //  */
    // primary: PropTypes.bool,
    // /**
    //  * What background color to use
    //  */
    // backgroundColor: PropTypes.oneOf(['blue', 'red']),
    // /**
    //  * How large should the button be?
    //  */
    // size: PropTypes.oneOf(['small', 'medium', 'large']),
    // /**
    //  * Button contents
    //  */
    // label: PropTypes.string.isRequired,
    // /**
    //  * Optional click handler
    //  */
    // onClick: PropTypes.func,
};

Tabs.defaultProps = {};

Tabs.displayName = 'Tabs';
