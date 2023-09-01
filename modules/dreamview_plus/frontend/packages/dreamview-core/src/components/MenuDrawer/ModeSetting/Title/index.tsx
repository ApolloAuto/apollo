import React, { useState } from 'react';
// import Collapse from 'rc-collapse';
// import 'rc-collapse/assets/index.css';
import { IconIcArrowsDown, Collapse } from '@dreamview/dreamview-ui';
import shortUUID from 'short-uuid';
import useStyle from './useStyle';

interface IModeSettingTitle {
    title: string | JSX.Element;
    expendChild?: JSX.Element;
    defaultExpend?: boolean;
}

function IcArrowsDown(props: any) {
    const { isActive } = props;
    return <IconIcArrowsDown rotate={isActive ? 180 : 0} />;
}
function ModeSettingTitle(props: React.PropsWithChildren<IModeSettingTitle>) {
    const { title, expendChild, defaultExpend = true } = props;
    const { classes, cx } = useStyle();
    const hasExpendChild = !!expendChild;
    const [uid] = useState(shortUUID.generate);

    if (!hasExpendChild) {
        return <div className={classes['mode-setting-title']}>{title}</div>;
    }
    const items = [
        {
            label: (
                <div className={cx(classes['mode-setting-title'], classes['mode-setting-title-expend'])}>{title}</div>
            ),
            key: uid,
            children: expendChild,
        },
    ];

    return (
        <Collapse
            defaultActiveKey={defaultExpend && uid}
            accordion
            className={classes['mode-setting-collapse']}
            items={items}
            expandIcon={IcArrowsDown}
        />
    );
}

export default React.memo(ModeSettingTitle);
