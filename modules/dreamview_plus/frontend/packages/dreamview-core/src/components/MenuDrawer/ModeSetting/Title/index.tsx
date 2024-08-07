import React, { useEffect, useRef, useState } from 'react';
import { IconPark, Collapse } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';

interface IModeSettingTitle {
    title: string | JSX.Element;
    expendChild?: JSX.Element;
    defaultExpend?: boolean;
}

function ModeSettingTitle(props: React.PropsWithChildren<IModeSettingTitle>) {
    const { title, expendChild, defaultExpend = true } = props;
    const { classes, cx } = useStyle();
    const hasExpendChild = !!expendChild;
    const [isChildShow, setIsChildShow] = useState(defaultExpend);
    const [scrollHeight, setScrollHeight] = useState<string>('auto');

    const onToogleDisplay = () => {
        setIsChildShow((prev) => !prev);
    };

    const obverDomRef = useRef<any>();

    useEffect(() => {
        let resizeObsizeOberver: any;
        let timer: number;
        if (hasExpendChild) {
            resizeObsizeOberver = new ResizeObserver(() => {
                if (hasExpendChild) {
                    clearTimeout(timer);
                    timer = window.setTimeout(() => {
                        clearTimeout(timer);
                        setScrollHeight(
                            obverDomRef.current.offsetHeight ? `${obverDomRef.current.offsetHeight}px` : 'auto',
                        );
                    });
                }
            });
            resizeObsizeOberver.observe(obverDomRef.current);
        }
        return () => {
            clearTimeout(timer);
            if (resizeObsizeOberver) {
                resizeObsizeOberver.disconnect();
            }
        };
    }, []);

    if (!hasExpendChild) {
        return <div className={classes['mode-setting-title']}>{title}</div>;
    }

    return (
        <>
            <div
                onClick={onToogleDisplay}
                className={cx(classes['mode-setting-title'], {
                    [classes['mode-setting-icon-active']]: isChildShow,
                })}
            >
                {title}
                <IconPark name='IcArrowsDown' />
            </div>
            <div
                style={{
                    height: isChildShow ? scrollHeight : 0,
                }}
                className={cx(classes['mode-setting-expend'])}
            >
                <div className={cx(classes['overflow-hidden'])} ref={obverDomRef}>
                    {expendChild}
                </div>
            </div>
        </>
    );
}

export default React.memo(ModeSettingTitle);
