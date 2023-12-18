import React, { useEffect, useRef, useState } from 'react';
import { IconIcArrowsDown, Collapse } from '@dreamview/dreamview-ui';
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
        let div: any;
        let resizeObsizeOberver: any;
        if (hasExpendChild) {
            div = document.createElement('div');
            resizeObsizeOberver = new ResizeObserver(() => {
                if (hasExpendChild) {
                    div.innerHTML = obverDomRef.current?.innerHTML;
                    div.style.overflow = 'hidden';
                    div.style.position = 'absolute';
                    div.style.top = '-9999px';
                    div.style.visibility = 'hidden';
                    document.documentElement.appendChild(div);
                    setTimeout(() => {
                        setScrollHeight(div.offsetHeight ? `${div.offsetHeight}px` : 'auto');
                        div.remove();
                    });
                }
            });
            resizeObsizeOberver.observe(obverDomRef.current);
        }
        return () => {
            if (div) {
                div.remove();
            }
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
                <IconIcArrowsDown />
            </div>
            <div
                style={{
                    height: isChildShow ? scrollHeight : 0,
                }}
                className={cx(classes['mode-setting-expend'])}
            >
                <div ref={obverDomRef}>{expendChild}</div>
            </div>
        </>
    );
}

export default React.memo(ModeSettingTitle);
