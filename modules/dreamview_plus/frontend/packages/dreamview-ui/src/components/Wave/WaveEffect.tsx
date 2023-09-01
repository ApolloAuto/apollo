/* eslint-disable @typescript-eslint/no-non-null-asserted-optional-chain */
/* eslint-disable @typescript-eslint/no-non-null-assertion */
/* eslint-disable react/function-component-definition */
import classNames from 'classnames';
import CSSMotion from 'rc-motion';
import { render, unmount } from 'rc-util/lib/React/render';
import raf from 'rc-util/lib/raf';
import * as React from 'react';
import { isValidWaveColor } from '../../tools/isValidWaveColor/isValidWaveColor';
import './index.less';

function validateNum(value: number) {
    return Number.isNaN(value) ? 0 : value;
}

export function getTargetWaveColor(node: HTMLElement) {
    const { borderTopColor, borderColor, backgroundColor } = getComputedStyle(node);
    if (isValidWaveColor(borderTopColor)) {
        return borderTopColor;
    }
    if (isValidWaveColor(borderColor)) {
        return borderColor;
    }
    if (isValidWaveColor(backgroundColor)) {
        return backgroundColor;
    }
    return null;
}

export interface WaveEffectProps {
    className: string;
    target: HTMLElement;
}

const WaveEffect: React.FC<WaveEffectProps> = (props) => {
    const { className, target } = props;
    const divRef = React.useRef<HTMLDivElement>(null);

    const [color, setWaveColor] = React.useState<string | null>(null);
    const [borderRadius, setBorderRadius] = React.useState<number[]>([]);
    const [left, setLeft] = React.useState(0);
    const [top, setTop] = React.useState(0);
    const [width, setWidth] = React.useState(0);
    const [height, setHeight] = React.useState(0);
    const [enabled, setEnabled] = React.useState(false);

    const waveStyle = {
        left,
        top,
        width,
        height,
        borderRadius: borderRadius.map((radius) => `${radius}px`).join(' '),
    } as React.CSSProperties & {
        [name: string]: number | string;
    };

    if (color) {
        waveStyle['background-color'] = color;
    }

    function syncPos() {
        const nodeStyle = getComputedStyle(target);

        setWaveColor(getTargetWaveColor(target));

        const isStatic = nodeStyle.position === 'static';

        const { borderLeftWidth, borderTopWidth } = nodeStyle;
        setLeft(isStatic ? target.offsetLeft : validateNum(-parseFloat(borderLeftWidth)));
        setTop(isStatic ? target.offsetTop : validateNum(-parseFloat(borderTopWidth)));
        setWidth(target.offsetWidth);
        setHeight(target.offsetHeight);

        // 获取border
        const { borderTopLeftRadius, borderTopRightRadius, borderBottomLeftRadius, borderBottomRightRadius } =
            nodeStyle;

        setBorderRadius(
            [borderTopLeftRadius, borderTopRightRadius, borderBottomRightRadius, borderBottomLeftRadius].map((radius) =>
                validateNum(parseFloat(radius)),
            ),
        );
    }

    // eslint-disable-next-line consistent-return
    React.useEffect(() => {
        if (target) {
            // requestAnimationFramework

            const id = raf(() => {
                syncPos();

                setEnabled(true);
            });

            // 观测target对象
            let resizeObserver: ResizeObserver;
            if (typeof ResizeObserver !== 'undefined') {
                resizeObserver = new ResizeObserver(syncPos);

                resizeObserver.observe(target);
            }

            return () => {
                raf.cancel(id);
                resizeObserver?.disconnect();
            };
        }

        return () => null;
    }, []);

    if (!enabled) {
        return null;
    }

    return (
        <CSSMotion
            visible
            motionAppear
            motionLeave
            motionEnter
            motionName='wave-motion'
            motionDeadline={600}
            onAppearEnd={(_, event) => {
                if (event.deadline || (event as TransitionEvent).propertyName === 'opacity') {
                    const holder = divRef.current?.parentElement!;
                    unmount(holder).then(() => {
                        holder?.remove();
                    });
                }
                return false;
            }}
        >
            {({ className: motionClassName }) => (
                <div ref={divRef} className={classNames(className, motionClassName)} style={waveStyle} />
            )}
        </CSSMotion>
    );
};

export default function showWaveEffect(node: HTMLElement, className: string) {
    // Create holder
    const holder = document.createElement('div');
    holder.style.position = 'absolute';
    holder.style.left = '0px';
    holder.style.top = '0px';
    node?.insertBefore(holder, node?.firstChild);

    render(<WaveEffect target={node} className={className} />, holder);
}
