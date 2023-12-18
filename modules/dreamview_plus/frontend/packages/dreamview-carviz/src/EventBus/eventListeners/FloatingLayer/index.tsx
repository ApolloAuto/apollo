import { useEffect, useState, memo, useRef, useMemo } from 'react';
import { useSpring, animated, interpolate } from '@react-spring/web';
import { eventBus, FunctionType, MouseEventType } from '@dreamview/dreamview-carviz/src/EventBus';
import { throttle } from 'lodash';
import { useTranslation } from 'react-i18next';

import './style.css';

interface IFloatingLayerProps {
    name: string;
}

function FloatingLayer(props: IFloatingLayerProps) {
    const { name } = props;

    const { t } = useTranslation('carviz');

    const floatLayerRef = useRef<HTMLDivElement>(null);

    // 设置阶段,用于判断显示不同的Tooltip
    const [phase, setPhase] = useState<'Start' | 'End'>('Start');

    const [visible, setVisible] = useState(false);

    const visibilitySpringProps = useSpring({
        opacity: visible ? 1 : 0,
        from: { opacity: 0 },
        config: { tension: 220, friction: 20 },
    });

    const [springProps, set] = useSpring(() => ({
        x: 0,
        y: 0,
        config: { tension: 220, friction: 20 },
    }));
    const [worldCoordinates, setWorldCoordinates] = useState({ x: 0, y: 0 });

    const [length, setLength] = useState(0);

    // 航向角
    const [heading, setHeading] = useState();

    // 显示坐标还是长度;
    const [showCoordinates, setShowCoordinates] = useState(true);

    const lengthText = useMemo(() => {
        return `${t('Length')}:${length}m`;
    }, [length, t]);

    const worldCoordinatesText = useMemo(
        () => (
            <>
                &#x005B;
                {worldCoordinates.x}
                {', '}
                {worldCoordinates.y}
                &#x005D; &nbsp;
                {heading}
            </>
        ),
        [worldCoordinates.x, worldCoordinates.y, heading],
    );

    const calculatePosition = (event: MouseEvent) => {
        const xOffset = 20;
        const yOffset = 20;
        const tooltipWidth = floatLayerRef.current?.offsetWidth || 238;
        const tooltipHeight = floatLayerRef.current?.offsetHeight || 86;
        let x = event.clientX + xOffset;
        let y = event.clientY + yOffset;

        if (x + tooltipWidth > window.innerWidth) {
            x = event.clientX - xOffset - tooltipWidth;
        }

        if (y + tooltipHeight > window.innerHeight) {
            y = event.clientY - yOffset - tooltipHeight;
        }

        set({ x, y }); // 更新动画的位置
    };

    const handleMouseMove: FunctionType<any, any> = (data, nativeEvent: MouseEvent) => {
        const { x, y, length, heading, phase = 'Start' } = data;

        if (length !== undefined) {
            setShowCoordinates(false);
            setLength(length);
        }

        if (heading !== undefined) {
            setHeading(heading);
        } else {
            setHeading(undefined);
        }

        if (x !== undefined || y !== undefined) {
            setShowCoordinates(true);
            setWorldCoordinates({ x, y });
        }
        calculatePosition(nativeEvent);
        setPhase(phase);
        setVisible(true);
    };

    const ToolTipName = useMemo(() => `${name}${phase}`, [phase, name]);

    useEffect(() => {
        const handleCustomEvent = throttle((data, nativeEvent) => {
            handleMouseMove(data, nativeEvent);
            setVisible(true);
        }, 100);

        let hideTimeout = null;

        const handleHideEvent = () => {
            if (handleCustomEvent.cancel) {
                handleCustomEvent.cancel();
            }
            clearTimeout(hideTimeout);
            hideTimeout = setTimeout(() => {
                setVisible(false);
            }, 100);
        };

        eventBus.on(MouseEventType.CURRENT_COORDINATES, handleCustomEvent);
        eventBus.on(MouseEventType.CURRENT_LENGTH, handleCustomEvent);
        eventBus.on(MouseEventType.HIDE_CURRENT_COORDINATES, handleHideEvent);
        return () => {
            eventBus.off(MouseEventType.CURRENT_COORDINATES, handleCustomEvent);
            eventBus.off(MouseEventType.CURRENT_LENGTH, handleCustomEvent);
            eventBus.off(MouseEventType.HIDE_CURRENT_COORDINATES, handleHideEvent);
        };
    }, []);

    if (!visible && visibilitySpringProps.opacity.get() === 0) return null;

    const { x, y } = springProps;

    return (
        <animated.div
            ref={floatLayerRef}
            className='dvc-floating-layer'
            style={{
                ...visibilitySpringProps,
                transform: interpolate([x, y], (x, y) => `translate(${x}px, ${y}px)`),
            }}
        >
            <div className='dvc-floating-layer__coordinates'>
                <span>{showCoordinates ? worldCoordinatesText : lengthText}</span>
            </div>
            <div className='dvc-floating-layer__tooltip'>{t(ToolTipName)}</div>
        </animated.div>
    );
}

export default memo(FloatingLayer);
