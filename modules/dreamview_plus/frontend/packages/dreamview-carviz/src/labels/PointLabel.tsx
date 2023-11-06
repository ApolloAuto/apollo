import { memo, useEffect, useRef } from 'react';

interface PointLabelProps {
    coordinate: { x: number; y: number };
}

function PointLabel(props: PointLabelProps) {
    const {
        coordinate = {
            x: 0,
            y: 0,
        },
    } = props;

    const labelRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        if (labelRef.current) {
            labelRef.current.style.transform = 'translate(-60%, 50%)';
        }
    }, []);

    return (
        <div
            ref={labelRef}
            style={{
                fontFamily: 'PingFangSC-Regular',
                fontSize: '14px',
                color: '#fff',
                lineHeight: '22px',
                fontWeight: 400,
                padding: '5px 8px',
                background: '#505866',
                borderRadius: '6px',
                boxShadow: '0 6px 12px 6px rgb(0 0 0 / 20%)',
            }}
        >
            &#x005B;
            {coordinate.x}
            {', '}
            {coordinate.y}
            &#x005D;
        </div>
    );
}

export default memo(PointLabel);
