import { memo } from 'react';

interface AngleLabelProps {
    angle: number;
}

function AngleLabel(props: AngleLabelProps) {
    const { angle } = props;
    return (
        <div
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
            {`${angle.toFixed(2)}°`}
        </div>
    );
}

export default memo(AngleLabel);
