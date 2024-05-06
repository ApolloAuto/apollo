import { memo, useEffect } from 'react';
// @ts-ignore
import IcClose from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcClose.png';
import { IPolyMarker } from '../render/type';

interface CloseLabelProps {
    polyline: IPolyMarker;
    clearThePolyline: (polyline: IPolyMarker) => void;
}

function CloseLabel(props: CloseLabelProps) {
    const { polyline, clearThePolyline } = props;

    const handleClick = () => {
        clearThePolyline(polyline);
    };
    return (
        <div
            style={{
                width: '20px',
                height: '20px',
                pointerEvents: 'auto',
                cursor: 'pointer',
                zIndex: 990,
            }}
            onClick={handleClick}
        >
            <img
                style={{
                    width: '20px',
                    height: '20px',
                }}
                src={IcClose}
                alt='关闭'
            />
        </div>
    );
}

export default memo(CloseLabel);
