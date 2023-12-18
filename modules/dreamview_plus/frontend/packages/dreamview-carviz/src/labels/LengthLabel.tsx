import { memo, useEffect, useMemo, useRef } from 'react';
import { useTranslation } from 'react-i18next';

type LengthLabelProps = { length: number; totalLength?: never } | { length?: never; totalLength: number };

function LengthLabel(props: LengthLabelProps) {
    const { length, totalLength } = props;

    const { t } = useTranslation('carviz');

    const lengthText = useMemo(() => {
        if (length) {
            return `${t('Length')}: ${length.toFixed(2)}m`;
        }
        if (totalLength) {
            return `${t('TotalLength')}: ${totalLength.toFixed(2)}m`;
        }
        return '';
    }, [length, t, totalLength]);

    const labelRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        if (labelRef.current) {
            if (length) {
                labelRef.current.style.transform = 'translate(-60%, 50%)';
            }
            if (totalLength) {
                labelRef.current.style.transform = 'translate(80%, -50%)';
            }
        }
    }, [length, totalLength]);

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
            {lengthText}
        </div>
    );
}

export default memo(LengthLabel);
