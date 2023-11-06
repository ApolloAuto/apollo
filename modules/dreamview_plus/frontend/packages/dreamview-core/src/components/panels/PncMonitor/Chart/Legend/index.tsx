import { useMakeStyle } from '@dreamview/dreamview-theme';
import React, { useEffect, useMemo, useState } from 'react';
import { IconIcClassificationNotes } from '@dreamview/dreamview-ui';

const colors = ['#3288FA', '#1FCC4D', '#FF8D26', '#44D7B6', '#FFEC3D', '#B37FEB', '#F75660'];

function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'legend-flex-container': {
            textAlign: 'center',
            marginBottom: '-6px',
            marginTop: theme.tokens.margin.speace2,
        },
        'legend-container': {
            display: 'inline-flex',
            flexWrap: 'wrap',
            margin: `0 6px 0px ${theme.tokens.margin.speace2}`,
        },
        'legend-item': {
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            userSelect: 'none',
            color: theme.tokens.colors.fontColor2,
            ...theme.tokens.typography.sideText,
            marginRight: '10px',
            '& .anticon': {
                marginRight: '6px',
            },
        },
        'legend-unactive': {
            position: 'relative',
            color: theme.tokens.colors.fontColor4,
            '& .anticon': {
                color: `${theme.tokens.colors.fontColor4} !important`,
            },
            '&::after': {
                content: '""',
                position: 'absolute',
                left: 0,
                right: 0,
                margin: 'auto',
                height: '1px',
                background: theme.tokens.colors.fontColor4,
            },
        },
        'legend-icon': {},
    }));

    return hoc();
}

interface LegendProps {
    legends: string[];
    onClick: (key: string) => void;
}

function Legend(props: LegendProps) {
    const { legends, onClick: propOnClick } = props;
    const { classes, cx } = useStyle();
    const [selected, setSelected] = useState<Record<string, boolean>>({});

    useEffect(() => {
        const result: Record<string, boolean> = {};
        legends.forEach((key) => {
            if (key in selected) {
                result[key] = selected[key];
            } else {
                result[key] = true;
            }
        });
        setSelected(result);
    }, [legends]);

    const onClick = (key: string) => {
        setSelected((prev) => ({
            ...prev,
            [key]: !prev[key],
        }));
        propOnClick(key);
    };

    if (legends.length <= 1) {
        return null;
    }

    return (
        <div className={classes['legend-flex-container']}>
            <div className={classes['legend-container']}>
                {legends.map((item, index) => (
                    <div
                        className={cx(classes['legend-item'], {
                            [classes['legend-unactive']]: !selected[item],
                        })}
                        onClick={() => onClick(item)}
                        key={item}
                    >
                        <IconIcClassificationNotes className={classes} style={{ color: colors[index] }} />
                        {item}
                    </div>
                ))}
            </div>
        </div>
    );
}

export default React.memo(Legend);
