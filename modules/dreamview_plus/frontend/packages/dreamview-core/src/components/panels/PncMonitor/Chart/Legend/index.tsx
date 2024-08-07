import { makeStyles, useThemeContext } from '@dreamview/dreamview-theme';
import React, { useEffect, useMemo, useState } from 'react';
import { IconPark } from '@dreamview/dreamview-ui';

const useStyle = makeStyles((theme) => ({
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
        color: theme.tokens.colors.fontColor5,
        ...theme.tokens.typography.sideText,
        marginRight: '10px',
        '& .anticon': {
            marginRight: '6px',
        },
    },
    'legend-unactive': {
        position: 'relative',
        color: theme.tokens.colors.fontColor1,
        '& .anticon': {
            color: `${theme.tokens.colors.fontColor1} !important`,
        },
        '&::after': {
            content: '""',
            position: 'absolute',
            left: 0,
            right: 0,
            margin: 'auto',
            height: '1px',
            background: theme.tokens.colors.fontColor1,
        },
    },
    'legend-icon': {},
}));

interface LegendProps {
    legends: Array<{ name: string; color: string }>;
    onClick: (key: string) => void;
}

function Legend(props: LegendProps) {
    const { legends, onClick: propOnClick } = props;
    const { classes, cx } = useStyle();
    const [selected, setSelected] = useState<Record<string, boolean>>({});
    const { tokens } = useThemeContext();
    useEffect(() => {
        const result: Record<string, boolean> = {};
        legends.forEach(({ name }) => {
            if (name in selected) {
                result[name] = selected[name];
            } else {
                result[name] = true;
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
                            [classes['legend-unactive']]: !selected[item.name],
                        })}
                        onClick={() => onClick(item.name)}
                        key={`${item.name}_${index + 1}`}
                    >
                        <IconPark
                            name='IcClassificationNotes'
                            // className={classes}
                            style={{ color: item.color || tokens.components.pncMonitor.chartColors[index] }}
                        />
                        {item.name}
                    </div>
                ))}
            </div>
        </div>
    );
}

export default React.memo(Legend);
