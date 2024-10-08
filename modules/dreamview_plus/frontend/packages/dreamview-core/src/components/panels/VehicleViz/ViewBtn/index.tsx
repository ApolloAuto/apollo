import React, { useCallback, useState, PropsWithChildren, useEffect } from 'react';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { Nullable } from '@dreamview/dreamview-core/src/util/similarFunctions';
import { Carviz, RoutingEditor } from '@dreamview/dreamview-carviz';
import { useTranslation } from 'react-i18next';
// import Popover from '@dreamview/dreamview-core/src/components/CustomPopover';
import useStyle from '../useStyle';
import { MutexToolNameEnum } from '../RoutingEditing/RoutingEditingFunctionalArea/types';

interface IViewMenuProps extends PropsWithChildren {
    carviz: Carviz | RoutingEditor;
    changeActiveName?: (name: MutexToolNameEnum) => void;
    activeName?: Nullable<MutexToolNameEnum>;
}

type FunctionalNameEnum = 'Rule' | 'Copy' | 'Magnify' | 'Shrink';

function ViewMenu(props: IViewMenuProps) {
    const { carviz, children, changeActiveName, activeName } = props;

    const { t } = useTranslation('carviz');

    const { classes } = useStyle();

    const [checkedItem, setCheckedItem] = useState<Nullable<FunctionalNameEnum>>(null);

    const handleClick = useCallback(
        (name: FunctionalNameEnum) => () => {
            if (checkedItem === name) {
                if (Object.values(MutexToolNameEnum).includes(name)) {
                    carviz?.deactiveAll();
                }
                setCheckedItem(null);
                return;
            }

            setCheckedItem(name);
            switch (name) {
                case 'Rule':
                    carviz?.deactiveAll();
                    changeActiveName?.(MutexToolNameEnum.RULE);
                    carviz?.rulerMarker.active();
                    break;
                case 'Copy':
                    carviz?.deactiveAll();
                    changeActiveName?.(MutexToolNameEnum.COPY);
                    carviz?.copyMarker.active();
                    break;
                case 'Magnify':
                    carviz.view?.updateViewDistance(-10);
                    break;
                case 'Shrink':
                    carviz.view?.updateViewDistance(10);
                    break;
                default:
                    break;
            }
        },
        [checkedItem, carviz],
    );

    useEffect(() => {
        if (!Object.values(MutexToolNameEnum).includes(activeName)) return;
        if (![MutexToolNameEnum.RULE, MutexToolNameEnum.COPY].includes(activeName)) {
            setCheckedItem(null);
        }
    }, [activeName, carviz]);

    return (
        <>
            <Popover trigger='hover' placement='left' content={t('RuleIcon')}>
                <div className={classes['view-menu-btn-item-only']} onClick={handleClick(MutexToolNameEnum.RULE)}>
                    <IconPark name='IcRanging'
                        style={{
                            fontSize: '16px',
                            color: checkedItem === MutexToolNameEnum.RULE ? '#3388FA' : '#96A5C1',
                        }}
                    />
                </div>
            </Popover>

            <Popover trigger='hover' placement='left' content={t('CopyIcon')}>
                <div className={classes['view-menu-btn-item-only']} onClick={handleClick(MutexToolNameEnum.COPY)}>
                    <IconPark name='IcCopy'
                        style={{
                            fontSize: '16px',
                            color: checkedItem === MutexToolNameEnum.COPY ? '#3388FA' : '#96A5C1',
                        }}
                    />
                </div>
            </Popover>
            {children && <>{children}</>}
            <div className={classes['view-menu-scale-btn-container']}>
                <div className={classes['view-menu-btn-item']} onClick={handleClick('Magnify')}>
                    <IconPark name='IcAmplification' style={{ fontSize: '16px', color: '#96A5C1' }} />
                </div>
                <div className={classes['view-menu-btn-item']} onClick={handleClick('Shrink')}>
                    <IconPark name='IcReduce' style={{ fontSize: '16px', color: '#96A5C1' }} />
                </div>
            </div>
        </>
    );
}

export default React.memo(ViewMenu);
