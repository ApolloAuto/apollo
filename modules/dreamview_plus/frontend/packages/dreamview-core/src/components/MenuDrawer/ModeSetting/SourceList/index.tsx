import React, { useCallback, useMemo, useState } from 'react';
import { IconPark, useImagePrak, Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { useMenuStore, ENUM_MENU_KEY } from '@dreamview/dreamview-core/src/store/MenuStore';
import { useThemeContext } from '@dreamview/dreamview-theme';
import { UpdateMenuAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import useStyle from './useStyle';
import { RECORDER_LOAD_STATUS, usePickHmiStore } from '../../../../store/HmiStore';
import { Spinner } from '../Spinner';

interface ISourceList1<T> {
    items: {
        label: string;
        id: string;
        content: T;
        preLoad?: RECORDER_LOAD_STATUS;
    }[];
    onChange: (v: T, preLoad?: RECORDER_LOAD_STATUS) => void;
    activeId: string;
    type?: string;
}

export function SourceList1<T>(props: ISourceList1<T>) {
    const { items, onChange, activeId, type } = props;
    const { t } = useTranslation('modeSettings');
    const { tokens } = useThemeContext();
    const { classes, cx } = useStyle({});
    const [, dispatch] = useMenuStore();
    const [hmi] = usePickHmiStore();
    const SourceEmptyImg = useImagePrak('ic_empty_page_no_data');

    const onClick = (item: T, preLoad?: RECORDER_LOAD_STATUS) => {
        if (type === 'HDMap' && hmi.envResourcesHDMapDisable) {
            return;
        }
        onChange(item, preLoad);
    };

    const onChangeIntoResource = () => {
        dispatch(UpdateMenuAction(ENUM_MENU_KEY.PROFILE_MANAGEER));
    };

    const renderRightByPreload = useCallback((preLoad: RECORDER_LOAD_STATUS) => {
        switch (preLoad) {
            case RECORDER_LOAD_STATUS.LOADING:
                return (
                    <div style={{ marginRight: '14px' }}>
                        <Spinner />
                    </div>
                );
            case RECORDER_LOAD_STATUS.LOADED:
                return (
                    <Popover trigger='hover' content={t('use')}>
                        <IconPark
                            name='IcProfileAngerNormal'
                            className={cx(classes['source-list-operate'], 'source-list-operate-hover')}
                        />
                    </Popover>
                );
            case RECORDER_LOAD_STATUS.NOT_LOAD:
                return null;
            default:
                return (
                    <Popover trigger='hover' content={t('use')}>
                        <IconPark
                            name='IcProfileAngerNormal'
                            className={cx(classes['source-list-operate'], 'source-list-operate-hover')}
                        />
                    </Popover>
                );
        }
    }, []);

    const renderFontColorByPreload = useCallback(
        (preLoad: RECORDER_LOAD_STATUS) => {
            // 灰色字体
            const grayColorFont = {
                color: tokens.components.sourceItem.disabledColor,
            };
            switch (preLoad) {
                case RECORDER_LOAD_STATUS.LOADING:
                    return grayColorFont;
                case RECORDER_LOAD_STATUS.LOADED:
                    return {};
                case RECORDER_LOAD_STATUS.NOT_LOAD:
                    return grayColorFont;
                default:
                    return {};
            }
        },
        [tokens],
    );

    return (
        <div className={classes['source-list-container']}>
            {items.length ? (
                <>
                    {items.map((item) => (
                        <div
                            className={cx(classes['source-list-container-item'], {
                                [classes.active]: activeId === item.id,
                                'source-list-container-item-disbale': type === 'HDMap' && hmi.envResourcesHDMapDisable,
                            })}
                            key={item.id}
                            style={renderFontColorByPreload(item.preLoad)}
                            onClick={onClick.bind(null, item.content, item.preLoad)}
                        >
                            <span title={item.label} className={classes['source-list-name']}>
                                {item.label}
                            </span>

                            {/* {item.preLoad === RECORDER_LOAD_STATUS.LOADING && <Spinner />} */}
                            {activeId === item.id ? (
                                <IconPark
                                    name='IcSucceed'
                                    className={cx(classes['source-list-operate'], 'source-list-operate-hover')}
                                />
                            ) : (
                                renderRightByPreload(item.preLoad)
                            )}
                        </div>
                    ))}
                </>
            ) : (
                <div className={classes.empty}>
                    <img alt='resource_empty' src={SourceEmptyImg} />
                    <div>{t('noData')}</div>
                    <div className={classes['empty-msg']}>
                        {t('noDataMsg1')}
                        <span onClick={onChangeIntoResource}>{t('noDataMsg2')}</span>
                        {t('noDataMsg3')}
                    </div>
                </div>
            )}
        </div>
    );
}

interface ISourceList2Item<T> {
    onChange: (v: ISourceList2Item<T>['item'], i: T) => void;
    activeId: string;
    onExpand: (v: string) => void;
    expandChildId: string;
    item: {
        label: string;
        id: string;
        child: ISourceList1<T>['items'];
    };
}

function SourceList2Item<T>(props: ISourceList2Item<T>) {
    const { item, expandChildId, onChange: propsOnChange, activeId, onExpand } = props;
    const { classes, cx } = useStyle({ height: (item.child?.length || 0) * 32 });
    const expand = expandChildId === item.id;
    const onClick = () => {
        onExpand(item.id);
    };

    const onChange = useCallback(
        (subItem: T) => {
            propsOnChange(item, subItem);
        },
        [propsOnChange],
    );

    return (
        <>
            <div onClick={onClick} className={cx(classes['source-list-title'])}>
                <IconPark
                    name='IcPullDownExpansion'
                    className={cx(classes['source-list-title-icon'], {
                        [classes['source-list-title-icon-expand']]: expand,
                    })}
                />
                <span
                    className={cx(classes['source-list-title-text'], 'source-list-title-text-hover')}
                    title={item.label}
                >
                    {item.label}
                </span>
            </div>
            <div className={cx(classes['source-list-close'], { [classes['source-list-expand']]: expand })}>
                {expand ? (
                    <div>
                        <SourceList1<T> activeId={activeId} onChange={onChange} items={item.child} />
                    </div>
                ) : null}
            </div>
        </>
    );
}

interface ISourceList2<T> {
    activeId: string;
    items: ISourceList2Item<T>['item'][];
    onChange: (v: ISourceList2Item<T>['item'], i: T) => void;
}

export function SourceList2<T>(props: ISourceList2<T>) {
    const { items, onChange, activeId } = props;
    const { classes } = useStyle({});
    const [, dispatch] = useMenuStore();
    const [expandChildId, setExpandChildId] = useState<string>(null);
    const SourceEmptyImg = useImagePrak('ic_empty_page_no_data');

    const { t } = useTranslation('modeSettings');

    const onExpand = useCallback((id: string) => {
        setExpandChildId((prev: string) => (prev === id ? null : id));
    }, []);

    const onChangeIntoResource = () => {
        dispatch(UpdateMenuAction(ENUM_MENU_KEY.PROFILE_MANAGEER));
    };

    return (
        <div className={classes['source-list-container']}>
            {items.length ? (
                items.map((item) => (
                    <SourceList2Item<T>
                        onExpand={onExpand}
                        expandChildId={expandChildId}
                        activeId={activeId}
                        onChange={onChange}
                        item={item}
                        key={item.id}
                    />
                ))
            ) : (
                <div className={classes.empty}>
                    <img alt='resource_empty' src={SourceEmptyImg} />
                    <div>{t('noData')}</div>
                    <div className={classes['empty-msg']}>
                        {t('noDataMsg1')}
                        <span onClick={onChangeIntoResource}>{t('noDataMsg2')}</span>
                        {t('noDataMsg3')}
                    </div>
                </div>
            )}
        </div>
    );
}
