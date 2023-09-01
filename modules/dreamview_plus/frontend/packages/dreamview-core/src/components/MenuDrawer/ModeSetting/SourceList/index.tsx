import React, { useMemo, useState } from 'react';
import { IconIcProfileAngerNormal, IconIcSucceed, IconIcPullDownExpansion } from '@dreamview/dreamview-ui';
import SourceEmptyImg from '@dreamview/dreamview-core/src/assets/ic_default_page_no_data.png';
import { useTranslation } from 'react-i18next';
import { useMenuStore, ENUM_MENU_KEY } from '@dreamview/dreamview-core/src/store/MenuStore';
import { UpdateMenuAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import useStyle from './useStyle';
import CustomPopover from '../../../CustomPopover';

interface ISourceList1<T> {
    items: {
        label: string;
        id: string;
        content: T;
    }[];
    onChange: (v: T) => void;
    activeId: string;
}

export function SourceList1<T>(props: ISourceList1<T>) {
    const { items, onChange, activeId } = props;
    const { t } = useTranslation('modeSettings');
    const { classes, cx } = useStyle()();
    const [, dispatch] = useMenuStore();

    const onClick = (item: T) => {
        onChange(item);
    };

    const onChangeIntoResource = () => {
        dispatch(UpdateMenuAction(ENUM_MENU_KEY.PROFILE_MANAGEER));
    };

    return (
        <div className={classes['source-list-container']}>
            {items.length ? (
                <>
                    {items.map((item) => (
                        <div
                            className={cx(classes['source-list-container-item'], {
                                [classes.active]: activeId === item.id,
                            })}
                            key={item.id}
                            onClick={onClick.bind(null, item.content)}
                        >
                            <span title={item.label} className={classes['source-list-name']}>
                                {item.label}
                            </span>
                            {activeId === item.id ? (
                                <IconIcSucceed
                                    className={cx(classes['source-list-operate'], 'source-list-operate-hover')}
                                />
                            ) : (
                                <CustomPopover trigger='hover' content={t('use')}>
                                    <IconIcProfileAngerNormal
                                        className={cx(classes['source-list-operate'], 'source-list-operate-hover')}
                                    />
                                </CustomPopover>
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
    onChange: (v: T) => void;
    activeId: string;
    item: {
        label: string;
        id: string;
        child: ISourceList1<T>['items'];
    };
}

function SourceList2Item<T>(props: ISourceList2Item<T>) {
    const { item, onChange, activeId } = props;
    const { classes, cx } = useStyle()({ height: (item.child?.length || 0) * 40 });
    const [expand, setExpand] = useState<boolean>(false);
    const onClick = () => {
        setExpand((prev) => !prev);
    };

    return (
        <>
            <div onClick={onClick} className={cx(classes['source-list-title'])}>
                <IconIcPullDownExpansion
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
                <div>
                    <SourceList1<T> activeId={activeId} onChange={onChange} items={item.child} />
                </div>
            </div>
        </>
    );
}

interface ISourceList2<T> {
    activeId: string;
    items: ISourceList2Item<T>['item'][];
    onChange: (v: T) => void;
}

export function SourceList2<T>(props: ISourceList2<T>) {
    const { items, onChange, activeId } = props;
    const { classes } = useStyle()();

    return (
        <div className={classes['source-list-container']}>
            {items.map((item) => (
                <SourceList2Item<T> activeId={activeId} onChange={onChange} item={item} key={item.id} />
            ))}
        </div>
    );
}
