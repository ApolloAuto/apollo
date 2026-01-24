import React, { useMemo } from 'react';
import { useTranslation } from 'react-i18next';
import { IconPark } from '@dreamview/dreamview-ui';
import { usePanelCatalogContext } from '@dreamview/dreamview-core/src/store/PanelCatalogStore';
import { usePanelLayoutStore } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { update } from '@dreamview/dreamview-core/src/store/PanelLayoutStore/actions';
import { getPanelTypeByPanelId } from '@dreamview/dreamview-core/src/util/layout';
import { useMenuStore, ENUM_MENU_KEY } from '@dreamview/dreamview-core/src/store/MenuStore';
import { useThemeContext } from '@dreamview/dreamview-theme';
import CustomScroll from '@dreamview/dreamview-core/src/components/CustomScroll';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import useStyle from './useStyle';
import ImgAddPanel from './icon_add_panel.png';

const spaceWidth = 12;

const space4 = `${spaceWidth * 4}px`;
const space2 = `${spaceWidth * 2}px`;

function AddIcon(prop: { className: string }) {
    const { theme } = useThemeContext();
    if (theme === 'drak') {
        return <IconPark name='IcAddPanel' />;
    }

    return (
        <div className={prop.className}>
            <img alt='add panel' src={ImgAddPanel} />
        </div>
    );
}

function PanelEmpty() {
    const { t } = useTranslation('panels');
    const { allPanel } = usePanelCatalogContext();
    const [, dispatch] = usePanelLayoutStore();
    const [hmi] = usePickHmiStore();
    const [{ activeMenu }] = useMenuStore();
    const isMenuHide = activeMenu === ENUM_MENU_KEY.HIDDEN;
    const aotuWith = { width: isMenuHide ? `calc((100% - ${space4}) / 5)` : `calc((100% - ${space2}) / 3)` };

    const { classes, cx } = useStyle();
    const onAddPanel = (type: string) => {
        dispatch(update({ mode: hmi.currentMode, layout: getPanelTypeByPanelId(type) }));
    };
    const flexFillElem = useMemo(
        () => new Array(4).fill(0).map((_, index: number) => <div style={aotuWith} key={`${index + 1}`} />),
        [],
    );
    return (
        <div className={classes['panel-empty-container']}>
            <div className={classes['panel-empty-title']}>{t('panelEmptyTitle')}</div>
            <CustomScroll className={classes['panel-list-container']}>
                <ul className={classes['panel-list']}>
                    {allPanel.map((item) => (
                        <li style={aotuWith} className={classes['panel-item']} key={item.type}>
                            <div
                                style={{ backgroundImage: `url(${item.thumbnail})` }}
                                className={classes['panel-item-img']}
                            />
                            <p title={item.title} className={classes['panel-item-title']}>
                                {item.title}
                            </p>
                            <p title={item.description} className={classes['panel-item-desc']}>
                                {item.description}
                            </p>
                            <div
                                onClick={() => onAddPanel(item.type)}
                                className={cx(classes['panel-item-add'], 'panel-item-add-hover')}
                            >
                                <AddIcon className={classes['panel-item-add-icon']} />
                                <span>{t('addPanel')}</span>
                            </div>
                        </li>
                    ))}
                    {flexFillElem}
                </ul>
            </CustomScroll>
        </div>
    );
}

export default React.memo(PanelEmpty);
