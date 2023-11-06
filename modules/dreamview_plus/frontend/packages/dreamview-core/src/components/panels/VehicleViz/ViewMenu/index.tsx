import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import useStyle from '../useStyle';

function ViewMenu(props: any) {
    const { carviz, setCurrentView } = props;
    const { classes, cx } = useStyle();
    const [view, setView] = useState('Default');
    const changeView = (viewType: string) => {
        setView(viewType);
        carviz.view.changeViewType(viewType);
        setCurrentView(viewType.charAt(0));
    };
    const { t } = useTranslation('viewMenu');

    return (
        <div className={classes['view-menu-container']}>
            <div className={classes['view-menu-header']}>{t('switchViews')}</div>
            <div
                className={cx(classes['view-menu-item'], {
                    [classes['view-menu-active']]: view === 'Default',
                })}
                onClick={() => changeView('Default')}
            >
                {t('default')}
            </div>
            <div
                className={cx(classes['view-menu-item'], {
                    [classes['view-menu-active']]: view === 'Near',
                })}
                onClick={() => changeView('Near')}
            >
                {t('near')}
            </div>
            <div
                className={cx(classes['view-menu-item'], {
                    [classes['view-menu-active']]: view === 'Overhead',
                })}
                onClick={() => changeView('Overhead')}
            >
                {t('overhead')}
            </div>
            <div
                className={cx(classes['view-menu-item'], {
                    [classes['view-menu-active']]: view === 'Map',
                })}
                onClick={() => changeView('Map')}
            >
                {t('map')}
            </div>
        </div>
    );
}

export default React.memo(ViewMenu);
