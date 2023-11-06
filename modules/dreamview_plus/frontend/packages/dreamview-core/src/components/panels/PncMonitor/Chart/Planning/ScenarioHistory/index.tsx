import React from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';

export default function ScenarioHistory() {
    const { t } = useTranslation('pncMonitor');
    const { planningData } = usePNCMonitorContext();
    const { classes } = useStyle();

    return (
        <div className={classes['moniter-item-container']}>
            <div className={classes['moniter-item-title']}>{t('scenarioHistory')}</div>
            <div className={classes['moniter-item-content']}>
                <div className={classes['moniter-item-left']}>
                    {planningData.scenarioHistory.map((scenario: any) => (
                        <div title={scenario.timeString} key={scenario.timeString}>
                            {scenario.timeString}
                        </div>
                    ))}
                </div>
                <div className={classes['moniter-item-mid']}>
                    {planningData.scenarioHistory.map((scenario: any) => (
                        <div title={scenario.timeString} key={scenario.timeString}>
                            {scenario.scenarioPluginType}
                        </div>
                    ))}
                </div>
                <div className={classes['moniter-item-right']}>
                    {planningData.scenarioHistory.map((scenario: any) => (
                        <div title={scenario.timeString} key={scenario.timeString}>
                            {scenario.stagePluginType}
                        </div>
                    ))}
                </div>
            </div>
        </div>
    );
}
