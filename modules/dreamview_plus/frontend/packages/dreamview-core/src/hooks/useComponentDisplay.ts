import { useMemo } from 'react';
import { usePickHmiStore, HMIModeOperation } from '@dreamview/dreamview-core/src/store/HmiStore';
import { ENUM_MENU_KEY, useMenuStore } from '@dreamview/dreamview-core/src/store/MenuStore';

type UseComponentDisplay = [
    {
        isMenuDrawerHidden: boolean;
        isMenuDrawerShow: boolean;
        isBottomBarShow: boolean;
        isBottomBarHidden: boolean;
        isScenarioHistoryShow: boolean;
        isDynamicalModelsShow: boolean;
    },
    { bottomBarHeight: number; bottomBarHeightString: string; menuDrawerWidthString: string },
];

export default function useComponentDisplay(): UseComponentDisplay {
    const [{ activeMenu }] = useMenuStore();
    const [hmi] = usePickHmiStore();

    return useMemo(() => {
        const displayStatus = {
            // 侧边栏抽屉隐藏状态
            isMenuDrawerHidden: activeMenu === ENUM_MENU_KEY.HIDDEN,
            isMenuDrawerShow: activeMenu !== ENUM_MENU_KEY.HIDDEN,
            // 底部bar隐藏状态
            isBottomBarHidden: hmi.currentOperation !== HMIModeOperation.PLAY_RECORDER,
            isBottomBarShow: [
                HMIModeOperation.PLAY_RECORDER,
                HMIModeOperation.SIM_CONTROL,
                HMIModeOperation.SCENARIO,
                HMIModeOperation.AUTO_DRIVE,
                HMIModeOperation.WAYPOINT_FOLLOW,
            ].some((item) => item === hmi.currentOperation),
            // PnCmonitor scenario history
            // isScenarioHistoryShow: [HMIModeOperation.SIM_CONTROL, HMIModeOperation.SCENARIO].some(
            //     (item) => item === hmi.currentOperation,
            // ),
            isScenarioHistoryShow: true,
            isDynamicalModelsShow: [HMIModeOperation.SIM_CONTROL, HMIModeOperation.SCENARIO].includes(
                hmi.currentOperation,
            ),
        };

        const displayHeightOrWidth = {
            // 底部bar高度
            bottomBarHeight: displayStatus.isBottomBarHidden ? 0 : 63,
            bottomBarHeightString: displayStatus.isBottomBarHidden ? '0px' : '63px',
            // 侧边栏抽屉宽度
            menuDrawerWidthString: (() => {
                if (activeMenu === ENUM_MENU_KEY.HIDDEN) {
                    return '0px';
                }
                if (activeMenu === ENUM_MENU_KEY.PROFILE_MANAGEER) {
                    return 'calc(100vw - 64px)';
                }
                return '370px';
            })(),
        };
        return [displayStatus, displayHeightOrWidth];
    }, [activeMenu, hmi]);
}
