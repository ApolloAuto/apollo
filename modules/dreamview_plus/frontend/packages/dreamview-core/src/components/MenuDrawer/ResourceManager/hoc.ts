import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { useMenuStore } from '@dreamview/dreamview-core/src/store/MenuStore';
import { ENUM_MENU_KEY } from '@dreamview/dreamview-core/src/store/MenuStore/actionTypes';
import { ENUM_DOWNLOAD_STATUS } from '@dreamview/dreamview-core/src/services/api/types';
import debounce from 'lodash/debounce';
import { useProfileManagerContext, ENUM_PROFILEMANAGER_TAB } from './provider';
import useComponentDisplay from '../../../hooks/useComponentDisplay';

interface IDatasource {
    percentage: string;
    status: ENUM_DOWNLOAD_STATUS;
    name: string;
    type: string;
    id: string;
    public: boolean;
    category: string;
}

interface IuseDataSource<T> {
    format: (v: T) => IDatasource[];
    api: () => Promise<T>;
    apiConnected: boolean;
    tabKey: ENUM_PROFILEMANAGER_TAB;
}

export function useDataSource<T>(props: IuseDataSource<T>) {
    const { format, api, apiConnected, tabKey } = props;
    const [{ activeMenu }] = useMenuStore();
    const { filter, activeTab } = useProfileManagerContext();
    const [originData, setOriginData] = useState<T>();

    const refreshList = useCallback(() => {
        if (apiConnected) {
            api().then((r: any) => {
                setOriginData(r);
            });
        }
    }, [apiConnected]);

    useEffect(() => {
        refreshList();
    }, [apiConnected]);

    const data = useMemo(() => {
        if (filter.downLoadStatus === 'all') {
            return format(originData);
        }
        return format(originData).filter((item: any) => item.status === filter.downLoadStatus);
    }, [filter.downLoadStatus, originData]);

    useEffect(() => {
        if (activeMenu === ENUM_MENU_KEY.PROFILE_MANAGEER && activeTab === tabKey) {
            refreshList();
        }
    }, [activeMenu, activeTab]);

    return {
        data,
        originData,
        setOriginData,
        refreshList,
    };
}

export function useScrollHeight() {
    const [, { bottomBarHeight }] = useComponentDisplay();

    const [scrollHeight, setScrollHeight] = useState<number>();

    useEffect(() => {
        const resizeHandler = debounce(() => {
            const windowHeight = document.documentElement.clientHeight;
            setScrollHeight(windowHeight - 250 - bottomBarHeight);
        }, 200);
        resizeHandler();
        window.addEventListener('resize', resizeHandler);
        return () => {
            window.removeEventListener('resize', resizeHandler);
        };
    }, [bottomBarHeight]);
    return scrollHeight;
}
