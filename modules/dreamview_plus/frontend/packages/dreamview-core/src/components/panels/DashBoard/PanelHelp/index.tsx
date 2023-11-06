import React from 'react';
import { SubContent, SubHeader, SubItem } from '../../base/PanelHelpContent';

export function DashBoardHelpOrigin() {
    return (
        <>
            <SubHeader>面板介绍</SubHeader>
            <SubContent>车辆行驶相关信息展示面板</SubContent>
            <SubHeader>功能描述</SubHeader>
            <SubItem>用于展示车辆速度，刹车，角度，驾驶模式以及红绿灯等信息</SubItem>
        </>
    );
}

export const DashBoardHelp = React.memo(DashBoardHelpOrigin);
