import React, { ComponentType, LazyExoticComponent, useMemo } from 'react';
import Panel from '../Panel';

export const withDisplayName = (
    LazyComponent: LazyExoticComponent<ComponentType<any>>,
    displayName: string,
): ComponentType<any> & { displayName: string } => {
    function WrappedComponent(props: any) {
        return <LazyComponent {...props} />;
    }
    WrappedComponent.displayName = displayName;
    return WrappedComponent as ComponentType<any> & { displayName: string };
};
export const createDymaticPanel = (LazyPanel: LazyExoticComponent<ComponentType<any>>) => {
    const LazyPanelWithDisplayName = withDisplayName(LazyPanel, 'LazyPanel');

    function DymaticPanel(props: any) {
        const C = useMemo(
            () =>
                Panel({
                    PanelComponent: LazyPanelWithDisplayName,
                    panelId: props.panelId,
                }),
            [],
        );

        return <C {...props} />;
    }

    return React.memo(DymaticPanel);
};
