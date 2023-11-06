import React from 'react';

export const PNCMonitorContext = React.createContext(null);

export function usePNCMonitorContext() {
    return React.useContext(PNCMonitorContext);
}
