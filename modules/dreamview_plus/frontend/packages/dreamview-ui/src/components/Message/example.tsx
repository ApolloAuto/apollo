import React, { useEffect } from 'react';
import { message } from '.';

export function Template() {
    useEffect(() => {
        message({ type: 'info', content: 'info', duration: 3 });
        message({ type: 'success', content: 'success', duration: 3 });
        message({ type: 'warning', content: 'warning', duration: 3 });
        message({ type: 'error', content: 'error', duration: 3 });
    }, []);
    return <div />;
}
