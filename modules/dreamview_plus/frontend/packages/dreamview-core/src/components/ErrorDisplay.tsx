import React, { ErrorInfo } from 'react';

interface ErrorDisplay {
    error: Error;
    errorInfo: ErrorInfo;
}

function ErrorDisplay(props: ErrorDisplay) {
    return <div>{props.error.message}</div>;
}

export default React.memo(ErrorDisplay);
