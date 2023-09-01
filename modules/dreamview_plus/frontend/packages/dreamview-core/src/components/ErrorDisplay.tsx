import React, { ErrorInfo } from 'react';

interface ErrorDisplay {
    error: Error;
    errorInfo: ErrorInfo;
}

export default function ErrorDisplay(props: ErrorDisplay) {
    return <div>{props.error.message}</div>;
}
