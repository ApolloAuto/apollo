import React, { PropsWithChildren, ErrorInfo } from 'react';

type State = {
    currentError: {
        error: Error;
        errorInfo: ErrorInfo;
    };
};

interface Props {
    errComponent?: any;
}

class ErrorBoundary extends React.Component<PropsWithChildren<Props>, State> {
    constructor(props: any) {
        super(props);
        this.state = {
            currentError: null,
        };
    }

    override componentDidCatch(error: Error, errorInfo: ErrorInfo): void {
        this.setState({
            currentError: {
                error,
                errorInfo,
            },
        });
    }

    override render() {
        if (this.state.currentError) {
            return this.props.errComponent || <div>啊哦，页面出错了</div>;
        }
        return this.props.children;
    }
}

export default React.memo(ErrorBoundary);
