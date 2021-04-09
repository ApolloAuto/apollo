import React from "react";

export default class WindowResizeControl extends React.PureComponent {
    getMaximizingIcon() {
        return (
            <svg viewBox="0 0 20 20">
                <defs>
                    <path d="M20 0L0 20h20V0z" id="a" />
                    <path d="M11.53 18.5l-.03-7h7" id="b" />
                    <path d="M12 12l7 7" id="c" />
                </defs>
                <use xlinkHref="#a" opacity=".6" fill="#549BFF" />
                <use xlinkHref="#b" fillOpacity="0" stroke="#006AFF" strokeWidth="2" />
                <use xlinkHref="#c" fillOpacity="0" stroke="#006AFF" strokeWidth="2" />
            </svg>
        );
    }

    getMinimizingIcon() {
        return (
            <svg viewBox="0 0 20 20">
                <defs>
                    <path d="M20 0L0 20h20V0z" id="a" />
                    <path d="M18.47 11.5l.03 7h-7" id="b" />
                    <path d="M11 11l7 7" id="c" />
                </defs>
                <use xlinkHref="#a" opacity=".6" fill="#549BFF" />
                <use xlinkHref="#b" fillOpacity="0" stroke="#006AFF" strokeWidth="2" />
                <use xlinkHref="#c" fillOpacity="0" stroke="#006AFF" strokeWidth="2" />
            </svg>
        );
    }

    render() {
        const { type, onClick } = this.props;

        let icon = null;
        switch (type) {
            case "minimizing":
                icon = this.getMinimizingIcon();
                break;
            case "maximizing":
                icon = this.getMaximizingIcon();
                break;
        }

        return (
            <div className="window-resize-control" onClick={onClick}>
                {icon}
            </div>
        );
    }
}
