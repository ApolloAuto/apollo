import React from "react";

export default class MonitorSection extends React.PureComponent {
    render() {
        const { title, icon, children } = this.props;

        return (
            <div className="section">
                <div className="section-title">
                    <img height="20px" width="20px" src={icon} />
                    <span>{title}</span>
                </div>
                <div className="section-content">
                    {children}
                </div>
            </div>
        );
    }
}