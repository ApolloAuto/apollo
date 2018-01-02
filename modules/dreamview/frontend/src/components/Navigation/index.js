import React from "react";

export default class Navigation extends React.Component {
    render() {
        const {height} = this.props;

        return (
            <div className="main-view" style={{height: height}}>
                <iframe src="components/Navigation/navigation_viewer.html"
                        style={{width: "100%", height: "100%"}}/>
            </div>
        );
    }
}