import React from "react";
import { observer } from "mobx-react";

import removeAllIcon from "assets/images/icons/remove_all.png";
import removeLastIcon from "assets/images/icons/remove_last.png";
import sendRouteIcon from "assets/images/icons/send_request.png";
import AddDefaultEndPointIcon from "assets/images/icons/add_default_end_point.png";

@observer
class RouteEditingButton extends React.Component {
    render() {
        const { label, icon, onClick } = this.props;

        return (
            <button onClick={onClick}
                    className="button">
                <img src={icon} />
                <span>{label}</span>
            </button>
        );
    }
}


@observer
export default class EditingPanel extends React.Component {

    render() {
        const { clickAddDefaultEndPoint, clickRemoveLast,
                clickRemoveAll, clickSendRoute  } = this.props;

        return (
            <div className="editing-panel">
                <RouteEditingButton label="Add Default End Point"
                                    icon={AddDefaultEndPointIcon}
                                    onClick={clickAddDefaultEndPoint}/>
	            <RouteEditingButton label="Remove Last Point"
	                                icon={removeLastIcon}
                                    onClick={clickRemoveLast}/>
	            <RouteEditingButton label="Remove All Points"
                                    icon={removeAllIcon}
	                                onClick={clickRemoveAll}/>
	            <RouteEditingButton label="Send Routing Request"
	                                icon={sendRouteIcon}
	                                onClick={clickSendRoute}/>
            </div>
        );
    }
}
