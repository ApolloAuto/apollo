import React from "react";

export default class DashCamButton extends React.Component {
    constructor(props) {
      super(props);
      this.onClickHandler = this.onClickHandler.bind(this);
    }

    onClickHandler() {
      this.fileInput.value = null;
      this.fileInput.click();
    }

    render() {
        const { disabled, onClick, video } = this.props;

        return (
          <div>
            <input  style={{display: "none"}}
                    ref={(input) => {
                        this.fileInput = input;
                    }}
                    type="file"
                    accept="video/*"
                    onChange={onClick}/>
            <button onClick={this.onClickHandler}
                    disabled={disabled}
                    className="button">
                DashCam Video
            </button>
          </div>
        );
    }
}
