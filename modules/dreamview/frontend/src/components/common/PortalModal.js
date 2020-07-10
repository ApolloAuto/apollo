import 'styles/Modal.scss';

import React from 'react';
import ReactDOM from 'react-dom';

class Modal extends React.Component {
  constructor(props) {
    super(props);

    this.setOkButtonRef = (element) => {
      this.okButton = element;
    };
  }

  componentDidMount() {
    // .focus() has browser restrictions in some cases, where
    // the element is not visible when trying to focus().
    // The workaround is setting a setTimeout as below.
    setTimeout(() => {
      if (this.okButton) {
        this.okButton.focus();
      }
    }, 0);
  }

  componentDidUpdate() {
    if (this.okButton) {
      this.okButton.focus();
    }
  }

  render() {
    const { open, header } = this.props;
    if (!open) {
      return null;
    }

    return (
            <div>
                <div className="modal-background" />
                <div className="modal-content">
                    <div role="dialog" className="modal-dialog">
                        {header && (
                            <header>
                                <span>{this.props.header}</span>
                            </header>
                        )}
                        {this.props.children}
                    </div>
                    <button
                        ref={this.setOkButtonRef}
                        className="ok-button"
                        onClick={() => this.props.onClose()}
                    >
                        OK
                    </button>
                </div>
            </div>
    );
  }
}

export default class PortalModal extends React.Component {
  constructor(props) {
    super(props);

    this.rootSelector = document.getElementById('root');
    this.container = document.createElement('div');
  }

  componentDidMount() {
    this.rootSelector.appendChild(this.container);
  }

  componentWillUnmount() {
    this.rootSelector.removeChild(this.container);
  }

  render() {
    return ReactDOM.createPortal(<Modal {...this.props} />, this.container);
  }
}
