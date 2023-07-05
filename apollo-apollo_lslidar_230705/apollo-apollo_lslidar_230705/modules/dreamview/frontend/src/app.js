import * as ReactDOM from 'react-dom';
import React from 'react';
import { Provider } from 'mobx-react';

import 'styles/main.scss';
import 'styles/antd-reset.scss';

import STORE from 'store';
import Dreamview from 'components/Dreamview';

ReactDOM.render(
    <Provider store={STORE}>
        <Dreamview />
    </Provider>,
    document.getElementById('root'),
);
