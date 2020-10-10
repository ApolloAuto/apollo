/* import "whatwg-fetch";
 * import "font-awesome-webpack"; */

import * as ReactDOM from 'react-dom';
import React from 'react';
import { Provider } from 'mobx-react';

import 'styles/main.scss';
import STORE from 'store';
import Offlineview from 'components/Offlineview';

ReactDOM.render(
    <Provider store={STORE}>
        <Offlineview />
    </Provider>,
    document.getElementById('root'),
);
