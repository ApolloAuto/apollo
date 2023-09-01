import Logger from '@dreamview/log';
import Store from './Store';

const logger = Logger.getInstance('StoreMiddleware');

export type AsyncAction<S, A> = (dispatch: (action: A) => void, state: S) => void | Promise<void>;

export function asyncActionMiddleware<S, A>(
    store: Store<S, A>,
    next: (action: A) => void,
    action: AsyncAction<S, A> | A,
): void {
    if (typeof action === 'function') {
        (action as AsyncAction<S, A>)(store.dispatch, store.getState());
    } else {
        next(action);
    }
}

export function loggerMiddleware<S, A>(store: Store<S, A>, next: (action: A) => void, action: A): void {
    logger.debug('dispatching:', <object>action);
    next(action);
    logger.debug('next state:', <object>store.getState());
}

// @ts-ignore
export function crashReporterMiddleware<S, A>(store: Store<S, A>, next: (action: A) => void, action: A): void {
    try {
        next(action);
    } catch (err) {
        console.error('Caught an exception!', err);
        throw err;
    }
}

export function reduxDevToolsMiddleware<S, A>(store: Store<S, A>, next: (action: A) => void, action: A): void {
    next(action);

    if (typeof action === 'function') {
        return;
    }

    // eslint-disable-next-line no-underscore-dangle
    if (window.__REDUX_DEVTOOLS_EXTENSION__) {
        // eslint-disable-next-line no-underscore-dangle
        window.__REDUX_DEVTOOLS_EXTENSION__.send(action, store.getState());
    }
}
