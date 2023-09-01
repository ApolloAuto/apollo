import { BehaviorSubject } from 'rxjs';
import { produce } from 'immer';
import Logger from '@dreamview/log';
import { Persistor } from './Persistor';

const logger = Logger.getInstance(__dirname);

class Store<S, A> {
    private state: BehaviorSubject<S>;

    private readonly reducer: (state: S, action: A) => S;

    // eslint-disable-next-line no-use-before-define
    private middleware: ((store: Store<S, A>, next: (action: A) => void, action: A) => void)[] = [];

    constructor(initialState: S, reducer: (state: S, action: A) => S) {
        this.state = new BehaviorSubject(initialState);
        this.reducer = reducer;
    }

    static create<S, A>(initialState: S, reducer: (state: S, action: A) => S) {
        return new Store(initialState, reducer);
    }

    applyMiddleware(...middleware: ((store: Store<S, A>, next: (action: A) => void, action: A) => void)[]): void {
        this.middleware = middleware;
    }

    getState() {
        return this.state.getValue();
    }

    dispatch = (action: A) => {
        const dispatchChain = this.middleware
            .reverse()
            // eslint-disable-next-line @typescript-eslint/no-shadow
            .reduce((next, middleware) => (action) => middleware(this, next, action), this.baseDispatch.bind(this));
        dispatchChain(action);
    };

    private baseDispatch(action: A) {
        const newState = this.reducer(this.getState(), action);
        this.state.next(newState);
    }

    subscribe(listener: (state: S) => void) {
        return this.state.subscribe(listener);
    }

    async persist(persistor: Persistor<S>) {
        if ('load' in persistor && 'save' in persistor) {
            const initialState = await persistor.load();
            logger.debug(initialState as object);
            this.state.next(
                produce(this.getState(), (draft) => {
                    Object.assign(draft, initialState);
                }),
            );
            this.state.subscribe((newState) => {
                persistor.save(newState);
            });
        } else if ('loadSync' in persistor && 'saveSync' in persistor) {
            this.state.subscribe((newState) => {
                persistor.saveSync(newState);
            });
        } else {
            throw new Error('Invalid persistor');
        }
    }
}

export default Store;
