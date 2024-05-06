export interface Action<T = any> {
    type: T;
}

export interface PayloadAction<T = any, P = any> extends Action<T> {
    payload: P;
}

export type Reducer<S, A> = (state: S, action: A) => S;

export interface Reducers<S, A> {
    [key: string]: Reducer<S, A>;
}
