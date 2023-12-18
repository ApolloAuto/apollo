export const noop = (): null => null;

export type Nullable<T> = T | null;

export type Emptyable<T> = T | undefined;

export type Arrayable<T> = T | T[];
export const returnZero = () => 0;

export const returnEmptyString = () => '';

export const returnEmptyObject = () => ({});
