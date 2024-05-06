export default class ListNode<T> {
    public data: T;

    // eslint-disable-next-line no-use-before-define
    public next: ListNode<T> | null = null;

    // eslint-disable-next-line no-use-before-define
    public prev: ListNode<T> | null = null;

    constructor(data: T) {
        this.data = data;
    }
}
