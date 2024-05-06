export default class MultiKeyMapNode<T> {
    // eslint-disable-next-line no-use-before-define
    children: Map<string, MultiKeyMapNode<T>>;

    values: Set<T>;

    constructor() {
        this.children = new Map();
        this.values = new Set();
    }
}
