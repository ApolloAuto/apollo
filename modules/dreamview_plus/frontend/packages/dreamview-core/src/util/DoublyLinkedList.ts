import ListNode from './ListNode';

export default class DoublyLinkedList<T> {
    private head: ListNode<T> | null = null;

    private tail: ListNode<T> | null = null;

    private length = 0;

    public append(data: T): void {
        const newNode = new ListNode<T>(data);
        if (!this.head) {
            this.head = newNode;
            this.tail = newNode;
        } else {
            newNode.prev = this.tail;
            if (this.tail) {
                this.tail.next = newNode;
            }
            this.tail = newNode;
        }
        this.length += 1;
    }

    public prepend(data: T): void {
        const newNode = new ListNode<T>(data);
        if (!this.head) {
            this.head = newNode;
            this.tail = newNode;
        } else {
            newNode.next = this.head;
            if (this.head) {
                this.head.prev = newNode;
            }
            this.head = newNode;
        }
        this.length += 1;
    }

    public insert(data: T, index: number): void {
        if (index < 0 || index > this.length) {
            throw new Error('Index out of range');
        }
        if (index === this.length) {
            this.append(data);
            return;
        }
        const newNode = new ListNode<T>(data);
        if (index === 0) {
            newNode.next = this.head;
            if (this.head) {
                this.head.prev = newNode;
            }
            this.head = newNode;
        } else {
            let current = this.head;
            for (let i = 0; i < index - 1; i += 1) {
                current = current?.next ?? null;
            }
            if (current) {
                newNode.next = current.next;
                newNode.prev = current;
                current.next = newNode;
                if (newNode.next) {
                    newNode.next.prev = newNode;
                }
            }
        }
        this.length += 1;
    }

    public removeFirst(): T | null {
        if (!this.head) {
            return null;
        }
        const data = this.head.data;
        this.head = this.head.next;
        if (this.head) {
            this.head.prev = null;
        } else {
            this.tail = null;
        }
        this.length -= 1;
        return data;
    }

    public removeLast(): T | null {
        if (!this.tail) {
            return null;
        }
        const data = this.tail.data;
        this.tail = this.tail.prev;
        if (this.tail) {
            this.tail.next = null;
        } else {
            this.head = null;
        }
        this.length -= 1;
        return data;
    }

    public remove(node: ListNode<T>): T | null {
        if (node.prev) {
            node.prev.next = node.next;
        } else {
            this.head = node.next;
        }
        if (node.next) {
            node.next.prev = node.prev;
        } else {
            this.tail = node.prev;
        }
        this.length -= 1;
        return node.data;
    }

    public clear(): void {
        this.head = null;
        this.tail = null;
        this.length = 0;
    }

    public getHead(): ListNode<T> | null {
        return this.head;
    }

    public getTail(): ListNode<T> | null {
        return this.tail;
    }

    get isEmpty(): boolean {
        return this.length === 0;
    }

    get size(): number {
        return this.length;
    }
}
