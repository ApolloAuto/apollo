import { Subscription } from 'rxjs';
import { Emptyable } from '../../util/similarFunctions';

export class SubscriptionRegistry {
    private subscriptions = new Map<string, Subscription[]>();

    add(id: string, subscription: Subscription): void {
        if (!this.subscriptions.has(id)) {
            this.subscriptions.set(id, [subscription]);
        }
    }

    get(id: string): Emptyable<Subscription> {
        return this.subscriptions.get(id)?.[0];
    }

    push(id: string, subscription: Subscription) {
        if (!this.subscriptions.has(id)) {
            this.subscriptions.set(id, []);
        }
        return this.subscriptions.get(id)?.push(subscription);
    }

    pop(id: string): Emptyable<Subscription> {
        return this.subscriptions.get(id)?.pop();
    }

    unshift(id: string, subscription: Subscription): void {
        if (!this.subscriptions.has(id)) {
            this.subscriptions.set(id, []);
        }
        this.subscriptions.get(id)?.unshift(subscription);
    }

    shift(id: string): Emptyable<Subscription> {
        return this.subscriptions.get(id)?.shift();
    }

    delete(id: string): void {
        this.subscriptions.delete(id);
    }

    clear(): void {
        this.subscriptions.clear();
    }
}
