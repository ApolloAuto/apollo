import { Observer, Subject, Subscription } from 'rxjs';
import Logger from '@dreamview/log';

const logger = Logger.getInstance('WebSocketManager');

export default class CountedSubject<T> extends Subject<T> {
    private subscriptionCount = 0;

    private isCompleted = false;

    constructor(public name: string, public channel?: string) {
        super();
    }

    override subscribe(): Subscription;

    override subscribe(observer: Partial<Observer<T>>): Subscription;

    override subscribe(next: (value: T) => void, error?: (error: unknown) => void, complete?: () => void): Subscription;

    override subscribe(
        observerOrNext?: Partial<Observer<T>> | ((value: T) => void),
        error?: (error: unknown) => void,
        complete?: () => void,
    ): Subscription {
        this.subscriptionCount += 1;
        logger.debug(
            `Added subscriber to ${this.name} ${
                this.channel ? `with channel ${this.channel}` : 'with Upstream'
            }. Total: ${this.subscriptionCount}`,
        );

        // Depending on the parameters, call the appropriate method of the super class
        let subscription: Subscription;
        if (typeof observerOrNext === 'function') {
            subscription = super.subscribe(observerOrNext, error, () => {
                if (complete) {
                    complete();
                }
                this.isCompleted = true;
            });
        } else {
            subscription = super.subscribe(observerOrNext as Partial<Observer<T>>);
        }

        // Override unsubscribe to decrease count
        const originalUnsubscribe = subscription.unsubscribe.bind(subscription);
        subscription.unsubscribe = () => {
            this.subscriptionCount -= 1;
            if (this.subscriptionCount < 0) {
                this.subscriptionCount = 0;
            }
            logger.debug(
                `Removed subscriber from ${this.name} ${
                    this.channel ? `with channel ${this.channel}` : 'with Upstream'
                }. Total: ${this.subscriptionCount}`,
            );
            originalUnsubscribe();
        };

        return subscription;
    }

    override complete() {
        this.isCompleted = true;
        super.complete();
    }

    get completed(): boolean {
        return this.isCompleted;
    }

    get count(): number {
        return this.subscriptionCount;
    }
}
