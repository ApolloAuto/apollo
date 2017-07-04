import { observable, computed, action } from "mobx";

export default class Monitor {
    @observable lastUpdateTimestamp = 0;
    @observable hasActiveNotification = false;
    @observable items = [];
    refreshTimer = null;

    startRefresh() {
        this.clearRefreshTimer();
        this.refreshTimer = setInterval(() => {
            if (Date.now() - this.lastUpdateTimestamp > 6000) {
                this.setHasActiveNotification(false);
                this.clearRefreshTimer();
            }
        }, 500);
    }

    clearRefreshTimer() {
        if (this.refreshTimer !== null) {
            clearInterval(this.refreshTimer);
            this.refreshTimer = null;
        }
    }

    @action setHasActiveNotification(value) {
        this.hasActiveNotification = value;
    }

    @action update(world) {
        if (!world.monitor) {
            return;
        }

        const { item, header } = world.monitor;

        const newTimestamp = Math.floor(header.timestampSec * 1000);

        if (newTimestamp > this.lastUpdateTimestamp) {
            this.hasActiveNotification = true;
            this.lastUpdateTimestamp = newTimestamp;
            this.items.replace(item);
            this.startRefresh();
        }
    }

    // Inserts the provided message into the items. This is for
    // debug purpose only.
    @action insert(level, message, timestamp) {
        const newItems = [];
        newItems.push({
            msg: message,
            logLevel: level
        });

        for (let i = 0; i < this.items.length; ++i) {
            if (i < 29) {
                newItems.push(this.items[i]);
            }
        }

        this.hasActiveNotification = true;
        this.lastUpdateTimestamp = timestamp;
        this.items.replace(newItems);
        this.startRefresh();
    }
}
