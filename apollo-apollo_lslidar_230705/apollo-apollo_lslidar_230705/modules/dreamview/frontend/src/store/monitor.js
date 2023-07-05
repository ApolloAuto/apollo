import { observable, action } from 'mobx';

export default class Monitor {
    @observable hasActiveNotification = false;

    @observable items = [];

    @observable rssInfo = [];

    @observable isSirenOn = false;

    lastUpdateTimestamp = 0;

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

    update(world) {
      this.updateMonitorItems(world);
      this.updateEmergencyItem(world);
    }

    @action updateEmergencyItem(world) {
      this.isSirenOn = world.isSirenOn;
    }

    @action updateMonitorItems(world) {
      if (!world.monitor && !world.notification) {
        return;
      }

      let newItems = [];
      if (world.notification) {
        newItems = world.notification
          .reverse()
          .map((notification) => Object.assign(notification.item, {
            timestampMs: notification.timestampSec * 1000,
          }))
          .sort((notification1, notification2) =>
            notification2.timestampMs - notification1.timestampMs);
      } else if (world.monitor) {
        // deprecated: no timestamp for each item
        newItems = world.monitor.item;
      }

      if (this.hasNewNotification(this.items, newItems)) {
        this.updateMonitorMessages(newItems, Date.now());
      }
    }

    hasNewNotification(items, newItems) {
      if (items.length === 0 && newItems.length === 0) {
        return false;
      }
      if (items.length === 0 || newItems.length === 0) {
        return true;
      }
      return JSON.stringify(this.items[0]) !== JSON.stringify(newItems[0]);
    }

    /**
   * Inserts the provided message into the items. This message won't send to backend
   * @param level {'FATAL' | 'WARN' | 'ERROR' | 'INFO'}
   * @param message {string}
   * @param timestamp {number}
   */
    @action insert(level, message, timestamp) {
      const newItems = [];
      newItems.push({
        msg: message,
        logLevel: level,
        timestampMs: timestamp,
      });

      for (let i = 0; i < this.items.length; ++i) {
        if (i < 29) {
          newItems.push(this.items[i]);
        }
      }

      this.updateMonitorMessages(newItems, timestamp);
    }

    updateMonitorMessages(newItems, newTimestamp) {
      this.hasActiveNotification = true;
      this.lastUpdateTimestamp = newTimestamp;

      this.items = [];
      this.rssInfo = [];
      newItems.forEach((item) => {
        if (item && item.msg && item.msg.startsWith('RSS')) {
          this.rssInfo.push(item);
        } else {
          this.items.push(item);
        }
      });
    }
}
