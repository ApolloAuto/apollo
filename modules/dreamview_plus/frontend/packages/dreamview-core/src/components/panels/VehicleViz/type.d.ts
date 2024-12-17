import { Subscription } from 'rxjs';

export type channelType = 'pointCloud' | 'curbPointCloud';

export type subscriptionRefType = {
    [type in channelType]: {
        name: string;
        channel: string;
        subscription: Subscription;
    };
};
