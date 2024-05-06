import { Meta } from '@storybook/react';
import PublishSubscribe from '../../components/EventHandler/PublishSubscribe';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof PublishSubscribe> = {
    title: 'Dreamview/PublishSubscribe',
    component: PublishSubscribe,
    tags: ['autodocs'],
};

export const Primary = PublishSubscribe.bind({});

export default meta;
