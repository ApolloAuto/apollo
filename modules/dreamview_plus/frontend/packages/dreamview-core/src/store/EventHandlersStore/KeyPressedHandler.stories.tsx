import { Meta } from '@storybook/react';
import KeyPressed from '@dreamview/dreamview-core/src/components/EventHandler/KeyPressedHandler';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof KeyPressed> = {
    title: 'Dreamview/KeyPressed',
    component: KeyPressed,
    tags: ['autodocs'],
};

export const Primary = KeyPressed.bind({});

export default meta;
