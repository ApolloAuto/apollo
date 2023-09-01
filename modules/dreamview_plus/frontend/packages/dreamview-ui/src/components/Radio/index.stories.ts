import { Meta, StoryObj } from '@storybook/react';
import { Radio } from './index';
import { RadioTemplate } from './radio.example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Radio> = {
    title: 'Dreamview/Radio',
    component: Radio,
    tags: ['autodocs'],
    argTypes: {},
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary = RadioTemplate.bind({});

export default meta;
