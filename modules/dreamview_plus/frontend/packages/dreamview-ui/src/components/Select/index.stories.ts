import { Meta, StoryObj } from '@storybook/react';
import { Select } from './index';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Select> = {
    title: 'Dreamview/Select',
    component: Select,
    tags: ['autodocs'],
    argTypes: {},
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary: StoryObj<typeof Select> = {
    args: {},
};

export const Secondary: StoryObj<typeof Select> = {
    args: {},
};

export default meta;
