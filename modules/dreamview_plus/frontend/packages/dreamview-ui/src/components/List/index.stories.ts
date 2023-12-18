import { Meta, StoryObj } from '@storybook/react';
import { List } from './index';
import { Template } from './example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof List> = {
    title: 'Dreamview/List',
    component: List,
    tags: ['autodocs'],
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary = Template.bind({});
// Primary.args = {};

export default meta;
