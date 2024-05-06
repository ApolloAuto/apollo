import { Meta, StoryObj } from '@storybook/react';
import { Collapse } from './index';
import { Template } from './example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Collapse> = {
    title: 'Dreamview/Collapse',
    component: Collapse,
};

export const Primary = Template.bind({});
// Primary.args = {};

export default meta;
