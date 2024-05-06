import { Meta, StoryObj } from '@storybook/react';
import { Modal } from './index';
import { Template } from './example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Modal> = {
    title: 'Dreamview/Modal',
    component: Modal,
    tags: ['autodocs'],
    args: {},
    argTypes: {},
};

// // More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
// export const Primary: StoryObj<typeof Template> = {
//     args: {},
// };

export const Primary = Template.bind({});
// Primary.args = {};

export default meta;
