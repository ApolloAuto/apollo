import { ModalProps } from 'antd';
import React, { useState } from 'react';
import { Button } from '../Button';
import { Modal } from '.';

export function Template(props: ModalProps) {
    const [isModalOpen, setIsModalOpen] = useState(false);

    const showModal = () => {
        setIsModalOpen(true);
    };

    const handleOk = () => {
        setIsModalOpen(false);
    };

    const handleCancel = () => {
        setIsModalOpen(false);
    };

    return (
        <>
            <Button type='primary' onClick={showModal}>
                Open Modal
            </Button>
            <Modal title='Basic Modal' open={isModalOpen} onOk={handleOk} onCancel={handleCancel} {...props}>
                <p>Some contents...</p>
                <p>Some contents...</p>
                <p>Some contents...</p>
            </Modal>
        </>
    );
}
