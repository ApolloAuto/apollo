import React from 'react';
import { ExclamationCircleOutlined } from '@ant-design/icons';
import { makeStyles } from '@dreamview/dreamview-theme';

const useStyle = makeStyles(() => ({
    'setting-modal-alert': {
        minHeight: '28px',
        background: 'rgba(255,141,38,0.25)',
        borderRadius: '4px',
        width: '100%',
        display: 'flex',
        fontFamily: 'PingFangSC-Regular',
        fontSize: '14px',
        color: '#FF8D26',
        alignItems: 'flex-start',
        fontWeight: 400,
        marginBottom: '8px',
        '.anticon': {
            marginLeft: '21px',
            marginTop: '7px',
        },
    },
    'setting-modal-text': {
        marginLeft: '7px',
        lineHeight: '20px',
        marginTop: '4px',
        marginBottom: '4px',
        flex: 1,
    },
}));

export default function Alert(props: { text: string }) {
    const { classes } = useStyle();
    return (
        <div className={classes['setting-modal-alert']}>
            <ExclamationCircleOutlined />
            <div className={classes['setting-modal-text']}>{props.text}</div>
        </div>
    );
}
