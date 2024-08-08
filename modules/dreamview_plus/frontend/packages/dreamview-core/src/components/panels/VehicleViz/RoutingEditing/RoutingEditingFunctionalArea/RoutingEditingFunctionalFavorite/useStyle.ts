import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles(() => ({
    'favorite-scroll': {
        width: '284px',
        maxHeight: '402px',
        padding: '16px 16px',
    },

    'favorite-creating-op': {
        width: '252px',
        height: '40px',
        marginBottom: '10px',
    },

    'favorite-common-co': {
        '& > div:last-child': {
            borderBottom: 'none',
        },
    },

    'favorite-common-item': {
        height: '40px',
        color: '#A6B5CC',
        fontSize: '14px',
        fontWeight: '400',
        fontFamily: 'PingFangSC-Regular',
        borderBottom: '1px solid #383B45',
        cursor: 'pointer',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        '& .favorite-common-item-op-hover': {
            display: 'none',
        },
        '&:hover': {
            width: '268px',
            background: 'rgba(115,193,250,0.08)',
            borderRadius: '6px',
            margin: '0px -8px 0px -8px',
            padding: '0px 8px 0px 8px',
            '& .favorite-common-item-op-no-hover': {
                display: 'none',
            },
            '& .favorite-common-item-op-hover': {
                display: 'block',
            },
        },
    },
    'favorite-common-item-active': {
        background: '#3288FA !important',
        borderRadius: '6px',
        margin: '0px -8px 0px -8px',
        padding: '0px 8px 0px 8px',
        '& .favorite-common-item-name-cx': {
            color: '#FFFFFF',
        },
        '& .favorite-common-item-op-no-hover-val-cx': {
            background: '#3288FA',
        },
        '& .favorite-common-item-op-no-hover-title-cx': {
            color: '#FFFFFF !important',
        },
        '&: hover': {
            '& .favorite-common-item-op-hover': {
                display: 'none',
            },
            '& .favorite-common-item-op-no-hover': {
                display: 'block',
            },
        },
    },
    'favorite-common-item-op-no-hover-title': {
        color: '#808B9D',
    },
    'favorite-common-item-op-no-hover-val': {
        width: '18px',
        height: '18px',
        color: '#FFFFFF',
        fontSize: '12px',
        textAlign: 'center',
        lineHeight: '18px',
        marginLeft: '4px',
        background: '#343C4D',
        borderRadius: '4px',
        display: 'inline-block',
    },
    'favorite-common-item-op-hover-remove': {
        color: '#FFFFFF',
        marginLeft: '23px',
    },
    'favorite-common-item-name': {
        width: '150px',
        overflow: 'hidden',
        textOverflow: 'ellipsis',
        whiteSpace: 'nowrap',
    },

    'favorite-warning-co': {
        padding: '14px 0px 32px 0px',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
    },
    'favorite-warning-co-desc': {
        width: '195px',
        color: '#A6B5CC',
        fontSize: '12px',
        fontWeight: '400',
        fontFamily: 'PingFangSC-Regular',
    },
    'favorite-warning-co-desc-active': {
        color: '#3288FA',
        cursor: 'pointer',
    },
}));
