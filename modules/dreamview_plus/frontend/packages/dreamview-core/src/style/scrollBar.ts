export default {
    '::-webkit-scrollbar': {
        display: 'none',
    },
    // mac-scrollbar组件样式重置
    '.ms-track.ms-active, .ms-track:hover': {
        background: 'transparent !important',
        borderColor: 'transparent !important',
        opacity: 1,
    },

    '.ms-track.ms-y.ms-active .ms-thumb, .ms-track.ms-y:hover .ms-thumb, .ms-track.ms-y .ms-thumb': {
        width: '4px',
    },
    '.ms-track.ms-x .ms-thumb, .ms-track.ms-x.ms-active .ms-thumb, .ms-track.ms-x:hover .ms-thumb': {
        height: '4px',
    },
    '.ms-track .ms-thumb': {
        backgroundColor: '#3D434E',
    },
};
