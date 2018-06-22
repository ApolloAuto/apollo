# link map_data to dist/assets
SYMLINK=/apollo/modules/dreamview/frontend/dist/assets/map_data
if [ ! -e ${SYMLINK} ] ; then
    ln -sf /apollo/modules/map/data $SYMLINK
fi

# generate protobuf bundles
./gen_pbjs.sh
