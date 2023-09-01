#!/bin/bash

# yarn gen:component <component_name>

set -e

COMPONENT_NAME=$1

echo "create $COMPONENT_NAME"

# 复制template
mkdir src/components/$COMPONENT_NAME
cp -R template/* src/components/$COMPONENT_NAME/

# 组件名替换
find src/components/$COMPONENT_NAME -type f -exec sed -i "s/Component/${COMPONENT_NAME}/g" {} \;
