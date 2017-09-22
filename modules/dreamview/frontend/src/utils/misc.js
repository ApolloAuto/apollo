export function copyProperty(toObj, fromObj) {
    for (const property in fromObj) {
        if (fromObj.hasOwnProperty(property)) {
            toObj[property] = fromObj[property];
        }
    }
}

export function hideArrayObjects(objects, startIdx = 0) {
    if (objects.constructor === Array && objects.length > 0) {
        for (;startIdx < objects.length; startIdx++) {
            objects[startIdx].visible = false;
        }
    }
}
