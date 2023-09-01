import React, { useEffect, useState } from 'react';

export default function useLocalStorage<T>(key: string, initValue: T): [T, (value: T) => void, () => void] {
    const storedValue = localStorage.getItem(key);

    const initial = storedValue ? JSON.parse(storedValue) : { value: initValue };

    const [storageValue, setStorageValue] = useState(initial);

    const updateStorage = (updateStorageValue: T) => {
        setStorageValue({ value: updateStorageValue });
        localStorage.setItem(key, JSON.stringify({ value: updateStorageValue }));
    };

    const clearStorage = () => {
        setStorageValue({ value: null });
        localStorage.removeItem(key);
    };

    return [storageValue.value, updateStorage, clearStorage];
}
