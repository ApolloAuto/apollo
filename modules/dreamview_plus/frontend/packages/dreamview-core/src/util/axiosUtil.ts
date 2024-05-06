import axios from 'axios';

const TREM_URL = 'http://127.0.0.1:8889';

export const termAxios = axios.create({
    baseURL: TREM_URL,
});
