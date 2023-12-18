import { useEffect } from 'react';
import { message } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';

type CopyMessageProps = {
    success?: boolean;
};

export default function CopyMessage({ success }: CopyMessageProps = { success: false }) {
    const { t } = useTranslation('carviz');

    useEffect(() => {
        if (success) {
            message({ type: 'success', content: t('CopySuccessful'), duration: 3 });
        } else {
            message({ type: 'error', content: t('CopyFailed'), duration: 3 });
        }
    }, [success, t]);

    return null;
}
