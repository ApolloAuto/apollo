import React from 'react';

import ProfileWarningIcon from 'assets/images/icons/profile_warning.png';
import ProfileCertificateArrowIcon from 'assets/images/icons/profile_certificate_arrow.png';

export function ScenarioNoCertificate() {
  return (
    <div className='data-profile_no_certificate'>
      <img src={ProfileWarningIcon} className='data-profile_no_certificate__icons' alt='profile-warning' />
      <h6 style={{ color: '#FFFFFF' }}>Please install the certificate first</h6>
      <p>
        After installing the certificate, you can download
        the Apollo studio cloud ScenarioGroup to the local
      </p>
      <button
        className='command-button'
        onClick={() => {
          // todo: 跳转安装证书页面
        }}
      >
        View the certificate installation tutorial
      </button>
    </div>
  );
}

export function ScenarioCertificateInvalid() {
  return (
    <div className='data-profile_invalid_certificate'>
      <img src={ProfileWarningIcon} className='data-profile_invalid_certificate__icons' alt='profile-warning' />
      <div className='data-profile_invalid_certificate__content'>
        <h6>certificate status is abnormal, please reinstall.</h6>
        <p
          onClick={() => {
            // todo: 跳转安装证书页面
          }
          }
        >View the certificate installation tutorial
          <img src={ProfileCertificateArrowIcon} alt='profile_certificate_arrow' />
        </p>
      </div>
    </div>
  );
}
