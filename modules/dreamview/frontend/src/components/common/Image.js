import React from 'react';
import PropTypes from 'prop-types';

const Image = (props) => {
  const {
    image, style, className, ...otherProps
  } = props;
  const finalStyle = {
    ...(style || {}),
    backgroundImage: `url(${image})`,
    backgroundSize: 'cover',
  };
  const finalClassName = className ? `${className} dreamview-image`
    : 'dreamview-image';
  return <div className={finalClassName} style={finalStyle} />;
};

Image.propTypes = {
  image: PropTypes.string.isRequired,
  style: PropTypes.object,
};

export default Image;
