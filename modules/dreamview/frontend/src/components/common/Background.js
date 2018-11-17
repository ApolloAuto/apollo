import React from "react";
import PropTypes from "prop-types";

const Image = (props) => {
    const { image, style, ...otherProps } = props;
    const finalStyle = Object.assign({}, style ? style : {}, {
        backgroundImage: `url(${image})`,
        backgroundSize: "cover",
    });
    return <div className="dreamview-background" style={finalStyle} />;
};

/* Image.propTypes = {
 *     image: React.PropTypes.string.isRequired,
 *     style: React.PropTypes.object,
 * };*/

export default Image;
