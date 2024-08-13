import React from 'react';
 
const iframeStyle = {
  border: '1px solid #ccc', // Add border to iframe
  borderRadius: '5px', // Add border radius to iframe
  boxShadow: '0 2px 5px rgba(0, 0, 0, 0.1)', // Add box shadow to iframe for a subtle elevation effect
  overflow: 'hidden',
};
 
const headingStyle = {
  fontFamily: 'Arial, sans-serif', // Change font family for heading
  fontSize: '24px', // Increase font size for heading
  color: '#333', // Change color for heading text
  marginBottom: '20px', // Add margin below heading
};
 
const paragraphStyle = {
  fontFamily: 'Arial, sans-serif', // Change font family for paragraph
  fontSize: '14px', // Change font size for paragraph
  color: '#666', // Change color for paragraph text
};
 
const RobotState1 = () => {
  return (
<div>
<h1 style={headingStyle}>Video Stream</h1> {/* Apply custom styles to heading */}
<iframe
        src="http://10.155.1.193:5000/video_feed"
        width="500"
        height="480"
        title="Video Stream"
        style={iframeStyle}
></iframe>
<p style={paragraphStyle}>Capgemini &copy; 2023</p> {/* Apply custom styles to paragraph */}
</div>
  );
};
 
export default RobotState1;