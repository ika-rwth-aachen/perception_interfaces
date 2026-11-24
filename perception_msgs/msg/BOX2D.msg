# Geometry in the original image: x right, y down; rotation in radians.
uint8 MODEL_ID=22
uint8 CONTINUOUS_STATE_SIZE=5
uint8 DISCRETE_STATE_SIZE=6
uint8 CENTER_X=0
uint8 CENTER_Y=1
uint8 WIDTH=2
uint8 HEIGHT=3
uint8 ROTATION=4
# Source image and integer region represented by the crop/mask (before resizing).
uint8 IMAGE_WIDTH=0
uint8 IMAGE_HEIGHT=1
uint8 ROI_X=2
uint8 ROI_Y=3
uint8 ROI_WIDTH=4
uint8 ROI_HEIGHT=5
# ObjectState.header is the acquisition header; sensor_id contains one camera.
# Pixels are not metric coordinates and must never be transformed with TF.
