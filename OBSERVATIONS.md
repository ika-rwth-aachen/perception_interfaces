# Objects with observations

The observation API separates geometry, semantics and raw sensor payloads. It is
additive: existing messages, model IDs and classification IDs are unchanged.

```text
ObjectListWithObservation
  header
  objects[]: ObjectWithObservation
    object: Object
      state: geometry model + classifications
    observation: exactly one image or point cloud
```

An image object uses `BOX2D` (model 22). A point-cloud object uses a supported
metric model. A wrapper always contains one observation from one sensor; fused
camera estimates instead use a plain `ObjectList`. An empty image payload is
allowed when geometry alone is published. An absent observation is not allowed.

## Geometry and decoding

`BOX2D.continuous_state` is `[center_x, center_y, width, height, rotation]` in the
original image's pixels, with x right and y down. Rotation is radians from the
image x axis toward its y axis. The center is the box reference; the metric
`reference_point` offset does not apply. Covariance is empty or 5x5 in that order.

`BOX2D.discrete_state` is `[image_width, image_height, roi_x, roi_y, roi_width,
roi_height]`. The integer ROI is clipped to the source image and describes the
region represented by both crop and mask. The floating box can extend outside
the source image. The state header is the acquisition timestamp/optical frame;
`state.sensor_id` contains exactly one camera ID.

`ImageObservation` contains `sensor_msgs/Image crop` and optional `mask`. Each
retains its encoding, dimensions, row stride and acquisition header. The mask is
mono8: 0 background, 255 foreground. Crop and mask may be resized independently.
For continuous pixel-edge coordinates, source coordinates are
`roi_x + payload_x * roi_width / payload_width` and analogously for y. For sample
centers, use `payload_x + 0.5` and `payload_y + 0.5`. Empty payloads have zero
width, height and stride. Geometry belongs to the state, not the payload.

`PointCloudObservation` contains `sensor_msgs/PointCloud2 points`, including field
layout, stride, endianness and acquisition header. The state and cloud must share
the same timestamp/frame and one acquisition sensor. Sensor-frame observations
must be transformed together with their state by a cloud-aware consumer; the
utilities deliberately provide no generic observation TF overload.

Metric TF helpers reject `BOX2D` explicitly. Never treat pixel X/Y as metres.
Use `sanityCheckObjectObservation` / `sanity_check_object_observation` at input
boundaries, and ordinary state initialization with the `BOX2D` model ID.

## Classification

`ObjectClassTaxonomy` is a constants-only catalog for the unchanged
`ObjectClassification.type` field. C++/Python helpers expose matching labels.

| IDs | Meaning |
| --- | --- |
| Existing IDs | Unchanged road-user classes, UNCLASSIFIED and UNKNOWN |
| 101–114 | Traffic light, unknown, off, red, yellow, green, red-yellow and seven compound classes |
| 128–129 | Traffic sign, unknown sign subtype |
| 130–172 | Existing 43 sign categories, in legacy sign-type order |

The precise allocation is in `ObjectClassTaxonomy.msg`; never reuse an assigned
ID. A generic traffic-light/sign class means detection before subtype recognition.
`TRAFFIC_LIGHT_UNKNOWN` means an unresolved signal, not OFF. `TRAFFIC_SIGN_UNKNOWN`
means an unresolved sign subtype. Preserve compound labels such as `left,red` and
`red,right,straight`; their green arrows must not be discarded in the native API.

Classification entries are alternative hypotheses for the complete object label.
Their probabilities are semantic confidence, independent of the detector's
`Object.existence_probability`. Published winners can have probability below one;
the list need not contain every hypothesis. Unsupported taxonomy growth beyond
uint8 requires a separately versioned interface, not renumbering existing IDs.

## Pipeline and compatibility

Segmentation publishes `observations/{road_users,traffic_lights,traffic_signs}`
under each camera prefix. Recognizers publish classified 2D wrappers on
`<prefix>/observations`. Localization produces generic `HEXAMOTION` objects on
separate plain-ObjectList topics. Unmeasured motion/orientation/size values are
NaN; covariance follows existing invalid/unknown conventions.

Camera images stop at classification. Missing calibration, TF or parallax leaves
the detection available on the 2D stream and produces no fabricated 3D object.
Stereo may use both images internally; its 3D output contains both contributing
sensor IDs and the primary camera's object ID. IDs are scoped to source sensors;
they are not globally unique across cameras. Map associations retain those IDs.

Legacy evidence topics, messages and specialized models remain supported.
`input.use_observations=false` selects legacy recognizer/localizer input;
`input.use_objects=false` selects legacy fusion/matcher input. Keep legacy topic
parameter files when selecting those modes. The new demo/platform composition
uses the new interfaces. Compatibility projections reuse existing numerical
algorithms and renderers internally; they do not expose specialized models on the
new topics. Conversion from fused evidence to a single observation is rejected
rather than silently dropping a sensor.

Fusion records the preceding metric camera-frame state in bounded state history
before moving the current state into `map`. Matching uses that measurement's
header to recover the viewing origin, without attaching image data to 3D objects.
For already-map-frame inputs without such history, matching uses the configured
ego origin. This history is metric only; pixel states do not enter 3D histories.

## Branch baseline and retained fixes

The coordinated branches start at these local main revisions:

| Repository | Main baseline |
| --- | --- |
| perception_interfaces | 284adea |
| image_instance_segmentation | c9bc0e9 |
| traffic_sign_recognition | f9644df |
| traffic_light_recognition | f575e33 |
| monitoring | c970300 |
| karl-platform | 0a49ffd6 |

The existing evidence feature commits are carried forward to preserve their public
interfaces, utilities, TF behavior and displays. Segmentation's lifecycle IDs,
header checks and unmeasured-state fixes and the platform's recognition-service
and Zenoh changes are retained. Recognizer main already includes preprocessing,
TF, covariance, bounded scheduling and native downstream fixes. Runtime CI uses
non-recursive dependency import and required-package pruning for both recognizers.

Build/publish the interface branch first, then producer/recognizer and monitoring
branches, before starting branch-image demos/platform profiles. Existing ignored
`demo/.env` overrides may still select older local images: update their runtime
image variables to match `.env.example` or explicitly supply matching local builds.
The model image variables and weight provisioning do not change.
