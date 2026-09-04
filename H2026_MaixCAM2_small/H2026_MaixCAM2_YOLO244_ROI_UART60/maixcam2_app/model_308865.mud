
[basic]
type = axmodel
model_npu = model_308865_npu.axmodel
model_vnpu = model_308865_vnpu.axmodel

[extra]
model_type = yolov5
type=detector
input_type = rgb

input_cache = true
output_cache = true
input_cache_flush = false
output_cache_inval = true

anchors = 10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326
labels = ball

mean = 0, 0, 0
scale = 0.00392156862745098, 0.00392156862745098, 0.00392156862745098

