import sensor, image, time, ml, math, uos, gc, pyb
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(False)
sensor.set_brightness(3)
sensor.set_contrast(-3)
led1=pyb.LED(1)
clock = time.clock()
sensor.set_windowing((80, 60, 125, 125))
uart = pyb.UART(1, 115200, timeout_char=1000)
net = None
labels = None
min_confidence = 0.5
try:
	net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
	raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
try:
	labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
	raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
colors = [
	(255,   0,   0),
	(  0, 255,   0),
	(255, 255,   0),
	(  0,   0, 255),
	(255,   0, 255),
	(  0, 255, 255),
	(255, 255, 255),
]
threshold_list = [(math.ceil(min_confidence * 255), 255)]
def fomo_post_process(model, inputs, outputs):
	ob, oh, ow, oc = model.output_shape[0]
	x_scale = inputs[0].roi[2] / ow
	y_scale = inputs[0].roi[3] / oh
	scale = min(x_scale, y_scale)
	x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
	y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]
	l = [[] for i in range(oc)]
	for i in range(oc):
		img = image.Image(outputs[0][0, :, :, i] * 255)
		blobs = img.find_blobs(
			threshold_list, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1
		)
		for b in blobs:
			rect = b.rect()
			x, y, w, h = rect
			score = (
				img.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0
			)
			x = int((x * scale) + x_offset)
			y = int((y * scale) + y_offset)
			w = int(w * scale)
			h = int(h * scale)
			l[i].append((x, y, w, h, score))
	return l
clock = time.clock()
while(True):
	clock.tick()
	img = sensor.snapshot()
	animal_counts = [0, 0, 0, 0]
	total_detections = 0
	for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
		if i == 0: continue
		if len(detection_list) == 0: continue
		print("********** %s **********" % labels[i])
		detection_count = len(detection_list)
		if i <= 4:
			animal_counts[i-1] = min(detection_count, 255)
		total_detections += detection_count
		for x, y, w, h, score in detection_list:
			center_x = math.floor(x + (w / 2))
			center_y = math.floor(y + (h / 2))
			print(f"x {center_x}\ty {center_y}\tscore {score}")
			img.draw_circle((center_x, center_y, 12), color=colors[i])
	if total_detections > 0:
		data = bytearray(animal_counts)
		uart.write(data)
		print(f"Sent via UART3: {animal_counts}")
	led1.off()
	print(clock.fps(), "fps", end="\n\n")