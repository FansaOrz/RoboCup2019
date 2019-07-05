from ctypes import *
import math
import cv2
import random

def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1

def c_array(ctype, values):
    arr = (ctype*len(values))()
    arr[:] = values
    return arr

class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]

class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]

class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]

    

#lib = CDLL("/home/pjreddie/documents/darknet/libdarknet.so", RTLD_GLOBAL)
lib = CDLL("/home/fansa/Src/pepper_example/RoboCup2019/human_detection/libdarknet.so", RTLD_GLOBAL)
lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

set_gpu = lib.cuda_set_device
set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)


class human_detector():
    def __init__(self):
        self.image_path = "/home/fansa/Src/pepper_example/RoboCup2019/human_detection/data/human_detection.jpg"
        self.net = load_net("/home/fansa/Src/pepper_example/RoboCup2019/human_detection/cfg/yolov3.cfg", "/home/fansa/Src/pepper_example/RoboCup2019/human_detection/weights/yolov3.weights", 0)
        self.meta = load_meta("/home/fansa/Src/pepper_example/RoboCup2019/human_detection/data/voc.data")

    def detect(self, net, meta, image, thresh=.6, hier_thresh=.5, nms=.45):
        im = load_image(image, 0, 0)
        # im = image
        num = c_int(0)
        pnum = pointer(num)
        predict_image(net, im)
        dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, None, 0, pnum)
        num = pnum[0]
        if (nms): do_nms_obj(dets, num, meta.classes, nms);

        res = []
        for j in range(num):
            for i in range(meta.classes):
                if dets[j].prob[i] > 0:
                    b = dets[j].bbox
                    res.append((meta.names[i], dets[j].prob[i], (b.x, b.y, b.w, b.h)))
        res = sorted(res, key=lambda x: -x[1])
        free_image(im)
        free_detections(dets, num)
        # print res
        if len(res) == 0:
            return []
        else:
            return res

    def main(self, frame_name):
        r = self.detect(self.net, self.meta, frame_name)
        if r == "none":
            return "none"
        img = cv2.imread(frame_name)
        # print r
        right_human_size = 0
        right_human = []

        for i in range(len(r)):
            if r[i][1] > 0:
                name = r[i][0]
                rect = r[i][2]
                # print len(rect)
                # for i in range(len(rect)):
                cv2.putText(img, name, (int(rect[0] - rect[2] / 2), int(rect[1] - rect[3] / 2 - 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                cv2.rectangle(img, (int(rect[0] - rect[2] / 2), int(rect[1] - rect[3] / 2 - 10)),
                              (int(rect[0] + rect[2] / 2), int(rect[1] + rect[3] / 2)), (0, 0, 255), 3)
                rect_left = int(rect[0] - rect[2] / 2)
                rect_top = int(rect[1] - rect[3] / 2 - 10)
                rect_right = int(rect[0] + rect[2] / 2)
                rect_bottom = int(rect[1] + rect[3] / 2)
                if (rect_right - rect_left)*(rect_bottom - rect_top) < 40000:
                    continue
                print "this person size:===========", (rect_right - rect_left)*(rect_bottom - rect_top)
                if (rect_right - rect_left) * (rect_bottom - rect_top) > right_human_size:
                    right_human_size = rect_right
                    right_human = [rect_left, rect_top, rect_right, rect_bottom]

        cv2.imshow("aa", img)
        cv2.imwrite("./waving_detection.jpg", img)
        cv2.waitKey(1)
        print type(r)
        print right_human
        return right_human

#
if __name__ == "__main__":
    a = human_detector()
    a.main("/home/fansa/Software/darknet/training/images/frame3590.jpg")
#     cap = cv2.VideoCapture(0)
#     ok, frame = cap.read()
#     cv2.namedWindow("aa", cv2.WINDOW_NORMAL)
#     while ok:
#         cv2.imwrite(image_path, frame)
#         r = detect(net, meta, image_path)
#         img = frame
#         print r
#         for i in range(len(r)):
#             if r[i][1] > 0:
#                 name =  r[i][0]
#                 rect = r[i][2]
#                 for i in range(len(rect)):
#                     cv2.putText(img, name, (int(rect[0] - rect[2] / 2), int(rect[1] - rect[3] / 2)), cv2.FONT_HERSHEY_SIMPLEX,
#                                 1, (255, 0, 0), 3)
#                     cv2.rectangle(img, (int(rect[0] - rect[2] / 2), int(rect[1] - rect[3] / 2 - 10)), (int(rect[0] + rect[2] / 2), int(rect[1] + rect[3] / 2)), (0,0,255), 3)
#         cv2.imshow("aa", img)
#         c = cv2.waitKey(1)
#         if c & 0xFF == ord('q'):
#             break
#         ok,frame = cap.read()
#     cap.release()
#     cv2.destroyAllWindows()

    

