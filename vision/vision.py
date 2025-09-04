import cv2 as cv 
import numpy as np 
from math import sqrt
import time 
from api_helper.ctrlx_api import CtrlxDlAPi
from hdv.object_detection2d import Result, OrientedBoundingBox
from ctrlxdatalayer.variant import Variant
from vision.error_codes import ErrorCodes

lookup = {0: 'x',1:'l', 2:'r', 3:'t', 4: 'c' }



def get_corners(img): 
    kernel_size =13
    blur = cv.GaussianBlur(img, (kernel_size, kernel_size), 0)
    circles = cv.HoughCircles(blur,
                                        cv.HOUGH_GRADIENT,
                                        dp=1.2,
                                        minDist=60,
                                        param1=150,
                                        param2=30,
                                        minRadius=8,
                                        maxRadius=13)
    centers = []
    dists = []
    screws = [[527, 119], [249, 698], [1109, 395], [825, 981]]
    l = 642
    d =912
    candidates = []
    if circles is None:
        return np.array([])
    for x,y,r in circles[0]:
        centers.append(np.array([x,y]))
    for i in range(len(circles[0])):
        dist = []
        for j in range(len(circles[0])):
            if i == j:
                continue
            dist.append(sqrt((circles[0][i][0] - circles[0][j][0])**2 + (circles[0][i][1] - circles[0][j][1])**2))
        diag = 0
        side = 0
        for k in range(len(dist)):
            if l - 0.1*l< dist[k] < l + 0.1*l:
                side += 1
            if d - 0.1*d < dist[k] < d + 0.1*d:
                diag += 1
        if side >=2 and diag >=1:
            candidates.append([circles[0][i][0], circles[0][i][1]])
        dists.append(dist)
    src_arr = np.zeros((4,2))
    for c in candidates:
        dist_screw = []
        for s in screws:
            dist_screw.append(sqrt((c[0] - s[0])**2 + (c[1] - s[1])**2))
        src_arr[np.argmin(np.array(dist_screw))] = np.array(c)
    return src_arr.astype(np.float32)

lookup = {0: 'x',1:'l', 2:'r', 3:'t', 4: 'c' }

def get_J(H, x, y):
    h11, h12, h13 = H[0]
    h21, h22, h23 = H[1]
    h31, h32, h33 = H[2]

    denom = (h31*x + h32*y + h33)**2

    dx_dx = (h11 * (h31*x + h32*y + h33) - (h11*x + h12*y + h13) * h31) / denom
    dx_dy = (h12 * (h31*x + h32*y + h33) - (h11*x + h12*y + h13) * h32) / denom
    dy_dx = (h21 * (h31*x + h32*y + h33) - (h21*x + h22*y + h23) * h31) / denom
    dy_dy = (h22 * (h31*x + h32*y + h33) - (h21*x + h22*y + h23) * h32) / denom

    return np.array([[dx_dx, dx_dy],
                     [dy_dx, dy_dy]])

def draw_box(cx,cy,w,h,angle, img):
    rect = ((cx, cy), (w, h), angle)  # ((center_x, center_y), (width, height), angle)

    # Get the 4 corner points of the rectangle
    box = cv.boxPoints(rect)
    box = box.astype(int)

    # Draw the rotated bounding box
    cv.polylines(img, [box], isClosed=True, color=0, thickness=4)

def send_data(loc,api:CtrlxDlAPi,node:str):
    robot_coords = dict()
    rob_data = dict()
    robot_coords['X'] = loc['x']
    robot_coords['Y'] = loc['y']
    robot_coords['Z'] = 10
    angle = loc['angle']
    angle = angle + 90
    if angle <= 0:
        angle = 360 + angle
    robot_coords['Rotation'] = angle
    rob_data['type'] = 'object'
    rob_data['value'] = robot_coords
    r = api.write_node(node, rob_data)


def get_inference(dl_node:CtrlxDlAPi, addr):
    bool_value = Variant() 
    bool_value.set_bool8(True)
    dl_node.write_node(addr + '/control/inference/request',bool_value)
    data = dl_node.read_node(addr + '/control/inference/ready')
    while data.get_bool8():
        time.sleep(0.01)
        data = dl_node.read_node(addr + '/control/inference/ready')

    bool_value = Variant() 
    bool_value.set_bool8(False)
    dl_node.write_node(addr + '/control/inference/request', bool_value)

    while not data.get_bool8():
        time.sleep(0.01)
        data = dl_node.read_node(addr + '/control/inference/ready')

    fb_data = dl_node.read_node(addr + '/output/result')
    result = Result.Result.GetRootAs(fb_data.get_flatbuffers())
    data = dict() 
    data['instances'] = []
    for i in range(result.InstancesLength()): 
        inst = result.Instances(i)
        score = inst.Score()
        class_index = inst.ClassIndex()
        bounding_box = OrientedBoundingBox.OrientedBoundingBox()
        bounding_box = inst.OrientedBoundingBox(bounding_box) 
        x_center = bounding_box.CenterX() 
        y_center = bounding_box.CenterY() 
        angle = bounding_box.Angle() 
        width = bounding_box.Width()
        height = bounding_box.Height()
        instance = dict() 
        instance['score'] = score 
        instance['class_index'] = class_index
        box = dict() 
        box['center_x'] = x_center 
        box['center_y'] = y_center 
        box['angle'] = angle
        box['width'] = width
        box['height'] = height
        instance['oriented_bounding_box'] = box
        data['instances'].append(instance)
        
    letters = dict()
    if len(data['instances']) == 0: 
        return None,None,None,None,None
    
    for inst in data['instances']:
        if lookup[inst['class_index']] in letters.keys():
            letter = letters[lookup[inst['class_index']]]
            if inst['score'] > letter['score']:
                letters[lookup[inst['class_index']]] = inst
        else:
            letters[lookup[inst['class_index']]] = inst

    c_inst = letters.get('c',None)
    c = None 
    t = None 
    r = None 
    l = None 
    x = None 
    if c_inst is not None:
        c = c_inst['oriented_bounding_box']
        c['score'] = c_inst['score']
    t_inst = letters.get('t',None)
    if t_inst is not None:
        t = t_inst['oriented_bounding_box']
        t['score'] = t_inst['score']
    r_inst = letters.get('r',None)
    if r_inst is not None:
        r = r_inst['oriented_bounding_box']
        r['score'] = r_inst['score']
    l_inst = letters.get('l',None)
    if l_inst is not None:
        l = l_inst['oriented_bounding_box']
        l['score'] = l_inst['score']
    x_inst = letters.get('x',None)
    if x_inst is not None:
        x = x_inst['oriented_bounding_box']
        x['score'] = x_inst['score']
    return c,t,r,l,x

def transform(H,resp):
    x = resp['center_x']
    y = resp['center_y']
    angle = resp['angle']
    rad = angle * np.pi / 180
    v1 = np.array([np.cos(rad),np.sin(rad)])
    J = get_J(H, x, y)
    v2 = J @ v1
    angle2 = np.arctan2(v2[1],v2[0])* 180/np.pi
    point = np.array([x,y,1]).reshape((3, 1))
    point2 = np.dot(H,point)
    x2 = point2[1]/point2[2]
    y2 = point2[0]/point2[2]
    return {'x': x2[0], 'y': y2[0], 'angle': angle2}

def run_vision(dl_node:CtrlxDlAPi, camera_node, inference_node, img_loc) -> tuple[bool,ErrorCodes]:
    data = dl_node.read_node(camera_node +'/control/connect-camera/ready')
    if data.get_bool8():
        dl_node.write_node(camera_node + '/control/connect-camera/request', data)
        while data.get_bool8():
            data = dl_node.read_node(camera_node + '/control/connect-camera/ready')
        data.set_bool8(False)
        dl_node.write_node(camera_node + '/control/connect-camera/request',
                           data)
    payload = Variant()
    payload.set_bool8(True)
    dl_node.write_node(camera_node + '/control/capture/request', payload)
    data = dl_node.read_node(camera_node + '/control/capture/ready')
    while data.get_bool8():
        data = dl_node.read_node(camera_node + '/control/capture/ready')
        time.sleep(0.050)

    payload.set_bool8(False)
    dl_node.write_node(camera_node + '/control/capture/request', payload)
    data = dl_node.read_node(camera_node + '/control/capture/ready')
    while not data.get_bool8():
        data = dl_node.read_node(camera_node +'/control/capture/ready')
        time.sleep(0.050)
    r = dl_node.read_image(camera_node + '/output/image/data')
    data = np.frombuffer(r, dtype=np.uint8)
    if data.size == 0:
        return
    data = data.astype(np.float64)
    data = data.reshape((1080, 1440))
    _max = np.max(data)
    data = 1.2 * data
    data = np.where(data > _max, _max, data)
    _min = np.min(data)
    data = data - _min
    _max = np.max(data)
    data = (data * 255) / _max
    data = data.astype(np.uint8)
    try:
        pt_src = get_corners(data)
    except Exception as e: 
        cv.imwrite(img_loc,data)
        return False, ErrorCodes.NO_SCREWS
    found = True
    if pt_src.size == 0:
        cv.imwrite(img_loc,data)
        return False, ErrorCodes.NO_SCREWS
    for pt in pt_src:
        if pt[0] ==0 and pt[1] == 0:
            found = False
            break
    if not found:
        return False, ErrorCodes.NO_SCREWS
   
    screw_from_edge = 15
    back_size = 250
    pts_dest = np.array([[screw_from_edge, screw_from_edge],
                             [screw_from_edge, back_size - screw_from_edge],
                             [back_size - screw_from_edge, screw_from_edge],
                             [back_size - screw_from_edge, back_size - screw_from_edge]],
                            dtype=np.float32)
    H, status = cv.findHomography(pt_src, pts_dest)
    try:
        c,t,r,l,x = get_inference(dl_node, inference_node)
    except Exception as e: 
        cv.imwrite(img_loc,data)
        return False, ErrorCodes.VISION_EXCEPTION
    locations = [] 
    if c is not None:
        loc_c = transform(H, c)
        loc_c['class_index'] = 4
        loc_c['score'] = c['score']
        draw_box(c['center_x'], c['center_y'], c['width'], c['height'],c['angle'],data)
        locations.append(loc_c)
    if t is not None:
        loc_t = transform(H, t)
        loc_t['class_index'] = 3
        loc_t['score'] = t['score']
        draw_box(t['center_x'], t['center_y'], t['width'],t['height'],t['angle'],data)
        locations.append(loc_t)
    if r is not None:
        loc_r = transform(H, r)
        loc_r['class_index']= 2
        loc_r['score'] = r['score']
        draw_box(r['center_x'], r['center_y'], r['width'], r['height'],r['angle'],data)
        locations.append(loc_r)
    if l is not None:
        loc_l = transform(H, l)
        loc_l['class_index']= 1
        loc_l['score'] = l['score']
        draw_box(l['center_x'], l['center_y'], l['width'], l['height'],l['angle'],data)
        locations.append(loc_l)
    if x is not None:
        loc_x = transform(H, x)
        loc_x['class_index']= 0
        loc_x['score'] = x['score']
        draw_box(x['center_x'], x['center_y'], x['width'], x['height'],x['angle'],data)
        locations.append(loc_x)
    cv.imwrite(img_loc,data)
    dl_node.write_locations(locations)


    return True, ErrorCodes.NO_ERROR
