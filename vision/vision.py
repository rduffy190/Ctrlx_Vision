import cv2 as cv 
import numpy as np 
from math import sqrt
import time 
from api_helper.ctrlx_api import CtrlxDlAPi
from hdv.object_detection2d import Result, OrientedBoundingBox
from ctrlxdatalayer.variant import Variant
from vision.error_codes import ErrorCodes
from scipy import ndimage
from time import time_ns

#lookup = {1: 'x',2:'l', 3:'r', 4:'t', 0: 'c' }
lookup = {0: 'x',1:'l', 2:'r', 3:'t', 4: 'c' }

def template_match(template, img, use_mask = False, use_noise= False, rot = 36, rot_fact = 10, function = cv.TM_CCORR_NORMED, invert = False):
    img_rot = 0
    n_rot = 0
    last_max = 0
    tmask = None
    bRotate = False
    if use_noise:
        noise = np.random.uniform(0,256,template.shape).astype(np.uint8)
    else:
        noise = 220
    while n_rot < rot:
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.rotate.html
        if use_noise or use_mask: 
            c_val = -1
        else: 
            c_val = 220
        rot_img = ndimage.rotate(template, rot_fact * n_rot, reshape=False,mode='constant', cval=c_val, order=3)
        if use_noise:
            rot_img = np.where(rot_img < 0,noise,rot_img)
        if use_mask:
            tmask = np.where(rot_img <0, 0, 1).astype(np.uint8)
        match = cv.matchTemplate(img,rot_img, function,mask=tmask)
        #cv.imwrite('rot_temp' + str(n_rot) + '.png', rot_img)
        if invert:
            match = -match
        max_val = np.max(match)
        if max_val > last_max:
            last_max = max_val
            img_rot = n_rot
            bRotate = True
        n_rot = n_rot + 1
    internal_template = rot_img
    if bRotate:
        internal_template = ndimage.rotate(template, rot_fact * img_rot, reshape=False,mode='constant', cval=220, order=3)
        if use_noise:
            internal_template = np.where(internal_template == 220,noise,internal_template)
    if use_mask:
        tmask = np.where(internal_template == -1, 0, 1).astype(np.uint8)
    match = cv.matchTemplate(img,internal_template, function, mask = tmask)
    if invert:
        match = - match
    #cv2.imshow('x', internal_template)
    #cv2.waitKey(0)
    points = np.argwhere(match == match.max())
    point = points.mean(axis=0)
    point[0] = point[0] + internal_template.shape[0]//2
    point[1] = point[1] + internal_template.shape[1]//2
    return [point[1],point[0]], -1*img_rot*rot_fact

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
    angle2 = np.arctan2(v2[0],v2[1])* 180/np.pi
    point = np.array([x,y,1]).reshape((3, 1))
    point2 = np.dot(H,point)
    x2 = point2[1]/point2[2]
    y2 = point2[0]/point2[2]
    return {'x': x2[0], 'y': y2[0], 'angle': angle2}

def run_vision(dl_node:CtrlxDlAPi, camera_node, inference_node, img_loc,use_template, template_loc, simulate, run_count = 0, sim_images = 0) -> tuple[bool,ErrorCodes]:
    if simulate:
        data = dl_node.read_node(camera_node + '/control/load/ready')
        if not data.get_bool8():
            payload = Variant()
            payload.set_bool8(True)
            dl_node.write_node(camera_node + '/control/list-images/request',payload)
            data = dl_node.read_node(camera_node + '/control/list-images/ready')
            while data.get_bool8(): 
               data = dl_node.read_node(camera_node + '/control/list-images/ready')
               time.sleep(0.01)
            payload.set_bool8(False)
            dl_node.write_node(camera_node + '/control/list-images/request',payload)
    else:
        data = dl_node.read_node(camera_node +'/control/connect-camera/ready')
        count = 0 
        if data.get_bool8():
            dl_node.write_node(camera_node + '/control/connect-camera/request', data)
            while data.get_bool8():
                data = dl_node.read_node(camera_node + '/control/connect-camera/ready')
                count = count + 1
                time.sleep(0.01)
                if count > 200: 
                    payload = Variant()
                    payload.set_bool8(True)
                    dl_node.write_node(camera_node + '/control/disconnect-camera/request',payload)
                    time.sleep(0.05)
                    data = dl_node.read_node(camera_node +'/control/disconnect-camera/ready')
                    cnt2 = 0
                    while data.get_bool8(): 
                        data = dl_node.read_node(camera_node +'/control/disconnect-camera/ready')
                        time.sleep(0.01)
                        cnt2 = cnt2 + 1 
                        if cnt2 > 100: 
                            break
                    payload.set_bool8(False) 
                    dl_node.write_node(camera_node + '/control/disconnect-camera/request',payload)
                    payload.set_bool8(True)
                    dl_node.write_node(camera_node + '/control/reset/request', payload)
                    time.sleep(0.05)
                    data = dl_node.read_node(camera_node + '/control/reset/ready')
                    cnt2 = 0
                    while data.get_bool8(): 
                        data = dl_node.read_node(camera_node + '/control/reset/ready')
                        time.sleep(0.01)
                        cnt2 = cnt2 + 1 
                        if cnt2 > 50: 
                            break
                    payload.set_bool8(False)
                    dl_node.write_node(camera_node + '/control/reset/request', payload)
                    return False, ErrorCodes.CAMMERA_CONNECTION_FAIL
            data.set_bool8(False)
            count = 0
            dl_node.write_node(camera_node + '/control/connect-camera/request',
                           data)
    payload = Variant()
    if simulate: 
        start = time_ns()
        index_read = Variant()
        payload.set_bool8(True)
        index_read.set_uint32(run_count % sim_images)
        dl_node.write_node(camera_node + '/input/index-of-image-to-load', index_read)
        time.sleep(0.01)
        dl_node.write_node(camera_node +'/control/load/request', payload)
        data = dl_node.read_node(camera_node + '/control/load/ready')
        while data.get_bool8():
            data = dl_node.read_node(camera_node + '/control/load/ready')
            time.sleep(0.01)
        count = 0
        while not data.get_bool8(): 
            payload.set_bool8(False)
            dl_node.write_node(camera_node +'/control/load/request',payload)
            time.sleep(0.01)
            count = count + 1
            data = dl_node.read_node(camera_node + '/control/load/ready')
            if count >50 : 
                return False, ErrorCodes.DL_FAIL
        end = time_ns()
        dl_node.write_cature_time_node((end - start)* 1E-6)

    else:
        data = dl_node.read_node(camera_node + '/control/capture/ready')
        count = 0
        while not data.get_bool8(): 
            payload.set_bool8(False)
            dl_node.write_node(camera_node + '/control/capture/request', payload)
            data = dl_node.read_node(camera_node + '/control/capture/ready')
            time.sleep(0.010)
            count = count + 1
            if count >50 : 
                return False, ErrorCodes.DL_FAIL

        payload.set_bool8(True)
        dl_node.write_node(camera_node + '/control/capture/request', payload)
        start = time_ns()
        data = dl_node.read_node(camera_node + '/control/capture/ready')
        count = 0
        while data.get_bool8():
            data = dl_node.read_node(camera_node + '/control/capture/ready')
            time.sleep(0.010)
            count = count + 1
            if count >50 : 
                payload.set_bool8(False)
                dl_node.write_node(camera_node + '/control/capture/request', payload)
                return False, ErrorCodes.DL_FAIL
            
        end = time_ns() 
        dl_node.write_cature_time_node((end - start)* 1E-6)
        payload.set_bool8(False)
        dl_node.write_node(camera_node + '/control/capture/request', payload)
        data = dl_node.read_node(camera_node + '/control/capture/ready')
        while not data.get_bool8():
            data = dl_node.read_node(camera_node +'/control/capture/ready')
            time.sleep(0.010)
    r = dl_node.read_image(camera_node + '/output/image/data')
    img = np.frombuffer(r, dtype=np.uint8)
    #cv.imwrite('img' + str(run_count) + '.png',img.reshape((1080,1440)))
    data = img.astype(np.float64)
    if data.size == 0:
        return
    data = img.astype(np.float64)
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
        data = cv.resize(data,(720,540))
        data = data.astype(np.uint8).flatten()
        dl_node.write_bounding_box_img(data)
        dl_node.write_new_img(True)
        return False, ErrorCodes.NO_SCREWS
    found = True
    if pt_src.size == 0:
        cv.imwrite(img_loc,data)
        data = cv.resize(data,(720,540))
        data = data.astype(np.uint8).flatten()
        dl_node.write_bounding_box_img(data)
        dl_node.write_new_img(True)
        return False, ErrorCodes.NO_SCREWS
    for pt in pt_src:
        if pt[0] ==0 and pt[1] == 0:
            found = False
            break
    if not found:
        cv.imwrite(img_loc,data)
        data = cv.resize(data,(720,540))
        data = data.astype(np.uint8).flatten()
        dl_node.write_bounding_box_img(data)
        dl_node.write_new_img(True)
        return False, ErrorCodes.LESS_THAN_FOUR_SCREWS_OR_DIST_WRONG
   
    screw_from_edge = 15
    zero_offset = 15
    back_size = 250
    pts_dest = np.array([[screw_from_edge - zero_offset, screw_from_edge - zero_offset],
                             [screw_from_edge - zero_offset, back_size - screw_from_edge - zero_offset],
                             [back_size - screw_from_edge - zero_offset, screw_from_edge - zero_offset],
                             [back_size - screw_from_edge - zero_offset, back_size - screw_from_edge - zero_offset]],
                            dtype=np.float32)
    H, status = cv.findHomography(pt_src, pts_dest)
    try:
        start = time_ns()
        c,t,r,l,x = get_inference(dl_node, inference_node)
        end = time_ns() 
        ms_time = (end - start)*1E-6
        dl_node.write_inference_time_node(ms_time)
    except Exception as e: 
        cv.imwrite(img_loc,data)
        return False, ErrorCodes.VISION_EXCEPTION
    locations = [] 
    tm_data = None
    for key in use_template.keys(): 
        if use_template[key]:
            tm_data = img.astype(np.float64)
            tm_data = tm_data.reshape((1080, 1440))
            _max = np.max(tm_data)
            tm_data = 2.5 * tm_data
            tm_data = np.where(data > _max, _max, tm_data)
            _min = np.min(tm_data)
            tm_data = tm_data - _min
            _max = np.max(tm_data)
            tm_data = (tm_data * 255) / _max
            tm_data = tm_data.astype(np.uint8)
            break
    if c is not None:
        
        if 'c' in use_template: 
            if use_template['c']: 
                c_data = tm_data[round(c['center_y'])-100 : round(c['center_y'])+ 100, round(c['center_x']-100) : round(c['center_x']+100)]
                c_temp = template_loc['c']
                c_temp = c_temp.astype(np.float32)
                c_temp = np.pad(c_temp, ((0, 0), (12, 12)), mode='constant', constant_values=-1)
                noise = np.random.uniform(np.random.randint(0, 256, c_temp.shape, dtype=np.uint8)).astype(np.float32)
                c_temp = np.where(c_temp == -1, noise,c_temp)
                p = template_match(c_temp,c_data.astype(np.float32),rot = 180, rot_fact = 2, use_noise = True)
                c['center_x'] = p[0][0] - 100 + c['center_x']
                c['center_y'] = p[0][1] - 100 + c['center_y']
                ang = p[1]
                if ang < -180: 
                    ang = ang + 360
                c['angle'] = ang
               
                c['width'] = 146 
                c['height'] = 170
        draw_box(c['center_x'], c['center_y'], c['width'], c['height'],c['angle'],data)
        loc_c = transform(H, c)
        loc_c['class_index'] = 4
        loc_c['score'] = c['score']
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
        if 'l' in use_template: 
            if use_template['l']: 
                l_data = tm_data[round(l['center_y']-140) : round(l['center_y']+ 140), round(l['center_x']-140) : round(l['center_x'] + 140)]
                l_temp = template_loc['l']
                l_temp = l_temp.astype(np.float32)
                l_temp = np.pad(l_temp, ((1, 1), (90, 90)), mode='constant', constant_values=-1)
                #cv.imwrite('/home/riley/Dev/Ctrlx_Vision/l_temp.png',l_temp.astype(np.uint8))
                p = template_match(l_temp,l_data.astype(np.float32),rot = 72, rot_fact = 2.5, use_mask=True)
                l['center_x'] = p[0][0] - 140 + l['center_x']
                l['center_y'] = p[0][1] - 140 + l['center_y']
                ang = p[1]
                if ang < -180: 
                    ang += 360
                l['angle'] = ang
                l['width'] = 59 
                l['height'] = 233

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
    data = cv.resize(data,(720,540))
    data = data.astype(np.uint8).flatten()
    dl_node.write_bounding_box_img(data)
    dl_node.write_new_img(True)
    return True, ErrorCodes.NO_ERROR
