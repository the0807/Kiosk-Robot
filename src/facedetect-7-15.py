#!/usr/bin/env python2

'''
face detection using haar cascades

USAGE:
    facedetect.py [--cascade <cascade_fn>] [--nested-cascade <cascade_fn>] [<video_source>]
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2 as cv
import multiprocessing



# local modules
from video import create_capture
from common import clock, draw_str

#function send data to server
def sender(queue):
    import socket

    HOST = '192.168.1.1'
    # Enter IP or Hostname of your server
    PORT = 12345
    # Pick an open Port (1000+ recommended), must match the server port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    
    while True:
    #Lets loop awaiting for your input
        if not queue.empty():
            try:
                data = queue.get()
                s.send("{},{},{}".format(data[0], data[1], data[2]))
                reply = s.recv(1024)
                #print('child',data)
            except:
                print('disconnected')
                break

 


def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30),
                                     flags=cv.CASCADE_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv.rectangle(img, (x1, y1), (x2, y2), color, 2)

def main():
    import sys, getopt

    args, video_src = getopt.getopt(sys.argv[1:], '', ['cascade=', 'nested-cascade='])
    try:
        video_src = video_src[0]
    except:
        video_src = 0
    args = dict(args)
    cascade_fn = args.get('--cascade', "data/haarcascades/haarcascade_frontalface_alt.xml")
    nested_fn  = args.get('--nested-cascade', "data/haarcascades/haarcascade_eye.xml")

    cascade = cv.CascadeClassifier(cv.samples.findFile(cascade_fn))
    nested = cv.CascadeClassifier(cv.samples.findFile(nested_fn))

    cam = create_capture(video_src, fallback='synth:bg={}:noise=0.05'.format(cv.samples.findFile('samples/data/lena.jpg')))

    queue = multiprocessing.Queue()
    sendp = multiprocessing.Process(target = sender, args = (queue,)) ###################
    sendp.start()

    while True:
        ret, img = cam.read()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        gray = cv.equalizeHist(gray)
        tmp_rects = 0
        eyes_distance = 0
        midX = 0
        t = clock()
        rects = detect(gray, cascade)
        vis = img.copy()
        #size of webcam = 1280 X 960
        draw_rects(vis, rects, (0, 255, 0))
        if len(rects) != 0:
            tmp_rects = rects[0][3]-rects[0][1]
            #print('face length', tmp_rects)  ##############
        if not nested.empty():
            for x1, y1, x2, y2 in rects:
                roi = gray[y1:y2, x1:x2]
                vis_roi = vis[y1:y2, x1:x2]
                subrects = detect(roi.copy(), nested)
                draw_rects(vis_roi, subrects, (255, 0, 0))
                if len(subrects) == 2:
                    down = subrects[0] if subrects[0][3]<subrects[1][3] else subrects[1] 
                    up = subrects[0] if subrects[0][3]>subrects[1][3] else subrects[1]
                    if down[3] > up[1] :
                        left = subrects[0] if subrects[0][2]<subrects[1][2] else subrects[1]
                        right = subrects[0] if subrects[0][2]>subrects[1][2] else subrects[1]
                        eyes_distance = right[0] - left[2]
                        if eyes_distance<0 :
                            eyes_distance = 0;
                midX = (x1+x2)/2

        print('face length',tmp_rects,'eyes_distance',eyes_distance,'position', midX)
        queue.put([tmp_rects, eyes_distance, midX])
        dt = clock() - t


        draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
        #cv.imshow('facedetect', vis)

        if cv.waitKey(5) == 27:
            break

    print('Done')


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
