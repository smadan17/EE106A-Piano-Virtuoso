import sys 
if sys.version_info[0] == 3:
	from tkinter import *
else:
	from Tkinter import *
from PIL import Image, ImageTk, ImageDraw
import json
import yaml
import os.path
import os
import cv2
import time
import numpy as np

with open("cfg.yml", "r") as ymlfile:
    cfg = yaml.load(ymlfile)

cwd = os.getcwd() + "/"
jsonFile = cwd + cfg['outputDir'] + cfg['imgName'] + ".json"
imgFile = cwd + cfg['inputDir'] + cfg['imgName'] + cfg['imgFormat']

#returns downsampled img, crop coords (possibly shrunk) in that img, and cropping function
def get_cropped_img(shrink=1):
    if not os.path.isfile(jsonFile):
        get_crop_points()
    with open(jsonFile, "r") as r_js:
        coords = json.load(r_js)
    img = cv2.imread(imgFile)
    downsample = cfg['downsample']
    coords = [[int(d/downsample) for d in c] for c in coords]
    avg = np.average(coords, axis=0)
    coords = (coords - avg) * shrink + avg
    img = cv2.resize(img, (0, 0), fx=1.0/downsample, fy=1.0/downsample)

    bbox = []
    for i in [0, 1]:
        bbox.append(min([c[i] for c in coords]))
        bbox.append(max([c[i] for c in coords]))

    def crop_img(img):
        return img[int(bbox[2]):int(bbox[3]), int(bbox[0]):int(bbox[1])]

    return img, coords, crop_img


globalimage = None
coords = []


def get_crop_points():
    root = Tk()
    img = Image.open(imgFile)
    w, h = img.size
    root.geometry(str(w + 150) + "x" + str(h + 150) + "+0+0")

    # setting up a tkinter canvas with scrollbars
    frame = Frame(root, bd=2, relief=SUNKEN)
    frame.grid_rowconfigure(0, weight=1)
    frame.grid_columnconfigure(0, weight=1)
    canvas = Canvas(frame, bd=0)
    canvas.grid(row=0, column=0, sticky=N+S+E+W)
    frame.pack(fill=BOTH, expand=1)

    # adding the image
    photoimg = ImageTk.PhotoImage(img)
    globalimage = photoimg
    canvasimg = canvas.create_image(0, 0, image=globalimage, anchor="nw")

    # function to be called when mouse is clicked
    def clickpoint(event):
        global globalimage
        global coords

        # close on 5th click
        if len(coords) == 5:
            root.destroy()
        else:
            coords.append((event.x, event.y))
            if len(coords) == 4:
                with open(jsonFile, "w") as w_js:
                    json.dump(coords, w_js)
                coords = coords + [coords[0]]
            if len(coords) > 1:
                img = Image.open(imgFile)
                draw = ImageDraw.Draw(img)
                draw.line(coords, fill=128, width=5)
                del draw
                globalimage = ImageTk.PhotoImage(img)
                canvas.itemconfig(canvasimg, image=globalimage)
    # mouseclick event
    canvas.bind("<Button 1>", clickpoint)
    root.mainloop()
