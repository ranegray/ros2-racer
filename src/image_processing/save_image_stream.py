import pyrealsense2 as rs
import matplotlib.pyplot as plot
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.image as image
import os


W, H, FPS = 640, 480, 30  # width, heigh, frames per second

pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
j = 0
path = os.getcwd()
dirname = os.path.join(path, 'run{}'.format(j))
while os.path.exists(dirname):          
   j = j+1
   dirname = os.path.join(path, 'run{}'.format(j))
os.makedirs(dirname)
print("Starting pipeline…")
pipe.start(cfg)
frames = pipe.wait_for_frames(5000)   # 5s timeout
color  = frames.get_color_frame()
img = np.asanyarray(color.get_data())[:,:,::-1]  # already BGR
im = plot.imshow(img)
i = 0
def update(self):
        frames = pipe.wait_for_frames(5000)   # 5s timeout
        color  = frames.get_color_frame()
        img = np.asanyarray(color.get_data())[:,:,::-1]  # already BGR
        im.set_data(img)
        imgname = 'Imageframe{x}.png'.format(i)
        while os.path.exists(os.path.join(dirname,imgname)):                 
            i = i+1            
            imgname = 'Imageframe{x}.png'.format(i)
        image.imsave(os.path.join(dirname, imgname), img)
        return im


ani = FuncAnimation(plot.gcf(), update, frames=range(100), interval=5, blit=False)
plot.show()


