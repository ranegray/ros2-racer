import pyrealsense2 as rs
import matplotlib.pyplot as plot
import numpy as np
from matplotlib.animation import FuncAnimation

W, H, FPS = 640, 480, 30  # width, heigh, frames per second

pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)

print("Starting pipeline…")
pipe.start()
frames = pipe.wait_for_frames(5000)   # 5s timeout
color  = frames.get_depth_frame()
img = np.asanyarray(color.get_data()) # already BGR
im = plot.imshow(img)

def update(self):
        frames = pipe.wait_for_frames(5000)   # 5s timeout
        color  = frames.get_depth_frame()
        img = np.asanyarray(color.get_data())  # already BGR
        im.set_data(img)
        return im


ani = FuncAnimation(plot.gcf(), update, frames=range(100), interval=5, blit=False)
plot.show()
