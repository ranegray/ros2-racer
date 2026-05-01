import pyrealsense2 as rs
import matplotlib.pyplot as plot
import numpy as np
import matplotlib.image as image
import scipy.ndimage as ndimage
from scipy.ndimage import find_objects
import colorsys
import os
import sys
from matplotlib.animation import FuncAnimation



class ProcessImage():

    def __init__(self,saveFile = False, filepath =  None, streamImages = True, *args, **kwargs):


        self.saveFile = saveFile
        self.readFile = False
        self.streamImages = streamImages
        self.pipe = rs.pipeline()
        self.readPath = filepath
        self.dirname = ''

        self.fig, ((ax1, ax2),) = plot.subplots(1,2, figsize=(8, 8), squeeze=False)

        self.graph = ax1.imshow([[0],[0]])
        self.MaskImg = ax2.imshow([[0],[0]])


        j = 0
        blobx = 0
        bloby = 0
        if self.saveFile:
            path = os.getcwd()
            self.dirname = os.path.join(path, 'run{}'.format(j))
            while os.path.exists(self.dirname):          
               j = j+1
               self.dirname = os.path.join(path, 'run{}'.format(j))
            os.makedirs(self.dirname)


        if self.streamImages:
            self.Stream_config(rs)             #configure the pipeline
            self.pipe.start()                #start data stream

            img = self.get_sensor_data()           #retrieve raw data

            mask, blobx, bloby = self.image_mask(img)
            self.graph = ax1.imshow(img)  
            self.MaskImg = ax2.imshow(mask, cmap = 'binary')
            self.visualize_data()
        else:
            self.readFile = True

        if self.readFile:
            img, mask = self.read_ImageFile()   
            self.FileImg = ax1.imshow(img)
            self.MaskImg = ax2.imshow(mask)  
            plot.show()
        else:
            self.streamImages = True

        
    def Stream_config(self, rs):

        cfg  = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        print("Starting pipeline…")

    def get_sensor_data(self):
        frames = self.pipe.wait_for_frames(5000)   # 5s timeout
        color  = frames.get_color_frame()
        greyScale  = frames.get_color_frame()
        img = np.asanyarray(color.get_data()) #[:,:,::-1]  # Convert BGR to RGB
        return img

    def image_greyscale(self, img): 
              
        lum =  [0.1, 0.1, 0.8] #<---- best greyscale luminosity weights RGB [0.33, 0.33, 0.34] 
        img2 = img*lum 
        img2 = np.sum(img2, axis=2)/255
        return img2

    def image_mask(self, img):
        
        blue = np.zeros((480,640,3))
        img2 = np.copy(img)

        color_sum = np.sum(img2, axis = 2)

        blue[:,:,:][img2[:,:,2]/color_sum[:,:] > .3665] = 1
        blue = ndimage.gaussian_filter(blue, sigma=2)
        blue[blue < 1] = 0
        centroid = ndimage.center_of_mass(blue)
        y_center = np.uint32(centroid[0])
        x_center = np.uint32(centroid[1])
        print(x_center,y_center)

        return blue, x_center, y_center

    def edgeDetection(self, img):
        
        sobel_h = ndimage.sobel(img, 0)  # horizontal gradient
        sobel_v = ndimage.sobel(img, 1)  # vertical gradient
        magnitude = np.sqrt(sobel_h**2 + sobel_v**2)
        magnitude *= 1.0 / np.max(magnitude)  # normalization
        magnitude[magnitude < 0.101] = 0
        magnitude[magnitude > 0.191] = 0


        return magnitude

    def contour(self, img, mask, edge, grey):
            #img = color
            #mask = grayscale mask
            #edge = edges
        tmask = np.copy(mask)
        tedge = np.copy(edge)
        tgrey = np.copy(grey)
        timg = np.copy(img)
        i = 0
        #print(timg[317][446])
        
        
        for x in timg:
            for y in x:
                y[y[0]+y[1]>y[2]+.16] = [0,0,0]

        #for x in timg[1]:
            #x[x[2]<0.8] = [0,0,0]
        timg = np.sum(timg, axis=2)
        timg[timg <.7] = 0
        timg[timg >.7] = 1
        #print(timg[317][446])
        
        return timg
                

    def update(self, event=None):
        #hello = input('exit')

        blobx = 0
        bloby = 0
        if not self.readFile:
            img = self.get_sensor_data()

        else:
            img = image.imread(self.readPath)[:,:,:3]

        self.graph.set_data(img)

        mask,blobx,bloby = self.image_mask(img)
        self.MaskImg.set_data(mask) 

        return img, mask 

    def end_Stream(self):
         self.pipe.stop()

    def read_ImageFile(self):

        img_arr, mask = self.update()
        if self.saveFile:
            self.save_Imagefile(img_arr)
            self.save_Imagefile(grey)

        return img_arr, mask
        
    def save_Imagefile(self, img):

        i = 0
        imgName = 'ProcessedImageframe{}.png'.format(i)

        while os.path.exists(os.path.join(self.dirname,imgName)):                 
            i = i+1            
            imgName = 'ProcessedImageframe{}.png'.format(i)
        image.imsave(os.path.join(self.dirname, imgName), img)

        if (type(img[0][0]) == 'numpy.float64'):
            image.imsave(os.path.join(self.dirname, imgName), img)
        else:
            image.imsave(os.path.join(self.dirname, imgName), img, cmap = 'binary')           

    def visualize_data(self):
        #hello = input('exit')
        ani = FuncAnimation(self.fig, self.update, frames=100, interval =5, blit=False, cache_frame_data =False)
        plot.show()
     
        if self.saveFile:
            i = 0
            imgName = 'ProcessedImageStream{}.gif'.format(i)
            while os.path.exists(os.path.join(self.dirname,imgName)):                 
                i = i+1            
                imgName = 'ProcessedImageStream{}.gif'.format(i)
            name = os.path.join(self.dirname,imgName)        
            #ani.save(name, writer="ffmpeg")


path = '/home/senayt/ADVROBO_Final/ros2-racer/src/image_processing/run5/Imageframe35.png'
test = ProcessImage(saveFile = False, filepath =  path, streamImages = True)

"""def main() -> int:
    path = '/home/senayt/ADVROBO_Final/ros2-racer/src/image_processing/run5/Imageframe35.png'
    test = ProcessImage(saveFile = False, filepath =  path, streamImages = True)
    test.__init__()
    test.end_Stream()
    return 0

if __name__ == '__main__':
    sys.exit(main()) """






