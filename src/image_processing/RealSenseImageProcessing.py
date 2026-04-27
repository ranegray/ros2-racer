import pyrealsense2 as rs
import matplotlib.pyplot as plot
import numpy as np
import matplotlib.image as image
import scipy.ndimage as ndimage
import colorsys
import os
import sys
from matplotlib.animation import FuncAnimation



class ProcessImage():

    def __init__(self,saveFile = True, filepath =  None, streamImages = False, *args, **kwargs):


        self.saveFile = saveFile
        self.readFile = False
        self.streamImages = streamImages
        self.pipe = rs.pipeline()
        self.readPath = filepath
        self.dirname = ''

        self.fig, ((ax1, ax2),(ax3, ax4)) = plot.subplots(2,2, figsize=(8, 8), squeeze=False)
        plot.tight_layout()

        self.graph = ax1.imshow([[0],[0]])
        self.FileImg = ax2.imshow([[0],[0]])

        self.EdgeImg = ax3.imshow([[0],[0]])
        self.MaskImg = ax4.imshow([[0],[0]])

        #ax1[0, 0].plot()
        #ax2[0, 1].plot()
        #ax3[1, 0].plot()
        #ax4[1, 1].plot()

        j = 0
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
            grey = self.image_greyscale(img)
            edge = self.edgeDetection(grey)
            mask = self.image_mask(grey)

            self.FileImg = ax2.imshow(grey, cmap = 'binary')
            self.graph = ax1.imshow(img)     
            self.EdgeImg = ax3.imshow(edge, cmap = 'binary')
            self.MaskImg = ax4.imshow(mask, cmap = 'binary')
            #ani = FuncAnimation(fig, self.update, frames=100, interval =5, blit=False, cache_frame_data =False)   
            self.visualize_data()
            #plot.show()  
        else:
            self.readFile = True

        if self.readFile:
            img, grey, edge, mask = self.read_ImageFile()
            self.graph = ax1.imshow(grey, cmap = 'binary')     
            self.FileImg = ax2.imshow(img)
            self.EdgeImg = ax3.imshow(edge, cmap = 'binary')
            self.MaskImg = ax4.imshow(mask, cmap = 'binary')  
            plot.show()
        else:
            self.streamImages = True


   
        #image_greyscale()             #Apply threshold + shape detection
        #image_mask()                #create overlay with shapes detected
                                      #display raw data with overlay
        
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

    def image_mask(self, grey):
        
             #Color value of pixel at y = 145, x = 386: 0.0014609765772726022   Too Low
                                   #Color value of pixel at y = 426, x = 509: 0.002122030075741749   Too High
        #shade = 0.002 too high
        #shade = 0.0019 better, maybe too high
        #shade = 0.0018 probably good lower bound
        Lowshade = 0.0016234
        Highshade = 0.002023
        #print('Color value of pixel at y = 426, x = 509: ', shade)
        mask = np.copy(grey)
        mask[mask < Lowshade] = 0
        mask[mask > Highshade] = 0
        return mask

    def edgeDetection(self, img):
        
        sobel_h = ndimage.sobel(img, 0)  # horizontal gradient
        sobel_v = ndimage.sobel(img, 1)  # vertical gradient
        magnitude = np.sqrt(sobel_h**2 + sobel_v**2)
        magnitude *= 1.0 / np.max(magnitude)  # normalization
        magnitude[magnitude < 0.101] = 0
        magnitude[magnitude > 0.191] = 0

        return magnitude

    def contour(self, mask, edge, grey):
            #img = color
            #mask = grayscale mask
            #edge = edges
        tmask = np.copy(mask)
        tedge = np.copy(edge)
        tgrey = np.copy(grey)

        #tmask += tedge 
        #tmask = ndimage.median_filter(tedge, size=2)
        #tmask = tmask*tgrey
        tmask += tedge 
        tmask *= tedge 
        #tmask = ndimage.binary_opening(tmask, structure=np.ones((3,3))).astype(int)
        #tedge = ndimage.binary_opening(tedge)
        #tedge = ndimage.binary_erosion(tedge)
        #tedge = ndimage.binary_propagation(tedge, mask=tmask)
        
        return tmask
                

    def update(self, event=None):
        
        #img = image.imread(self.readPath)[:,:,:3]
        if not self.readFile:
            img = self.get_sensor_data()

        self.graph.set_data(img)


        grey = self.image_greyscale(img)

        edge = self.edgeDetection(grey)
        self.EdgeImg.set_data(edge)

        mask = self.image_mask(grey)
        self.MaskImg.set_data(mask) 
    

        con = self.contour(mask, edge, grey)
        self.FileImg.set_data(grey)
        #hello = input('enter')

        return img, con, edge, mask

    def end_Stream(self):
         self.pipe.stop()

    def read_ImageFile(self):

        img_arr, grey, edge, mask = self.update()
        if self.saveFile:
            self.save_Imagefile(img_arr)
            self.save_Imagefile(grey)

        return img_arr, grey, edge, mask
        
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

        ani = FuncAnimation(self.fig, self.update, frames=100, interval =5, blit=False, cache_frame_data =False)
        plot.show()
     
        if self.saveFile:
            i = 0
            imgName = 'ProcessedImageStream{}.gif'.format(i)
            while os.path.exists(os.path.join(self.dirname,imgName)):                 
                i = i+1            
                imgName = 'ProcessedImageStream{}.gif'.format(i)
            name = os.path.join(self.dirname,imgName)        
            ani.save(name, writer="ffmpeg")


def main() -> int:
    path = '/home/senayt/ADVROBO_Final/ros2-racer/src/image_processing/run5/Imageframe35.png'
    test = ProcessImage(saveFile = False, filepath =  path, streamImages = True)
    test.__init__()
    test.end_Stream()
    return 0

if __name__ == '__main__':
    sys.exit(main()) 






