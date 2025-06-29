from rosbags.highlevel import AnyReader # imported rosbags modules
from pathlib import Path # imported the path
import cv2 # imported the cv2 module 
from rosbags.image import message_to_cvimage # importing the image 


# function for getting the images
def get_images(bagdir, outdir):
    outdir = Path(outdir) # setting up the ouput dir
    if not outdir.exists(): # if not existing 
        outdir.mkdir()
    count = 0
    
    with AnyReader([Path(bagdir)]) as reader:
        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype) #print he connection topic and msgtype
        for connection, timestamp, rawdata in reader.messages():
            if (
                connection.topic == "/camera/camera/color/image_raw" # check if the topic exist or not 
            ):  # topic Name of images
                msg = reader.deserialize(rawdata, connection.msgtype)
                # change encoding type if needed
                img = message_to_cvimage(msg, "bgr8") # function to convert the messaget to cv image 
                cv2.imwrite(outdir / f"img_{count}.png", img)
                count += 1


bagdir = "/home/midhun.eldose/Downloads/rosbag2_2025_06_22-17_37_00" # directory for the ros2bag
out_dir = "/home/midhun.eldose/Desktop/" # directory for the output

get_images(bagdir, out_dir) # calling the function 
