from botXsrc.botXexport import botXexport
import time

"""
botXexport is a dictionary containing all the reusable components you
developed for the project, and you will use them in the main program.
"""
def main():
    print('starting app ...')
    server = botXexport['rosbridge_suit_component']['module']()
    server.setup()
    time.sleep(10)
    server.subscribe(topic='/chatter', type='std_msgs/String')
    server.advertise(topic='/test', type='std_msgs/String')
    time.sleep(50)
    server.shutdown()

"""
This is the only script that should be running from terminal so that the
program can gather modules correctly, so we need to specify main as entry point.
"""
if __name__ == '__main__':
    main()
