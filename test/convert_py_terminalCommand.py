import os

path = "/home/archiep/Desktop"
# command ="ls"
command ="sudo chmod a+rw /dev/ttyUSB0"

def main():
    print("Current directory is ", os.getcwd())
    os.chdir(path)
    os.system(command)
    x = os.system("ls /dev/ttyUSB0")
    if x==0:
        print("connected")
    else:
        print("disconnected")
    
    print(os.access('/dev/ttyUSB0', os.R_OK))

    # print("Current directory is ", os.getcwd())

if __name__ == '__main__':
    main()