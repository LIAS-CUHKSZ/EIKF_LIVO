import argparse

from rtk2xyz import rtk2xyz

parser=argparse.ArgumentParser(description='Convert rtk data to xyz.')
parser.add_argument('--output', '-o')
parser.add_argument('--input', '-i')
args=parser.parse_args()


rtk2xyz(args.input,args.output,flag=True)
