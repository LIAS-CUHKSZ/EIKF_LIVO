
from opttracker_helper import align_time
import argparse

parser=argparse.ArgumentParser(description='align groundtruth generated from tracker with the odom.')
parser.add_argument('--input_odom', '-i')
parser.add_argument('--tracker', '-t')
parser.add_argument('--output', '-o')
args=parser.parse_args()


align_time(args.input_odom, args.tracker, args.output)   


