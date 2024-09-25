import argparse

from z0 import z0

parser=argparse.ArgumentParser(description='don not compare z0.')
parser.add_argument('--input', '-o')
parser.add_argument('--output', '-i')
args=parser.parse_args()


z0(args.input, args.output)
