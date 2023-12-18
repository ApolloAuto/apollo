import os
import sys
import argparse
from numpy import genfromtxt

APOLLO_ROOT = "/apollo"


def main():
    """
    Main cyber
    """
    parser = argparse.ArgumentParser(description='Obtain relevant data based on CSV files')
    parser.add_argument(
        '-f',
        '--file',
        help=' files name ',
        default='garage')

    args = vars(parser.parse_args())
    record_file = os.path.join(APOLLO_ROOT, 'data/log/' + args['file'])

    try:
        file_handler = open(record_file, 'r')
    except (IOError) as ex:
        sys.exit(1)
    data = genfromtxt(file_handler, delimiter=',', names=True)
    f = data['time'][0]
    l = len(data)
    e = data['time'][l - 1]
    print(e - f)

if __name__ == '__main__':
    main()
