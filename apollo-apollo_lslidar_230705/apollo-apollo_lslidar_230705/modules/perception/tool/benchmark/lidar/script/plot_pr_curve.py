#!/usr/bin/env python3

"""
plot pr curve provided precision-recall samples
and plot sr curve provided similarity-recall samples
"""

import sys

import matplotlib.pyplot as plt
import numpy


def plot_curve(in_path, out_path, x_label, y_label, title):
    """ plot curve
    """
    data = numpy.loadtxt(in_path)
    plt.gcf().clear()

    num = int(data.shape[0] / 5)
    names = ['OTH', 'PED', 'CYC', 'VEH', 'ALL']
    for i in range(0, 5):
        plt.plot(data[i * num: (i + 1) * num, 1],
                 data[i * num: (i + 1) * num, 0],
                 label=names[i], linestyle='-')

    plt.grid()
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)
    plt.ylim(0, 1.1)
    #plt.legend(bbox_to_anchor=(0.98, 0.99), ncol=len(names), labels=names, prop={'size': 10})
    plt.legend(ncol=len(names), labels=names, prop={'size': 10})

    plt.savefig(out_path, dpi=150)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        parent = '.'
    else:
        parent = sys.argv[1]
    prc_in = parent + '/prc_sample'
    prc_out = parent + '/pr_curve.png'
    print("prc in is %s" % prc_in)
    plot_curve(prc_in, prc_out, 'Recall', 'Precision', 'PR Curve')
    src_in = parent + '/src_sample'
    src_out = parent + '/sr_curve.png'
    plot_curve(src_in, src_out, 'Recall', 'Similarity', 'SR Curve')
