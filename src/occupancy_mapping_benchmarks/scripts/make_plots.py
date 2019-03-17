#!/usr/bin/env python

"""
Draw pretty plots.
"""

import os

import matplotlib
matplotlib.use('Agg')

from matplotlib import pyplot as plt
from tabulate import tabulate
import numpy as np
import pandas as pd
import seaborn as sns

def main():
    df = pd.read_csv('bench_output.txt', header=None, names=['name', 'Time'])

    # table

    with open('bench_table.md', 'w') as f:
        f.write(tabulate(df, tablefmt='pipe', showindex=False, headers=['Library', 'Runtime [sec]']))

    # plot

    sns.set()
    f, ax = plt.subplots(figsize=(5, 3.75))
    df.plot.bar(ax=ax, x='name', y='Time', rot=0, legend=False, color=sns.color_palette()[0])
    ax.set_title('Map building time')
    ax.set_xlabel('')
    ax.set_ylabel('Runtime [sec]')

    plt.tight_layout()
    f.savefig('bench_plot.png', bbox_inches='tight')


if __name__ == '__main__':
    main()
