#!/usr/bin/env python3
""" Script for generating an example IMacExecutor run for unit tests.

Author: Charlie Street
Owner: Charlie Street
"""

import csv


def gen_run(out_file):
    """Writes out a dummy IMac run to file for the purpose of a unit test.

    Args:
        out_file: The file to write the run to
    """

    with open(out_file, "w") as csvfile:
        writer = csv.writer(csvfile, delimiter=", ")
        for t in range(11):
            row = [t]
            for y in range(3):
                for x in range(2):
                    row.append(x)
                    row.append(y)
                    if (x == 1 and y == 0 and t == 0) or (x == 2 and y == 0 and t < 8):
                        row.append(1)
                    else:
                        row.append(0)
            writer.writerow(row)


if __name__ == "__main__":
    out_path = "../../data/tests/energyFunctionalIMacRun.csv"
    gen_run(out_path)
