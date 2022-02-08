import sys
import numpy as np


def f(x):
    # Implementation here...
    rval = 0

    return rval


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        print(
            "Usage:\n$ python myfirstscript.py <a number>")
        sys.exit()
    x = float(sys.argv[1])
    rval = f(x)
    print(f"\nf({x}) = {rval}")
