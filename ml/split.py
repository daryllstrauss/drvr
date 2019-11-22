import json
import sys
from random import random


def split(dirname: str, testsize: float) -> None:
    with open(f"{dirname}/actions.json", "r") as fp:
        data = json.load(fp)
    test = []
    train = []
    for rec in data:
        if random() < testsize:
            test.append(rec)
        else:
            train.append(rec)
    with open("train.json", "w") as fp:
        json.dump(train, fp)
    with open("test.json", "w") as fp:
        json.dump(test, fp)


if __name__ == "__main__":
    split(sys.argv[-1], 0.2)
