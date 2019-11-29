
import os
import json
from os.path import join, isfile
import sys
from PIL import Image


def tagOne(name:str, output:str) -> str:
    im = Image.open(name).rotate(90)
    im.show()
    im = im.resize((18, 32)).convert('LA')
    im.save(output)
    while True:
        v = input("Action? ")
        if v == "l" or v == "r" or v == "s":
            return v

def trainDir(dirname: str) -> None:
    count = 1
    recs = []
    for name in os.listdir(dirname):
        inname = f"{dirname}/{name}"
        if isfile(inname) and inname[-4:] == ".png":
            outname = f"{dirname}/out-{count:04}.png"
            action = tagOne(inname, outname)
            count = count + 1
            recs.append({"orig": inname, "filename": outname, "result": action})
    with open(f"{dirname}/actions.json", "w") as fp:
        json.dump(recs, fp)


def main(dir:str):
    trainDir(dir)

if __name__ == '__main__':
    main(sys.argv[-1])
