import numpy as np
from PIL import Image, ImageDraw
from os import listdir
import csv


def save_features(file, params):
    np.savez_compressed(f'params_lib/{file}', params)
    return


def test_features(value):
    all_vals = get_all_files()
    all_vals.sort()
    if all_vals.size == 0:
        return False

    idx = np.searchsorted(all_vals, value)
    if all_vals[idx] >= int(value-0.05*value):
        return all_vals[idx]
    return False


def get_all_files() -> np.ndarray:
    return np.array([float(f.strip('.npz')) for f in listdir("params_lib/")])


def save(id, sys, sub=""):
    save_png(f'outputs/{sub}test_{id}.png', sys, 10)


def save_params(id, params, fit):
    with open(f"outputs/test_{id}.par", "w") as f:
        f.write(" ".join([str(i) for i in params]))


def save_png(filename, data, scaling):
    data, boundingBox = reposition(data, scaling)
    im = Image.new('L', boundingBox, 255)
    draw = ImageDraw.Draw(im)
    for i in np.nditer(np.arange(len(data) - 1)):
        draw.line(data[i] + data[i + 1], fill=0, width=1)
    del draw
    # Image origin is top left, convert to CAD-style bottom left
    #im = ImageOps.flip(im)
    try:
        im.save(filename, "PNG")
    except SystemError as e:
        print(f"{e} | cannot save")


def reposition(data, scaling=1):
    '''Returns data, boundingBox'''
    xMin = data[0][0]
    yMin = data[0][1]
    xMax = data[0][0]
    yMax = data[0][1]

    for i in range(len(data)):
        if data[i][0] < xMin and data[i][0] != float("nan"):
            xMin = data[i][0]
        if data[i][1] < yMin and data[i][1] != float("nan"):
            yMin = data[i][1]
        if data[i][0] > xMax and data[i][0] != float("nan"):
            xMax = data[i][0]
        if data[i][1] > yMax and data[i][1] != float("nan"):
            yMax = data[i][1]

    repoData = []

    for i in np.arange(len(data)):
        try:
            repoData.append((int((data[i][0] - xMin) * scaling), (int(
                (data[i][1] - yMin) * scaling))))
        except ValueError:
            continue

    return repoData, (int(
        (xMax - xMin) * scaling)+1, int((yMax - yMin) * scaling)+1)


def save_cords(sys):
    with open("outputs\\base.cor", "w") as f:
        for x, i in enumerate(sys):
            if x > 181:
                break
            f.write(f"{i[0]},{i[1]}\n")


def log(filename, id=None, error=None, time=None) -> None:
    with open(f"logs\\{filename}.csv", "a", encoding="UTF-8") as f:
        writer = csv.writer(f)
        writer.writerow((id, error, time))