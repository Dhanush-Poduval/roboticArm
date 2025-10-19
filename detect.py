import os
import json
import shutil
from sklearn.model_selection import train_test_split
from PIL import Image
import numpy as np

input_folder = '/Users/dhanushpoduval/Downloads/archive/LabPicsChemistry/Train'
output_folder = '/Users/dhanushpoduval/Desktop/interDomain/yolov5/data/labpics_yolo'
classes = ["vessel", "jar", "vial"]
val_ratio = 0.2


for split in ["train", "val"]:
    os.makedirs(os.path.join(output_folder, "images", split), exist_ok=True)
    os.makedirs(os.path.join(output_folder, "labels", split), exist_ok=True)


all_samples = [os.path.join(input_folder, d) for d in os.listdir(input_folder)
               if os.path.isdir(os.path.join(input_folder, d))]
train_samples, val_samples = train_test_split(all_samples, test_size=val_ratio, random_state=42)


def mask_to_bbox(folder, mask_path):
    """Return bounding box [x_min, y_min, x_max, y_max] from a binary mask."""
    mask_full = os.path.join(folder, mask_path.lstrip("/"))
    if not os.path.exists(mask_full):
        return None
    mask = np.array(Image.open(mask_full).convert("L"))
    ys, xs = np.where(mask > 0)
    if len(xs) == 0 or len(ys) == 0:
        return None
    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()
    return [x_min, y_min, x_max, y_max]


def process_samples(samples, split):
    copied_images = 0
    for folder in samples:
        json_file = os.path.join(folder, "Data.json")
        image_file = os.path.join(folder, "Image.jpg")
        if not os.path.exists(image_file):
            print(f"Skipping {folder}, no image found")
            continue

        img = Image.open(image_file)
        w, h = img.size

        yolo_lines = []
        if os.path.exists(json_file):
            with open(json_file) as f:
                data = json.load(f)
            vessels = data.get("Vessels", {}).values()
            for obj in vessels:
                for type_name in obj.get("VesselType_ClassNames", []):
                    label = type_name.lower()
                    if label not in classes:
                        continue
                    bbox = mask_to_bbox(folder , obj.get("MaskFilePath", ""))
                    if not bbox:
                        continue
                    x_min, y_min, x_max, y_max = bbox
                    x_center = (x_min + x_max) / 2 / w
                    y_center = (y_min + y_max) / 2 / h
                    width = (x_max - x_min) / w
                    height = (y_max - y_min) / h
                    class_idx = classes.index(label)
                    yolo_lines.append(f"{class_idx} {x_center} {y_center} {width} {height}")


        dest_image = os.path.join(output_folder, "images", split, os.path.basename(folder) + ".jpg")
        shutil.copy(image_file, dest_image)
        copied_images += 1


        dest_label = os.path.join(output_folder, "labels", split, os.path.basename(folder) + ".txt")
        with open(dest_label, "w") as f:
            f.write("\n".join(yolo_lines))

    print(f"{copied_images} images processed for {split}")


process_samples(train_samples, "train")
process_samples(val_samples, "val")
print("YOLO dataset creation complete!")
