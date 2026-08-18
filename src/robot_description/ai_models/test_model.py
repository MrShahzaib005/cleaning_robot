#!/usr/bin/env python3
"""
Floor Type Model Tester
Tests floor_model.h5 on a single image or folder of images
Usage:
  python test_model.py --image path/to/image.jpg
  python test_model.py --folder path/to/folder/
"""

import os
import sys
import argparse
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing import image
import json
from PIL import Image
import matplotlib.pyplot as plt

# ===== PATHS =====
BASE_DIR   = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR  = os.path.join(BASE_DIR, 'model')
MODEL_PATH = os.path.join(MODEL_DIR, 'floor_model.h5')
CLASS_PATH = os.path.join(MODEL_DIR, 'class_indices.json')

IMG_SIZE = 224

# ===== LOAD MODEL =====
print("Loading model...")
model = load_model(MODEL_PATH)

# Load class indices and reverse them  {0: 'carpet', 1: 'tiles', 2: 'wood'}
with open(CLASS_PATH, 'r') as f:
    class_indices = json.load(f)
idx_to_class = {v: k for k, v in class_indices.items()}
print(f"Classes: {idx_to_class}")
print("Model loaded!\n")

# ===== FLOOR TYPE ACTIONS =====
FLOOR_ACTIONS = {
    'carpet': {
        'speed':   'SLOW',
        'suction': 'HIGH',
        'brush':   'ON',
        'message': 'Carpet detected — high suction, brush on, slow speed'
    },
    'tiles': {
        'speed':   'FAST',
        'suction': 'MEDIUM',
        'brush':   'OFF',
        'message': 'Tiles detected — medium suction, brush off, fast speed'
    },
    'wood': {
        'speed':   'MEDIUM',
        'suction': 'LOW',
        'brush':   'OFF',
        'message': 'Wood detected — low suction, brush off, medium speed'
    }
}

def predict_image(img_path):
    """Predict floor type for a single image"""
    # Load and preprocess
    img = image.load_img(img_path, target_size=(IMG_SIZE, IMG_SIZE))
    img_array = image.img_to_array(img)
    img_array = img_array / 255.0          # rescale same as training
    img_array = np.expand_dims(img_array, axis=0)

    # Predict
    predictions = model.predict(img_array, verbose=0)
    predicted_idx = np.argmax(predictions[0])
    confidence = predictions[0][predicted_idx] * 100
    predicted_class = idx_to_class[predicted_idx]

    # All class probabilities
    all_probs = {idx_to_class[i]: f"{predictions[0][i]*100:.1f}%" 
                 for i in range(len(idx_to_class))}

    return predicted_class, confidence, all_probs

def show_result(img_path, predicted_class, confidence, all_probs):
    """Display image with prediction result"""
    action = FLOOR_ACTIONS[predicted_class]

    print("=" * 50)
    print(f"  Image     : {os.path.basename(img_path)}")
    print(f"  Predicted : {predicted_class.upper()} ({confidence:.1f}%)")
    print(f"  All probs : {all_probs}")
    print("-" * 50)
    print(f"  🤖 ROBOT ACTION:")
    print(f"     Speed   : {action['speed']}")
    print(f"     Suction : {action['suction']}")
    print(f"     Brush   : {action['brush']}")
    print(f"     → {action['message']}")
    print("=" * 50)

    # Show image with result
    img = Image.open(img_path)
    plt.figure(figsize=(6, 5))
    plt.imshow(img)
    plt.title(f"Predicted: {predicted_class.upper()} ({confidence:.1f}%)\n{action['message']}", 
              fontsize=11, color='green' if confidence > 80 else 'orange')
    plt.axis('off')
    plt.tight_layout()
    plt.show()

def test_folder(folder_path):
    """Test all images in a folder and show summary"""
    extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')
    images = [f for f in os.listdir(folder_path) 
              if f.lower().endswith(extensions)]

    if not images:
        print(f"No images found in {folder_path}")
        return

    print(f"\nTesting {len(images)} images in {folder_path}\n")
    results = {'carpet': 0, 'tiles': 0, 'wood': 0}
    low_confidence = []

    for img_name in images:
        img_path = os.path.join(folder_path, img_name)
        predicted_class, confidence, all_probs = predict_image(img_path)
        results[predicted_class] += 1
        status = "✅" if confidence > 80 else "⚠️ "
        print(f"  {status} {img_name:30s} → {predicted_class:8s} ({confidence:.1f}%)")
        if confidence < 80:
            low_confidence.append((img_name, confidence))

    print(f"\n{'='*50}")
    print(f"  SUMMARY:")
    print(f"  Carpet : {results['carpet']} images")
    print(f"  Tiles  : {results['tiles']} images")
    print(f"  Wood   : {results['wood']} images")
    if low_confidence:
        print(f"\n  ⚠️  Low confidence predictions:")
        for name, conf in low_confidence:
            print(f"     {name} ({conf:.1f}%)")
    print(f"{'='*50}")

# ===== MAIN =====
parser = argparse.ArgumentParser(description='Test floor type model')
parser.add_argument('--image',  type=str, help='Path to single image')
parser.add_argument('--folder', type=str, help='Path to folder of images')
args = parser.parse_args()

if args.image:
    if not os.path.exists(args.image):
        print(f"Image not found: {args.image}")
        sys.exit(1)
    predicted_class, confidence, all_probs = predict_image(args.image)
    show_result(args.image, predicted_class, confidence, all_probs)

elif args.folder:
    if not os.path.exists(args.folder):
        print(f"Folder not found: {args.folder}")
        sys.exit(1)
    test_folder(args.folder)

else:
    print("Usage:")
    print("  Test single image : python test_model.py --image path/to/image.jpg")
    print("  Test full folder  : python test_model.py --folder path/to/folder/")