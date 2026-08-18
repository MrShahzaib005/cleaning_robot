#!/usr/bin/env python3
"""
Floor Type AI Model Trainer
Trains MobileNetV2 on carpet / tiles / wood images
Output: ai_models/model/floor_model.h5
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Dense, GlobalAveragePooling2D, Dropout, BatchNormalization
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau
from tensorflow.keras import regularizers
import json
from PIL import ImageFile
ImageFile.LOAD_TRUNCATED_IMAGES = True

# ===== PATHS =====
BASE_DIR    = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR = os.path.join(BASE_DIR, 'dataset')
MODEL_DIR   = os.path.join(BASE_DIR, 'model')
os.makedirs(MODEL_DIR, exist_ok=True)

# ===== CONFIG =====
IMG_SIZE    = 224
BATCH_SIZE  = 8
EPOCHS      = 60
LR          = 0.0001
CLASSES     = ['carpet', 'tiles', 'wood']

print("=" * 50)
print("   FLOOR TYPE AI MODEL TRAINER (MobileNetV2)")
print("=" * 50)
print(f"Dataset : {DATASET_DIR}")
print(f"Model   : {MODEL_DIR}")
print(f"Classes : {CLASSES}")
print()

# ===== CHECK DATASET =====
for cls in CLASSES:
    path = os.path.join(DATASET_DIR, cls)
    count = len(os.listdir(path)) if os.path.exists(path) else 0
    print(f"  {cls}: {count} images")
print()

# ===== DATA AUGMENTATION =====
# rescale=1./255 works best for MobileNetV2 with frozen base
train_datagen = ImageDataGenerator(
    rescale=1./255,
    validation_split=0.2,
    rotation_range=10,
    width_shift_range=0.1,
    height_shift_range=0.1,
    shear_range=0.08,
    zoom_range=0.1,
    horizontal_flip=True,
    vertical_flip=False,
    brightness_range=[0.85, 1.15],
    fill_mode='nearest'
)

val_datagen = ImageDataGenerator(
    rescale=1./255,
    validation_split=0.2
)

train_generator = train_datagen.flow_from_directory(
    DATASET_DIR,
    target_size=(IMG_SIZE, IMG_SIZE),
    batch_size=BATCH_SIZE,
    class_mode='categorical',
    subset='training',
    shuffle=True
)

val_generator = val_datagen.flow_from_directory(
    DATASET_DIR,
    target_size=(IMG_SIZE, IMG_SIZE),
    batch_size=BATCH_SIZE,
    class_mode='categorical',
    subset='validation',
    shuffle=False
)

print(f"\nTraining samples   : {train_generator.samples}")
print(f"Validation samples : {val_generator.samples}")
print(f"Classes found      : {train_generator.class_indices}")

# Save class indices
class_indices_path = os.path.join(MODEL_DIR, 'class_indices.json')
with open(class_indices_path, 'w') as f:
    json.dump(train_generator.class_indices, f)
print(f"Class indices saved: {class_indices_path}")

# ===== BUILD MODEL =====
print("\nBuilding MobileNetV2 model...")
base_model = MobileNetV2(
    weights='imagenet',
    include_top=False,
    input_shape=(IMG_SIZE, IMG_SIZE, 3)
)

# Fully frozen base
base_model.trainable = False

x = base_model.output
x = GlobalAveragePooling2D()(x)
x = Dense(512, activation='relu', kernel_regularizer=regularizers.l2(1e-4))(x)
x = BatchNormalization()(x)
x = Dropout(0.4)(x)
x = Dense(256, activation='relu', kernel_regularizer=regularizers.l2(1e-4))(x)
x = BatchNormalization()(x)
x = Dropout(0.3)(x)
x = Dense(128, activation='relu', kernel_regularizer=regularizers.l2(1e-4))(x)
x = Dropout(0.2)(x)
output = Dense(len(CLASSES), activation='softmax')(x)

model = Model(inputs=base_model.input, outputs=output)

model.compile(
    optimizer=Adam(learning_rate=LR),
    loss='categorical_crossentropy',
    metrics=['accuracy']
)

print(f"Model built! Total params: {model.count_params():,}")

# ===== CALLBACKS =====
callbacks = [
    ModelCheckpoint(
        os.path.join(MODEL_DIR, 'floor_model.h5'),
        monitor='val_accuracy',
        save_best_only=True,
        verbose=1
    ),
    EarlyStopping(
        monitor='val_accuracy',
        patience=15,
        restore_best_weights=True,
        verbose=1
    ),
    ReduceLROnPlateau(
        monitor='val_loss',
        factor=0.5,
        patience=5,
        min_lr=1e-7,
        verbose=1
    )
]

# ===== TRAIN =====
print("\n" + "=" * 50)
print("   STARTING TRAINING...")
print("=" * 50)

history = model.fit(
    train_generator,
    epochs=EPOCHS,
    validation_data=val_generator,
    callbacks=callbacks,
    verbose=1
)

# ===== RESULTS =====
best_acc = max(history.history['val_accuracy'])
print("\n" + "=" * 50)
print(f"   TRAINING COMPLETE!")
print(f"   Best Accuracy: {best_acc*100:.2f}%")
print(f"   Model saved : {MODEL_DIR}/floor_model.h5")
print("=" * 50)

# ===== SAVE PLOT =====
plt.figure(figsize=(12, 4))
plt.subplot(1, 2, 1)
plt.plot(history.history['accuracy'], label='Train')
plt.plot(history.history['val_accuracy'], label='Validation')
plt.title('Model Accuracy')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(history.history['loss'], label='Train')
plt.plot(history.history['val_loss'], label='Validation')
plt.title('Model Loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()

plt.tight_layout()
plot_path = os.path.join(MODEL_DIR, 'training_results.png')
plt.savefig(plot_path)
print(f"Training plot saved: {plot_path}")