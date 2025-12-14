---
title: Synthetic Data Generation and Perception
sidebar_position: 3
---

# Synthetic Data Generation and Perception

## Introduction to Synthetic Data in Robotics

Synthetic data generation has become a cornerstone of modern robotics development, particularly for training perception systems. Unlike traditional data collection that requires physical robots and extensive real-world testing, synthetic data provides unlimited, diverse, and perfectly annotated training samples. NVIDIA Isaac's synthetic data capabilities enable the creation of large-scale, high-quality datasets that can significantly accelerate perception system development.

## Why Synthetic Data Matters

### Challenges with Real Data
- **Limited Diversity**: Real-world data may not cover all possible scenarios
- **Annotation Cost**: Manual annotation is expensive and time-consuming
- **Safety Concerns**: Some scenarios are dangerous to collect data from
- **Weather/Lighting Constraints**: Limited by environmental conditions
- **Long Collection Times**: Gathering sufficient data takes considerable time

### Advantages of Synthetic Data
- **Infinite Scenarios**: Generate any situation imaginable
- **Perfect Annotations**: Automatic ground truth generation
- **Cost-Effective**: Generate thousands of samples quickly
- **Controlled Environment**: Systematically vary parameters
- **Safety**: No risk to robots or humans

## Isaac's Synthetic Data Pipeline

### Core Components
1. **Scene Generation**: Creating diverse environments and configurations
2. **Domain Randomization**: Systematically varying visual properties
3. **Sensor Simulation**: Accurate modeling of real sensors
4. **Annotation Generation**: Automatic labeling of all objects
5. **Data Export**: Export in standard formats for ML frameworks

### Data Generation Workflow
```python
import omni.replicator.core as rep
from omni.isaac.synthetic_utils import plot
import numpy as np

# Define the synthetic data generation workflow
def create_synthetic_dataset():
    # 1. Setup replicator
    rep.orchestrator.init_orchestrator()

    # 2. Define annotation types needed
    with rep.new_layer():
        setup_scene()
        setup_domain_randomization()
        setup_annotation_collection()
        setup_data_export()

    # 3. Run the generation process
    rep.orchestrator.run()
    rep.orchestrator.stop()

def setup_scene():
    """Define the base scene with assets and layout"""
    # Create multiple environment variants
    for env_type in ['warehouse', 'office', 'outdoor']:
        create_environment_variant(env_type)

def setup_domain_randomization():
    """Configure domain randomization parameters"""
    with rep.trigger.on_frame():
        # Randomize materials
        randomize_materials()

        # Randomize lighting
        randomize_lighting()

        # Randomize object placement
        randomize_object_poses()

def setup_annotation_collection():
    """Configure what annotations to collect"""
    # RGB images
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach(camera.get_render_product_path())

    # Depth maps
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth_annotator.attach(camera.get_render_product_path())

    # Instance segmentation
    seg_annotator = rep.AnnotatorRegistry.get_annotator("instance_segmentation")
    seg_annotator.attach(camera.get_render_product_path())

    # Bounding boxes
    bbox_annotator = rep.AnnotatorRegistry.get_annotator("bbox_2d_tight")
    bbox_annotator.attach(camera.get_render_product_path())

def setup_data_export():
    """Configure data export format and location"""
    writer = rep.WriterRegistry.get("BasicInstanceDataWriter")
    writer.initialize(
        output_dir="synthetic_dataset",
        frames_per_capture=1,
        semantic_labels=True
    )
    writer.attach([rgb_annotator, depth_annotator, seg_annotator, bbox_annotator])
```

## Domain Randomization Techniques

### Visual Domain Randomization
```python
def randomize_materials():
    """Randomize visual properties of materials"""
    # Get all materials in the scene
    materials = rep.get.materials()

    for material in materials:
        # Randomize base color
        color = rep.distribution.uniform((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        material.get_attribute("rgb").set(color)

        # Randomize roughness (0.0-1.0)
        roughness = rep.distribution.uniform(0.1, 0.9)
        material.get_attribute("roughness").set(roughness)

        # Randomize metallic (0.0-1.0)
        metallic = rep.distribution.uniform(0.0, 1.0)
        material.get_attribute("metallic").set(metallic)

        # Randomize normal map intensity
        normal_intensity = rep.distribution.uniform(0.0, 1.0)
        material.get_attribute("normalmapIntensity").set(normal_intensity)

def randomize_lighting():
    """Randomize lighting conditions"""
    # Get dome light
    dome_light = rep.get.prim_at_path("/World/DomeLight")

    # Randomize intensity (1000-10000)
    intensity = rep.distribution.uniform(1000, 10000)
    dome_light.get_attribute("intensity").set(intensity)

    # Randomize color temperature (3000K-8000K)
    color_temp = rep.distribution.uniform(3000, 8000)
    dome_light.get_attribute("inputs:color").set(kelvin_to_rgb(color_temp))

    # Add random directional lights
    for i in range(3):
        light = rep.randomizer.placement(
            prim_path=f"/World/RandomLight{i}",
            positions=rep.distribution.uniform((-5, -5, 5), (5, 5, 10)),
            rotations=rep.distribution.uniform((0, 0, 0, 1), (360, 360, 360, 1))
        )
        light.get_attribute("intensity").set(rep.distribution.uniform(200, 800))

def randomize_textures():
    """Randomize textures and surface details"""
    # Apply random textures from a library
    texture_library = [
        "/path/to/texture1.jpg",
        "/path/to/texture2.jpg",
        "/path/to/texture3.jpg"
    ]

    for prim in get_objects_with_materials():
        # Randomly assign texture
        texture_path = rep.distribution.choice(texture_library)
        apply_texture_to_prim(prim, texture_path)
```

### Physical Domain Randomization
```python
def randomize_physics_properties():
    """Randomize physical properties for realistic simulation"""
    objects = rep.get.objects()

    for obj in objects:
        # Randomize friction coefficients
        static_friction = rep.distribution.uniform(0.1, 1.0)
        dynamic_friction = rep.distribution.uniform(0.1, 1.0)
        restitution = rep.distribution.uniform(0.0, 0.5)

        obj.get_attribute("physics:staticFriction").set(static_friction)
        obj.get_attribute("physics:dynamicFriction").set(dynamic_friction)
        obj.get_attribute("physics:restitution").set(restitution)

        # Randomize mass for non-fixed objects
        if not obj.get_attribute("physics:kinematicEnabled").get():
            # Scale mass based on volume
            base_mass = obj.get_attribute("physics:mass").get()
            mass_multiplier = rep.distribution.uniform(0.8, 1.2)
            obj.get_attribute("physics:mass").set(base_mass * mass_multiplier)

def randomize_object_poses():
    """Randomize object positions and orientations"""
    objects = rep.get.movable_objects()

    for obj in objects:
        # Random position within bounds
        position = rep.distribution.uniform(
            (-2.0, -2.0, 0.1),  # min bounds
            (2.0, 2.0, 2.0)    # max bounds
        )

        # Random rotation
        rotation = rep.distribution.uniform(
            (0, 0, 0, 1),      # min rotation (quat)
            (360, 360, 360, 1) # max rotation
        )

        obj.set_world_pos(position)
        obj.set_world_rotation(rotation)
```

## Sensor Simulation and Data Collection

### Camera Simulation
```python
def setup_camera_simulation():
    """Configure realistic camera simulation"""
    from omni.replicator.core import Camera
    import numpy as np

    # Create camera with realistic parameters
    camera = Camera(
        prim_path="/World/Robot/Camera",
        frequency=30,  # 30 Hz
        resolution=(640, 480),
        position=(0.0, 0.0, 0.3),  # 30cm above ground
        orientation=(0.0, 0.0, 0.0, 1.0)  # Looking forward
    )

    # Configure camera intrinsics
    camera.set_focal_length(35.0)  # 35mm equivalent
    camera.set_horizontal_aperture(36.0)  # Full frame
    camera.set_vertical_aperture(24.0)

    # Add noise models
    camera.set_noise_model("RGBNoise",
                          noise_intensity=0.01,  # 1% noise
                          noise_type="gaussian")

    return camera

def collect_camera_data():
    """Collect synchronized camera data"""
    # RGB data
    rgb_data = camera.get_rgb_data()

    # Depth data
    depth_data = camera.get_depth_data()

    # Semantic segmentation
    seg_data = camera.get_semantic_segmentation()

    # Instance segmentation
    instance_data = camera.get_instance_segmentation()

    # Normal maps
    normal_data = camera.get_normal_data()

    return {
        'rgb': rgb_data,
        'depth': depth_data,
        'semantic': seg_data,
        'instance': instance_data,
        'normals': normal_data
    }
```

### LiDAR Simulation
```python
def setup_lidar_simulation():
    """Configure realistic LiDAR simulation"""
    from omni.isaac.sensor import LidarRtx

    # Create realistic LiDAR sensor
    lidar = LidarRtx(
        prim_path="/World/Robot/Lidar",
        translation=np.array([0.0, 0.0, 0.5]),  # 50cm high
        config="Example_Rotary",  # Standard 360-degree configuration
        rotation_frequency=20,    # 20 Hz rotation
        points_per_second=25000,  # 25k points per second
        horizontal_resolution=1,  # 1 degree horizontal
        vertical_resolution=2,    # 2 degree vertical
        horizontal_min=0,         # Start angle
        horizontal_max=360,       # End angle
        range_min=0.1,            # 0.1m minimum range
        range_max=25.0,           # 25m maximum range
        # Add realistic noise
        noise_mean=0.0,
        noise_std=0.01          # 1cm standard deviation
    )

    return lidar

def collect_lidar_data():
    """Collect LiDAR point cloud data"""
    # Get raw point cloud
    point_cloud = lidar.get_point_cloud()

    # Get range data (for 2D LiDAR simulation)
    range_data = lidar.get_range_data()

    # Get intensity data
    intensity_data = lidar.get_intensity_data()

    return {
        'point_cloud': point_cloud,
        'ranges': range_data,
        'intensities': intensity_data
    }
```

## Perception Pipeline Integration

### Training Data Format
```python
def format_training_data(annotation_data, output_format="coco"):
    """Convert synthetic data to standard ML formats"""

    if output_format == "coco":
        return format_coco(annotation_data)
    elif output_format == "kitti":
        return format_kitti(annotation_data)
    elif output_format == "darknet":
        return format_darknet(annotation_data)
    else:
        raise ValueError(f"Unsupported format: {output_format}")

def format_coco(annotation_data):
    """Format data in COCO format for object detection"""
    coco_format = {
        "info": {
            "description": "Synthetic Robot Perception Dataset",
            "version": "1.0",
            "year": 2025,
            "contributor": "Isaac Synthetic Data Generator",
            "date_created": "2025-01-01"
        },
        "licenses": [
            {
                "id": 1,
                "name": "Synthetic Data License",
                "url": "http://creativecommons.org/licenses/by/4.0/"
            }
        ],
        "categories": [],
        "images": [],
        "annotations": []
    }

    # Add categories
    category_id = 0
    for category_name in get_unique_categories(annotation_data):
        coco_format["categories"].append({
            "id": category_id,
            "name": category_name,
            "supercategory": "object"
        })
        category_id += 1

    # Process each image
    image_id = 0
    annotation_id = 0

    for img_data in annotation_data:
        # Add image info
        image_info = {
            "id": image_id,
            "width": img_data["width"],
            "height": img_data["height"],
            "file_name": f"synthetic_{image_id:06d}.jpg",
            "license": 1,
            "date_captured": "2025-01-01 00:00:00"
        }
        coco_format["images"].append(image_info)

        # Add annotations for this image
        for obj in img_data["objects"]:
            bbox = obj["bbox"]  # [x, y, width, height]
            segmentation = obj["segmentation"]

            annotation_info = {
                "id": annotation_id,
                "image_id": image_id,
                "category_id": get_category_id(obj["category"]),
                "bbox": [float(x) for x in bbox],
                "area": bbox[2] * bbox[3],  # width * height
                "iscrowd": 0,
                "segmentation": [segmentation] if segmentation else [],
                "pose": obj.get("pose", [0, 0, 0]),  # [x, y, z] pose
                "dimensions": obj.get("dimensions", [1, 1, 1])  # [w, h, l] dimensions
            }
            coco_format["annotations"].append(annotation_info)
            annotation_id += 1

        image_id += 1

    return coco_format

def get_unique_categories(annotation_data):
    """Extract unique category names from annotation data"""
    categories = set()
    for img_data in annotation_data:
        for obj in img_data["objects"]:
            categories.add(obj["category"])
    return sorted(list(categories))

def get_category_id(category_name):
    """Map category name to ID (implementation depends on your mapping)"""
    # This would be based on your specific category mapping
    category_mapping = {
        "person": 0,
        "robot": 1,
        "obstacle": 2,
        "shelf": 3,
        # ... add more categories as needed
    }
    return category_mapping.get(category_name, -1)
```

### Data Augmentation
```python
def augment_synthetic_data(image_data, annotations):
    """Apply augmentation to synthetic data"""
    augmented_samples = []

    # Original sample
    augmented_samples.append({
        'image': image_data,
        'annotations': annotations
    })

    # Horizontal flip
    flipped_img, flipped_ann = horizontal_flip(image_data, annotations)
    augmented_samples.append({
        'image': flipped_img,
        'annotations': flipped_ann
    })

    # Brightness adjustment
    for brightness_factor in [0.8, 1.2]:
        bright_img = adjust_brightness(image_data, brightness_factor)
        augmented_samples.append({
            'image': bright_img,
            'annotations': annotations  # Annotations remain the same
        })

    # Gaussian noise
    noisy_img = add_gaussian_noise(image_data, std=0.01)
    augmented_samples.append({
        'image': noisy_img,
        'annotations': annotations
    })

    return augmented_samples

def horizontal_flip(image, annotations):
    """Apply horizontal flip to image and adjust annotations"""
    import cv2

    # Flip image
    flipped_image = cv2.flip(image, 1)

    # Adjust bounding boxes
    flipped_annotations = []
    for ann in annotations:
        if 'bbox' in ann:
            x, y, w, h = ann['bbox']
            # Flip x coordinate
            flipped_x = image.shape[1] - (x + w)
            ann['bbox'] = [flipped_x, y, w, h]

        # Adjust other spatial annotations as needed
        flipped_annotations.append(ann)

    return flipped_image, flipped_annotations
```

## Perception Model Training

### Integration with ML Frameworks
```python
def train_perception_model_with_synthetic_data():
    """Example training pipeline using synthetic data"""
    import torch
    import torchvision
    from torch.utils.data import DataLoader, Dataset

    class SyntheticDataset(Dataset):
        def __init__(self, data_dir, transforms=None):
            self.data_dir = data_dir
            self.transforms = transforms
            self.samples = self.load_sample_list()

        def __len__(self):
            return len(self.samples)

        def __getitem__(self, idx):
            sample = self.samples[idx]

            # Load image
            image_path = f"{self.data_dir}/images/{sample['image']}"
            image = torchvision.io.read_image(image_path)

            # Load annotations
            boxes = torch.tensor(sample['annotations']['boxes'], dtype=torch.float32)
            labels = torch.tensor(sample['annotations']['labels'], dtype=torch.int64)

            target = {
                'boxes': boxes,
                'labels': labels,
                # Add other targets as needed
            }

            if self.transforms:
                image, target = self.transforms(image, target)

            return image, target

        def load_sample_list(self):
            """Load list of samples from annotation file"""
            # Load from COCO format or other format
            with open(f"{self.data_dir}/annotations.json", 'r') as f:
                annotations = json.load(f)
            return annotations['images']

    # Create dataset
    dataset = SyntheticDataset("synthetic_dataset", transforms=get_transforms())
    dataloader = DataLoader(dataset, batch_size=8, shuffle=True, num_workers=4)

    # Load pre-trained model
    model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)

    # Replace the classifier with the number of classes in your dataset
    num_classes = 10  # background + 9 object classes
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

    # Training loop
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    model.to(device)

    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(params, lr=0.005, momentum=0.9, weight_decay=0.0005)
    lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=3, gamma=0.1)

    num_epochs = 10

    for epoch in range(num_epochs):
        train_one_epoch(model, optimizer, dataloader, device, epoch, print_freq=10)
        lr_scheduler.step()

    return model

def get_transforms():
    """Get data transforms for training"""
    transforms = []
    transforms.append(torchvision.transforms.ToTensor())
    # Add other transforms as needed
    return torchvision.transforms.Compose(transforms)
```

## Quality Assessment and Validation

### Synthetic vs. Real Similarity
```python
def assess_synthetic_data_quality(synthetic_data, real_data):
    """Assess the quality and similarity of synthetic data to real data"""

    # Statistical comparison
    synthetic_stats = compute_image_statistics(synthetic_data)
    real_stats = compute_image_statistics(real_data)

    # Compare distributions
    distribution_similarity = compare_distributions(synthetic_stats, real_stats)

    # Feature space comparison
    synthetic_features = extract_features(synthetic_data)
    real_features = extract_features(real_data)

    # Calculate domain gap
    domain_gap = calculate_domain_gap(synthetic_features, real_features)

    return {
        'distribution_similarity': distribution_similarity,
        'domain_gap': domain_gap,
        'quality_score': compute_quality_score(distribution_similarity, domain_gap)
    }

def compute_image_statistics(images):
    """Compute statistical properties of images"""
    means = []
    stds = []

    for img in images:
        # Convert to numpy if needed
        if hasattr(img, 'cpu'):
            img = img.cpu().numpy()

        # Compute mean and std per channel
        means.append([img[:, :, i].mean() for i in range(img.shape[2])])
        stds.append([img[:, :, i].std() for i in range(img.shape[2])])

    return {
        'mean': np.mean(means, axis=0),
        'std': np.mean(stds, axis=0),
        'histograms': compute_color_histograms(images)
    }

def calculate_domain_gap(synthetic_features, real_features):
    """Calculate the domain gap between synthetic and real features"""
    from sklearn.metrics.pairwise import cosine_similarity

    # Compute MMD (Maximum Mean Discrepancy) or other distance metric
    # This is a simplified version - in practice, use more sophisticated metrics

    syn_mean = np.mean(synthetic_features, axis=0)
    real_mean = np.mean(real_features, axis=0)

    # Euclidean distance between feature means
    domain_gap = np.linalg.norm(syn_mean - real_mean)

    return domain_gap
```

## Practical Exercise: Object Detection Dataset

Create a complete synthetic dataset for object detection with:

1. **Scene Setup**: Create multiple indoor environments (warehouse, office, home)
2. **Object Placement**: Randomly place various objects with realistic poses
3. **Domain Randomization**: Apply visual and physical randomization
4. **Data Collection**: Generate RGB, depth, segmentation, and bounding box annotations
5. **Format Conversion**: Export in COCO format for training
6. **Quality Assessment**: Validate synthetic data quality

This exercise will provide hands-on experience with the complete synthetic data generation pipeline.

## Best Practices

### Data Quality
- **Realistic Physics**: Ensure objects behave according to physical laws
- **Proper Lighting**: Use realistic lighting conditions
- **Accurate Sensors**: Model sensor noise and limitations
- **Consistent Annotation**: Maintain annotation quality across samples

### Performance
- **Efficient Rendering**: Use appropriate LOD and rendering settings
- **Batch Processing**: Generate data in batches for efficiency
- **Parallel Generation**: Use multiple instances for faster generation
- **Storage Optimization**: Compress data appropriately

### Validation
- **Reality Check**: Validate synthetic data against real data
- **Transfer Learning**: Test model performance on real data
- **Domain Gap Analysis**: Measure and minimize the domain gap
- **Iterative Improvement**: Refine generation process based on results

## Summary

Synthetic data generation with NVIDIA Isaac provides:
- **Unlimited Training Data**: Generate as much data as needed
- **Perfect Annotations**: Automatic ground truth generation
- **Controlled Variation**: Systematically vary parameters
- **Cost-Effective**: No physical robot required
- **Safe Testing**: No risk to equipment or people

The combination of photorealistic rendering, accurate physics, and domain randomization makes Isaac's synthetic data generation a powerful tool for developing robust perception systems that can transfer from simulation to reality.