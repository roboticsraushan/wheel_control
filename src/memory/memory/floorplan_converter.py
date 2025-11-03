#!/usr/bin/env python3
"""
Floorplan Converter - Converts uploaded images to ROS map format (PGM + YAML).

Supports:
  - Input formats: PNG, JPG, JPEG, BMP, TIFF
  - Output format: PGM (Portable Graymap) with accompanying YAML metadata
  - Automatic thresholding for binary maps
  - Configurable resolution and origin
  - Proper ROS map format compliance (white=free, black=occupied, gray=unknown)

Usage:
  - Call the /memory/convert_floorplan service with an image file path
  - Or use the web interface to upload an image
"""

import cv2
import numpy as np
import yaml
from pathlib import Path
from datetime import datetime
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('FloorplanConverter')


class FloorplanConverter:
    """Converts various image formats to ROS-compatible map format."""

    # Supported input formats
    SUPPORTED_FORMATS = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.tif']

    # ROS map thresholds (OpenCV uses 0=black, 255=white)
    THRESHOLD_LOWER = 127  # Pixels below this are obstacles (black -> 0)
    THRESHOLD_UPPER = 255  # Pixels at/above this are free space (white -> 255)

    def __init__(self, output_dir: str = 'data/maps'):
        """
        Initialize converter.

        Args:
            output_dir: Directory to save converted maps
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        logger.info(f'Floorplan converter initialized. Output dir: {self.output_dir}')

    def convert(
        self,
        image_path: str,
        output_name: str = None,
        resolution: float = 0.05,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        auto_threshold: bool = True,
        invert: bool = False,
    ) -> dict:
        """
        Convert image to ROS map format.

        Args:
            image_path: Path to input image file
            output_name: Name for output files (without extension). If None, uses image filename.
            resolution: Map resolution in meters/pixel. Default 0.05 (20 pixels per meter)
            origin_x: Map origin X coordinate in meters
            origin_y: Map origin Y coordinate in meters
            auto_threshold: If True, automatically threshold grayscale to binary
            invert: If True, invert the image (swap black/white)

        Returns:
            dict with keys:
                - success: bool indicating conversion success
                - pgm_path: path to generated PGM file
                - yaml_path: path to generated YAML file
                - message: status message
                - map_metadata: dict with map information
        """
        try:
            input_path = Path(image_path)

            # Validate input file
            if not input_path.exists():
                return {
                    'success': False,
                    'message': f'Input file not found: {image_path}',
                    'pgm_path': None,
                    'yaml_path': None,
                    'map_metadata': None,
                }

            if input_path.suffix.lower() not in self.SUPPORTED_FORMATS:
                return {
                    'success': False,
                    'message': f'Unsupported format: {input_path.suffix}. '
                    f'Supported: {", ".join(self.SUPPORTED_FORMATS)}',
                    'pgm_path': None,
                    'yaml_path': None,
                    'map_metadata': None,
                }

            # Load image
            logger.info(f'Loading image from {image_path}')
            img = cv2.imread(str(input_path))

            if img is None:
                return {
                    'success': False,
                    'message': f'Failed to load image: {image_path}',
                    'pgm_path': None,
                    'yaml_path': None,
                    'map_metadata': None,
                }

            # Convert to grayscale
            if len(img.shape) == 3:
                # Color image -> grayscale
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                logger.info(f'Converted color image to grayscale')
            else:
                gray = img

            # Apply thresholding if requested
            if auto_threshold:
                _, binary = cv2.threshold(
                    gray, self.THRESHOLD_LOWER, self.THRESHOLD_UPPER, cv2.THRESH_BINARY
                )
                logger.info('Applied automatic thresholding')
            else:
                binary = gray

            # Invert if requested (swap black/white)
            if invert:
                binary = cv2.bitwise_not(binary)
                logger.info('Inverted image colors')

            # Generate output filename if not provided
            if output_name is None:
                output_name = input_path.stem

            # Ensure unique filename (avoid overwriting)
            base_name = output_name
            counter = 1
            pgm_path = self.output_dir / f'{base_name}.pgm'
            while pgm_path.exists():
                base_name = f'{output_name}_{counter}'
                pgm_path = self.output_dir / f'{base_name}.pgm'
                counter += 1

            yaml_path = self.output_dir / f'{base_name}.yaml'

            # Save PGM file
            success = cv2.imwrite(str(pgm_path), binary)
            if not success:
                return {
                    'success': False,
                    'message': f'Failed to write PGM file: {pgm_path}',
                    'pgm_path': None,
                    'yaml_path': None,
                    'map_metadata': None,
                }

            logger.info(f'Saved PGM file: {pgm_path}')

            # Create YAML metadata
            map_metadata = {
                'image': str(pgm_path.name),
                'resolution': resolution,
                'origin': [origin_x, origin_y, 0.0],
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
                'negate': 0,
                # Additional metadata
                'map_name': base_name,
                'created': datetime.now().isoformat(),
                'source_image': input_path.name,
                'size': {'width': binary.shape[1], 'height': binary.shape[0]},
                'notes': f'Converted from {input_path.name} using FloorplanConverter',
            }

            # Write YAML file
            with open(yaml_path, 'w') as f:
                yaml.dump(map_metadata, f, default_flow_style=False, sort_keys=False)

            logger.info(f'Saved YAML metadata: {yaml_path}')

            return {
                'success': True,
                'message': f'Successfully converted {input_path.name} to ROS map format',
                'pgm_path': str(pgm_path),
                'yaml_path': str(yaml_path),
                'map_metadata': map_metadata,
            }

        except Exception as e:
            logger.error(f'Conversion error: {e}', exc_info=True)
            return {
                'success': False,
                'message': f'Conversion error: {str(e)}',
                'pgm_path': None,
                'yaml_path': None,
                'map_metadata': None,
            }

    def convert_with_settings(
        self,
        image_path: str,
        settings: dict = None,
    ) -> dict:
        """
        Convert image using settings dictionary.

        Args:
            image_path: Path to input image
            settings: Dict with keys:
                - output_name: output filename
                - resolution: map resolution
                - origin_x, origin_y: map origin
                - auto_threshold: whether to auto-threshold
                - invert: whether to invert colors

        Returns:
            Conversion result dict
        """
        if settings is None:
            settings = {}

        return self.convert(
            image_path=image_path,
            output_name=settings.get('output_name'),
            resolution=settings.get('resolution', 0.05),
            origin_x=settings.get('origin_x', 0.0),
            origin_y=settings.get('origin_y', 0.0),
            auto_threshold=settings.get('auto_threshold', True),
            invert=settings.get('invert', False),
        )


# Command-line interface for testing
if __name__ == '__main__':
    import argparse
    import json

    parser = argparse.ArgumentParser(
        description='Convert floorplan images to ROS map format (PGM + YAML)'
    )
    parser.add_argument('image_path', help='Path to input image file')
    parser.add_argument(
        '-o',
        '--output',
        help='Output filename (without extension)',
        default=None,
    )
    parser.add_argument(
        '-r',
        '--resolution',
        type=float,
        default=0.05,
        help='Map resolution in meters/pixel (default: 0.05)',
    )
    parser.add_argument(
        '-x',
        '--origin-x',
        type=float,
        default=0.0,
        help='Map origin X coordinate (default: 0.0)',
    )
    parser.add_argument(
        '-y',
        '--origin-y',
        type=float,
        default=0.0,
        help='Map origin Y coordinate (default: 0.0)',
    )
    parser.add_argument(
        '--invert',
        action='store_true',
        help='Invert image colors (black <-> white)',
    )
    parser.add_argument(
        '--no-threshold',
        action='store_true',
        help='Skip automatic thresholding',
    )
    parser.add_argument(
        '-d',
        '--output-dir',
        default='data/maps',
        help='Output directory (default: data/maps)',
    )

    args = parser.parse_args()

    converter = FloorplanConverter(output_dir=args.output_dir)
    result = converter.convert(
        image_path=args.image_path,
        output_name=args.output,
        resolution=args.resolution,
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        auto_threshold=not args.no_threshold,
        invert=args.invert,
    )

    print(json.dumps(result, indent=2))

    if result['success']:
        print(f"\n✅ Conversion successful!")
        print(f"PGM file: {result['pgm_path']}")
        print(f"YAML file: {result['yaml_path']}")
    else:
        print(f"\n❌ Conversion failed: {result['message']}")
