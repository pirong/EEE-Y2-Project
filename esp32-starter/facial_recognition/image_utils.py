from PIL import Image, ExifTags
import os

def load_image_corrected(path, resize=(640, 480)):
    """
    Loads an image from disk, corrects orientation based on EXIF metadata,
    resizes it, and converts to RGB PIL Image.
    
    Args:
        path (str): Path to the image file.
        resize (tuple): Optional resize dimensions (width, height).
        
    Returns:
        PIL.Image.Image: Processed image.
    """
    img = Image.open(path)

    try:
        exif = img._getexif()
        if exif:
            orientation_key = next((k for k, v in ExifTags.TAGS.items() if v == 'Orientation'), None)
            if orientation_key and orientation_key in exif:
                orientation = exif[orientation_key]
                if orientation == 3:
                    img = img.rotate(180, expand=True)
                elif orientation == 6:
                    img = img.rotate(270, expand=True)
                elif orientation == 8:
                    img = img.rotate(90, expand=True)
    except Exception as e:
        print(f"EXIF auto-rotation failed for {os.path.basename(path)}: {e}")

    return img.resize(resize).convert("RGB")
