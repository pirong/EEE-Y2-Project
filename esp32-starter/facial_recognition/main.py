import os
from face_recognizer import FaceRecognizer
from image_utils import load_image_corrected  # Uses EXIF-corrected, resized PIL loader

def main():
    """
    Main function to initialize the FaceRecognizer, load the database,
    and test face recognition on images in the 'dataset/input' folder.
    """
    recognizer = FaceRecognizer()
    recognizer.load_database()

    input_folder = "dataset/input"
    if not os.path.exists(input_folder):
        print(f"❌ Error: '{input_folder}' not found.")
        print("📂 Please create this folder and place images you want to test inside it.")
        return

    # Check if the database was loaded successfully
    if recognizer.embeddings is None or recognizer.embeddings.numel() == 0:
        print("⚠️ Warning: Database is empty. Cannot perform recognition.")
        if os.path.exists(recognizer.dataset_path) and len(os.listdir(recognizer.dataset_path)) > 1:
            print("📌 Consider running 'python build_db.py' to build the database first.")
        return

    image_files_found = False
    for file_name in os.listdir(input_folder):
        if file_name.lower().endswith(('.jpg', '.jpeg', '.png')):
            image_files_found = True
            path = os.path.join(input_folder, file_name)
            print(f"\n🖼️ --- Testing: {file_name} ---")

            try:
                img = load_image_corrected(path)
            except Exception as e:
                print(f"❌ Error loading image {file_name}: {e}")
                continue  # Skip to the next file

            results = recognizer.recognize(img)  # Always returns a list

            # Print results for each face in the image
            if not results:
                print("⚠️ No result found for this image.")
            else:
                for name in results:
                    if name.lower() == "no face":
                        print("🙈 No face detected.")
                    elif name.lower() == "database empty":
                        print("🚫 Not recognised (Database empty).")
                    elif name.lower() == "unknown":
                        print("❓ Not recognised.")
                    else:
                        print(f"✅ Recognised: {name}")

    if not image_files_found:
        print(f"📂 No image files found in '{input_folder}'. Please place images (JPG, JPEG, PNG) there.")

if __name__ == "__main__":
    main()
