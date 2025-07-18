import torch
from facenet_pytorch import MTCNN, InceptionResnetV1
from PIL import Image
import os
import torch.nn.functional as F
from image_utils import load_image_corrected 
from typing import List, Tuple
import cv2 # Import OpenCV
import numpy as np # Import NumPy for image array conversion

# --- New Function for Image Preprocessing ---
def _preprocess_image_for_detection(pil_img: Image.Image) -> Image.Image:
    """
    Applies image enhancement techniques to improve face detection in challenging conditions.
    Converts PIL image to OpenCV format, applies CLAHE, and converts back to PIL.
    """
    # Convert PIL Image to OpenCV format (NumPy array, BGR color)
    cv_img = np.array(pil_img.convert('RGB')) # Convert to RGB first
    cv_img = cv_img[:, :, ::-1].copy() # Convert RGB to BGR for OpenCV
    
    # Optional: Convert to grayscale for contrast enhancement if color isn't critical
    # gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    
    # Convert to LAB color space for better contrast enhancement (CLAHE on L-channel)
    lab = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    # Apply Contrast Limited Adaptive Histogram Equalization (CLAHE) to the L-channel
    # clipLimit: threshold for contrast limiting. Larger values give more contrast.
    # tileGridSize: size of grid for histogram equalization. Smaller means more local.
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)) # Tunable parameters
    cl = clahe.apply(l)
    
    # Merge the enhanced L-channel back with original A and B channels
    limg = cv2.merge((cl, a, b))
    
    # Convert back to BGR and then to PIL Image
    enhanced_cv_img = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    enhanced_pil_img = Image.fromarray(enhanced_cv_img[:, :, ::-1]) # Convert BGR back to RGB for PIL

    return enhanced_pil_img

class FaceRecognizer:
    """
    A class for performing facial recognition using MTCNN for face detection
    and InceptionResnetV1 for embedding generation. It supports building
    and loading a database of known faces for recognition.
    """
    def __init__(self, dataset_path: str = 'dataset', recognition_threshold: float = 0.75,
                 mtcnn_thresholds: Tuple[float, float, float] = (0.6, 0.7, 0.7)): # Make thresholds configurable
        """
        Initializes the FaceRecognizer with device setup, model loading,
        and database path.

        Args:
            dataset_path (str): The root directory containing subfolders for
                                each person's images (e.g., dataset/PersonA, dataset/PersonB).
            recognition_threshold (float): The cosine similarity threshold above which
                                           a face is considered recognized.
            mtcnn_thresholds (Tuple[float, float, float]): Thresholds for MTCNN's P-Net, R-Net, and O-Net.
                                                            Lower values make detection more permissive.
        """
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Using device: {self.device}")
        
        self.mtcnn = MTCNN(image_size=160, margin=20, device=self.device, keep_all=True,
                           thresholds=mtcnn_thresholds) # Pass thresholds here
        
        self.model = InceptionResnetV1(pretrained='vggface2').eval().to(self.device)
        
        self.dataset_path = dataset_path
        self.recognition_threshold = recognition_threshold
        self.embeddings: torch.Tensor = None
        self.names: List[str] = []

    def build_database(self) -> None:
        """
        Builds the face recognition database by processing images from the
        `self.dataset_path`. It detects faces, generates embeddings, and
        saves them to 'embeddings.pth'.
        """
        print(f"Building database from: {self.dataset_path}")
        new_embeddings_list: List[torch.Tensor] = []
        new_names: List[str] = []
        
        # --- Debugging counters (keep them for now) ---
        total_images_processed = 0
        total_faces_detected_for_db = 0
        # --- End debugging counters ---

        if not os.path.exists(self.dataset_path):
            print(f"Error: Dataset path '{self.dataset_path}' not found.")
            return

        for person_name in os.listdir(self.dataset_path):
            person_path = os.path.join(self.dataset_path, person_name)
            if not os.path.isdir(person_path) or person_name.lower() == "input":
                continue

            print(f"Processing images for: {person_name}")
            for file_name in os.listdir(person_path):
                img_path = os.path.join(person_path, file_name)
                if not file_name.lower().endswith(('.jpg', '.jpeg', '.png')):
                    continue
                
                total_images_processed += 1

                try:
                    img = load_image_corrected(img_path)
                except Exception as e:
                    print(f"Error loading image {img_path}: {e}")
                    continue

                # --- Apply preprocessing before MTCNN detection for DB images ---
                processed_img = _preprocess_image_for_detection(img)
                # --- End preprocessing ---

                faces = self.mtcnn(processed_img) # Use the processed image for detection
                
                if faces is not None and faces.dim() == 4 and len(faces) > 0:
                    num_faces_in_current_image = faces.shape[0]
                    total_faces_detected_for_db += num_faces_in_current_image
                    
                    print(f"  --> Detected {num_faces_in_current_image} face(s) in {file_name}")
                    
                    embeddings_batch = self.model(faces.to(self.device))
                    embeddings_batch = F.normalize(embeddings_batch, p=2, dim=1).detach()
                    
                    for i in range(embeddings_batch.shape[0]):
                        new_embeddings_list.append(embeddings_batch[i].cpu()) 
                        new_names.append(person_name)
                else:
                    print(f"  --> No face detected in {file_name}")

        if new_embeddings_list:
            self.embeddings = torch.stack(new_embeddings_list).to(self.device)
            self.names = new_names
            torch.save({'embeddings': self.embeddings.cpu(), 'names': self.names}, 'embeddings.pth')
            print(f"Database built and saved with {len(new_names)} entries.")
            print(f"Summary: Processed {total_images_processed} image files. Total faces added to DB: {total_faces_detected_for_db}")
        else:
            print("No valid faces found in the dataset. Database not built.")
            self.embeddings = None
            self.names = []

    def load_database(self) -> None:
        """
        Loads face embeddings and names from 'embeddings.pth' into memory.
        """
        try:
            data = torch.load('embeddings.pth', map_location=self.device)
            self.embeddings = data['embeddings'].to(self.device)
            self.names = data['names']
            print(f"Database loaded with {len(self.names)} entries.")
        except FileNotFoundError:
            print("Error: embeddings.pth not found. Please build the database first (run build_db.py).")
            self.embeddings = None
            self.names = []
        except Exception as e:
            print(f"Error loading database: {e}")
            self.embeddings = None
            self.names = []

    def recognize(self, frame: Image.Image) -> List[str]:
        """
        Recognizes faces in the given input image frame.

        Args:
            frame (PIL.Image.Image): The input image frame (PIL Image object).

        Returns:
            List[str]: A list of recognized names for each detected face, or
                       "Unknown", "No face", or "Database empty". If multiple
                       faces are detected, a list of results is returned.
        """
        img = frame # Expecting a PIL Image

        # --- Apply preprocessing before MTCNN detection for recognition images ---
        processed_img = _preprocess_image_for_detection(img)
        # --- End preprocessing ---

        faces_detected = self.mtcnn(processed_img) # Use the processed image for detection
        
        if faces_detected is None or len(faces_detected) == 0:
            return ["No face"]

        if self.embeddings is None or self.embeddings.numel() == 0:
            return ["Database empty"] * faces_detected.shape[0] if faces_detected.dim() == 4 else ["Database empty"]

        results: List[str] = []

        if faces_detected.dim() == 4:
            input_embeddings = self.model(faces_detected.to(self.device))
            input_embeddings = F.normalize(input_embeddings, p=2, dim=1).detach()

            similarities_matrix = torch.mm(input_embeddings, self.embeddings.T)

            max_sim_values, idx_values = torch.max(similarities_matrix, dim=1)

            for i in range(max_sim_values.shape[0]):
                max_sim = max_sim_values[i].item()
                idx = idx_values[i].item()
                
                print(f"Max similarity for face {i+1}: {max_sim:.3f}, Match: {self.names[idx]}")
                
                if max_sim > self.recognition_threshold:
                    results.append(self.names[idx])
                else:
                    results.append("Unknown")
        else:
            print("Warning: MTCNN did not return a batched tensor. Processing faces individually.")
            for face_tensor in faces_detected:
                if face_tensor.dim() == 3:
                    input_embedding = self.model(face_tensor.to(self.device).unsqueeze(0))
                    input_embedding = F.normalize(input_embedding, p=2, dim=1).detach()
                    
                    similarities = F.cosine_similarity(input_embedding, self.embeddings)
                    max_sim, idx = torch.max(similarities, dim=0)

                    print(f"Max similarity: {max_sim.item():.3f}, Match: {self.names[idx.item()]}")
                    if max_sim.item() > self.recognition_threshold:
                        results.append(self.names[idx.item()])
                    else:
                        results.append("Unknown")
                else:
                    results.append("Invalid face tensor format")

        return results