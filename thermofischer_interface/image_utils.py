from PIL import Image as Im
import io
import base64
import numpy as np


def image_to_base64(image: np.ndarray) -> str:
    image = Im.fromarray(image)
    buffered = io.BytesIO()
    image.save(buffered, format="JPEG")
    img_str = base64.b64encode(buffered.getvalue())
    img_str = str(img_str)
    img_str = img_str.lstrip("'b")
    img_str = img_str.rstrip("'")
    response_img = "data:image/jpeg;base64," + img_str
    return response_img

def base64_to_image(base64_string: str) -> np.ndarray:
    decoded = base64.b64decode(base64_string)
    image = Im.open(io.BytesIO(decoded))
    image = np.array(image)
    return image