import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt

# Intel RealSense pipeline'i oluştur
pipeline = rs.pipeline()

# Konfigürasyon ayarlarını yap
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Pipeline'i başlat
pipeline.start(config)

def format_coord(x, y):
    """İmleç koordinatlarını ve derinlik bilgisini formatlar."""
    x = int(x + 0.5)
    y = int(y + 0.5)
    if x >= 0 and x < depth_image.shape[1] and y >= 0 and y < depth_image.shape[0]:
        z = depth_image[y, x]
        return f'x={x}, y={y}, z={z:.2f} meters'
    else:
        return f'x={x}, y={y}, z=N/A'

try:
    # Plot hazırlığı
    plt.ion()  # Interactive mode'u aç
    fig, ax = plt.subplots()
    depth_image = np.zeros((480, 640))  # Boş bir görüntü ile başla
    im = ax.imshow(depth_image, cmap='jet', interpolation='nearest')  # jet renk haritası ile başla
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label('Depth Scale (meters)')
    plt.title('Intel RealSense Depth Image')
    ax.format_coord = format_coord  # İmleç koordinat formatlama fonksiyonunu ayarla

    while True:
        # Frame set'i bekle
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        if not depth_frame:
            continue

        # Derinlik verilerini numpy dizisine çevir
        depth_image = np.asanyarray(depth_frame.get_data())

        # Maksimum derinlik mesafesi (örneğin 5 metre)
        max_depth = 5.0
        
        # Derinlik verilerini normalize et
        depth_image = depth_image * depth_frame.get_units()  # Derinlik verilerini metreye çevir
        depth_image = np.clip(depth_image, 0, max_depth)  # Belirtilen maksimum derinlik mesafesine göre sınırla
        depth_image_normalized = (depth_image / max_depth * 255).astype(np.uint8)  # 0-255 aralığında normalize et

        # Görüntüyü güncelle
        im.set_data(depth_image_normalized)
        im.set_clim(0, 255)  # Renk haritası sınırlarını ayarla

        # Renk çubuğu etiketlerini güncelle
        tick_labels = np.linspace(0, max_depth, num=6)
        cbar.set_ticks(np.linspace(0, 255, num=6))
        cbar.set_ticklabels([f"{label:.2f}" for label in tick_labels])
        
        plt.draw()
        plt.pause(0.01)

finally:
    # Pipeline'i durdur
    pipeline.stop()

